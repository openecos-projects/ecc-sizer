// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_path.h"

#include "design_edit.h"
#include "repair_backend.h"
#include "repair_path_feedback.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "sta/PathExpanded.hh"
#include "sta/PathEnd.hh"
#include "sta/TimingRole.hh"

namespace ecc {
namespace {
using detail::NetState;

struct PhaseTimer {
    double& elapsed;
    std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    ~PhaseTimer() {
        elapsed += std::chrono::duration< double, std::milli >(
                       std::chrono::steady_clock::now() - start)
                       .count();
    }
};

// A topology plan is immutable until the entire ECO is accepted. Nodes are
// bottom-up: every child index precedes its parent, and the root is last.
struct PathTreeNode {
    BufferLocation location;
    std::vector<size_t> loads, children;
    float cap = 0;
};
struct PathTree {
    std::vector<PathTreeNode> nodes;
    std::vector<odb::dbObject*> terminals;
};


struct PathBufferCandidate {
    odb::dbNet* net = nullptr;
    std::vector< odb::dbObject* > loads;
    odb::dbMaster* master = nullptr;
    BufferLocation location;
    float slack = 0;
    float predicted_gain = 0;
    size_t path_index = 0;
    float predicted_wns_gain = 0;
    float predicted_tns_gain = 0;
    path_detail::CandidateQuality quality;
    bool electrical_retry = false;
    unsigned affected_endpoints = 0;
    unsigned variant_rank = 0;
    std::string feedback_key;
    std::string group_key;
    std::string load_group_key;
    std::shared_ptr<const std::string> feedback_context;
    double ranked_wns_gain = 0, ranked_tns_gain = 0, rank_score = 0;
    bool deferred_retry = false;
    unsigned feedback_priority = 0;
    unsigned family_attempts = 0;
    double covered_tns_gain = 0;
    unsigned covered_endpoints = 0;
    std::vector< odb::dbInst* > cone;
    bool recovery = false;
    std::shared_ptr<const PathTree> tree;
    // Saved on the committed netlist, never read from deferred rollback STA.
    std::vector<std::pair<const sta::Pin*, float>> endpoints;
    double objective_wns = 0, objective_tns = 0;
};

struct CandidateKey {
    odb::dbNet* net = nullptr;
    std::vector< odb::dbObject* > loads;

    bool operator==(const CandidateKey& other) const {
        return net == other.net && loads == other.loads;
    }
};

struct CandidateKeyHash {
    size_t operator()(const CandidateKey& key) const {
        size_t hash = std::hash< const void* >{}(key.net);
        for(auto* load : key.loads) {
            hash ^= std::hash< const void* >{}(load) + 0x9e3779b9u +
                    (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

struct CandidateCollection {
    std::vector< PathBufferCandidate > candidates;
    unsigned duplicate_candidates = 0;
};

std::string terminalKey(odb::dbObject* term) {
    return std::to_string(int(term->getObjectType())) + '/' + std::to_string(term->getId());
}

std::string actionKey(const PathBufferCandidate& candidate, bool physical = true) {
    const auto master = std::to_string(candidate.master->getLib()->getId()) + '/' +
                        std::to_string(candidate.master->getId());
    std::vector<path_detail::ActionNode> nodes;
    if(candidate.tree) {
        for(const auto& node : candidate.tree->nodes) {
            path_detail::ActionNode out{master, node.location.x, node.location.y, {}, node.children};
            for(auto index : node.loads) out.terminals.push_back(terminalKey(candidate.tree->terminals.at(index)));
            nodes.push_back(std::move(out));
        }
    } else {
        path_detail::ActionNode out{master, candidate.location.x, candidate.location.y, {}, {}};
        for(auto* load : candidate.loads) out.terminals.push_back(terminalKey(load));
        nodes.push_back(std::move(out));
    }
    return path_detail::actionFingerprint(std::to_string(candidate.net->getId()), nodes, physical);
}

// Match OpenSTA's fanout-load semantics: output ports do not contribute;
// Liberty input weights override the library default, and a missing Liberty
// pin contributes one. A missing weight on an existing Liberty pin is zero.
float fanoutWeight(sta::dbNetwork* network, odb::dbObject* term) {
    auto* pin = network->dbToSta(term);
    if(!network->isLoad(pin) || network->isTopLevelPort(pin))
        return 0;
    auto* port = network->libertyPort(pin);
    if(!port)
        return 1;
    float weight = 0;
    bool exists = false;
    port->fanoutLoad(weight, exists);
    if(!exists)
        port->libertyLibrary()->defaultFanoutLoad(weight, exists);
    return exists ? weight : 0;
}

std::vector< PathFanoutCheck > fanoutChecks(
    sta::dbSta* sta, odb::dbITerm* driver, const char* phase) {
    std::vector< PathFanoutCheck > result;
    auto* network = sta->getDbNetwork();
    auto* pin = network->dbToSta(driver);
    for(auto* mode : sta->modes()) {
        PathFanoutCheck check;
        check.phase = phase;
        check.net = driver->getNet()->getName();
        check.pin = network->pathName(pin);
        check.mode = mode->name();
        sta->checkFanout(pin, mode, sta::MinMax::max(),
                         check.load, check.limit, check.slack);
        result.push_back(std::move(check));
    }
    return result;
}

float bufferFanoutLimit(sta::dbSta* sta, const detail::Buffer& buffer) {
    auto* network = sta->getDbNetwork();
    float limit = detail::kInf, value;
    bool exists;
    const auto merge = [&]() { if(exists) limit = std::min(limit, value); };
    for(auto* mode : sta->modes()) {
        mode->sdc()->fanoutLimit(network->cell(network->topInstance()),
                                sta::MinMax::max(), value, exists); merge();
        mode->sdc()->fanoutLimit(network->dbToSta(buffer.master),
                                sta::MinMax::max(), value, exists); merge();
    }
    buffer.output->fanoutLimit(sta::MinMax::max(), value, exists);
    if(!exists) buffer.cell->libertyLibrary()->defaultMaxFanout(value, exists);
    merge();
    return limit;
}

float bufferCapLimit(sta::dbSta* sta, const detail::Buffer& buffer) {
    auto* network = sta->getDbNetwork();
    float limit = buffer.max_cap, value; bool exists;
    for(auto* mode : sta->modes()) {
        mode->sdc()->capacitanceLimit(network->cell(network->topInstance()),
                                     sta::MinMax::max(), value, exists);
        if(exists) limit = std::min(limit, value);
        mode->sdc()->capacitanceLimit(network->dbToSta(buffer.master),
                                     sta::MinMax::max(), value, exists);
        if(exists) limit = std::min(limit, value);
    }
    return limit;
}

float bufferSlewLimit(sta::dbSta* sta, const detail::Buffer& buffer) {
    float limit = detail::kInf, value;
    bool exists = false;
    buffer.output->slewLimit(sta::MinMax::max(), value, exists);
    if(!exists) buffer.cell->libertyLibrary()->defaultMaxSlew(value, exists);
    if(exists) limit = std::min(limit, value);
    auto* network = sta->getDbNetwork();
    for(auto* mode : sta->modes()) {
        mode->sdc()->slewLimit(network->cell(network->topInstance()),
                              sta::MinMax::max(), value, exists);
        if(exists) limit = std::min(limit, value);
        mode->sdc()->slewLimit(network->dbToSta(buffer.master),
                              sta::MinMax::max(), value, exists);
        if(exists) limit = std::min(limit, value);
    }
    return limit;
}

float libertyFanoutWeight(sta::LibertyPort* port) {
    float weight; bool exists;
    port->fanoutLoad(weight, exists);
    if(!exists) port->libertyLibrary()->defaultFanoutLoad(weight, exists);
    return exists ? weight : 0;
}

// Spatial packing uses Liberty/SDC electrical limits, with no group-size or
// depth budget. Strict reduction of the frontier guarantees termination.
PathTree planPathTree(const NetState& state, const detail::Buffer& buffer,
                      sta::dbSta* sta, BufferLocation root_location,
                      double cap_per_length, float cap_limit, float fanout_limit) {
    PathTree tree;
    struct Item { size_t index; bool leaf; int x, y; double cap, weight; };
    std::vector<Item> frontier;
    for(size_t i = 0; i < state.loads.size(); ++i) {
        const auto& load = state.loads[i];
        const float weight = fanoutWeight(sta->getDbNetwork(), load.term);
        if(!std::isfinite(weight) || weight < 0 || weight > fanout_limit ||
           load.capacitance > cap_limit) return {};
        frontier.push_back({i, true, load.x, load.y, load.capacitance, weight});
        tree.terminals.push_back(load.term);
    }
    const auto center = [](const std::vector<Item>& items) {
        long long x = 0, y = 0;
        for(const auto& item : items) { x += item.x; y += item.y; }
        return BufferLocation{int(x / items.size()), int(y / items.size())};
    };
    const auto cap = [&](const std::vector<Item>& items, BufferLocation p) {
        double c = 0;
        for(const auto& item : items)
            c += item.cap + cap_per_length *
                (std::abs(double(p.x) - item.x) + std::abs(double(p.y) - item.y));
        return c;
    };
    const auto fits = [&](const std::vector<Item>& items, BufferLocation p) {
        double weight = 0;
        for(const auto& item : items) weight += item.weight;
        return weight <= fanout_limit && cap(items, p) <= cap_limit;
    };
    const auto append = [&](const std::vector<Item>& items, BufferLocation p) {
        PathTreeNode node; node.location = p; node.cap = cap(items, p);
        for(const auto& item : items)
            (item.leaf ? node.loads : node.children).push_back(item.index);
        tree.nodes.push_back(std::move(node));
        return Item{tree.nodes.size() - 1, false, p.x, p.y,
                    buffer.input->capacitance(), libertyFanoutWeight(buffer.input)};
    };
    while(!frontier.empty()) {
        if(fits(frontier, root_location)) {
            append(frontier, root_location);
            return tree;
        }
        int min_x = frontier.front().x, max_x = min_x;
        int min_y = frontier.front().y, max_y = min_y;
        for(const auto& item : frontier) {
            min_x = std::min(min_x, item.x); max_x = std::max(max_x, item.x);
            min_y = std::min(min_y, item.y); max_y = std::max(max_y, item.y);
        }
        const bool axis_x = double(max_x) - min_x >= double(max_y) - min_y;
        std::sort(frontier.begin(), frontier.end(), [&](const auto& a, const auto& b) {
            return std::make_tuple(axis_x ? a.x : a.y, axis_x ? a.y : a.x, a.index) <
                   std::make_tuple(axis_x ? b.x : b.y, axis_x ? b.y : b.x, b.index);
        });
        std::vector<Item> next, group;
        for(const auto& item : frontier) {
            group.push_back(item);
            if(!fits(group, center(group))) {
                group.pop_back();
                if(group.empty()) return {};
                next.push_back(append(group, center(group)));
                group = {item};
                if(!fits(group, center(group))) return {};
            }
        }
        if(!group.empty()) next.push_back(append(group, center(group)));
        // First-level singleton groups may replace heavy sinks by lighter
        // buffer inputs. On subsequent levels no reduction means infeasible.
        if(!frontier.front().leaf && next.size() >= frontier.size()) return {};
        frontier = std::move(next);
    }
    return {};
}

struct PathTreeTiming {
    std::vector<float> delay, slew;
    double max_slew = 0, max_cap = 0;
};
PathTreeTiming predictPathTree(detail::Repair& backend, const NetState& state,
    const detail::Buffer& buffer, const PathTree& tree, float driver_slew,
    double upstream_delay, double resistance_per_length) {
    PathTreeTiming result;
    result.delay.resize(state.loads.size()); result.slew.resize(state.loads.size());
    std::vector<double> arrivals(tree.nodes.size()), slews(tree.nodes.size());
    arrivals.back() = upstream_delay;
    // 10–90% transition of a first-order RC: ln(9) * Elmore.
    const auto wire_slew = [](double slew, double elmore) {
        return std::hypot(slew, std::log(9.0) * elmore);
    };
    slews.back() = wire_slew(driver_slew, upstream_delay);
    for(size_t i = tree.nodes.size(); i-- > 0;) {
        const auto& node = tree.nodes[i];
        const auto [delay, slew] = backend.predictedTiming(buffer, node.cap, slews[i]);
        result.max_slew = std::max({result.max_slew, slews[i], double(slew)});
        result.max_cap = std::max(result.max_cap, double(node.cap));
        const double arrival = arrivals[i] + delay;
        const auto wire = [&](int x, int y) {
            return resistance_per_length * node.cap *
                (std::abs(double(node.location.x) - x) + std::abs(double(node.location.y) - y));
        };
        for(auto child : node.children) {
            const auto p = tree.nodes[child].location;
            const double elmore = wire(p.x, p.y);
            arrivals[child] = arrival + elmore;
            slews[child] = wire_slew(slew, elmore);
        }
        for(auto load : node.loads) {
            const double elmore = wire(state.loads[load].x, state.loads[load].y);
            result.delay[load] = arrival + elmore;
            result.slew[load] = wire_slew(slew, elmore);
            result.max_slew = std::max(result.max_slew, double(result.slew[load]));
        }
    }
    return result;
}

// Recovery constraints change with the asynchronous input transition. This
// ranking estimate uses the real Liberty check arcs and fixed clock slew;
// actual polarity/path competition is still resolved by post-ECO STA.
float recoveryConstraintGain(sta::dbSta* sta, odb::dbObject* term, float new_slew) {
    if(term->getObjectType() != odb::dbITermObj) return 0;
    auto* input = static_cast<odb::dbITerm*>(term);
    auto* network = sta->getDbNetwork();
    auto* pin = network->dbToSta(input);
    auto* port = network->libertyPort(pin);
    if(!port) return 0;
    auto* vertex = sta->graph()->pinLoadVertex(pin);
    if(!vertex) return 0;
    const float old_slew = float(sta->slew(vertex, sta::RiseFallBoth::riseFall(),
                                         sta->scenes(), sta::MinMax::max()));
    const auto* pvt = sta->cmdMode()->sdc()->operatingConditions(sta::MinMax::max());
    float old_check = -detail::kInf, new_check = -detail::kInf;
    for(auto* set : port->libertyCell()->timingArcSets()) {
        if(set->to() != port || set->role() != sta::TimingRole::recovery() || !set->from()) continue;
        auto* clock = input->getInst()->findITerm(set->from()->name().c_str());
        if(!clock) continue;
        auto* clock_vertex = sta->graph()->pinLoadVertex(network->dbToSta(clock));
        if(!clock_vertex) continue;
        const float clock_slew = float(sta->slew(clock_vertex, sta::RiseFallBoth::riseFall(),
                                               sta->scenes(), sta::MinMax::max()));
        for(auto* arc : set->arcs()) {
            auto* model = arc->checkModel(sta->cmdScene(), sta::MinMax::max());
            if(!model) continue;
            old_check = std::max(old_check, float(model->checkDelay(pvt, clock_slew, old_slew,
                0, sta::MinMax::max(), sta->pocvMode())));
            new_check = std::max(new_check, float(model->checkDelay(pvt, clock_slew, new_slew,
                0, sta::MinMax::max(), sta->pocvMode())));
        }
    }
    return std::isfinite(old_check) && std::isfinite(new_check) ? old_check - new_check : 0;
}

// Runs entirely inside the caller's ECO, before placement/extraction/timing.
// Only complete a tree when the original single-buffer proposal increases
// the violation count. Existing violations may remain, as in STA acceptance.
// Every added level strictly reduces root fanout; no arbitrary depth budget.
bool completeFanoutTrees(
    sta::dbSta* sta, const std::vector< BufferInsertionResult >& roots,
    const std::vector< odb::dbITerm* >& original_drivers,
    unsigned baseline_violations, unsigned buffer_allowance,
    double area_allowance, const PathRepairOptions& options,
    unsigned& buffer_count, double initial_added_area) {
    auto* network = sta->getDbNetwork();
    sta->checkFanoutPreamble();
    const auto violation_count = [&]() {
        return sta->fanoutViolationCount(sta::MinMax::max(), sta->modes());
    };
    if(violation_count() <= baseline_violations)
        return true;
    const auto report = [&](odb::dbITerm* driver, const char* phase) {
        if(options.path_fanout_observer)
            for(const auto& check : fanoutChecks(sta, driver, phase))
                if(check.slack < 0)
                    options.path_fanout_observer(check);
    };
    for(auto* driver : original_drivers)
        report(driver, "candidate_driver");
    for(const auto& root : roots)
        report(root.output_pin, "candidate_buffer");
    double added_area = initial_added_area;
    for(const auto& root : roots) {
        auto* master = root.buffer->getMaster();
        for(;;) {
            float limit = std::numeric_limits<float>::infinity();
            float previous_load = 0;
            for(const auto& check : fanoutChecks(sta, root.output_pin, "tree"))
                if(check.slack < 0) {
                    limit = std::min(limit, check.limit);
                    previous_load = std::max(previous_load, check.load);
                }
            if(!std::isfinite(limit))
                break;
            if(!(limit > 0))
                return false;
            struct WeightedLoad {
                odb::dbObject* term;
                float weight;
                int x, y;
            };
            std::vector< WeightedLoad > loads;
            for(auto* term : root.output_net->getITerms()) {
                if(!network->isLoad(network->dbToSta(term)))
                    continue;
                int x, y;
                term->getInst()->getLocation(x, y);
                loads.push_back({term, fanoutWeight(network, term), x, y});
            }
            // Top-level output ports carry zero fanout and can stay at root.
            if(loads.empty())
                return false;
            int min_x = loads.front().x, max_x = min_x;
            int min_y = loads.front().y, max_y = min_y;
            for(const auto& load : loads) {
                min_x = std::min(min_x, load.x); max_x = std::max(max_x, load.x);
                min_y = std::min(min_y, load.y); max_y = std::max(max_y, load.y);
            }
            const bool axis_x = double(max_x) - min_x >= double(max_y) - min_y;
            std::sort(loads.begin(), loads.end(), [&](const auto& a, const auto& b) {
                return std::make_tuple(axis_x ? a.x : a.y, axis_x ? a.y : a.x, a.term->getId()) <
                       std::make_tuple(axis_x ? b.x : b.y, axis_x ? b.y : b.x, b.term->getId());
            });
            std::vector< std::vector< WeightedLoad > > groups(1);
            float sum = 0;
            for(const auto& load : loads) {
                if(!std::isfinite(load.weight) || load.weight < 0 || load.weight > limit)
                    return false;
                if(sum + load.weight > limit && !groups.back().empty()) {
                    groups.emplace_back();
                    sum = 0;
                }
                groups.back().push_back(load);
                sum += load.weight;
            }
            // An additional level must decrease the root's weighted load.
            const float input_weight = fanoutWeight(network, root.input_pin);
            if(!(groups.size() * double(input_weight) < previous_load) ||
               buffer_count + groups.size() > buffer_allowance ||
               added_area + groups.size() * detail::area(master) > area_allowance)
                return false;
            std::vector< odb::dbITerm* > children;
            for(const auto& group : groups) {
                std::vector< odb::dbObject* > terms;
                long long x = 0, y = 0;
                for(const auto& load : group) {
                    terms.push_back(load.term); x += load.x; y += load.y;
                }
                BufferInsertionOptions insertion;
                insertion.location = BufferLocation{int(x / group.size()), int(y / group.size())};
                insertion.buffer_name_prefix = "sizer_path_buf";
                insertion.net_name_prefix = "sizer_path_net";
                const auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
                    root.output_net, terms, master, insertion);
                if(!edit)
                    throw std::runtime_error(edit.message);
                children.push_back(edit.output_pin);
                ++buffer_count;
                added_area += detail::area(master);
            }
            sta->checkFanoutPreamble();
            // STA is authoritative, including exceptions and all active modes.
            for(auto* child : children)
                for(const auto& check : fanoutChecks(sta, child, "tree_buffer"))
                    if(check.slack < 0) {
                        if(options.path_fanout_observer) options.path_fanout_observer(check);
                        return false;
                    }
        }
    }
    return violation_count() <= baseline_violations;
}

std::vector< NetState > analyze(detail::Repair& backend,
                                DesignRepairMetrics& metrics) {
    return backend.analyze(metrics, true);
}

const DesignRepairMetrics& globalGainReference(const PathRepairOptions& options,
                                               const DesignRepairMetrics& initial,
                                               const DesignRepairMetrics& current) {
    return options.gain_scope == "current_global" ? current : initial;
}

std::vector< std::string > rejectionReasons(const DesignRepairMetrics& before,
                                            const DesignRepairMetrics& after,
                                            double area_limit,
                                            const DesignRepairMetrics& initial,
                                            const PathRepairOptions& options,
                                            const PathBufferTrial& trial) {
    const auto& tolerance = options.common.tolerances;
    const float improvement = options.min_improvement;
    const bool local = options.gain_scope == "endpoints";
    const auto& reference = globalGainReference(options, initial, before);
    const double objective_wns_before = local ? trial.objective_wns_before : before.setup_wns;
    const double objective_tns_before = local ? trial.objective_tns_before : before.setup_tns;
    const double objective_wns_after = local ? trial.objective_wns_after : after.setup_wns;
    const double objective_tns_after = local ? trial.objective_tns_after : after.setup_tns;
    const double wns_improvement = std::max(double(improvement),
        std::abs(local ? objective_wns_before : double(reference.setup_wns)) * options.min_relative_improvement);
    const double tns_improvement = std::max(double(improvement),
        std::abs(local ? objective_tns_before : double(reference.setup_tns)) * options.min_relative_improvement);
    const float tns_tolerance =
        detail::tnsTolerance(before.setup_tns, tolerance);
    std::vector< std::string > reasons;
    if(!(after.area <= area_limit))
        reasons.push_back("area");
    if(!(after.setup_wns + tolerance.wns_absolute >= before.setup_wns))
        reasons.push_back("setup_wns");
    if(!(after.setup_tns + tns_tolerance >= before.setup_tns))
        reasons.push_back("setup_tns");
    if(!(after.hold_wns + tolerance.wns_absolute >= before.hold_wns))
        reasons.push_back("hold_wns");
    if(!(after.hold_tns + detail::tnsTolerance(before.hold_tns, tolerance) >=
         before.hold_tns))
        reasons.push_back("hold_tns");
    const double objective_tns_tolerance = local
        ? detail::tnsTolerance(float(objective_tns_before), tolerance) : tns_tolerance;
    if(!(objective_wns_after > objective_wns_before + wns_improvement ||
         objective_tns_after > objective_tns_before + std::max(tns_improvement, objective_tns_tolerance)))
        reasons.push_back("no_setup_gain");
    // Local gains cannot justify a globally neutral/numerically insignificant
    // ECO. Keep full-design monotonic progress in addition to DRV/hold guards.
    if(local && !(after.setup_wns > before.setup_wns + improvement ||
                  after.setup_tns > before.setup_tns + std::max(improvement, tns_tolerance)))
        reasons.push_back("no_global_gain");
    if(after.total_slew_violations > before.total_slew_violations)
        reasons.push_back("slew");
    if(after.total_cap_violations > before.total_cap_violations)
        reasons.push_back("cap");
    if(after.total_fanout_violations > before.total_fanout_violations)
        reasons.push_back("fanout");
    if(!(after.severity <= before.severity + tolerance.drv_severity))
        reasons.push_back("drv_severity");
    return reasons;
}

CandidateCollection collectSequentialCandidates(
    detail::Repair& backend, sta::dbSta* sta,
    const std::vector< NetState >& states, const PathRepairOptions& options,
    bool recovery = false) {
    CandidateCollection collection;
    auto& result = collection.candidates;
    auto* network = sta->getDbNetwork();
    std::unordered_map< odb::dbNet*, const NetState* > by_net;
    by_net.reserve(states.size());
    for(const auto& state : states)
        by_net.emplace(state.net, &state);

    sta::StringSeq groups;
    const auto ends = sta->findPathEnds(
        nullptr, nullptr, nullptr, false, sta->scenes(), sta::MinMaxAll::max(),
        options.max_paths, options.paths_per_endpoint, true, false,
        -detail::kInf, 0, true, groups, !recovery, false, recovery, false, false,
        false);
    unsigned path_count = 0;
    for(auto* end : ends) {
        if(path_count++ >= options.max_paths)
            break;
        const float slack = float(end->slack(sta));
        if(slack >= 0)
            continue;

        sta::PathExpanded expanded(end->path(), sta);
        std::unordered_set< odb::dbNet* > seen_nets;
        std::vector< PathBufferCandidate > path_candidates;
        for(size_t i = expanded.startIndex(); i + 1 < expanded.size(); ++i) {
            auto* output_object = network->staToDb(expanded.path(i)->pin(sta));
            if(!output_object ||
               output_object->getObjectType() != odb::dbITermObj)
                continue;
            auto* output = static_cast< odb::dbITerm* >(output_object);
            if(output->getIoType() != odb::dbIoType::OUTPUT ||
               !output->getNet())
                continue;
            auto* net = output->getNet();
            if(net->isSpecial() || net->isDoNotTouch() ||
               sta->isClock(network->dbToSta(net), sta->cmdMode()) ||
               !seen_nets.insert(net).second)
                continue;

            auto* load = network->staToDb(expanded.path(i + 1)->pin(sta));
            if(!load || (load->getObjectType() != odb::dbITermObj &&
                         load->getObjectType() != odb::dbBTermObj))
                continue;
            const auto load_net =
                load->getObjectType() == odb::dbITermObj
                    ? static_cast< odb::dbITerm* >(load)->getNet()
                    : static_cast< odb::dbBTerm* >(load)->getNet();
            if(load_net != net)
                continue;
            const bool is_load =
                load->getObjectType() == odb::dbITermObj
                    ? (static_cast< odb::dbITerm* >(load)->getIoType() ==
                           odb::dbIoType::INPUT ||
                       static_cast< odb::dbITerm* >(load)->getIoType() ==
                           odb::dbIoType::INOUT)
                    : (static_cast< odb::dbBTerm* >(load)->getIoType() ==
                           odb::dbIoType::OUTPUT ||
                       static_cast< odb::dbBTerm* >(load)->getIoType() ==
                           odb::dbIoType::INOUT);
            if(!is_load)
                continue;

            const auto state_it = by_net.find(net);
            if(state_it == by_net.end() || state_it->second->driver == nullptr)
                continue;
            const auto& state = *state_it->second;
            std::vector< odb::dbObject* > selected_loads;
            float selected_cap = 0;
            for(const auto& state_load : state.loads) {
                if(state_load.term != load) {
                    selected_loads.push_back(state_load.term);
                    selected_cap += state_load.capacitance;
                }
            }
            // A net without side loads may still benefit from a buffer on a
            // wire-dominated path.  Otherwise use split-load semantics.
            if(selected_loads.empty()) {
                for(const auto& state_load : state.loads) {
                    if(state_load.term == load) {
                        selected_loads.push_back(state_load.term);
                        selected_cap += state_load.capacitance;
                        break;
                    }
                }
            }
            if(selected_loads.empty() || state.loads.empty())
                continue;
            float pin_cap = 0;
            for(const auto& state_load : state.loads)
                pin_cap += state_load.capacitance;
            const float wire_cap = std::max(0.0f, state.capacitance - pin_cap) /
                                   float(state.loads.size());
            const auto* buffer = backend.select(
                selected_cap + wire_cap, state.slew_limit, state.input_slew);
            if(buffer == nullptr)
                continue;

            const auto original = backend.driverModel(state);
            if(!original.cell)
                continue;
            const float before_delay = backend.predictedDelay(
                original, state.capacitance, state.input_slew);
            const float reduced_cap =
                std::max(0.0f, state.capacitance - selected_cap - wire_cap) +
                buffer->input->capacitance();
            const float after_delay =
                backend.predictedDelay(original, reduced_cap, state.input_slew);
            const float predicted_gain =
                std::isfinite(before_delay) && std::isfinite(after_delay)
                    ? std::max(0.0f, before_delay - after_delay)
                    : 0.0f;

            long long x = 0, y = 0;
            for(auto* selected : selected_loads) {
                if(selected->getObjectType() == odb::dbITermObj) {
                    int sx = 0, sy = 0;
                    static_cast< odb::dbITerm* >(selected)
                        ->getInst()
                        ->getLocation(sx, sy);
                    x += sx;
                    y += sy;
                }
                else {
                    const auto box =
                        static_cast< odb::dbBTerm* >(selected)->getBBox();
                    x += box.xCenter();
                    y += box.yCenter();
                }
            }
            const BufferLocation location{int(x / selected_loads.size()),
                                          int(y / selected_loads.size())};
            std::sort(selected_loads.begin(), selected_loads.end(),
                      [](const auto* lhs, const auto* rhs) {
                          return std::less< const void* >{}(lhs, rhs);
                      });
            path_candidates.push_back({net, std::move(selected_loads),
                                       buffer->master, location, slack,
                                       predicted_gain, i});
            auto& candidate = path_candidates.back();
            candidate.recovery = recovery;
            const auto* endpoint = end->path()->pin(sta);
            const float pin_slack = std::min(0.0f, float(sta->slack(endpoint,
                sta::RiseFallBoth::riseFall(), sta->scenes(), sta::MinMax::max())));
            candidate.endpoints.emplace_back(endpoint, pin_slack);
            candidate.objective_wns = candidate.objective_tns = pin_slack;
        }
        std::sort(
            path_candidates.begin(), path_candidates.end(),
            [](const PathBufferCandidate& a, const PathBufferCandidate& b) {
                if(a.slack != b.slack)
                    return a.slack < b.slack;
                if(a.path_index != b.path_index)
                    return a.path_index > b.path_index;
                if(a.predicted_gain != b.predicted_gain)
                    return a.predicted_gain > b.predicted_gain;
                return a.net->getName() < b.net->getName();
            });
        if(path_candidates.size() > options.max_candidates)
            path_candidates.resize(options.max_candidates);
        result.insert(result.end(), path_candidates.begin(),
                      path_candidates.end());
    }
    std::sort(result.begin(), result.end(),
              [](const PathBufferCandidate& a, const PathBufferCandidate& b) {
                  if(a.slack != b.slack)
                      return a.slack < b.slack;
                  if(a.path_index != b.path_index)
                      return a.path_index > b.path_index;
                  if(a.predicted_gain != b.predicted_gain)
                      return a.predicted_gain > b.predicted_gain;
                  return a.net->getName() < b.net->getName();
              });
    std::unordered_set< CandidateKey, CandidateKeyHash > unique;
    unique.reserve(result.size());
    std::vector< PathBufferCandidate > deduplicated;
    deduplicated.reserve(result.size());
    for(auto& candidate : result) {
        CandidateKey key{candidate.net, candidate.loads};
        if(unique.insert(std::move(key)).second)
            deduplicated.push_back(std::move(candidate));
        else
            ++collection.duplicate_candidates;
    }
    result = std::move(deduplicated);
    return collection;
}
// The model is a ranking heuristic. Real placement, RC and global STA remain
// the acceptance authority; a negative estimate does not prune an ECO.
struct PathUse {
    size_t endpoint;
    odb::dbObject* load;
    float slack;
    size_t path = 0;
    bool recovery = false;
};

std::vector< odb::dbInst* > downstreamCone(odb::dbNet* net,
                                           sta::dbNetwork* network) {
    std::vector< odb::dbNet* > queue{net};
    std::unordered_set< odb::dbNet* > visited;
    std::unordered_set< odb::dbInst* > instances;
    for(auto* pin : net->getITerms())
        if(pin->getIoType() == odb::dbIoType::OUTPUT)
            instances.insert(pin->getInst());
    while(!queue.empty()) {
        auto* current = queue.back();
        queue.pop_back();
        if(!visited.insert(current).second)
            continue;
        for(auto* pin : current->getITerms()) {
            if(pin->getIoType() != odb::dbIoType::INPUT)
                continue;
            auto* inst = pin->getInst();
            instances.insert(inst);
            auto* cell = network->libertyCell(inst);
            if(!cell || cell->hasSequentials() || cell->isClockGate())
                continue;
            for(auto* output : inst->getITerms())
                if(output->getIoType() == odb::dbIoType::OUTPUT &&
                   output->getNet() && !output->getNet()->isSpecial())
                    queue.push_back(output->getNet());
        }
    }
    return {instances.begin(), instances.end()};
}

bool gainOrder(const PathBufferCandidate& a, const PathBufferCandidate& b) {
    if(a.predicted_wns_gain != b.predicted_wns_gain)
        return a.predicted_wns_gain > b.predicted_wns_gain;
    if(a.predicted_tns_gain != b.predicted_tns_gain)
        return a.predicted_tns_gain > b.predicted_tns_gain;
    // The RC/slew proxy may be optimistic for long, weakly driven trees.
    // Preserve path-margin information before using that proxy to break a
    // global-objective tie; neither estimate is an electrical proof.
    if(a.quality.path_gain != b.quality.path_gain)
        return a.quality.path_gain > b.quality.path_gain;
    if(a.quality.electrical_excess != b.quality.electrical_excess)
        return a.quality.electrical_excess < b.quality.electrical_excess;
    const double area_a = detail::area(a.master) * (a.tree ? a.tree->nodes.size() : 1);
    const double area_b = detail::area(b.master) * (b.tree ? b.tree->nodes.size() : 1);
    if(area_a != area_b) return area_a < area_b;
    return std::make_tuple(a.net->getName(), a.master->getName(), a.location.x,
                           a.location.y, a.loads) <
           std::make_tuple(b.net->getName(), b.master->getName(), b.location.x,
                           b.location.y, b.loads);
}

struct PathFeedback {
    struct Model {
        std::string context;
        path_detail::GainCalibration wns, tns;
    };
    path_detail::FailureHistory failures;
    std::unordered_map<std::string, Model> models;
    path_detail::FamilyCoverage coverage;
    unsigned generation = 0;
    const PathRepairOptions& options;
    PathBufferRepairResult& result;
    std::unordered_map<std::string, path_detail::ExplorationFrontier> frontiers;
    path_detail::ContextCoverage group_coverage;
    // One unselected representative per load set/master, not every position
    // and topology. These are activated only by a measured electrical failure
    // with sufficient actual timing gain, on this committed STA snapshot.
    std::vector<PathBufferCandidate> electrical_reserve;

    double tnsEstimate(const PathBufferCandidate& c) const {
        return options.endpoint_coverage && options.gain_scope != "endpoints"
                   ? c.covered_tns_gain : c.predicted_tns_gain;
    }

    path_detail::ExplorationFrontier::Decision explore(const PathBufferCandidate& c) {
        using Decision = path_detail::ExplorationFrontier::Decision;
        if(!options.adaptive_search || options.batch_size != 1)
            return Decision::Explore;
        const bool local = options.gain_scope == "endpoints";
        const auto& reference = globalGainReference(options, result.before, result.after);
        const double wns_floor = std::max(double(options.min_improvement),
            std::abs(local ? c.objective_wns : double(reference.setup_wns)) * options.min_relative_improvement);
        const double tns_floor = std::max({double(options.min_improvement),
            std::abs(local ? c.objective_tns : double(reference.setup_tns)) * options.min_relative_improvement,
            double(detail::tnsTolerance(local ? c.objective_tns : result.after.setup_tns, options.common.tolerances))});
        return frontiers[c.group_key].decide(
            {c.predicted_wns_gain, c.predicted_tns_gain}, {wns_floor, tns_floor});
    }

    void committed() {
        ++generation;
        // Even a distant commit can change an alternative path or placement.
        // Reopen every frontier rather than treating a structural hint as an
        // exact cross-generation dependency certificate.
        frontiers.clear();
    }

    void rank(PathBufferCandidate& candidate, bool count = false) {
        candidate.family_attempts = options.diversify
            ? (options.group_schedule
                   ? group_coverage.attempts(candidate.load_group_key, *candidate.feedback_context)
                   : coverage.attempts(candidate.group_key, generation)) : 0;
        candidate.ranked_wns_gain = candidate.predicted_wns_gain;
        candidate.ranked_tns_gain = tnsEstimate(candidate);
        if(options.calibrate) {
            const auto found = models.find(candidate.group_key);
            if(found != models.end() && found->second.context == *candidate.feedback_context) {
                candidate.ranked_wns_gain = found->second.wns.predict(candidate.predicted_wns_gain);
                candidate.ranked_tns_gain = found->second.tns.predict(tnsEstimate(candidate));
                if(count) ++result.calibrated_candidates;
            }
        }
        const bool local = options.gain_scope == "endpoints";
        const auto& reference = globalGainReference(options, result.before, result.after);
        const double wns_floor = std::max(double(options.min_improvement),
            std::abs(local ? candidate.objective_wns : double(reference.setup_wns)) * options.min_relative_improvement);
        const double tns_floor = std::max(double(options.min_improvement),
            std::abs(local ? candidate.objective_tns : double(reference.setup_tns)) * options.min_relative_improvement);
        candidate.rank_score = std::max(
            wns_floor > 0 ? candidate.ranked_wns_gain / wns_floor : candidate.ranked_wns_gain,
            tns_floor > 0 ? candidate.ranked_tns_gain / tns_floor : candidate.ranked_tns_gain);
        candidate.deferred_retry = options.failure_cache &&
            failures.status(candidate.feedback_key, *candidate.feedback_context, generation) == path_detail::Retry::Deferred &&
            !failures.opportunityChanged(candidate.feedback_key,
                candidate.predicted_wns_gain, candidate.predicted_tns_gain, wns_floor, tns_floor);
        const bool regression =
            candidate.ranked_wns_gain < -options.common.tolerances.wns_absolute ||
            candidate.ranked_tns_gain < -detail::tnsTolerance(result.after.setup_tns, options.common.tolerances);
        candidate.feedback_priority = (candidate.deferred_retry ? 2 : 0) + unsigned(regression);
    }
    static bool order(const PathBufferCandidate& a, const PathBufferCandidate& b) {
        if(a.feedback_priority != b.feedback_priority) return a.feedback_priority < b.feedback_priority;
        if(a.electrical_retry != b.electrical_retry) return a.electrical_retry;
        if(a.family_attempts != b.family_attempts) return a.family_attempts < b.family_attempts;
        if(a.rank_score != b.rank_score) return a.rank_score > b.rank_score;
        if(a.ranked_wns_gain != b.ranked_wns_gain) return a.ranked_wns_gain > b.ranked_wns_gain;
        if(a.ranked_tns_gain != b.ranked_tns_gain) return a.ranked_tns_gain > b.ranked_tns_gain;
        return gainOrder(a, b);
    }
    void select(std::vector<PathBufferCandidate>& variants) {
        std::unordered_set<std::string> actions;
        const auto end = std::remove_if(variants.begin(), variants.end(), [&](auto& c) {
            if(!actions.insert(c.feedback_key).second) { ++result.duplicate_candidates; return true; }
            if(options.failure_cache) {
                const auto retry = failures.status(c.feedback_key, *c.feedback_context, generation);
                if(retry == path_detail::Retry::Duplicate) { ++result.failure_cache_hits; return true; }
                if(retry == path_detail::Retry::Invalidated) ++result.invalidated_failures;
            }
            rank(c, true);
            if(c.deferred_retry) ++result.deferred_candidates;
            return false;
        });
        variants.erase(end, variants.end());
        std::stable_sort(variants.begin(), variants.end(), order);
        if(options.diversify) {
            std::unordered_map<std::string, unsigned> wave;
            for(auto& c : variants)
                c.variant_rank = wave[options.group_schedule ? c.load_group_key : c.group_key]++;
            std::stable_sort(variants.begin(), variants.end(), [](const auto& a, const auto& b) {
                if(a.feedback_priority != b.feedback_priority) return a.feedback_priority < b.feedback_priority;
                if(a.variant_rank != b.variant_rank) return a.variant_rank < b.variant_rank;
                return order(a, b);
            });
        }
        // Beam selection happens after feedback and family diversification.
        // Retain deferred, previously tested actions as a final validation
        // queue even if the active beam is full. They are never blacklisted.
        unsigned active = 0;
        std::unordered_set<std::string> strengths;
        std::vector<PathBufferCandidate> selected;
        for(auto& c : variants) {
            const auto strength = c.load_group_key + '/' +
                std::to_string(c.master->getLib()->getId()) + '/' + std::to_string(c.master->getId());
            const bool new_strength = strengths.insert(strength).second;
            if(c.deferred_retry || active++ < options.max_candidates)
                selected.push_back(std::move(c));
            else if(new_strength)
                electrical_reserve.push_back(std::move(c));
        }
        variants = std::move(selected);
        std::stable_sort(variants.begin(), variants.end(), order);
    }
    void recoverElectrical(const PathBufferCandidate& failed, const PathBufferTrial& trial,
                           std::vector<PathBufferCandidate>& pending, size_t next) {
        if(trial.rejection_reasons.empty() || !trial.metrics_complete ||
           !path_detail::needsFullVerification(trial.rejection_reasons)) return;
        PathBufferCandidate* best = nullptr;
        size_t reserve_index = electrical_reserve.size();
        const auto consider = [&](PathBufferCandidate& c) {
            if(c.load_group_key != failed.load_group_key || c.deferred_retry ||
               failures.status(c.feedback_key, *c.feedback_context, generation) == path_detail::Retry::Duplicate ||
               !path_detail::betterElectricalAlternative(c.quality, failed.quality)) return false;
            if(!best || path_detail::betterElectricalAlternative(c.quality, best->quality) ||
               (!path_detail::betterElectricalAlternative(best->quality, c.quality) && order(c, *best))) {
                best = &c;
                return true;
            }
            return false;
        };
        for(size_t i = next; i < pending.size(); ++i) consider(pending[i]);
        for(size_t i = 0; i < electrical_reserve.size(); ++i)
            if(consider(electrical_reserve[i])) reserve_index = i;
        if(!best) return;
        best->electrical_retry = true;
        if(reserve_index != electrical_reserve.size()) {
            pending.push_back(std::move(electrical_reserve[reserve_index]));
            electrical_reserve.erase(electrical_reserve.begin() + reserve_index);
        }
    }
    void covered(const PathBufferCandidate& plan) {
        coverage.observe(plan.group_key, generation);
        group_coverage.observe(plan.load_group_key, *plan.feedback_context);
    }
    void observe(const PathBufferCandidate& plan, const PathBufferTrial& trial) {
        covered(plan);
        const bool local = options.gain_scope == "endpoints";
        const double wns = local ? trial.objective_wns_after - trial.objective_wns_before
                                 : double(trial.after.setup_wns) - trial.before.setup_wns;
        const double tns = local ? trial.objective_tns_after - trial.objective_tns_before
                                 : double(trial.after.setup_tns) - trial.before.setup_tns;
        if(options.adaptive_search)
            frontiers[plan.group_key].observe(
                {plan.predicted_wns_gain, plan.predicted_tns_gain}, {wns, tns},
                !trial.rejection_reasons.empty(), trial.frontier_audit);
        if(options.calibrate) {
            auto& model = models[plan.group_key];
            if(model.context != *plan.feedback_context) model = {*plan.feedback_context, {}, {}};
            model.wns.observe(plan.predicted_wns_gain, wns);
            model.tns.observe(tnsEstimate(plan), tns);
            ++result.feedback_updates;
        }
        if(!trial.rejection_reasons.empty() && options.failure_cache)
            failures.record(plan.feedback_key, {*plan.feedback_context, trial.rejection_reasons, generation, wns, tns,
                plan.predicted_wns_gain, plan.predicted_tns_gain});
    }
};

CandidateCollection collectPathGainCandidates(
    detail::Repair& backend, sta::dbSta* sta,
    const std::vector< NetState >& states, const PathRepairOptions& options,
    const DesignRepairMetrics& metrics, PathFeedback* feedback) {
    backend.beginTimingPredictionEpoch();
    struct PredictionEpoch {
        detail::Repair& backend;
        ~PredictionEpoch() { backend.endTimingPredictionEpoch(); }
    } prediction_epoch{backend};
    CandidateCollection collection;
    if(feedback) feedback->electrical_reserve.clear();
    auto* network = sta->getDbNetwork();
    std::unordered_map< odb::dbNet*, std::vector< PathUse > > uses;
    std::unordered_map< odb::dbInst*, const NetState* > driven;
    for(const auto& state : states)
        if(state.driver)
            driven.emplace(state.driver->getInst(), &state);
    std::vector< float > endpoint_slacks;
    std::vector<const sta::Pin*> endpoint_pins;
    std::unordered_map< const sta::Pin*, size_t > endpoints;
    std::vector<path_detail::PathForecast> sampled_paths;
    std::map<size_t, double> actual_endpoint_slacks;
    sta::StringSeq groups;
    // Preserve each sampled path, including competing checks/paths at the
    // same endpoint. Generate physical actions once for their union.
    for(bool recovery : {false, true}) {
    if(recovery && !options.recovery) continue;
    const auto ends = sta->findPathEnds(
        nullptr, nullptr, nullptr, false, sta->scenes(), sta::MinMaxAll::max(),
        options.max_paths, options.paths_per_endpoint, true, false,
        -detail::kInf, 0, true, groups, !recovery, false, recovery, false, false,
        false);
    unsigned count = 0;
    for(auto* end : ends) {
        if(count++ >= options.max_paths)
            break;
        const float slack = float(end->slack(sta));
        if(!std::isfinite(slack) || slack >= 0)
            continue;
        const auto* endpoint = end->path()->pin(sta);
        const auto [it, added] = endpoints.emplace(endpoint, endpoints.size());
        if(added) {
            endpoint_slacks.push_back(slack);
            endpoint_pins.push_back(endpoint);
        }
        else
            endpoint_slacks[it->second] =
                std::min(endpoint_slacks[it->second], slack);
        const auto path_index = sampled_paths.size();
        sampled_paths.push_back({it->second, slack, slack, false});
        sta::PathExpanded path(end->path(), sta);
        for(size_t i = path.startIndex(); i + 1 < path.size(); ++i) {
            auto* output = network->staToDb(path.path(i)->pin(sta));
            auto* load = network->staToDb(path.path(i + 1)->pin(sta));
            if(!output || output->getObjectType() != odb::dbITermObj || !load)
                continue;
            auto* driver = static_cast< odb::dbITerm* >(output);
            if(driver->getIoType() != odb::dbIoType::OUTPUT ||
               !driver->getNet())
                continue;
            auto* load_net = load->getObjectType() == odb::dbITermObj
                                 ? static_cast< odb::dbITerm* >(load)->getNet()
                             : load->getObjectType() == odb::dbBTermObj
                                 ? static_cast< odb::dbBTerm* >(load)->getNet()
                                 : nullptr;
            if(load_net == driver->getNet())
                uses[load_net].push_back({it->second, load, slack, path_index, recovery});
        }
    }
    }
    if(endpoint_slacks.empty())
        return collection;
    // Cell limits are invariant throughout this STA snapshot.
    std::vector<std::tuple<float, float, float>> buffer_limits;
    for(const auto& buffer : backend.bufferChoices())
        buffer_limits.emplace_back(bufferCapLimit(sta, buffer) * (1 - options.common.margin),
                                   bufferFanoutLimit(sta, buffer), bufferSlewLimit(sta, buffer));
    // Slack queries may rebuild OpenSTA path groups. Do not issue them while
    // iterating the PathEnd objects returned by findPathEnds().
    for(size_t i = 0; i < endpoint_pins.size(); ++i)
        actual_endpoint_slacks.emplace(i, std::min(0.0, double(sta->slack(
            endpoint_pins[i], sta::RiseFallBoth::riseFall(), sta->scenes(), sta::MinMax::max()))));
    // Index the CURRENT worst max path of every endpoint once per committed
    // design, retaining only nets already chosen by the critical-path search.
    // This does not expand the candidate-net search or the acceptance scope.
    // Each endpoint contributes at most once per net, including reconvergence.
    // It is a scheduling forecast: competing paths after the ECO still require
    // global STA, and these profiles are never used to skip a trial.
    std::unordered_map<odb::dbNet*,
        std::unordered_map<odb::dbObject*, path_detail::EndpointSlackProfile>> endpoint_profiles;
    if(options.endpoint_coverage && options.gain_scope != "endpoints") {
        sta->findRequireds();
        std::unordered_map<const sta::Pin*, sta::Path*> worst_paths;
        for(auto* vertex : sta->endpoints()) {
            auto* path = sta->vertexWorstSlackPath(vertex, sta::MinMax::max());
            if(!path || !std::isfinite(double(path->slack(sta))) ||
               std::abs(double(path->slack(sta))) >= detail::kInf) continue;
            auto [it, added] = worst_paths.emplace(vertex->pin(), path);
            if(!added && path->slack(sta) < it->second->slack(sta)) it->second = path;
        }
        for(const auto& [endpoint, worst] : worst_paths) {
            sta::PathExpanded path(worst, sta);
            std::unordered_set<odb::dbNet*> seen;
            for(size_t i = path.startIndex(); i + 1 < path.size(); ++i) {
                auto* output = network->staToDb(path.path(i)->pin(sta));
                auto* load = network->staToDb(path.path(i + 1)->pin(sta));
                if(!output || output->getObjectType() != odb::dbITermObj || !load) continue;
                auto* driver = static_cast<odb::dbITerm*>(output);
                auto* net = driver->getNet();
                if(driver->getIoType() != odb::dbIoType::OUTPUT || !uses.count(net)) continue;
                auto* load_net = load->getObjectType() == odb::dbITermObj
                    ? static_cast<odb::dbITerm*>(load)->getNet()
                    : load->getObjectType() == odb::dbBTermObj
                        ? static_cast<odb::dbBTerm*>(load)->getNet() : nullptr;
                if(load_net == net && seen.insert(net).second)
                    endpoint_profiles[net][load].add(double(worst->slack(sta)));
            }
        }
        for(auto& [net, profiles] : endpoint_profiles)
            for(auto& [load, profile] : profiles) profile.finish();
    }
    for(const auto& state : states) {
        const auto use_it = uses.find(state.net);
        if(!state.driver || use_it == uses.end())
            continue;
        const auto& net_uses = use_it->second;
        int dx, dy;
        state.driver->getInst()->getLocation(dx, dy);
        const auto distance = [](int ax, int ay, int bx, int by) {
            return std::abs(double(ax) - bx) + std::abs(double(ay) - by);
        };
        const auto model = backend.driverModel(state);
        const auto [before_delay, before_slew] =
            backend.predictedTiming(model, state.capacitance, state.input_slew);
        if(!std::isfinite(before_delay))
            continue;
        std::unordered_map< odb::dbObject*, size_t > index;
        std::vector< float > slacks, elmore;
        double total_length = 0, pin_cap = 0;
        for(size_t i = 0; i < state.loads.size(); ++i) {
            const auto& load = state.loads[i];
            index[load.term] = i;
            pin_cap += load.capacitance;
            total_length += distance(dx, dy, load.x, load.y);
            auto* pin = network->dbToSta(load.term);
            slacks.push_back(
                float(sta->slack(pin, sta::RiseFallBoth::riseFall(),
                                 sta->scenes(), sta::MinMax::max())));
            float wire_delay = 0;
            for(const auto* rf :
                {sta::RiseFall::rise(), sta::RiseFall::fall()}) {
                float delay = 0;
                bool exists = false;
                sta->findElmore(network->dbToSta(state.driver), pin, rf,
                                sta::MinMax::max(), delay, exists);
                if(exists && std::isfinite(delay))
                    wire_delay = std::max(wire_delay, delay);
            }
            elmore.push_back(wire_delay);
        }
        // Keep sampled path loads critical even if the pin slack is
        // unavailable.
        for(const auto& use : net_uses)
            if(index.count(use.load))
                slacks[index.at(use.load)] =
                    std::min(slacks[index.at(use.load)], use.slack);
        const double wire_cap =
            std::max(0.0, double(state.capacitance) - pin_cap);
        const double cap_per_length =
            total_length > 0 ? wire_cap / total_length : 0;
        std::set< std::vector< size_t > > load_groups;
        const auto add_group = [&](std::vector< size_t > group) {
            if(group.empty())
                return;
            std::sort(group.begin(), group.end());
            if(!load_groups.insert(std::move(group)).second)
                ++collection.duplicate_candidates;
        };
        std::vector< size_t > all, side, critical;
        for(size_t i = 0; i < state.loads.size(); ++i) {
            all.push_back(i);
            (slacks[i] < 0 ? critical : side).push_back(i);
        }
        add_group(all);
        add_group(side);
        add_group(critical);
        for(const auto& use : net_uses) {
            if(!index.count(use.load))
                continue;
            std::vector< size_t > others;
            for(auto i : all)
                if(i != index.at(use.load))
                    others.push_back(i);
            add_group(std::move(others));
            add_group({index.at(use.load)});
        }
        // Spatial splits preserve coherent branches; no tuned distance
        // threshold.
        for(bool axis_x : {true, false}) {
            auto ordered = all;
            std::stable_sort(
                ordered.begin(), ordered.end(), [&](size_t a, size_t b) {
                    return axis_x ? state.loads[a].x < state.loads[b].x
                                  : state.loads[a].y < state.loads[b].y;
                });
            const auto middle = ordered.begin() + ordered.size() / 2;
            add_group({ordered.begin(), middle});
            add_group({middle, ordered.end()});
        }
        struct GateResponse {
            detail::Buffer model{};
            detail::Repair::DelayPrediction delay;
            const NetState* state = nullptr;
            float before = 0;
            std::unordered_map< float, float > delay_cache;
        };
        std::vector< GateResponse > gate_responses(state.loads.size());
        std::vector< double > wire_resistance(state.loads.size());
        for(size_t i = 0; i < state.loads.size(); ++i) {
            const auto& load = state.loads[i];
            const double length = distance(dx, dy, load.x, load.y);
            wire_resistance[i] = length > 0 && state.capacitance > 0
                                     ? elmore[i] / (length * state.capacitance)
                                     : 0;
            if(load.term->getObjectType() != odb::dbITermObj)
                continue;
            auto* sink_inst = static_cast<odb::dbITerm*>(load.term)->getInst();
            auto* sink_cell = network->libertyCell(sink_inst);
            if(sink_cell && sink_cell->hasSequentials()) continue;
            const auto next = driven.find(sink_inst);
            if(next == driven.end())
                continue;
            auto& response = gate_responses[i];
            response.state = next->second;
            response.model = backend.driverModel(*response.state);
            response.delay = backend.prepareDelayPrediction(response.model, response.state->capacitance);
            response.before = response.delay(response.state->input_slew);
        }
        std::unordered_map< float, std::pair< float, float > > driver_cache;
        std::ostringstream boundary;
        if(options.strategy == "feedback") {
            // SOFT transfer between structurally identical families. Global
            // legalization and RC extraction can perturb numerical boundaries
            // after any ECO; treating those as entirely new families repeatedly
            // forgets measured failures. Fresh forecasts include the new RC/
            // slew/placement state and can reopen an improved opportunity.
            // Cross-generation evidence NEVER skips the final STA trial.
            boundary << terminalKey(state.driver) << ':'
                     << state.driver->getInst()->getMaster()->getName();
            std::vector<std::string> topology_loads;
            for(size_t i = 0; i < state.loads.size(); ++i) {
                const auto& load = state.loads[i];
                std::string entry = terminalKey(load.term);
                if(load.term->getObjectType() == odb::dbITermObj)
                    entry += '/' + std::string(static_cast<odb::dbITerm*>(load.term)->getInst()->getMaster()->getName());
                topology_loads.push_back(std::move(entry));
            }
            std::sort(topology_loads.begin(), topology_loads.end());
            for(const auto& entry : topology_loads) boundary << ':' << entry.size() << ':' << entry;
        }
        // This is a dependency hint for deferring retries, not proof that all
        // global timing/placement dependencies are unchanged across commits.
        const auto boundary_key = std::make_shared<const std::string>(boundary.str());
        std::vector< PathBufferCandidate > variants;
        for(const auto& group : load_groups) {
            std::vector< bool > selected(state.loads.size(), false);
            double selected_cap = 0, removed_length = 0, weight = 0;
            long long x = 0, y = 0;
            for(auto i : group) {
                selected[i] = true;
                selected_cap += state.loads[i].capacitance;
                weight += fanoutWeight(network, state.loads[i].term);
                removed_length +=
                    distance(dx, dy, state.loads[i].x, state.loads[i].y);
                x += state.loads[i].x;
                y += state.loads[i].y;
            }
            const int cx = int(x / group.size()), cy = int(y / group.size());
            std::set< std::pair< int, int > > locations{
                {cx, cy},
                {dx, dy},
                {int((static_cast< long long >(dx) + cx) / 2),
                 int((static_cast< long long >(dy) + cy) / 2)}};
            for(const auto& [bx, by] : locations) {
                const double upstream_length = distance(dx, dy, bx, by);
                double downstream_length = 0;
                for(auto i : group)
                    downstream_length +=
                        distance(bx, by, state.loads[i].x, state.loads[i].y);
                const float buffer_cap =
                    selected_cap + cap_per_length * downstream_length;
                size_t buffer_index = 0;
                for(const auto& buffer : backend.bufferChoices()) {
                    const auto [cap_limit, fanout_limit, cell_slew_limit] = buffer_limits[buffer_index++];
                    std::shared_ptr<const PathTree> tree;
                    // Whole-net trees are a separate physical candidate. Other
                    // load groups retain the existing single-buffer search.
                    if(options.tree_candidates && group.size() == state.loads.size() &&
                       (buffer_cap > cap_limit || weight > fanout_limit)) {
                        auto plan = planPathTree(state, buffer, sta, {bx, by},
                                                 cap_per_length, cap_limit, fanout_limit);
                        if(plan.nodes.empty()) continue;
                        tree = std::make_shared<PathTree>(std::move(plan));
                    } else if(buffer_cap > cap_limit) continue;
                    const float new_cap =
                        std::max(0.0, double(state.capacitance) - selected_cap -
                                          removed_length * cap_per_length) +
                        buffer.input->capacitance() +
                        upstream_length * cap_per_length;
                    auto [driver_it, uncached] = driver_cache.emplace(
                        new_cap, std::pair< float, float >{});
                    if(uncached)
                        driver_it->second = backend.predictedTiming(model, new_cap, state.input_slew);
                    const auto [driver_delay, driver_slew] = driver_it->second;
                    const auto [buffer_delay, buffer_slew] =
                        backend.predictedTiming(buffer, tree ? tree->nodes.back().cap : buffer_cap, driver_slew);
                    if(!std::isfinite(driver_delay) ||
                       !std::isfinite(buffer_delay) ||
                       !std::isfinite(buffer_slew) ||
                       !std::isfinite(driver_slew))
                        continue;
                    PathBufferCandidate candidate;
                    candidate.net = state.net;
                    candidate.recovery = std::any_of(net_uses.begin(), net_uses.end(),
                        [](const auto& use) { return use.recovery; });
                    candidate.tree = tree;
                    candidate.master = buffer.master;
                    candidate.location = {bx, by};
                    candidate.slack = metrics.setup_wns;
                    candidate.predicted_gain = before_delay - driver_delay;
                    std::vector< float > gains(state.loads.size());
                    double side_wns_after = 0;
                    PathTreeTiming tree_timing;
                    if(tree) {
                        double r = 0;
                        for(double value : wire_resistance) r += value;
                        r /= std::max(size_t(1), wire_resistance.size());
                        tree_timing = predictPathTree(backend, state, buffer, *tree,
                            driver_slew, r * upstream_length * new_cap, r);
                        if(std::any_of(tree_timing.delay.begin(), tree_timing.delay.end(),
                                [](float v) { return !std::isfinite(v); }) ||
                           std::any_of(tree_timing.slew.begin(), tree_timing.slew.end(),
                                [](float v) { return !std::isfinite(v); })) continue;
                    }
                    for(size_t i = 0; i < state.loads.size(); ++i) {
                        const auto& load = state.loads[i];
                        // Effective branch R is calibrated from extracted
                        // Elmore.
                        const double resistance_per_length = wire_resistance[i];
                        double new_wire_delay =
                            elmore[i] * (state.capacitance > 0
                                             ? new_cap / state.capacitance
                                             : 1);
                        if(selected[i]) {
                            new_wire_delay =
                                resistance_per_length *
                                (upstream_length * new_cap +
                                 distance(bx, by, load.x, load.y) * buffer_cap);
                            candidate.loads.push_back(load.term);
                        }
                        gains[i] = candidate.predicted_gain + elmore[i] -
                            (tree ? tree_timing.delay[i] : new_wire_delay +
                                (selected[i] ? buffer_delay : 0));
                        // Include the first downstream gate's slew response.
                        auto& gate = gate_responses[i];
                        if(gate.state) {
                            const float new_input = std::max(
                                0.0f,
                                gate.state->input_slew +
                                    (tree ? tree_timing.slew[i] : selected[i] ? buffer_slew : driver_slew) -
                                    before_slew);
                            auto [gate_it, uncached_gate] =
                                gate.delay_cache.emplace(new_input, 0);
                            if(uncached_gate)
                                gate_it->second = gate.delay(new_input);
                            if(std::isfinite(gate.before) &&
                               std::isfinite(gate_it->second))
                                gains[i] += gate.before - gate_it->second;
                        }
                        if(std::isfinite(slacks[i]))
                            side_wns_after = std::min(side_wns_after, double(slacks[i]) + gains[i]);
                    }
                    auto forecasts = sampled_paths;
                    std::unordered_set< size_t > affected;
                    for(const auto& use : net_uses) {
                        if(!index.count(use.load))
                            continue;
                        const auto i = index.at(use.load);
                        const float constraint_gain = use.recovery
                            ? recoveryConstraintGain(sta, state.loads[i].term,
                                tree ? tree_timing.slew[i] : selected[i] ? buffer_slew : driver_slew) : 0;
                        const double slack = use.slack + gains[i] + constraint_gain;
                        auto& path = forecasts.at(use.path);
                        path.after = path.affected ? std::min(path.after, slack) : slack;
                        path.affected = true;
                        affected.insert(use.endpoint);
                    }
                    const auto forecast = path_detail::forecastObjective(forecasts,
                        actual_endpoint_slacks, metrics.setup_wns,
                        options.gain_scope == "endpoints");
                    candidate.predicted_wns_gain = forecast.wns_gain;
                    candidate.quality.path_gain = forecast.path_gain;
                    const double slew_limit = std::min(cell_slew_limit, state.slew_limit);
                    const double peak_slew = tree ? tree_timing.max_slew : std::max(driver_slew, buffer_slew);
                    const double peak_cap = tree ? tree_timing.max_cap : buffer_cap;
                    candidate.quality.electrical_excess = std::max({0.0,
                        slew_limit > 0 ? peak_slew / slew_limit - 1 : 0,
                        cap_limit > 0 ? peak_cap / cap_limit - 1 : 0});
                    // A side branch outside the sampled endpoints can still
                    // predict a global regression. Preserve this risk signal
                    // for ordering only; it is not a pruning proof.
                    if(options.gain_scope != "endpoints" && side_wns_after < metrics.setup_wns)
                        candidate.predicted_wns_gain = std::min(double(candidate.predicted_wns_gain),
                                                               side_wns_after - metrics.setup_wns);
                    candidate.predicted_tns_gain = forecast.tns_gain;
                    candidate.covered_tns_gain = forecast.tns_gain;
                    const auto profiles = endpoint_profiles.find(state.net);
                    if(profiles != endpoint_profiles.end()) {
                        candidate.covered_tns_gain = 0;
                        for(size_t i = 0; i < state.loads.size(); ++i) {
                            const auto profile = profiles->second.find(state.loads[i].term);
                            if(profile == profiles->second.end()) continue;
                            const float constraint_gain = candidate.recovery
                                ? recoveryConstraintGain(sta, state.loads[i].term,
                                    tree ? tree_timing.slew[i] : selected[i] ? buffer_slew : driver_slew) : 0;
                            candidate.covered_tns_gain += profile->second.gain(gains[i] + constraint_gain);
                            candidate.covered_endpoints += profile->second.size();
                        }
                    }
                    candidate.affected_endpoints = forecast.affected_endpoints;
                    for(size_t i = 0; i < endpoint_pins.size(); ++i) {
                        if(!affected.count(i)) continue;
                        // This collection uses one committed STA snapshot.
                        // Reuse its endpoint query across all tree variants.
                        const float pin_slack = actual_endpoint_slacks.at(i);
                        candidate.endpoints.emplace_back(endpoint_pins[i], pin_slack);
                        candidate.objective_wns = std::min(candidate.objective_wns, double(pin_slack));
                        candidate.objective_tns += pin_slack;
                    }
                    std::sort(candidate.loads.begin(), candidate.loads.end(),
                              [](auto* a, auto* b) {
                                  return std::less< const void* >{}(a, b);
                              });
                    if(options.strategy == "feedback")
                        candidate.feedback_key = actionKey(candidate);
                    candidate.feedback_context = boundary_key;
                    candidate.group_key = actionKey(candidate, false);
                    // All tree shapes/strengths/positions for one load set
                    // share a scheduling family, while calibration continues
                    // to distinguish the actual tree topology.
                    path_detail::ActionNode load_group;
                    for(auto* load : candidate.loads) load_group.terminals.push_back(terminalKey(load));
                    candidate.load_group_key = path_detail::actionFingerprint(
                        std::to_string(state.net->getId()), {load_group}, false);
                    variants.push_back(std::move(candidate));
                }
            }
        }
        if(feedback) feedback->select(variants);
        else {
            std::sort(variants.begin(), variants.end(), gainOrder);
            if(variants.size() > options.max_candidates)
                variants.resize(options.max_candidates);
        }
        const auto cone = options.batch_size > 1
                              ? downstreamCone(state.net, network)
                              : std::vector< odb::dbInst* >{};
        for(size_t i = 0; i < variants.size(); ++i) {
            variants[i].variant_rank = i;
            variants[i].cone = cone;
            collection.candidates.push_back(std::move(variants[i]));
        }
    }
    // Diversify nets before retrying another cell/location on the same net.
    if(options.strategy == "feedback")
        std::stable_sort(collection.candidates.begin(), collection.candidates.end(), PathFeedback::order);
    else std::sort(collection.candidates.begin(), collection.candidates.end(),
              [](const auto& a, const auto& b) {
                  if(a.variant_rank != b.variant_rank)
                      return a.variant_rank < b.variant_rank;
                  return gainOrder(a, b);
              });
    return collection;
}

CandidateCollection collectCandidates(detail::Repair& backend, sta::dbSta* sta,
                                      const std::vector< NetState >& states,
                                      const PathRepairOptions& options,
                                      const DesignRepairMetrics& metrics, PathFeedback* feedback) {
    const bool gain = options.strategy != "sequential";
    auto result = gain ? collectPathGainCandidates(backend, sta, states, options, metrics, feedback)
                       : collectSequentialCandidates(backend, sta, states, options);
    if(!gain && options.recovery) {
        auto async = collectSequentialCandidates(backend, sta, states, options, true);
        result.duplicate_candidates += async.duplicate_candidates;
        result.candidates.insert(result.candidates.end(),
            std::make_move_iterator(async.candidates.begin()),
            std::make_move_iterator(async.candidates.end()));
        std::stable_sort(result.candidates.begin(), result.candidates.end(),
            [](const auto& a, const auto& b) { return a.slack < b.slack; });
    }
    // Keep the best-ranked objective for an identical physical action. Pins
    // from either query remain valid; PathEnd pointers would not survive it.
    std::unordered_set<std::string> actions;
    auto end = std::remove_if(result.candidates.begin(), result.candidates.end(),
        [&](const auto& c) {
            if(actions.insert(actionKey(c)).second) return false;
            ++result.duplicate_candidates;
            return true;
        });
    result.candidates.erase(end, result.candidates.end());
    if(!gain && options.batch_size > 1)
        for(auto& candidate : result.candidates)
            candidate.cone = downstreamCone(candidate.net, sta->getDbNetwork());
    return result;
}

// Keep all provisional ECOs in one parent journal. Child commits merge into
// it, so final legalization (including moves of existing cells) can be undone
// together with every inserted buffer and reconnected load.
PathBufferRepairResult repairWithFinalPlacement(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function<void()>& refresh, const PathRepairOptions& options) {
    PathBufferRepairResult result;
    if(!options.common.prepare) {
        result.message = "final path placement requires a legalization callback";
        return result;
    }
    detail::Repair backend(block, sta, options.common);
    analyze(backend, result.before);
    result.after = result.before;
    const auto authoritative_refresh = [&]() {
        PhaseTimer timer{result.refresh_ms};
        ++result.refreshes;
        if(options.path_verification_refresh) options.path_verification_refresh();
        else refresh();
    };
    bool parent_active = false, final_active = false;
    const auto restore = [&]() {
        if(final_active) {
            odb::dbDatabase::undoEco(block);
            final_active = false;
        }
        if(parent_active) {
            odb::dbDatabase::undoEco(block);
            parent_active = false;
        }
        result.final_placement.rolled_back = true;
        result.inserted_buffers = result.accepted_trials = 0;
        ++result.rollback_refreshes;
        authoritative_refresh();
        PhaseTimer timer{result.measure_ms};
        analyze(backend, result.after);
        result.converged = result.after.setup_wns >= 0;
    };
    try {
        odb::dbDatabase::beginEco(block);
        odb::dbDatabase::endEco(block);
        parent_active = true;
        auto search_options = options;
        search_options.placement = "trial";
        search_options.common.prepare = {};
        if(options.verification == "final") {
            // Provisional ECOs still undergo global timing and electrical
            // acceptance. The parent journal makes the final full extraction
            // authoritative for the entire phase, including all child commits.
            search_options.verification = "trial";
            search_options.path_verification_refresh = {};
        }
        search_options.path_trial_observer = [&](unsigned index, const PathBufferTrial& trial) {
            if(options.path_trial_observer) {
                auto provisional = trial;
                provisional.placement_provisional = true;
                options.path_trial_observer(index, provisional);
            }
        };
        result = OpenDbPathRepairer::repair(block, sta, refresh, search_options);
        for(auto& trial : result.trials) trial.placement_provisional = true;
        result.final_placement.provisional_buffers = result.inserted_buffers;
        result.final_placement.before = result.after;
        if(!result.ok) {
            restore();
            return result;
        }
        if(!result.inserted_buffers) {
            if(options.verification == "final") {
                authoritative_refresh();
                PhaseTimer timer{result.measure_ms};
                analyze(backend, result.after);
            }
            odb::dbDatabase::commitEco(block);
            parent_active = false;
            return result;
        }
        result.final_placement.attempted = true;
        PhaseTimer finalize_timer{result.final_placement.elapsed_ms};
        odb::dbDatabase::beginEco(block);
        final_active = true;
        {
            PhaseTimer timer{result.prepare_ms};
            options.common.prepare();
        }
        authoritative_refresh();
        {
            PhaseTimer timer{result.measure_ms};
            analyze(backend, result.final_placement.after);
        }
        auto acceptance = options;
        // Final acceptance compares the legal result with the legal phase
        // input, even if trial ranking used sampled endpoint objectives.
        acceptance.gain_scope = "global";
        const double area_limit = options.max_area_growth == -1
            ? std::numeric_limits<double>::infinity()
            : result.before.area * (1 + options.max_area_growth);
        result.final_placement.rejection_reasons = rejectionReasons(
            result.before, result.final_placement.after, area_limit,
            result.before, acceptance, {});
        if(result.final_placement.rejection_reasons.empty()) {
            odb::dbDatabase::commitEco(block);
            final_active = false;
            odb::dbDatabase::commitEco(block);
            parent_active = false;
            result.after = result.final_placement.after;
            result.final_placement.accepted = true;
            result.converged = result.after.setup_wns >= 0;
            result.message += "; final placement and global STA accepted";
        } else {
            restore();
            result.message = "final path placement rejected; restored phase input";
            for(const auto& reason : result.final_placement.rejection_reasons)
                result.message += " " + reason;
        }
    } catch(const std::exception& error) {
        result.ok = false;
        result.message = std::string("final path placement failed: ") + error.what();
        if(parent_active || final_active) {
            try { restore(); }
            catch(const std::exception& failure) {
                result.message += std::string("; restore failed: ") + failure.what();
            }
        }
    }
    return result;
}

}  // namespace

PathBufferRepairResult OpenDbPathRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh, const PathRepairOptions& options) {
    PathBufferRepairResult result;
    try {
        options.validate();
    }
    catch(const std::exception& error) {
        result.message = error.what();
        return result;
    }
    if(!block || !sta || !refresh || sta->getDbNetwork()->block() != block ||
       sta->scenes().size() != 1) {
        result.message = "invalid path repair backend (one STA scene required)";
        return result;
    }
    if(options.verification == "final" &&
       (options.placement != "final" || !options.path_verification_refresh)) {
        result.message = "final path verification requires final placement and a full RC callback";
        return result;
    }
    if(options.placement == "final")
        return repairWithFinalPlacement(block, sta, refresh, options);

    const auto start = std::chrono::steady_clock::now();
    const auto& verify = options.path_verification_refresh
        ? options.path_verification_refresh : options.path_incremental_verification;
    bool needs_refresh = false;
    detail::Repair backend(block, sta, options.common);
    const auto update = [&](bool restoring) {
        PhaseTimer timer{result.refresh_ms};
        ++result.refreshes;
        if(restoring)
            ++result.rollback_refreshes;
        if(restoring && options.path_verification_refresh)
            options.path_verification_refresh();
        else
            refresh();
        needs_refresh = false;
    };
    const auto inspect = [&](DesignRepairMetrics& metrics) {
        PhaseTimer timer{result.measure_ms};
        if(options.incremental_analysis && options.path_analysis_changes &&
           !options.path_verification_refresh && options.path_incremental_verification && !options.common.prepare) {
            auto states = backend.analyzeIncremental(metrics, options.path_analysis_changes());
            const auto& stats = backend.analysisStatistics();
            result.electrical_full_scans = stats.full_scans;
            result.electrical_net_checks = stats.net_checks;
            result.electrical_instance_checks = stats.instance_checks;
            return states;
        }
        ++result.electrical_full_scans;
        return analyze(backend, metrics);
    };
    auto states = inspect(result.before);
    result.after = result.before;
    const double area_limit =
        options.max_area_growth == -1
            ? std::numeric_limits< double >::infinity()
            : result.before.area * (1 + options.max_area_growth);
    const auto trials_available = [&]() {
        return options.max_trials == 0 ||
               result.attempted_trials < options.max_trials;
    };
    PathFeedback feedback{{}, {}, {}, 0, options, result};
    path_detail::DependencyFailures dependency_failures;
    path_detail::EndpointFailures endpoint_failures;
    const auto endpoint_fanin = [&](sta::Vertex* endpoint) {
        path_detail::DependencyFailures::Cone nets;
        std::unordered_set<sta::Vertex*> visited{endpoint};
        std::vector<sta::Vertex*> queue{endpoint};
        auto* graph = sta->graph();
        auto* network = sta->getDbNetwork();
        for(size_t i = 0; i < queue.size(); ++i) {
            if(auto* net = network->net(queue[i]->pin()))
                if(auto* db_net = network->staToDb(net)) nets.insert(db_net->getId());
            // The input-pin slew also depends on its cell's output loads
            // through the delay model, even for side outputs outside fanin.
            auto* object = network->staToDb(queue[i]->pin());
            if(object && object->getObjectType() == odb::dbITermObj) {
                auto* inst = static_cast<odb::dbITerm*>(object)->getInst();
                for(auto* pin : inst->getITerms())
                    if(pin->getNet()) nets.insert(pin->getNet()->getId());
            }
            // Include timing checks: capture-clock/constraint changes must
            // invalidate a slack witness, not only its data arrival cone.
            sta::VertexInEdgeIterator edges(queue[i], graph);
            while(edges.hasNext()) {
                auto* vertex = edges.next()->from(graph);
                if(visited.insert(vertex).second) queue.push_back(vertex);
            }
        }
        auto result = std::make_shared<const path_detail::DependencyFailures::Cone>(std::move(nets));
        return result;
    };

    // No intermediate DP or full RC: only the inserted tree changes physical
    // dependencies. Final placement still receives authoritative global STA.
    const bool reuse_rejections = options.reuse_rejections && options.failure_cache &&
        options.strategy == "feedback" && sta->scenes().size() == 1 &&
        !options.common.prepare && !options.path_verification_refresh && options.path_incremental_verification &&
        options.batch_size == 1;
    std::unordered_map<odb::dbNet*, std::shared_ptr<const path_detail::DependencyFailures::Cone>> dependency_cones;
    const auto dependency_cone = [&](odb::dbNet* net) {
        if(const auto it = dependency_cones.find(net); it != dependency_cones.end()) return it->second;
        path_detail::DependencyFailures::Cone cone{net->getId()};
        for(auto* vertex : repairTimingCone(sta, {net})) {
            auto* sta_net = sta->getDbNetwork()->net(vertex->pin());
            if(sta_net) {
                auto* db_net = sta->getDbNetwork()->staToDb(sta_net);
                if(db_net) cone.insert(db_net->getId());
            }
        }
        auto shared = std::make_shared<const path_detail::DependencyFailures::Cone>(std::move(cone));
        dependency_cones.emplace(net, shared);
        return shared;
    };


    const auto budget_available = [&]() {
        // Reserve a final refresh for a rejected ECO. Never return stale STA.
        if(options.max_refreshes &&
           result.refreshes + (verify ? 3 : 2) >
               options.max_refreshes)
            return false;
        if(options.max_seconds > 0) {
            const double elapsed = std::chrono::duration< double >(
                                       std::chrono::steady_clock::now() - start)
                                       .count();
            if(elapsed >= options.max_seconds)
                return false;
        }
        return true;
    };
    try {
        while(budget_available() &&
              trials_available() &&
              result.after.setup_wns < 0 &&
              (options.max_inserted_buffers == 0 ||
               result.inserted_buffers <
                   options.max_inserted_buffers)) {
            CandidateCollection collection;
            {
                PhaseTimer timer{result.candidate_ms};
                collection = collectCandidates(backend, sta, states, options, result.after,
                    options.strategy == "feedback" ? &feedback : nullptr);
                if(options.strategy == "feedback") {
                    if(options.min_relative_improvement > 0) {
                        const auto& reference = globalGainReference(options, result.before, result.after);
                        const double wns_floor = std::max(double(options.min_improvement),
                            std::abs(double(reference.setup_wns)) * options.min_relative_improvement);
                        const double tns_floor = std::max(double(options.min_improvement),
                            std::abs(double(reference.setup_tns)) * options.min_relative_improvement);
                        const auto end = std::remove_if(collection.candidates.begin(), collection.candidates.end(),
                            [&](const auto& candidate) {
                                const bool local = options.gain_scope == "endpoints";
                                const double candidate_wns_floor = local ? std::max(double(options.min_improvement),
                                    std::abs(candidate.objective_wns) * options.min_relative_improvement) : wns_floor;
                                const double candidate_tns_floor = local ? std::max(double(options.min_improvement),
                                    std::abs(candidate.objective_tns) * options.min_relative_improvement) : tns_floor;
                                // The largest possible improvement is reaching
                                // zero negative slack. Sampled forecasts are
                                // NOT upper bounds and must not prune actions.
                                const double wns_bound = std::abs(local ? candidate.objective_wns : double(result.after.setup_wns));
                                const double tns_bound = std::abs(local ? candidate.objective_tns : double(result.after.setup_tns));
                                return !(wns_bound > candidate_wns_floor ||
                                         tns_bound > candidate_tns_floor);
                            });
                        result.screened_candidates += std::distance(end, collection.candidates.end());
                        collection.candidates.erase(end, collection.candidates.end());
                    }
                }
            }
            result.candidate_count += collection.candidates.size();
            result.duplicate_candidates += collection.duplicate_candidates;
            auto plans = std::move(collection.candidates);
            if(plans.empty())
                break;
            bool accepted = false;
            // Fanout feasibility is invariant to buffer position while this
            // committed topology and the constraints stay unchanged.
            std::unordered_set<std::string> infeasible_topologies;
            const auto topology_key = [&](const PathBufferCandidate& plan) {
                auto copy = plan;
                copy.location = {0, 0};
                return actionKey(copy);
            };
            size_t cursor = 0;
            size_t width =
                std::min(size_t(options.batch_size), plans.size());
            bool single_candidate_mode = false;
            std::vector<PathBufferCandidate> suspended;
            const auto reopen_suspended = [&]() {
                const auto end = std::remove_if(suspended.begin(), suspended.end(), [&](auto& plan) {
                    if(feedback.explore(plan) == path_detail::ExplorationFrontier::Decision::Suspend)
                        return false;
                    plans.push_back(std::move(plan));
                    ++result.frontier_reopened;
                    return true;
                });
                suspended.erase(end, suspended.end());
            };
            const auto rerank_remaining = [&]() {
                if(options.strategy != "feedback" || cursor >= plans.size()) return;
                for(size_t i = cursor; i < plans.size(); ++i) feedback.rank(plans[i]);
                std::stable_sort(plans.begin() + cursor, plans.end(), PathFeedback::order);
            };
            const auto backoff = [&]() {
                if(width > 1) {
                    // A failed aggregate does not imply that each member is
                    // bad. Probe the members individually, but do not spend
                    // additional trials on 8->4->2 combinations that cannot
                    // be distinguished without another STA run.
                    width = 1;
                    single_candidate_mode = true;
                }
                else {
                    ++cursor;
                    if(cursor < plans.size())
                        width =
                            single_candidate_mode
                                ? 1
                                : std::min(
                                      size_t(options.batch_size),
                                      plans.size() - cursor);
                }
            };
            while(cursor < plans.size() && budget_available() &&
                  trials_available()) {
                bool frontier_audit = false;
                if(options.strategy == "feedback" && width == 1) {
                    const auto decision = feedback.explore(plans[cursor]);
                    if(decision == path_detail::ExplorationFrontier::Decision::Suspend) {
                        ++result.frontier_suspended;
                        suspended.push_back(std::move(plans[cursor]));
                        backoff();
                        continue;
                    }
                    frontier_audit = decision == path_detail::ExplorationFrontier::Decision::Audit;
                }
                std::vector< size_t > selected;
                std::unordered_set< odb::dbNet* > touched_nets;
                std::unordered_set< odb::dbInst* > touched_cone;
                double planned_area = 0;
                unsigned planned_buffers = 0;
                for(size_t i = cursor;
                    i < std::min(cursor + width, plans.size()); ++i) {
                    const auto& plan = plans[i];
                    const unsigned count = plan.tree ? plan.tree->nodes.size() : 1;
                    if(options.fanout_repair &&
                       infeasible_topologies.count(topology_key(plan))) {
                        ++result.fanout_precheck_rejections;
                        continue;
                    }
                    const bool conflict = std::any_of(
                        plan.cone.begin(), plan.cone.end(),
                        [&](auto* inst) { return touched_cone.count(inst); });
                    if(conflict || touched_nets.count(plan.net) ||
                       result.after.area + planned_area +
                               count * detail::area(plan.master) >
                           area_limit ||
                       (options.max_inserted_buffers != 0 &&
                        result.inserted_buffers + planned_buffers + count >
                            options.max_inserted_buffers))
                        continue;
                    selected.push_back(i);
                    touched_nets.insert(plan.net);
                    touched_cone.insert(plan.cone.begin(), plan.cone.end());
                    planned_area += count * detail::area(plan.master);
                    planned_buffers += count;
                }
                if(selected.empty()) {
                    cursor += width;
                    if(cursor < plans.size())
                        width = std::min(size_t(options.batch_size),
                                         plans.size() - cursor);
                    continue;
                }
                if(reuse_rejections && selected.size() == 1 &&
                   (dependency_failures.rejects(plans[selected.front()].feedback_key,
                       result.after.setup_tns, detail::tnsTolerance(result.after.setup_tns, options.common.tolerances)) ||
                    endpoint_failures.rejects(plans[selected.front()].feedback_key,
                       result.after.setup_wns, options.common.tolerances.wns_absolute))) {
                    ++result.failure_cache_hits;
                    ++result.reused_rejections;
                    ++result.consecutive_rejections;
                    // A dependency-certified rejection completes this family's
                    // turn just like a measured rejection. Advance coverage,
                    // but do not train calibration/frontiers using an old
                    // global delta: other commits may have changed that delta.
                    feedback.covered(plans[selected.front()]);
                    if(options.max_consecutive_rejections &&
                       result.consecutive_rejections >= options.max_consecutive_rejections) {
                        result.stopped_on_rejections = true;
                        break;
                    }
                    backoff();
                    rerank_remaining();
                    continue;
                }
                std::shared_ptr<const path_detail::DependencyFailures::Cone> dependencies;
                if(reuse_rejections && selected.size() == 1)
                    dependencies = dependency_cone(plans[selected.front()].net);
                PathBufferTrial trial;
                trial.frontier_audit = frontier_audit;
                trial.before = result.after;
                bool repeated = false, deferred = false;
                for(const auto index : selected) {
                    const auto& plan = plans[index];
                    repeated |= feedback.failures.find(plan.feedback_key) != nullptr;
                    deferred |= plan.deferred_retry;
                    trial.candidates.push_back(
                        {plan.net->getName(), plan.master->getName(),
                         unsigned(plan.loads.size()), plan.predicted_gain,
                         plan.predicted_wns_gain, plan.predicted_tns_gain,
                         plan.affected_endpoints, plan.location.x,
                         plan.location.y, plan.recovery,
                         unsigned(plan.tree ? plan.tree->nodes.size() : 1),
                         actionKey(plan),
                         options.strategy == "feedback" ? plan.ranked_wns_gain : plan.predicted_wns_gain,
                         options.strategy == "feedback" ? plan.ranked_tns_gain : plan.predicted_tns_gain,
                         plan.deferred_retry, plan.covered_tns_gain, plan.covered_endpoints,
                         plan.family_attempts, feedback.generation, plan.load_group_key,
                         plan.quality.path_gain, plan.quality.electrical_excess, plan.electrical_retry});
                }
                std::unordered_map<const sta::Pin*, float> objective_pins;
                for(const auto index : selected)
                    for(const auto& [pin, slack] : plans[index].endpoints)
                        objective_pins.emplace(pin, slack);
                trial.objective_endpoints = objective_pins.size();
                for(const auto& [pin, slack] : objective_pins) {
                    trial.objective_wns_before = std::min(trial.objective_wns_before, double(slack));
                    trial.objective_tns_before += slack;
                }
                const double prepare_start = result.prepare_ms;
                const double refresh_start = result.refresh_ms;
                const double measure_start = result.measure_ms;
                const auto record_trial = [&]() {
                    trial.prepare_ms = result.prepare_ms - prepare_start;
                    trial.refresh_ms = result.refresh_ms - refresh_start;
                    trial.measure_ms = result.measure_ms - measure_start;
                    result.trials.push_back(std::move(trial));
                    if(options.path_trial_observer)
                        options.path_trial_observer(result.attempted_trials, result.trials.back());
                };
                std::vector< odb::dbITerm* > original_drivers;
                for(const auto index : selected)
                    for(auto* term : plans[index].net->getITerms())
                        if(term->getIoType() == odb::dbIoType::OUTPUT)
                            original_drivers.push_back(term);
                odb::dbDatabase::beginEco(block);
                bool active = true;
                try {
                    std::vector< BufferInsertionResult > roots;
                    for(const auto index : selected) {
                        const auto& plan = plans[index];
                        BufferInsertionOptions insertion;
                        insertion.location = plan.location;
                        insertion.buffer_name_prefix = "sizer_path_buf";
                        insertion.net_name_prefix = "sizer_path_net";
                        if(plan.tree) {
                            std::vector<BufferInsertionResult> edits;
                            for(const auto& node : plan.tree->nodes) {
                                std::vector<odb::dbObject*> terms;
                                for(auto i : node.loads) terms.push_back(plan.tree->terminals[i]);
                                for(auto i : node.children) terms.push_back(edits[i].input_pin);
                                insertion.location = node.location;
                                auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
                                    plan.net, terms, plan.master, insertion);
                                if(!edit) throw std::runtime_error(edit.message);
                                edits.push_back(edit);
                            }
                            roots.push_back(edits.back());
                            trial.buffers += edits.size();
                        } else {
                            const auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
                                plan.net, plan.loads, plan.master, insertion);
                            if(!edit) throw std::runtime_error(edit.message);
                            roots.push_back(edit);
                            ++trial.buffers;
                        }
                    }
                    if(options.fanout_repair) {
                        PhaseTimer timer{result.candidate_ms};
                        const auto allowance = detail::remainingBufferBudget(
                            options.max_inserted_buffers, result.inserted_buffers);
                        if(!completeFanoutTrees(sta, roots, original_drivers,
                                result.after.total_fanout_violations, allowance,
                                area_limit - result.after.area, options, trial.buffers, planned_area)) {
                            odb::dbDatabase::undoEco(block);
                            active = false;
                            needs_refresh = true;
                            ++result.fanout_precheck_rejections;
                            if(selected.size() == 1)
                                infeasible_topologies.insert(topology_key(plans[selected.front()]));
                            backoff();
                            continue;
                        }
                    }
                    if(trial.buffers > roots.size())
                        ++result.fanout_tree_trials;
                    ++result.attempted_trials;
                    result.frontier_audits += frontier_audit;
                    result.repeated_trials += repeated;
                    result.deferred_trials += deferred;
                    if(options.common.prepare) {
                        PhaseTimer timer{result.prepare_ms};
                        options.common.prepare();
                    }
                    update(false);
                    DesignRepairMetrics after;
                    std::vector<NetState> next_states;
                    const auto objective = [&]() {
                        trial.objective_wns_after = trial.objective_tns_after = 0;
                        for(const auto& [pin, old_slack] : objective_pins) {
                            const double slack = std::min(0.0, double(sta->slack(pin,
                                sta::RiseFallBoth::riseFall(), sta->scenes(), sta::MinMax::max())));
                            trial.objective_wns_after = std::min(trial.objective_wns_after, slack);
                            trial.objective_tns_after += slack;
                        }
                    };
                    if(options.timing_precheck) {
                        PhaseTimer timer{result.measure_ms};
                        sta->findRequireds();
                        // Placeholder electrical fields are used only by the
                        // timing rejection predicate and never exported as
                        // measured values or used to accept an ECO.
                        after = result.after;
                        after.setup_wns = std::min(0.0f, float(sta->worstSlack(sta::MinMax::max())));
                        after.setup_tns = float(sta->totalNegativeSlack(sta::MinMax::max()));
                        after.hold_wns = std::min(0.0f, float(sta->worstSlack(sta::MinMax::min())));
                        after.hold_tns = float(sta->totalNegativeSlack(sta::MinMax::min()));
                        objective();
                        trial.rejection_reasons = rejectionReasons(
                            result.after, after, area_limit, result.before, options, trial);
                    }
                    // Survivors are refreshed and inspected below (full RC or
                    // the accumulated timing cone). Do not scan electrical
                    // states twice before the same acceptance decision.
                    const bool verify_next = options.timing_precheck &&
                        verify && trial.rejection_reasons.empty();
                    if(trial.rejection_reasons.empty() && !verify_next) {
                        next_states = inspect(after);
                        objective();
                        trial.rejection_reasons = rejectionReasons(
                            result.after, after, area_limit, result.before, options, trial);
                    } else if(!trial.rejection_reasons.empty()) {
                        trial.metrics_complete = false;
                        ++result.timing_precheck_rejections;
                    } else {
                        trial.metrics_complete = false;
                    }
                    trial.after = after;
                    if(verify &&
                       path_detail::needsFullVerification(trial.rejection_reasons)) {
                        {
                            PhaseTimer timer{result.refresh_ms};
                            ++result.refreshes;
                            verify();
                        }
                        next_states = inspect(after);
                        trial.metrics_complete = true;
                        trial.after = after;
                        objective();
                        trial.rejection_reasons = rejectionReasons(
                            result.after, after, area_limit, result.before, options, trial);
                        trial.full_verified = bool(options.path_verification_refresh);
                    }
                    // A failed batch cannot be attributed to its members.
                    if(options.strategy == "feedback" && selected.size() == 1)
                        feedback.observe(plans[selected.front()], trial);
                    if(trial.rejection_reasons.empty()) {
                        odb::dbDatabase::commitEco(block);
                        active = false;
                        result.accepted_trials += trial.buffers;
                        result.inserted_buffers += trial.buffers;
                        result.consecutive_rejections = 0;
                        result.after = after;
                        states = std::move(next_states);
                        if(reuse_rejections) {
                            dependency_failures.committed(*dependencies);
                            endpoint_failures.committed(plans[selected.front()].net->getId());
                            dependency_cones.clear();
                        }
                        feedback.committed();
                        accepted = true;
                        record_trial();
                        break;
                    }
                    if(reuse_rejections && selected.size() == 1 &&
                       std::find(trial.rejection_reasons.begin(), trial.rejection_reasons.end(), "setup_tns") != trial.rejection_reasons.end())
                        dependency_failures.record(plans[selected.front()].feedback_key,
                            std::move(dependencies), trial.before.setup_tns, trial.after.setup_tns);
                    if(reuse_rejections && selected.size() == 1 &&
                       std::find(trial.rejection_reasons.begin(), trial.rejection_reasons.end(), "setup_wns") != trial.rejection_reasons.end()) {
                        sta::Slack slack; sta::Vertex* endpoint = nullptr;
                        sta->worstSlack(sta::MinMax::max(), slack, endpoint);
                        if(endpoint) endpoint_failures.record(plans[selected.front()].feedback_key,
                            endpoint_fanin(endpoint), float(slack));
                    }
                    odb::dbDatabase::undoEco(block);
                    active = false;
                    ++result.rejected_trials;
                    // Do not stop after a failed aggregate: its members must
                    // remain eligible for individual trials during backoff.
                    if(selected.size() == 1) ++result.consecutive_rejections;
                    needs_refresh = true;
                    if(!options.defer_rollback_refresh) {
                        update(true);
                        states = inspect(result.after);
                    }
                    if(options.strategy == "feedback" && selected.size() == 1)
                        feedback.recoverElectrical(plans[selected.front()], trial, plans, cursor + 1);
                    record_trial();
                    if(options.max_consecutive_rejections &&
                       result.consecutive_rejections >= options.max_consecutive_rejections) {
                        result.stopped_on_rejections = true;
                        break; // The common exit path restores RC/STA first.
                    }
                    backoff();
                    // An underestimated later probe can invalidate the error
                    // envelope. Restore earlier suspended siblings as well,
                    // including when the active queue has just become empty.
                    reopen_suspended();
                    rerank_remaining();
                }
                catch(...) {
                    if(active) {
                        odb::dbDatabase::undoEco(block);
                        needs_refresh = true;
                    }
                    throw;
                }
            }
            if(!accepted)
                break;
        }
    }
    catch(const std::exception& error) {
        result.message = error.what();
        if(needs_refresh) {
            try {
                update(true);
                states = inspect(result.after);
            }
            catch(const std::exception& restore) {
                result.message +=
                    std::string("; rollback refresh failed: ") + restore.what();
            }
        }
        return result;
    }
    if(needs_refresh) {
        try {
            update(true);
            states = inspect(result.after);
        }
        catch(const std::exception& error) {
            result.message = error.what();
            return result;
        }
    }

    result.ok = true;
    result.converged = result.after.setup_wns >= 0;
    result.message =
        result.converged ? "path buffer setup timing met"
        : result.stopped_on_rejections
            ? "path buffer stopped after consecutive rejections; search not exhausted"
        : !budget_available()
            ? "path buffer time/refresh budget exhausted; violations remain"
        : !trials_available()
            ? "path buffer trial budget exhausted; violations remain"
            : result.frontier_suspended
                ? "audited empirical path frontier exhausted; violations remain"
                : "no acceptable path buffer candidate; violations remain";
    return result;
}
}  // namespace ecc
