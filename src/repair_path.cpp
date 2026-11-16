// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_path.h"

#include "buffer_inserter.h"
#include "repair_acceptance.h"
#include "repair_backend.h"

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
    unsigned affected_endpoints = 0;
    unsigned variant_rank = 0;
    std::string feedback_key;
    unsigned feedback_priority = 0;
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

// History changes priority only: a globally coupled STA failure is never
// treated as proof that the candidate is still invalid after another edit.
std::string actionKey(const PathBufferCandidate& candidate) {
    std::ostringstream key;
    key << candidate.net->getId() << ':' << candidate.master->getId()
        << ':' << candidate.master->getLib()->getId() << ':'
        << candidate.location.x << ':' << candidate.location.y;
    for(auto* load : candidate.loads)
        key << ':' << int(load->getObjectType()) << '/' << load->getId();
    if(candidate.tree) key << ":tree";
    return key.str();
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

struct PathTreeTiming { std::vector<float> delay, slew; };
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
        const double arrival = arrivals[i] + backend.predictedDelay(buffer, node.cap, slews[i]);
        const double slew = backend.predictedSlew(buffer, node.cap, slews[i]);
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
    double area_allowance, const SetupRepairOptions& options,
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

std::vector< std::string > rejectionReasons(const DesignRepairMetrics& before,
                                            const DesignRepairMetrics& after,
                                            double area_limit,
                                            const DesignRepairMetrics& initial,
                                            const SetupRepairOptions& options,
                                            const PathBufferTrial& trial) {
    const auto& tolerance = options.common.tolerances;
    const float improvement = options.min_improvement;
    const bool local = options.path_buffer_gain_scope == "endpoints";
    const double objective_wns_before = local ? trial.objective_wns_before : before.setup_wns;
    const double objective_tns_before = local ? trial.objective_tns_before : before.setup_tns;
    const double objective_wns_after = local ? trial.objective_wns_after : after.setup_wns;
    const double objective_tns_after = local ? trial.objective_tns_after : after.setup_tns;
    const double wns_improvement = std::max(double(improvement),
        std::abs(local ? objective_wns_before : double(initial.setup_wns)) * options.path_buffer_min_relative_improvement);
    const double tns_improvement = std::max(double(improvement),
        std::abs(local ? objective_tns_before : double(initial.setup_tns)) * options.path_buffer_min_relative_improvement);
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
    const std::vector< NetState >& states, const SetupRepairOptions& options,
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
        options.path_buffer_max_paths, options.paths_per_endpoint, true, false,
        -detail::kInf, 0, true, groups, !recovery, false, recovery, false, false,
        false);
    unsigned path_count = 0;
    for(auto* end : ends) {
        if(path_count++ >= options.path_buffer_max_paths)
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
        if(path_candidates.size() > options.path_buffer_max_candidates)
            path_candidates.resize(options.path_buffer_max_candidates);
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
    const double area_a = detail::area(a.master) * (a.tree ? a.tree->nodes.size() : 1);
    const double area_b = detail::area(b.master) * (b.tree ? b.tree->nodes.size() : 1);
    if(area_a != area_b) return area_a < area_b;
    return std::make_tuple(a.net->getName(), a.master->getName(), a.location.x,
                           a.location.y, a.loads) <
           std::make_tuple(b.net->getName(), b.master->getName(), b.location.x,
                           b.location.y, b.loads);
}

CandidateCollection collectPathGainCandidates(
    detail::Repair& backend, sta::dbSta* sta,
    const std::vector< NetState >& states, const SetupRepairOptions& options,
    bool recovery = false) {
    CandidateCollection collection;
    auto* network = sta->getDbNetwork();
    std::unordered_map< odb::dbNet*, std::vector< PathUse > > uses;
    std::unordered_map< odb::dbInst*, const NetState* > driven;
    for(const auto& state : states)
        if(state.driver)
            driven.emplace(state.driver->getInst(), &state);
    std::vector< float > endpoint_slacks;
    std::vector<const sta::Pin*> endpoint_pins;
    std::unordered_map< const sta::Pin*, size_t > endpoints;
    sta::StringSeq groups;
    const auto ends = sta->findPathEnds(
        nullptr, nullptr, nullptr, false, sta->scenes(), sta::MinMaxAll::max(),
        options.path_buffer_max_paths, options.paths_per_endpoint, true, false,
        -detail::kInf, 0, true, groups, !recovery, false, recovery, false, false,
        false);
    unsigned count = 0;
    for(auto* end : ends) {
        if(count++ >= options.path_buffer_max_paths)
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
                uses[load_net].push_back({it->second, load, slack});
        }
    }
    if(endpoint_slacks.empty())
        return collection;
    const float baseline_wns =
        *std::min_element(endpoint_slacks.begin(), endpoint_slacks.end());
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
        const float before_delay =
            backend.predictedDelay(model, state.capacitance, state.input_slew);
        if(!std::isfinite(before_delay))
            continue;
        const float before_slew =
            backend.predictedSlew(model, state.capacitance, state.input_slew);
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
            response.before = backend.predictedDelay(
                response.model, response.state->capacitance,
                response.state->input_slew);
        }
        std::unordered_map< float, std::pair< float, float > > driver_cache;
        std::ostringstream boundary;
        if(options.path_buffer_strategy == "feedback") {
            // Retain failures only for the same local electrical/placement
            // state. Global interactions still require fresh STA verification.
            boundary << std::hexfloat << state.capacitance << ':' << state.input_slew
                     << ':' << state.slew << ':' << dx << ':' << dy << ':'
                     << state.driver->getInst()->getMaster()->getName();
            for(size_t i = 0; i < state.loads.size(); ++i) {
                const auto& load = state.loads[i];
                boundary << ':' << int(load.term->getObjectType()) << '/' << load.term->getId()
                         << '/' << load.capacitance << '/' << load.x << '/' << load.y
                         << '/' << slacks[i] << '/' << elmore[i];
            }
        }
        // Hash collisions can only change ordering, never suppress a trial or
        // reuse STA results. Avoid copying a large load boundary per variant.
        const auto boundary_key = std::to_string(std::hash< std::string >{}(boundary.str()));
        std::vector< PathBufferCandidate > variants;
        for(const auto& group : load_groups) {
            std::vector< bool > selected(state.loads.size(), false);
            double selected_cap = 0, removed_length = 0;
            long long x = 0, y = 0;
            for(auto i : group) {
                selected[i] = true;
                selected_cap += state.loads[i].capacitance;
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
                for(const auto& buffer : backend.bufferChoices()) {
                    const float cap_limit = bufferCapLimit(sta, buffer) * (1 - options.common.margin);
                    std::shared_ptr<const PathTree> tree;
                    const float fanout_limit = bufferFanoutLimit(sta, buffer);
                    double weight = 0;
                    for(auto i : group) weight += fanoutWeight(network, state.loads[i].term);
                    // Whole-net trees are a separate physical candidate. Other
                    // load groups retain the existing single-buffer search.
                    if(options.path_buffer_tree_candidates && group.size() == state.loads.size() &&
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
                        driver_it->second = {
                            backend.predictedDelay(model, new_cap,
                                                   state.input_slew),
                            backend.predictedSlew(model, new_cap,
                                                  state.input_slew)};
                    const auto [driver_delay, driver_slew] = driver_it->second;
                    const float buffer_delay =
                        backend.predictedDelay(buffer, tree ? tree->nodes.back().cap : buffer_cap, driver_slew);
                    const float buffer_slew =
                        backend.predictedSlew(buffer, tree ? tree->nodes.back().cap : buffer_cap, driver_slew);
                    if(!std::isfinite(driver_delay) ||
                       !std::isfinite(buffer_delay) ||
                       !std::isfinite(buffer_slew) ||
                       !std::isfinite(driver_slew))
                        continue;
                    PathBufferCandidate candidate;
                    candidate.net = state.net;
                    candidate.recovery = recovery;
                    candidate.tree = tree;
                    candidate.master = buffer.master;
                    candidate.location = {bx, by};
                    candidate.slack = baseline_wns;
                    candidate.predicted_gain = before_delay - driver_delay;
                    std::vector< float > gains(state.loads.size());
                    float predicted_wns = baseline_wns;
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
                                gate_it->second = backend.predictedDelay(
                                    gate.model, gate.state->capacitance,
                                    new_input);
                            if(std::isfinite(gate.before) &&
                               std::isfinite(gate_it->second))
                                gains[i] += gate.before - gate_it->second;
                        }
                        if(std::isfinite(slacks[i]))
                            predicted_wns =
                                std::min(predicted_wns, slacks[i] + gains[i]);
                    }
                    if(recovery) {
                        std::unordered_set<size_t> checked;
                        for(const auto& use : net_uses) {
                            const auto found = index.find(use.load);
                            if(found == index.end() || !checked.insert(found->second).second) continue;
                            const size_t i = found->second;
                            gains[i] += recoveryConstraintGain(sta, state.loads[i].term,
                                tree ? tree_timing.slew[i] : selected[i] ? buffer_slew : driver_slew);
                        }
                    }
                    auto after_slacks = endpoint_slacks;
                    std::unordered_set< size_t > affected;
                    for(const auto& use : net_uses) {
                        if(!index.count(use.load))
                            continue;
                        const float slack =
                            use.slack + gains[index.at(use.load)];
                        if(affected.insert(use.endpoint).second)
                            after_slacks[use.endpoint] = slack;
                        else
                            after_slacks[use.endpoint] =
                                std::min(after_slacks[use.endpoint], slack);
                    }
                    const float sampled_wns = *std::min_element(
                        after_slacks.begin(), after_slacks.end());
                    // Positive improvement comes from sampled endpoints; other
                    // affected loads can veto optimistic WNS predictions.
                    candidate.predicted_wns_gain =
                        predicted_wns < baseline_wns
                            ? predicted_wns - baseline_wns
                            : sampled_wns - baseline_wns;
                    for(size_t i = 0; i < after_slacks.size(); ++i)
                        candidate.predicted_tns_gain +=
                            std::min(0.0f, after_slacks[i]) -
                            endpoint_slacks[i];
                    candidate.affected_endpoints = affected.size();
                    for(size_t i = 0; i < endpoint_pins.size(); ++i) {
                        if(!affected.count(i)) continue;
                        const float pin_slack = std::min(0.0f, float(sta->slack(endpoint_pins[i],
                            sta::RiseFallBoth::riseFall(), sta->scenes(), sta::MinMax::max())));
                        candidate.endpoints.emplace_back(endpoint_pins[i], pin_slack);
                        candidate.objective_wns = std::min(candidate.objective_wns, double(pin_slack));
                        candidate.objective_tns += pin_slack;
                    }
                    std::sort(candidate.loads.begin(), candidate.loads.end(),
                              [](auto* a, auto* b) {
                                  return std::less< const void* >{}(a, b);
                              });
                    if(options.path_buffer_strategy == "feedback")
                        candidate.feedback_key = actionKey(candidate) + ':' + boundary_key;
                    variants.push_back(std::move(candidate));
                }
            }
        }
        std::sort(variants.begin(), variants.end(), gainOrder);
        if(variants.size() > options.path_buffer_max_candidates)
            variants.resize(options.path_buffer_max_candidates);
        const auto cone = options.path_buffer_batch_size > 1
                              ? downstreamCone(state.net, network)
                              : std::vector< odb::dbInst* >{};
        for(size_t i = 0; i < variants.size(); ++i) {
            variants[i].variant_rank = i;
            variants[i].cone = cone;
            collection.candidates.push_back(std::move(variants[i]));
        }
    }
    // Diversify nets before retrying another cell/location on the same net.
    if(options.path_buffer_strategy == "feedback")
        std::sort(collection.candidates.begin(), collection.candidates.end(), gainOrder);
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
                                      const SetupRepairOptions& options) {
    const bool gain = options.path_buffer_strategy != "sequential";
    auto result = gain ? collectPathGainCandidates(backend, sta, states, options)
                       : collectSequentialCandidates(backend, sta, states, options);
    if(options.path_buffer_recovery) {
        auto async = gain ? collectPathGainCandidates(backend, sta, states, options, true)
                          : collectSequentialCandidates(backend, sta, states, options, true);
        result.duplicate_candidates += async.duplicate_candidates;
        result.candidates.insert(result.candidates.end(),
            std::make_move_iterator(async.candidates.begin()),
            std::make_move_iterator(async.candidates.end()));
        if(gain) std::stable_sort(result.candidates.begin(), result.candidates.end(), gainOrder);
        else std::stable_sort(result.candidates.begin(), result.candidates.end(),
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
    if(!gain && options.path_buffer_batch_size > 1)
        for(auto& candidate : result.candidates)
            candidate.cone = downstreamCone(candidate.net, sta->getDbNetwork());
    return result;
}

}  // namespace

PathBufferRepairResult OpenDbPathRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh, const SetupRepairOptions& options) {
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

    const auto start = std::chrono::steady_clock::now();
    bool needs_refresh = false;
    detail::Repair backend(block, sta, options.common);
    const auto update = [&](bool restoring) {
        PhaseTimer timer{result.refresh_ms};
        ++result.refreshes;
        if(restoring)
            ++result.rollback_refreshes;
        refresh();
        needs_refresh = false;
    };
    const auto inspect = [&](DesignRepairMetrics& metrics) {
        PhaseTimer timer{result.measure_ms};
        return analyze(backend, metrics);
    };
    auto states = inspect(result.before);
    result.after = result.before;
    const double area_limit =
        options.path_buffer_max_area_growth == -1
            ? std::numeric_limits< double >::infinity()
            : result.before.area * (1 + options.path_buffer_max_area_growth);
    const auto trials_available = [&]() {
        return options.path_buffer_max_trials == 0 ||
               result.attempted_trials < options.path_buffer_max_trials;
    };
    std::unordered_set< std::string > failed_actions;

    const auto budget_available = [&]() {
        // Reserve a final refresh for a rejected ECO. Never return stale STA.
        if(options.path_buffer_max_refreshes &&
           result.refreshes + 2 > options.path_buffer_max_refreshes)
            return false;
        if(options.path_buffer_max_seconds > 0) {
            const double elapsed = std::chrono::duration< double >(
                                       std::chrono::steady_clock::now() - start)
                                       .count();
            if(elapsed >= options.path_buffer_max_seconds)
                return false;
        }
        return true;
    };
    try {
        while(budget_available() &&
              trials_available() &&
              result.after.setup_wns < 0 &&
              (options.path_buffer_max_inserted_buffers == 0 ||
               result.inserted_buffers <
                   options.path_buffer_max_inserted_buffers)) {
            CandidateCollection collection;
            {
                PhaseTimer timer{result.candidate_ms};
                collection = collectCandidates(backend, sta, states, options);
                if(options.path_buffer_strategy == "feedback") {
                    if(options.path_buffer_min_relative_improvement > 0) {
                        const double wns_floor = std::max(double(options.min_improvement),
                            std::abs(double(result.before.setup_wns)) * options.path_buffer_min_relative_improvement);
                        const double tns_floor = std::max(double(options.min_improvement),
                            std::abs(double(result.before.setup_tns)) * options.path_buffer_min_relative_improvement);
                        const auto end = std::remove_if(collection.candidates.begin(), collection.candidates.end(),
                            [&](const auto& candidate) {
                                const bool local = options.path_buffer_gain_scope == "endpoints";
                                const double candidate_wns_floor = local ? std::max(double(options.min_improvement),
                                    std::abs(candidate.objective_wns) * options.path_buffer_min_relative_improvement) : wns_floor;
                                const double candidate_tns_floor = local ? std::max(double(options.min_improvement),
                                    std::abs(candidate.objective_tns) * options.path_buffer_min_relative_improvement) : tns_floor;
                                return !(candidate.predicted_wns_gain > candidate_wns_floor ||
                                         candidate.predicted_tns_gain > candidate_tns_floor);
                            });
                        result.screened_candidates += std::distance(end, collection.candidates.end());
                        collection.candidates.erase(end, collection.candidates.end());
                    }
                    // Prefer untried promising actions, then failed promising
                    // actions, before speculative predicted regressions. All
                    // tiers remain eligible for a fresh global STA trial.
                    for(auto& candidate : collection.candidates) {
                        const bool promising =
                            candidate.predicted_wns_gain >= -options.common.tolerances.wns_absolute &&
                            candidate.predicted_tns_gain >= -detail::tnsTolerance(result.after.setup_tns, options.common.tolerances) &&
                            (candidate.predicted_wns_gain > options.min_improvement ||
                             candidate.predicted_tns_gain > options.min_improvement);
                        candidate.feedback_priority = (promising ? 0 : 2) +
                            unsigned(failed_actions.count(candidate.feedback_key));
                    }
                    std::stable_sort(collection.candidates.begin(), collection.candidates.end(),
                        [&](const auto& a, const auto& b) {
                            return a.feedback_priority < b.feedback_priority;
                        });
                }
            }
            result.candidate_count += collection.candidates.size();
            result.duplicate_candidates += collection.duplicate_candidates;
            const auto& plans = collection.candidates;
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
                std::min(size_t(options.path_buffer_batch_size), plans.size());
            bool single_candidate_mode = false;
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
                                      size_t(options.path_buffer_batch_size),
                                      plans.size() - cursor);
                }
            };
            while(cursor < plans.size() && budget_available() &&
                  trials_available()) {
                std::vector< size_t > selected;
                std::unordered_set< odb::dbNet* > touched_nets;
                std::unordered_set< odb::dbInst* > touched_cone;
                double planned_area = 0;
                unsigned planned_buffers = 0;
                for(size_t i = cursor;
                    i < std::min(cursor + width, plans.size()); ++i) {
                    const auto& plan = plans[i];
                    const unsigned count = plan.tree ? plan.tree->nodes.size() : 1;
                    if(options.path_buffer_fanout_repair &&
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
                       (options.path_buffer_max_inserted_buffers != 0 &&
                        result.inserted_buffers + planned_buffers + count >
                            options.path_buffer_max_inserted_buffers))
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
                        width = std::min(size_t(options.path_buffer_batch_size),
                                         plans.size() - cursor);
                    continue;
                }
                PathBufferTrial trial;
                trial.before = result.after;
                for(const auto index : selected) {
                    const auto& plan = plans[index];
                    trial.candidates.push_back(
                        {plan.net->getName(), plan.master->getName(),
                         unsigned(plan.loads.size()), plan.predicted_gain,
                         plan.predicted_wns_gain, plan.predicted_tns_gain,
                         plan.affected_endpoints, plan.location.x,
                         plan.location.y, plan.recovery,
                         unsigned(plan.tree ? plan.tree->nodes.size() : 1)});
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
                    if(options.path_buffer_fanout_repair) {
                        PhaseTimer timer{result.candidate_ms};
                        const auto allowance = detail::remainingBufferBudget(
                            options.path_buffer_max_inserted_buffers, result.inserted_buffers);
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
                    if(options.common.prepare) {
                        PhaseTimer timer{result.prepare_ms};
                        options.common.prepare();
                    }
                    update(false);
                    DesignRepairMetrics after;
                    auto next_states = inspect(after);
                    trial.after = after;
                    for(const auto& [pin, old_slack] : objective_pins) {
                        const double slack = std::min(0.0, double(sta->slack(pin,
                            sta::RiseFallBoth::riseFall(), sta->scenes(), sta::MinMax::max())));
                        trial.objective_wns_after = std::min(trial.objective_wns_after, slack);
                        trial.objective_tns_after += slack;
                    }
                    trial.rejection_reasons = rejectionReasons(
                        result.after, after, area_limit, result.before, options, trial);
                    if(trial.rejection_reasons.empty()) {
                        odb::dbDatabase::commitEco(block);
                        active = false;
                        result.accepted_trials += trial.buffers;
                        result.inserted_buffers += trial.buffers;
                        result.after = after;
                        states = std::move(next_states);
                        accepted = true;
                        record_trial();
                        break;
                    }
                    odb::dbDatabase::undoEco(block);
                    active = false;
                    ++result.rejected_trials;
                    if(options.path_buffer_strategy == "feedback")
                        for(const auto index : selected)
                            failed_actions.insert(plans[index].feedback_key);
                    needs_refresh = true;
                    if(!options.defer_rollback_refresh) {
                        update(true);
                        states = inspect(result.after);
                    }
                    record_trial();
                    backoff();
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
        : !budget_available()
            ? "path buffer time/refresh budget exhausted; violations remain"
        : !trials_available()
            ? "path buffer trial budget exhausted; violations remain"
            : "no acceptable path buffer candidate; violations remain";
    return result;
}
}  // namespace ecc
