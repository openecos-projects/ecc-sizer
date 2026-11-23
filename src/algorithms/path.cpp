// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "actions/netlist.h"
#include "design_analysis.h"
#include "sta/Graph.hh"
#include "sta/Sdc.hh"
#include "sta/Mode.hh"
#include "sta/Liberty.hh"
#include "algorithms/path.h"

#include <algorithm>
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
namespace path_detail {
using detail::NetState;
BufferTreePlan bufferPlan(const PathBufferCandidate& candidate) {
    BufferTreePlan plan;
    plan.net = candidate.net;
    if (candidate.tree) {
        for (const auto& source : candidate.tree->nodes) {
            BufferTreeNode node;
            node.master = candidate.master;
            node.location = source.location;
            node.children = source.children;
            for (auto i : source.loads) {
                node.loads.push_back(candidate.tree->terminals.at(i));
            }
            plan.nodes.push_back(std::move(node));
        }
    } else {
        plan.nodes.push_back(
            {candidate.master, candidate.location, candidate.loads, {}});
    }
    return plan;
}

// Match OpenSTA's fanout-load semantics: output ports do not contribute;
// Liberty input weights override the library default, and a missing Liberty
// pin contributes one. A missing weight on an existing Liberty pin is zero.
float fanoutWeight(sta::dbNetwork* network, odb::dbObject* term) {
    auto* pin = network->dbToSta(term);
    if (!network->isLoad(pin) || network->isTopLevelPort(pin)) {
        return 0;
    }
    auto* port = network->libertyPort(pin);
    if (!port) {
        return 1;
    }
    float weight = 0;
    bool exists = false;
    port->fanoutLoad(weight, exists);
    if (!exists) {
        port->libertyLibrary()->defaultFanoutLoad(weight, exists);
    }
    return exists ? weight : 0;
}

float bufferFanoutLimit(sta::dbSta* sta, const detail::CellModel& buffer) {
    auto* network = sta->getDbNetwork();
    float limit = detail::kInf, value;
    bool exists;
    const auto merge = [&]() {
        if (exists) {
            limit = std::min(limit, value);
        }
    };
    for (auto* mode : sta->modes()) {
        mode->sdc()->fanoutLimit(network->cell(network->topInstance()),
                                 sta::MinMax::max(), value, exists);
        merge();
        mode->sdc()->fanoutLimit(network->dbToSta(buffer.master), sta::MinMax::max(),
                                 value, exists);
        merge();
    }
    buffer.output->fanoutLimit(sta::MinMax::max(), value, exists);
    if (!exists) {
        buffer.cell->libertyLibrary()->defaultMaxFanout(value, exists);
    }
    merge();
    return limit;
}

float bufferCapLimit(sta::dbSta* sta, const detail::CellModel& buffer) {
    auto* network = sta->getDbNetwork();
    float limit = buffer.max_cap, value;
    bool exists;
    for (auto* mode : sta->modes()) {
        mode->sdc()->capacitanceLimit(network->cell(network->topInstance()),
                                      sta::MinMax::max(), value, exists);
        if (exists) {
            limit = std::min(limit, value);
        }
        mode->sdc()->capacitanceLimit(network->dbToSta(buffer.master),
                                      sta::MinMax::max(), value, exists);
        if (exists) {
            limit = std::min(limit, value);
        }
    }
    return limit;
}

float bufferSlewLimit(sta::dbSta* sta, const detail::CellModel& buffer) {
    float limit = detail::kInf, value;
    bool exists = false;
    buffer.output->slewLimit(sta::MinMax::max(), value, exists);
    if (!exists) {
        buffer.cell->libertyLibrary()->defaultMaxSlew(value, exists);
    }
    if (exists) {
        limit = std::min(limit, value);
    }
    auto* network = sta->getDbNetwork();
    for (auto* mode : sta->modes()) {
        mode->sdc()->slewLimit(network->cell(network->topInstance()),
                               sta::MinMax::max(), value, exists);
        if (exists) {
            limit = std::min(limit, value);
        }
        mode->sdc()->slewLimit(network->dbToSta(buffer.master), sta::MinMax::max(),
                               value, exists);
        if (exists) {
            limit = std::min(limit, value);
        }
    }
    return limit;
}

float libertyFanoutWeight(sta::LibertyPort* port) {
    float weight;
    bool exists;
    port->fanoutLoad(weight, exists);
    if (!exists) {
        port->libertyLibrary()->defaultFanoutLoad(weight, exists);
    }
    return exists ? weight : 0;
}

// Spatial packing uses Liberty/SDC electrical limits, with no group-size or
// depth budget. Strict reduction of the frontier guarantees termination.
PathTree planPathTree(const NetState& state, const detail::CellModel& buffer,
                      sta::dbSta* sta, BufferLocation root_location,
                      double cap_per_length, float cap_limit, float fanout_limit) {
    PathTree tree;
    struct Item {
        size_t index;
        bool leaf;
        int x, y;
        double cap, weight;
    };
    std::vector<Item> frontier;
    for (size_t i = 0; i < state.loads.size(); ++i) {
        const auto& load = state.loads[i];
        const float weight = fanoutWeight(sta->getDbNetwork(), load.term);
        if (!std::isfinite(weight) || weight < 0 || weight > fanout_limit ||
            load.capacitance > cap_limit) {
            return {};
        }
        frontier.push_back({i, true, load.x, load.y, load.capacitance, weight});
        tree.terminals.push_back(load.term);
    }
    const auto center = [](const std::vector<Item>& items) {
        long long x = 0, y = 0;
        for (const auto& item : items) {
            x += item.x;
            y += item.y;
        }
        return BufferLocation{int(x / items.size()), int(y / items.size())};
    };
    const auto cap = [&](const std::vector<Item>& items, BufferLocation p) {
        double c = 0;
        for (const auto& item : items) {
            c += item.cap + cap_per_length * (std::abs(double(p.x) - item.x) +
                                              std::abs(double(p.y) - item.y));
        }
        return c;
    };
    const auto fits = [&](const std::vector<Item>& items, BufferLocation p) {
        double weight = 0;
        for (const auto& item : items) {
            weight += item.weight;
        }
        return weight <= fanout_limit && cap(items, p) <= cap_limit;
    };
    const auto append = [&](const std::vector<Item>& items, BufferLocation p) {
        PathTreeNode node;
        node.location = p;
        node.cap = cap(items, p);
        for (const auto& item : items) {
            (item.leaf ? node.loads : node.children).push_back(item.index);
        }
        tree.nodes.push_back(std::move(node));
        return Item{tree.nodes.size() - 1,
                    false,
                    p.x,
                    p.y,
                    buffer.input->capacitance(),
                    libertyFanoutWeight(buffer.input)};
    };
    while (!frontier.empty()) {
        if (fits(frontier, root_location)) {
            append(frontier, root_location);
            return tree;
        }
        int min_x = frontier.front().x, max_x = min_x;
        int min_y = frontier.front().y, max_y = min_y;
        for (const auto& item : frontier) {
            min_x = std::min(min_x, item.x);
            max_x = std::max(max_x, item.x);
            min_y = std::min(min_y, item.y);
            max_y = std::max(max_y, item.y);
        }
        const bool axis_x = double(max_x) - min_x >= double(max_y) - min_y;
        std::sort(frontier.begin(), frontier.end(), [&](const auto& a, const auto& b) {
            return std::make_tuple(axis_x ? a.x : a.y, axis_x ? a.y : a.x, a.index) <
                   std::make_tuple(axis_x ? b.x : b.y, axis_x ? b.y : b.x, b.index);
        });
        std::vector<Item> next, group;
        for (const auto& item : frontier) {
            group.push_back(item);
            if (!fits(group, center(group))) {
                group.pop_back();
                if (group.empty()) {
                    return {};
                }
                next.push_back(append(group, center(group)));
                group = {item};
                if (!fits(group, center(group))) {
                    return {};
                }
            }
        }
        if (!group.empty()) {
            next.push_back(append(group, center(group)));
        }
        // First-level singleton groups may replace heavy sinks by lighter
        // buffer inputs. On subsequent levels no reduction means infeasible.
        if (!frontier.front().leaf && next.size() >= frontier.size()) {
            return {};
        }
        frontier = std::move(next);
    }
    return {};
}

struct PathTreeTiming {
    std::vector<float> delay, slew;
    double max_slew = 0, max_cap = 0;
};
PathTreeTiming predictPathTree(detail::DesignAnalysis& analysis, const NetState& state,
                               const detail::CellModel& buffer, const PathTree& tree,
                               float driver_slew, double upstream_delay,
                               double resistance_per_length) {
    PathTreeTiming result;
    result.delay.resize(state.loads.size());
    result.slew.resize(state.loads.size());
    std::vector<double> arrivals(tree.nodes.size()), slews(tree.nodes.size());
    arrivals.back() = upstream_delay;
    // 10–90% transition of a first-order RC: ln(9) * Elmore.
    const auto wire_slew = [](double slew, double elmore) {
        return std::hypot(slew, std::log(9.0) * elmore);
    };
    slews.back() = wire_slew(driver_slew, upstream_delay);
    for (size_t i = tree.nodes.size(); i-- > 0;) {
        const auto& node = tree.nodes[i];
        const auto [delay, slew] = analysis.predictedTiming(buffer, node.cap, slews[i]);
        result.max_slew = std::max({result.max_slew, slews[i], double(slew)});
        result.max_cap = std::max(result.max_cap, double(node.cap));
        const double arrival = arrivals[i] + delay;
        const auto wire = [&](int x, int y) {
            return resistance_per_length * node.cap *
                   (std::abs(double(node.location.x) - x) +
                    std::abs(double(node.location.y) - y));
        };
        for (auto child : node.children) {
            const auto p = tree.nodes[child].location;
            const double elmore = wire(p.x, p.y);
            arrivals[child] = arrival + elmore;
            slews[child] = wire_slew(slew, elmore);
        }
        for (auto load : node.loads) {
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
    if (term->getObjectType() != odb::dbITermObj) {
        return 0;
    }
    auto* input = static_cast<odb::dbITerm*>(term);
    auto* network = sta->getDbNetwork();
    auto* pin = network->dbToSta(input);
    auto* port = network->libertyPort(pin);
    if (!port) {
        return 0;
    }
    auto* vertex = sta->graph()->pinLoadVertex(pin);
    if (!vertex) {
        return 0;
    }
    const float old_slew = float(sta->slew(vertex, sta::RiseFallBoth::riseFall(),
                                           sta->scenes(), sta::MinMax::max()));
    const auto* pvt = sta->cmdMode()->sdc()->operatingConditions(sta::MinMax::max());
    float old_check = -detail::kInf, new_check = -detail::kInf;
    for (auto* set : port->libertyCell()->timingArcSets()) {
        if (set->to() != port || set->role() != sta::TimingRole::recovery() ||
            !set->from()) {
            continue;
        }
        auto* clock = input->getInst()->findITerm(set->from()->name().c_str());
        if (!clock) {
            continue;
        }
        auto* clock_vertex = sta->graph()->pinLoadVertex(network->dbToSta(clock));
        if (!clock_vertex) {
            continue;
        }
        const float clock_slew =
            float(sta->slew(clock_vertex, sta::RiseFallBoth::riseFall(), sta->scenes(),
                            sta::MinMax::max()));
        for (auto* arc : set->arcs()) {
            auto* model = arc->checkModel(sta->cmdScene(), sta::MinMax::max());
            if (!model) {
                continue;
            }
            old_check =
                std::max(old_check,
                         float(model->checkDelay(pvt, clock_slew, old_slew, 0,
                                                 sta::MinMax::max(), sta->pocvMode())));
            new_check =
                std::max(new_check,
                         float(model->checkDelay(pvt, clock_slew, new_slew, 0,
                                                 sta::MinMax::max(), sta->pocvMode())));
        }
    }
    return std::isfinite(old_check) && std::isfinite(new_check) ? old_check - new_check
                                                                : 0;
}

struct SampledPath {
    float slack;
    size_t start;
    std::vector<const sta::Pin*> pins;
};

std::vector<SampledPath> samplePaths(sta::dbSta* sta, const PathSearchOptions& options,
                                     bool recovery) {
    sta::StringSeq groups;
    const auto ends = sta->findPathEnds(
        nullptr, nullptr, nullptr, false, sta->scenes(), sta::MinMaxAll::max(),
        options.max_paths, options.paths_per_endpoint, true, false, -detail::kInf, 0,
        true, groups, !recovery, false, recovery, false, false, false);
    // STA slack queries may invalidate PathEnd and PathExpanded objects. Copy
    // every path before evaluating candidates or querying endpoint slack.
    std::vector<SampledPath> paths;
    unsigned path_count = 0;
    for (auto* end : ends) {
        if (path_count++ >= options.max_paths) {
            break;
        }
        const float slack = float(end->slack(sta));
        if (slack >= 0) {
            continue;
        }

        sta::PathExpanded expanded(end->path(), sta);
        SampledPath path{slack, expanded.startIndex(), {}};
        path.pins.reserve(expanded.size());
        for (size_t i = 0; i < expanded.size(); ++i) {
            path.pins.push_back(expanded.path(i)->pin(sta));
        }
        paths.push_back(std::move(path));
    }
    return paths;
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

std::vector<odb::dbInst*> downstreamCone(odb::dbNet* net, sta::dbNetwork* network) {
    std::vector<odb::dbNet*> queue{net};
    std::unordered_set<odb::dbNet*> visited;
    std::unordered_set<odb::dbInst*> instances;
    for (auto* pin : net->getITerms()) {
        if (pin->getIoType() == odb::dbIoType::OUTPUT) {
            instances.insert(pin->getInst());
        }
    }
    while (!queue.empty()) {
        auto* current = queue.back();
        queue.pop_back();
        if (!visited.insert(current).second) {
            continue;
        }
        for (auto* pin : current->getITerms()) {
            if (pin->getIoType() != odb::dbIoType::INPUT) {
                continue;
            }
            auto* inst = pin->getInst();
            instances.insert(inst);
            auto* cell = network->libertyCell(inst);
            if (!cell || cell->hasSequentials() || cell->isClockGate()) {
                continue;
            }
            for (auto* output : inst->getITerms()) {
                if (output->getIoType() == odb::dbIoType::OUTPUT && output->getNet() &&
                    !output->getNet()->isSpecial()) {
                    queue.push_back(output->getNet());
                }
            }
        }
    }
    return {instances.begin(), instances.end()};
}

// One committed STA snapshot shared by every candidate forecast. Own no
// Path/PathEnd pointers: querying slack can rebuild STA's path groups.
struct PathSamples {
    std::unordered_map<odb::dbNet*, std::vector<PathUse>> uses;
    std::vector<const sta::Pin*> endpoint_pins;
    std::vector<path_detail::PathForecast> sampled_paths;
    std::map<size_t, double> actual_endpoint_slacks;
    std::unordered_map<
        odb::dbNet*,
        std::unordered_map<odb::dbObject*, path_detail::EndpointSlackProfile>>
        endpoint_profiles;
    PathSamples(sta::dbSta* sta, const PathSearchOptions& options) {
        auto* network = sta->getDbNetwork();
        std::unordered_map<const sta::Pin*, size_t> endpoints;
        // Preserve each sampled path, including competing checks/paths at the
        // same endpoint. Generate physical actions once for their union.
        for (bool recovery : {false, true}) {
            if (recovery && !options.recovery) {
                continue;
            }
            for (const auto& path : samplePaths(sta, options, recovery)) {
                const float slack = path.slack;
                if (!std::isfinite(slack) || path.pins.empty()) {
                    continue;
                }
                const auto* endpoint = path.pins.back();
                const auto [it, added] = endpoints.emplace(endpoint, endpoints.size());
                if (added) {
                    endpoint_pins.push_back(endpoint);
                }
                const auto path_index = sampled_paths.size();
                sampled_paths.push_back({it->second, slack, slack, false});
                for (size_t i = path.start; i + 1 < path.pins.size(); ++i) {
                    auto* output = network->staToDb(path.pins[i]);
                    auto* load = network->staToDb(path.pins[i + 1]);
                    if (!output || output->getObjectType() != odb::dbITermObj ||
                        !load) {
                        continue;
                    }
                    auto* driver = static_cast<odb::dbITerm*>(output);
                    if (driver->getIoType() != odb::dbIoType::OUTPUT ||
                        !driver->getNet()) {
                        continue;
                    }
                    auto* load_net = load->getObjectType() == odb::dbITermObj
                                         ? static_cast<odb::dbITerm*>(load)->getNet()
                                     : load->getObjectType() == odb::dbBTermObj
                                         ? static_cast<odb::dbBTerm*>(load)->getNet()
                                         : nullptr;
                    if (load_net == driver->getNet()) {
                        uses[load_net].push_back(
                            {it->second, load, slack, path_index, recovery});
                    }
                }
            }
        }
        if (endpoint_pins.empty()) {
            return;
        }
        // Slack queries may rebuild OpenSTA path groups. Do not issue them while
        // iterating the PathEnd objects returned by findPathEnds().
        for (size_t i = 0; i < endpoint_pins.size(); ++i) {
            actual_endpoint_slacks.emplace(
                i, std::min(0.0, double(sta->slack(
                                     endpoint_pins[i], sta::RiseFallBoth::riseFall(),
                                     sta->scenes(), sta::MinMax::max()))));
        }
        collectEndpointProfiles(sta, options);
    }

    void collectEndpointProfiles(sta::dbSta* sta, const PathSearchOptions& options) {
        auto* network = sta->getDbNetwork();
        // Index the CURRENT worst max path of every endpoint once per committed
        // design, retaining only nets already chosen by the critical-path search.
        // This does not expand the candidate-net search or the acceptance scope.
        // Each endpoint contributes at most once per net, including reconvergence.
        // It is a scheduling forecast: competing paths after the ECO still require
        // global STA, and these profiles are never used to skip a trial.
        if (options.endpoint_coverage &&
            options.gain_scope != ecc::PathGainScope::Endpoints) {
            sta->findRequireds();
            std::unordered_map<const sta::Pin*, sta::Path*> worst_paths;
            for (auto* vertex : sta->endpoints()) {
                auto* path = sta->vertexWorstSlackPath(vertex, sta::MinMax::max());
                if (!path || !std::isfinite(double(path->slack(sta))) ||
                    std::abs(double(path->slack(sta))) >= detail::kInf) {
                    continue;
                }
                auto [it, added] = worst_paths.emplace(vertex->pin(), path);
                if (!added && path->slack(sta) < it->second->slack(sta)) {
                    it->second = path;
                }
            }
            for (const auto& [endpoint, worst] : worst_paths) {
                sta::PathExpanded path(worst, sta);
                std::unordered_set<odb::dbNet*> seen;
                for (size_t i = path.startIndex(); i + 1 < path.size(); ++i) {
                    auto* output = network->staToDb(path.path(i)->pin(sta));
                    auto* load = network->staToDb(path.path(i + 1)->pin(sta));
                    if (!output || output->getObjectType() != odb::dbITermObj ||
                        !load) {
                        continue;
                    }
                    auto* driver = static_cast<odb::dbITerm*>(output);
                    auto* net = driver->getNet();
                    if (driver->getIoType() != odb::dbIoType::OUTPUT ||
                        !uses.contains(net)) {
                        continue;
                    }
                    auto* load_net = load->getObjectType() == odb::dbITermObj
                                         ? static_cast<odb::dbITerm*>(load)->getNet()
                                     : load->getObjectType() == odb::dbBTermObj
                                         ? static_cast<odb::dbBTerm*>(load)->getNet()
                                         : nullptr;
                    if (load_net == net && seen.insert(net).second) {
                        endpoint_profiles[net][load].add(double(worst->slack(sta)));
                    }
                }
            }
            for (auto& [net, profiles] : endpoint_profiles) {
                for (auto& [load, profile] : profiles) {
                    profile.finish();
                }
            }
        }
    }
};

CandidateCollection generatePathCandidates(detail::DesignAnalysis& analysis,
                                           sta::dbSta* sta,
                                           const std::vector<NetState>& states,
                                           const PathSearchOptions& options,
                                           const DesignMetrics& metrics,
                                           PathPolicy& feedback) {
    analysis.beginTimingPredictionEpoch();
    struct PredictionEpoch {
        detail::DesignAnalysis& analysis;
        ~PredictionEpoch() { analysis.endTimingPredictionEpoch(); }
    } prediction_epoch{analysis};
    CandidateCollection collection;
    feedback.beginCollection();
    auto* network = sta->getDbNetwork();
    const PathSamples samples(sta, options);
    const auto& [uses, endpoint_pins, sampled_paths, actual_endpoint_slacks,
                 endpoint_profiles] = samples;
    if (endpoint_pins.empty()) {
        return collection;
    }
    std::unordered_map<odb::dbInst*, const NetState*> driven;
    for (const auto& state : states) {
        if (state.driver) {
            driven.emplace(state.driver->getInst(), &state);
        }
    }
    // Cell limits are invariant throughout this STA snapshot.
    std::vector<std::tuple<float, float, float>> buffer_limits;
    for (const auto& buffer : analysis.bufferChoices()) {
        buffer_limits.emplace_back(
            bufferCapLimit(sta, buffer) * (1 - options.common.margin),
            bufferFanoutLimit(sta, buffer), bufferSlewLimit(sta, buffer));
    }
    for (const auto& state : states) {
        const auto use_it = uses.find(state.net);
        if (!state.driver || use_it == uses.end()) {
            continue;
        }
        const auto& net_uses = use_it->second;
        int dx, dy;
        state.driver->getInst()->getLocation(dx, dy);
        const auto distance = [](int ax, int ay, int bx, int by) {
            return std::abs(double(ax) - bx) + std::abs(double(ay) - by);
        };
        const auto model = analysis.driverModel(state);
        const auto [before_delay, before_slew] =
            analysis.predictedTiming(model, state.capacitance, state.input_slew);
        if (!std::isfinite(before_delay)) {
            continue;
        }
        std::unordered_map<odb::dbObject*, size_t> index;
        std::vector<float> slacks, elmore;
        double total_length = 0, pin_cap = 0;
        for (size_t i = 0; i < state.loads.size(); ++i) {
            const auto& load = state.loads[i];
            index[load.term] = i;
            pin_cap += load.capacitance;
            total_length += distance(dx, dy, load.x, load.y);
            auto* pin = network->dbToSta(load.term);
            slacks.push_back(float(sta->slack(pin, sta::RiseFallBoth::riseFall(),
                                              sta->scenes(), sta::MinMax::max())));
            float wire_delay = 0;
            for (const auto* rf : {sta::RiseFall::rise(), sta::RiseFall::fall()}) {
                float delay = 0;
                bool exists = false;
                sta->findElmore(network->dbToSta(state.driver), pin, rf,
                                sta::MinMax::max(), delay, exists);
                if (exists && std::isfinite(delay)) {
                    wire_delay = std::max(wire_delay, delay);
                }
            }
            elmore.push_back(wire_delay);
        }
        // Keep sampled path loads critical even if the pin slack is
        // unavailable.
        for (const auto& use : net_uses) {
            if (index.contains(use.load)) {
                slacks[index.at(use.load)] =
                    std::min(slacks[index.at(use.load)], use.slack);
            }
        }
        const double wire_cap = std::max(0.0, double(state.capacitance) - pin_cap);
        const double cap_per_length = total_length > 0 ? wire_cap / total_length : 0;
        std::set<std::vector<size_t>> load_groups;
        const auto add_group = [&](std::vector<size_t> group) {
            if (group.empty()) {
                return;
            }
            std::sort(group.begin(), group.end());
            if (!load_groups.insert(std::move(group)).second) {
                ++collection.duplicate_candidates;
            }
        };
        std::vector<size_t> all, side, critical;
        for (size_t i = 0; i < state.loads.size(); ++i) {
            all.push_back(i);
            (slacks[i] < 0 ? critical : side).push_back(i);
        }
        add_group(all);
        add_group(side);
        add_group(critical);
        for (const auto& use : net_uses) {
            if (!index.contains(use.load)) {
                continue;
            }
            std::vector<size_t> others;
            for (auto i : all) {
                if (i != index.at(use.load)) {
                    others.push_back(i);
                }
            }
            add_group(std::move(others));
            add_group({index.at(use.load)});
        }
        // Spatial splits preserve coherent branches; no tuned distance
        // threshold.
        for (bool axis_x : {true, false}) {
            auto ordered = all;
            std::stable_sort(ordered.begin(), ordered.end(), [&](size_t a, size_t b) {
                return axis_x ? state.loads[a].x < state.loads[b].x
                              : state.loads[a].y < state.loads[b].y;
            });
            const auto middle = ordered.begin() + ordered.size() / 2;
            add_group({ordered.begin(), middle});
            add_group({middle, ordered.end()});
        }
        struct GateResponse {
            detail::CellModel model{};
            detail::DesignAnalysis::DelayPrediction delay;
            const NetState* state = nullptr;
            float before = 0;
            std::unordered_map<float, float> delay_cache;
        };
        std::vector<GateResponse> gate_responses(state.loads.size());
        std::vector<double> wire_resistance(state.loads.size());
        for (size_t i = 0; i < state.loads.size(); ++i) {
            const auto& load = state.loads[i];
            const double length = distance(dx, dy, load.x, load.y);
            wire_resistance[i] = length > 0 && state.capacitance > 0
                                     ? elmore[i] / (length * state.capacitance)
                                     : 0;
            if (load.term->getObjectType() != odb::dbITermObj) {
                continue;
            }
            auto* sink_inst = static_cast<odb::dbITerm*>(load.term)->getInst();
            auto* sink_cell = network->libertyCell(sink_inst);
            if (sink_cell && sink_cell->hasSequentials()) {
                continue;
            }
            const auto next = driven.find(sink_inst);
            if (next == driven.end()) {
                continue;
            }
            auto& response = gate_responses[i];
            response.state = next->second;
            response.model = analysis.driverModel(*response.state);
            response.delay = analysis.prepareDelayPrediction(
                response.model, response.state->capacitance);
            response.before = response.delay(response.state->input_slew);
        }
        std::unordered_map<float, std::pair<float, float>> driver_cache;
        std::ostringstream boundary;
        // SOFT transfer between structurally identical families. Global
        // legalization and RC extraction can perturb numerical boundaries
        // after any ECO; treating those as entirely new families repeatedly
        // forgets measured failures. Fresh forecasts include the new RC/
        // slew/placement state and can reopen an improved opportunity.
        // Cross-generation evidence NEVER skips the final STA trial.
        boundary << terminalKey(state.driver) << ':'
                 << state.driver->getInst()->getMaster()->getName();
        std::vector<std::string> topology_loads;
        for (const auto& load : state.loads) {
            std::string entry = terminalKey(load.term);
            if (load.term->getObjectType() == odb::dbITermObj) {
                entry += '/' + std::string(static_cast<odb::dbITerm*>(load.term)
                                               ->getInst()
                                               ->getMaster()
                                               ->getName());
            }
            topology_loads.push_back(std::move(entry));
        }
        std::sort(topology_loads.begin(), topology_loads.end());
        for (const auto& entry : topology_loads) {
            boundary << ':' << entry.size() << ':' << entry;
        }

        // This is a dependency hint for deferring retries, not proof that all
        // global timing/placement dependencies are unchanged across commits.
        const auto boundary_key = std::make_shared<const std::string>(boundary.str());
        std::vector<PathBufferCandidate> variants;
        for (const auto& group : load_groups) {
            std::vector<bool> selected(state.loads.size(), false);
            double selected_cap = 0, removed_length = 0, weight = 0;
            long long x = 0, y = 0;
            for (auto i : group) {
                selected[i] = true;
                selected_cap += state.loads[i].capacitance;
                weight += fanoutWeight(network, state.loads[i].term);
                removed_length += distance(dx, dy, state.loads[i].x, state.loads[i].y);
                x += state.loads[i].x;
                y += state.loads[i].y;
            }
            const int cx = int(x / group.size()), cy = int(y / group.size());
            std::set<std::pair<int, int>> locations{
                {cx, cy},
                {dx, dy},
                {int((static_cast<long long>(dx) + cx) / 2),
                 int((static_cast<long long>(dy) + cy) / 2)}};
            for (const auto& [bx, by] : locations) {
                const double upstream_length = distance(dx, dy, bx, by);
                double downstream_length = 0;
                for (auto i : group) {
                    downstream_length +=
                        distance(bx, by, state.loads[i].x, state.loads[i].y);
                }
                const float buffer_cap =
                    selected_cap + cap_per_length * downstream_length;
                size_t buffer_index = 0;
                for (const auto& buffer : analysis.bufferChoices()) {
                    const auto [cap_limit, fanout_limit, cell_slew_limit] =
                        buffer_limits[buffer_index++];
                    std::shared_ptr<const PathTree> tree;
                    // Whole-net trees are a separate physical candidate. Other
                    // load groups retain the existing single-buffer search.
                    if (options.tree_candidates && group.size() == state.loads.size() &&
                        (buffer_cap > cap_limit || weight > fanout_limit)) {
                        auto plan =
                            planPathTree(state, buffer, sta, {bx, by}, cap_per_length,
                                         cap_limit, fanout_limit);
                        if (plan.nodes.empty()) {
                            continue;
                        }
                        tree = std::make_shared<PathTree>(std::move(plan));
                    } else if (buffer_cap > cap_limit) {
                        continue;
                    }
                    const float new_cap =
                        std::max(0.0, double(state.capacitance) - selected_cap -
                                          removed_length * cap_per_length) +
                        buffer.input->capacitance() + upstream_length * cap_per_length;
                    auto [driver_it, uncached] =
                        driver_cache.emplace(new_cap, std::pair<float, float>{});
                    if (uncached) {
                        driver_it->second =
                            analysis.predictedTiming(model, new_cap, state.input_slew);
                    }
                    const auto [driver_delay, driver_slew] = driver_it->second;
                    const auto [buffer_delay, buffer_slew] = analysis.predictedTiming(
                        buffer, tree ? tree->nodes.back().cap : buffer_cap,
                        driver_slew);
                    if (!std::isfinite(driver_delay) || !std::isfinite(buffer_delay) ||
                        !std::isfinite(buffer_slew) || !std::isfinite(driver_slew)) {
                        continue;
                    }
                    PathBufferCandidate candidate;
                    candidate.net = state.net;
                    candidate.recovery =
                        std::any_of(net_uses.begin(), net_uses.end(),
                                    [](const auto& use) { return use.recovery; });
                    candidate.tree = tree;
                    candidate.master = buffer.master;
                    candidate.location = {bx, by};
                    candidate.slack = metrics.setup_wns;
                    candidate.predicted_gain = before_delay - driver_delay;
                    std::vector<float> gains(state.loads.size());
                    double side_wns_after = 0;
                    PathTreeTiming tree_timing;
                    if (tree) {
                        double r = 0;
                        for (double value : wire_resistance) {
                            r += value;
                        }
                        r /= std::max(size_t(1), wire_resistance.size());
                        tree_timing =
                            predictPathTree(analysis, state, buffer, *tree, driver_slew,
                                            r * upstream_length * new_cap, r);
                        if (std::any_of(tree_timing.delay.begin(),
                                        tree_timing.delay.end(),
                                        [](float v) { return !std::isfinite(v); }) ||
                            std::any_of(tree_timing.slew.begin(),
                                        tree_timing.slew.end(),
                                        [](float v) { return !std::isfinite(v); })) {
                            continue;
                        }
                    }
                    for (size_t i = 0; i < state.loads.size(); ++i) {
                        const auto& load = state.loads[i];
                        // Effective branch R is calibrated from extracted
                        // Elmore.
                        const double resistance_per_length = wire_resistance[i];
                        double new_wire_delay =
                            elmore[i] *
                            (state.capacitance > 0 ? new_cap / state.capacitance : 1);
                        if (selected[i]) {
                            new_wire_delay =
                                resistance_per_length *
                                (upstream_length * new_cap +
                                 distance(bx, by, load.x, load.y) * buffer_cap);
                            candidate.loads.push_back(load.term);
                        }
                        gains[i] =
                            candidate.predicted_gain + elmore[i] -
                            (tree ? tree_timing.delay[i]
                                  : new_wire_delay + (selected[i] ? buffer_delay : 0));
                        // Include the first downstream gate's slew response.
                        auto& gate = gate_responses[i];
                        if (gate.state) {
                            const float new_input =
                                std::max(0.0f, gate.state->input_slew +
                                                   (tree          ? tree_timing.slew[i]
                                                    : selected[i] ? buffer_slew
                                                                  : driver_slew) -
                                                   before_slew);
                            auto [gate_it, uncached_gate] =
                                gate.delay_cache.emplace(new_input, 0);
                            if (uncached_gate) {
                                gate_it->second = gate.delay(new_input);
                            }
                            if (std::isfinite(gate.before) &&
                                std::isfinite(gate_it->second)) {
                                gains[i] += gate.before - gate_it->second;
                            }
                        }
                        if (std::isfinite(slacks[i])) {
                            side_wns_after =
                                std::min(side_wns_after, double(slacks[i]) + gains[i]);
                        }
                    }
                    auto forecasts = sampled_paths;
                    std::unordered_set<size_t> affected;
                    for (const auto& use : net_uses) {
                        if (!index.contains(use.load)) {
                            continue;
                        }
                        const auto i = index.at(use.load);
                        const float constraint_gain =
                            use.recovery
                                ? recoveryConstraintGain(sta, state.loads[i].term,
                                                         tree ? tree_timing.slew[i]
                                                         : selected[i] ? buffer_slew
                                                                       : driver_slew)
                                : 0;
                        const double slack = use.slack + gains[i] + constraint_gain;
                        auto& path = forecasts.at(use.path);
                        path.after =
                            path.affected ? std::min(path.after, slack) : slack;
                        path.affected = true;
                        affected.insert(use.endpoint);
                    }
                    const auto forecast = path_detail::forecastObjective(
                        forecasts, actual_endpoint_slacks, metrics.setup_wns,
                        options.gain_scope == ecc::PathGainScope::Endpoints);
                    candidate.predicted_wns_gain = forecast.wns_gain;
                    candidate.quality.path_gain = forecast.path_gain;
                    const double slew_limit =
                        std::min(cell_slew_limit, state.slew_limit);
                    const double peak_slew = tree ? tree_timing.max_slew
                                                  : std::max(driver_slew, buffer_slew);
                    const double peak_cap = tree ? tree_timing.max_cap : buffer_cap;
                    candidate.quality.electrical_excess =
                        std::max({0.0, slew_limit > 0 ? peak_slew / slew_limit - 1 : 0,
                                  cap_limit > 0 ? peak_cap / cap_limit - 1 : 0});
                    // A side branch outside the sampled endpoints can still
                    // predict a global regression. Preserve this risk signal
                    // for ordering only; it is not a pruning proof.
                    if (options.gain_scope != ecc::PathGainScope::Endpoints &&
                        side_wns_after < metrics.setup_wns) {
                        candidate.predicted_wns_gain =
                            std::min(double(candidate.predicted_wns_gain),
                                     side_wns_after - metrics.setup_wns);
                    }
                    candidate.predicted_tns_gain = forecast.tns_gain;
                    candidate.covered_tns_gain = forecast.tns_gain;
                    const auto profiles = endpoint_profiles.find(state.net);
                    if (profiles != endpoint_profiles.end()) {
                        candidate.covered_tns_gain = 0;
                        for (size_t i = 0; i < state.loads.size(); ++i) {
                            const auto profile =
                                profiles->second.find(state.loads[i].term);
                            if (profile == profiles->second.end()) {
                                continue;
                            }
                            const float constraint_gain =
                                candidate.recovery
                                    ? recoveryConstraintGain(sta, state.loads[i].term,
                                                             tree ? tree_timing.slew[i]
                                                             : selected[i]
                                                                 ? buffer_slew
                                                                 : driver_slew)
                                    : 0;
                            candidate.covered_tns_gain +=
                                profile->second.gain(gains[i] + constraint_gain);
                            candidate.covered_endpoints += profile->second.size();
                        }
                    }
                    candidate.affected_endpoints = forecast.affected_endpoints;
                    for (size_t i = 0; i < endpoint_pins.size(); ++i) {
                        if (!affected.contains(i)) {
                            continue;
                        }
                        // This collection uses one committed STA snapshot.
                        // Reuse its endpoint query across all tree variants.
                        const float pin_slack = actual_endpoint_slacks.at(i);
                        candidate.endpoints.emplace_back(endpoint_pins[i], pin_slack);
                        candidate.objective_wns =
                            std::min(candidate.objective_wns, double(pin_slack));
                        candidate.objective_tns += pin_slack;
                    }
                    std::sort(candidate.loads.begin(), candidate.loads.end(),
                              [](auto* a, auto* b) {
                                  return std::less<const void*>{}(a, b);
                              });
                    candidate.feedback_key = actionKey(candidate);

                    candidate.feedback_context = boundary_key;
                    candidate.group_key = actionKey(candidate, false);
                    // All tree shapes/strengths/positions for one load set
                    // share a scheduling family, while calibration continues
                    // to distinguish the actual tree topology.
                    path_detail::ActionNode load_group;
                    for (auto* load : candidate.loads) {
                        load_group.terminals.push_back(terminalKey(load));
                    }
                    candidate.load_group_key = path_detail::actionFingerprint(
                        std::to_string(state.net->getId()), {load_group}, false);
                    variants.push_back(std::move(candidate));
                }
            }
        }
        feedback.select(variants);
        const auto cone = options.batch_size > 1 ? downstreamCone(state.net, network)
                                                 : std::vector<odb::dbInst*>{};
        for (size_t i = 0; i < variants.size(); ++i) {
            variants[i].variant_rank = i;
            variants[i].cone = cone;
            collection.candidates.push_back(std::move(variants[i]));
        }
    }
    // Diversify nets before retrying another cell/location on the same net.
    std::stable_sort(collection.candidates.begin(), collection.candidates.end(),
                     PathPolicy::order);
    return collection;
}

CandidateCollection collectCandidates(detail::DesignAnalysis& analysis, sta::dbSta* sta,
                                      const std::vector<NetState>& states,
                                      const PathSearchOptions& options,
                                      const DesignMetrics& metrics,
                                      PathPolicy& feedback) {
    auto result =
        generatePathCandidates(analysis, sta, states, options, metrics, feedback);
    // Keep the best-ranked objective for an identical physical action. Pins
    // from either query remain valid; PathEnd pointers would not survive it.
    std::unordered_set<std::string> actions;
    auto end = std::remove_if(result.candidates.begin(), result.candidates.end(),
                              [&](const auto& c) {
                                  if (actions.insert(actionKey(c)).second) {
                                      return false;
                                  }
                                  ++result.duplicate_candidates;
                                  return true;
                              });
    result.candidates.erase(end, result.candidates.end());
    return result;
}

} // namespace path_detail

std::vector<PathCandidate>
PathSearch::generateCandidates(odb::dbBlock* block, sta::dbSta* sta,
                               const PathSearchOptions& options,
                               const DesignConstraints& constraints) {
    using namespace path_detail;
    options.validate();
    if (!block || !sta || sta->getDbNetwork()->block() != block ||
        sta->scenes().size() != 1) {
        throw std::invalid_argument(
            "Path candidates require one live OpenDB/STA scene");
    }
    detail::DesignAnalysis analysis(block, sta, options.common, constraints);
    DesignMetrics metrics;
    const auto states = analysis.analyze(metrics, true);
    unsigned inserted_buffers = 0;
    PathSearchStatistics statistics;
    PathSearchState snapshot{metrics, metrics, inserted_buffers, statistics};
    PathPolicy policy{options, snapshot};
    const auto candidates = policy.collect(analysis, sta, states, metrics);
    std::vector<PathCandidate> result;
    result.reserve(candidates.candidates.size());
    for (const auto& candidate : candidates.candidates) {
        result.push_back({bufferPlan(candidate), candidate.slack,
                          candidate.predicted_wns_gain, candidate.predicted_tns_gain,
                          candidate.recovery});
    }
    return result;
}

} // namespace ecc

namespace ecc::path_detail {
std::string terminalKey(odb::dbObject* term) {
    return std::to_string(int(term->getObjectType())) + '/' +
           std::to_string(term->getId());
}

std::string actionKey(const PathBufferCandidate& candidate, bool physical) {
    const auto master = std::to_string(candidate.master->getLib()->getId()) + '/' +
                        std::to_string(candidate.master->getId());
    std::vector<path_detail::ActionNode> nodes;
    if (candidate.tree) {
        for (const auto& node : candidate.tree->nodes) {
            path_detail::ActionNode out{
                master, node.location.x, node.location.y, {}, node.children};
            for (auto index : node.loads) {
                out.terminals.push_back(
                    terminalKey(candidate.tree->terminals.at(index)));
            }
            nodes.push_back(std::move(out));
        }
    } else {
        path_detail::ActionNode out{
            master, candidate.location.x, candidate.location.y, {}, {}};
        for (auto* load : candidate.loads) {
            out.terminals.push_back(terminalKey(load));
        }
        nodes.push_back(std::move(out));
    }
    return path_detail::actionFingerprint(std::to_string(candidate.net->getId()), nodes,
                                          physical);
}

const DesignMetrics& globalGainReference(const PathSearchOptions& options,
                                         const DesignMetrics& initial,
                                         const DesignMetrics& current) {
    return options.gain_scope == ecc::PathGainScope::CurrentGlobal ? current : initial;
}

PathGainThreshold gainThreshold(const PathSearchOptions& options,
                                const DesignMetrics& initial,
                                const DesignMetrics& current, double endpoint_wns,
                                double endpoint_tns) {
    const auto& reference = globalGainReference(options, initial, current);
    const bool local = options.gain_scope == PathGainScope::Endpoints;
    const auto floor = [&](double value) {
        return std::max(double(options.min_improvement),
                        std::abs(value) * options.min_relative_improvement);
    };
    return {floor(local ? endpoint_wns : double(reference.setup_wns)),
            floor(local ? endpoint_tns : double(reference.setup_tns))};
}

bool gainOrder(const PathBufferCandidate& a, const PathBufferCandidate& b) {
    if (a.predicted_wns_gain != b.predicted_wns_gain) {
        return a.predicted_wns_gain > b.predicted_wns_gain;
    }
    if (a.predicted_tns_gain != b.predicted_tns_gain) {
        return a.predicted_tns_gain > b.predicted_tns_gain;
    }
    // Equal endpoint forecasts do not justify a larger edit. Prefer a smaller
    // tree before secondary proxy gains; STA still validates every proposal.
    const auto count_a = a.tree ? a.tree->nodes.size() : 1;
    const auto count_b = b.tree ? b.tree->nodes.size() : 1;
    if (count_a != count_b) {
        return count_a < count_b;
    }
    // The RC/slew proxy may be optimistic for long, weakly driven trees.
    // Preserve path-margin information before using that proxy to break a
    // global-objective tie; neither estimate is an electrical proof.
    if (a.quality.path_gain != b.quality.path_gain) {
        return a.quality.path_gain > b.quality.path_gain;
    }
    if (a.quality.electrical_excess != b.quality.electrical_excess) {
        return a.quality.electrical_excess < b.quality.electrical_excess;
    }
    const double area_a = detail::area(a.master) * count_a;
    const double area_b = detail::area(b.master) * count_b;
    if (area_a != area_b) {
        return area_a < area_b;
    }
    return std::make_tuple(a.net->getName(), a.master->getName(), a.location.x,
                           a.location.y, a.loads) <
           std::make_tuple(b.net->getName(), b.master->getName(), b.location.x,
                           b.location.y, b.loads);
}

PathPolicy::PathPolicy(const PathSearchOptions& policy, PathSearchState statistics)
    : options(policy), result(statistics) {
}
PathPolicy::~PathPolicy() = default;

bool PathPolicy::reuseRejections() const {
    return options.reuse_rejections && options.failure_cache && options.batch_size == 1;
}

CandidateCollection PathPolicy::collect(detail::DesignAnalysis& analysis,
                                        sta::dbSta* sta,
                                        const std::vector<detail::NetState>& states,
                                        const DesignMetrics& metrics) {
    return collectCandidates(analysis, sta, states, options, metrics, *this);
}

void PathPolicy::beginCollection() {
    electrical_reserve.clear();
}
bool PathPolicy::hasFailure(const PathBufferCandidate& candidate) const {
    return failures.find(candidate.feedback_key) != nullptr;
}
unsigned PathPolicy::generationIndex() const {
    return generation;
}

double PathPolicy::tnsEstimate(const PathBufferCandidate& c) const {
    return options.endpoint_coverage &&
                   options.gain_scope != ecc::PathGainScope::Endpoints
               ? c.covered_tns_gain
               : c.predicted_tns_gain;
}

path_detail::ExplorationFrontier::Decision
PathPolicy::explore(const PathBufferCandidate& c) {
    using Decision = path_detail::ExplorationFrontier::Decision;
    if (!options.adaptive_search || options.batch_size != 1) {
        return Decision::Explore;
    }
    const bool local = options.gain_scope == ecc::PathGainScope::Endpoints;
    const auto floor = gainThreshold(options, result.before, result.after,
                                     c.objective_wns, c.objective_tns);
    const double wns_floor = floor.wns;
    const double tns_floor = std::max(
        floor.tns,
        double(detail::tnsTolerance(local ? c.objective_tns : result.after.setup_tns,
                                    options.common.tolerances)));
    return frontiers[c.group_key].decide({c.predicted_wns_gain, c.predicted_tns_gain},
                                         {wns_floor, tns_floor});
}

void PathPolicy::committed() {
    ++generation;
    // Even a distant commit can change an alternative path or placement.
    // Reopen every frontier rather than treating a structural hint as an
    // exact cross-generation dependency certificate.
    frontiers.clear();
}

void PathPolicy::rank(PathBufferCandidate& candidate, bool count) {
    candidate.family_attempts =
        options.diversify ? (options.group_schedule
                                 ? group_coverage.attempts(candidate.load_group_key,
                                                           *candidate.feedback_context)
                                 : coverage.attempts(candidate.group_key, generation))
                          : 0;
    candidate.ranked_wns_gain = candidate.predicted_wns_gain;
    candidate.ranked_tns_gain = tnsEstimate(candidate);
    if (options.calibrate) {
        const auto found = models.find(candidate.group_key);
        if (found != models.end() &&
            found->second.context == *candidate.feedback_context) {
            candidate.ranked_wns_gain =
                found->second.wns.predict(candidate.predicted_wns_gain);
            candidate.ranked_tns_gain =
                found->second.tns.predict(tnsEstimate(candidate));
            if (count) {
                ++result.statistics.calibrated_candidates;
            }
        }
    }
    const auto [wns_floor, tns_floor] =
        gainThreshold(options, result.before, result.after, candidate.objective_wns,
                      candidate.objective_tns);
    candidate.rank_score =
        std::max(wns_floor > 0 ? candidate.ranked_wns_gain / wns_floor
                               : candidate.ranked_wns_gain,
                 tns_floor > 0 ? candidate.ranked_tns_gain / tns_floor
                               : candidate.ranked_tns_gain);
    candidate.deferred_retry =
        options.failure_cache &&
        failures.status(candidate.feedback_key, *candidate.feedback_context,
                        generation) == path_detail::Retry::Deferred &&
        !failures.opportunityChanged(
            candidate.feedback_key, candidate.predicted_wns_gain,
            candidate.predicted_tns_gain, wns_floor, tns_floor);
    const bool regression =
        candidate.ranked_wns_gain < -options.common.tolerances.wns_absolute ||
        candidate.ranked_tns_gain <
            -detail::tnsTolerance(result.after.setup_tns, options.common.tolerances);
    candidate.feedback_priority =
        (candidate.deferred_retry ? 2 : 0) + unsigned(regression);
}
bool PathPolicy::order(const PathBufferCandidate& a, const PathBufferCandidate& b) {
    if (a.feedback_priority != b.feedback_priority) {
        return a.feedback_priority < b.feedback_priority;
    }
    if (a.electrical_retry != b.electrical_retry) {
        return a.electrical_retry;
    }
    if (a.family_attempts != b.family_attempts) {
        return a.family_attempts < b.family_attempts;
    }
    if (a.rank_score != b.rank_score) {
        return a.rank_score > b.rank_score;
    }
    if (a.ranked_wns_gain != b.ranked_wns_gain) {
        return a.ranked_wns_gain > b.ranked_wns_gain;
    }
    if (a.ranked_tns_gain != b.ranked_tns_gain) {
        return a.ranked_tns_gain > b.ranked_tns_gain;
    }
    return gainOrder(a, b);
}
void PathPolicy::select(std::vector<PathBufferCandidate>& variants) {
    std::unordered_set<std::string> actions;
    const auto end = std::remove_if(variants.begin(), variants.end(), [&](auto& c) {
        if (!actions.insert(c.feedback_key).second) {
            ++result.statistics.duplicate_candidates;
            return true;
        }
        if (options.failure_cache) {
            const auto retry =
                failures.status(c.feedback_key, *c.feedback_context, generation);
            if (retry == path_detail::Retry::Duplicate) {
                ++result.statistics.failure_cache_hits;
                return true;
            }
            if (retry == path_detail::Retry::Invalidated) {
                ++result.statistics.invalidated_failures;
            }
        }
        rank(c, true);
        if (c.deferred_retry) {
            ++result.statistics.deferred_candidates;
        }
        return false;
    });
    variants.erase(end, variants.end());
    std::stable_sort(variants.begin(), variants.end(), order);
    if (options.diversify) {
        std::unordered_map<std::string, unsigned> wave;
        for (auto& c : variants) {
            c.variant_rank =
                wave[options.group_schedule ? c.load_group_key : c.group_key]++;
        }
        std::stable_sort(variants.begin(), variants.end(),
                         [](const auto& a, const auto& b) {
                             if (a.feedback_priority != b.feedback_priority) {
                                 return a.feedback_priority < b.feedback_priority;
                             }
                             if (a.variant_rank != b.variant_rank) {
                                 return a.variant_rank < b.variant_rank;
                             }
                             return order(a, b);
                         });
    }
    // Beam selection happens after feedback and family diversification.
    // Retain deferred, previously tested actions as a final validation
    // queue even if the active beam is full. They are never blacklisted.
    unsigned active = 0;
    std::unordered_set<std::string> strengths;
    std::vector<PathBufferCandidate> selected;
    for (auto& c : variants) {
        const auto strength = c.load_group_key + '/' +
                              std::to_string(c.master->getLib()->getId()) + '/' +
                              std::to_string(c.master->getId());
        const bool new_strength = strengths.insert(strength).second;
        if (c.deferred_retry || active++ < options.max_candidates) {
            selected.push_back(std::move(c));
        } else if (new_strength) {
            electrical_reserve.push_back(std::move(c));
        }
    }
    variants = std::move(selected);
    std::stable_sort(variants.begin(), variants.end(), order);
}
void PathPolicy::recoverElectrical(const PathBufferCandidate& failed,
                                   const PathObservation& trial,
                                   std::vector<PathBufferCandidate>& pending,
                                   size_t next) {
    if (trial.rejection_reasons.empty() || !trial.metrics_complete ||
        !path_detail::needsFullVerification(trial.rejection_reasons)) {
        return;
    }
    PathBufferCandidate* best = nullptr;
    size_t reserve_index = electrical_reserve.size();
    const auto consider = [&](PathBufferCandidate& c) {
        if (c.load_group_key != failed.load_group_key || c.deferred_retry ||
            failures.status(c.feedback_key, *c.feedback_context, generation) ==
                path_detail::Retry::Duplicate ||
            !path_detail::betterElectricalAlternative(c.quality, failed.quality)) {
            return false;
        }
        if (!best ||
            path_detail::betterElectricalAlternative(c.quality, best->quality) ||
            (!path_detail::betterElectricalAlternative(best->quality, c.quality) &&
             order(c, *best))) {
            best = &c;
            return true;
        }
        return false;
    };
    for (size_t i = next; i < pending.size(); ++i) {
        consider(pending[i]);
    }
    for (size_t i = 0; i < electrical_reserve.size(); ++i) {
        if (consider(electrical_reserve[i])) {
            reserve_index = i;
        }
    }
    if (!best) {
        return;
    }
    best->electrical_retry = true;
    if (reserve_index != electrical_reserve.size()) {
        pending.push_back(std::move(electrical_reserve[reserve_index]));
        electrical_reserve.erase(electrical_reserve.begin() + reserve_index);
    }
}
void PathPolicy::covered(const PathBufferCandidate& plan) {
    coverage.observe(plan.group_key, generation);
    group_coverage.observe(plan.load_group_key, *plan.feedback_context);
}
void PathPolicy::observe(const PathBufferCandidate& plan,
                         const PathObservation& trial) {
    covered(plan);
    const bool local = options.gain_scope == ecc::PathGainScope::Endpoints;
    const double wns = local ? trial.objective_wns_gain : trial.wns_gain;
    const double tns = local ? trial.objective_tns_gain : trial.tns_gain;
    if (options.adaptive_search) {
        frontiers[plan.group_key].observe(
            {plan.predicted_wns_gain, plan.predicted_tns_gain}, {wns, tns},
            !trial.rejection_reasons.empty(), trial.frontier_audit);
    }
    if (options.calibrate) {
        auto& model = models[plan.group_key];
        if (model.context != *plan.feedback_context) {
            model = {*plan.feedback_context, {}, {}};
        }
        model.wns.observe(plan.predicted_wns_gain, wns);
        model.tns.observe(tnsEstimate(plan), tns);
        ++result.statistics.feedback_updates;
    }
    if (!trial.rejection_reasons.empty() && options.failure_cache) {
        failures.record(plan.feedback_key,
                        {*plan.feedback_context, trial.rejection_reasons, generation,
                         wns, tns, plan.predicted_wns_gain, plan.predicted_tns_gain});
    }
}

void PathPolicy::screen(std::vector<PathBufferCandidate>& candidates) {
    if (options.min_relative_improvement > 0) {
        const auto end = std::remove_if(
            candidates.begin(), candidates.end(), [&](const auto& candidate) {
                const bool local = options.gain_scope == ecc::PathGainScope::Endpoints;
                const auto [candidate_wns_floor, candidate_tns_floor] =
                    gainThreshold(options, result.before, result.after,
                                  candidate.objective_wns, candidate.objective_tns);
                // The largest possible improvement is reaching
                // zero negative slack. Sampled forecasts are
                // NOT upper bounds and must not prune actions.
                const double wns_bound = std::abs(
                    local ? candidate.objective_wns : double(result.after.setup_wns));
                const double tns_bound = std::abs(
                    local ? candidate.objective_tns : double(result.after.setup_tns));
                return !(wns_bound > candidate_wns_floor ||
                         tns_bound > candidate_tns_floor);
            });
        result.statistics.screened_candidates += std::distance(end, candidates.end());
        candidates.erase(end, candidates.end());
    }
}

PathSchedule::PathSchedule(std::vector<PathBufferCandidate> candidates,
                           const PathSearchOptions& policy, PathSearchState statistics,
                           PathPolicy& learning)
    : plans(std::move(candidates)), options(policy), result(statistics),
      policy(learning), width(std::min(size_t(options.batch_size), plans.size())) {
}

std::string PathSchedule::topologyKey(const PathBufferCandidate& plan) {
    auto copy = plan;
    copy.location = {0, 0};
    return actionKey(copy);
}

void PathSchedule::fanoutRejected(const PathBufferCandidate& plan) {
    // Fanout feasibility does not change with buffer position on this snapshot.
    infeasible_topologies.insert(topologyKey(plan));
}

void PathSchedule::reopen() {
    const auto end =
        std::remove_if(suspended.begin(), suspended.end(), [&](auto& plan) {
            if (policy.explore(plan) == ExplorationFrontier::Decision::Suspend) {
                return false;
            }
            plans.push_back(std::move(plan));
            ++result.statistics.frontier_reopened;
            return true;
        });
    suspended.erase(end, suspended.end());
}

void PathSchedule::rerank() {
    if (!hasNext()) {
        return;
    }
    for (size_t i = cursor; i < plans.size(); ++i) {
        policy.rank(plans[i]);
    }
    std::stable_sort(plans.begin() + cursor, plans.end(), PathPolicy::order);
}

void PathSchedule::backoff() {
    if (width > 1) {
        // Rejecting an aggregate says nothing about each member. Retry singles
        // directly, rather than spending STA trials on 8 -> 4 -> 2 subsets.
        width = 1;
        single_candidate_mode = true;
    } else {
        ++cursor;
        if (hasNext()) {
            width = single_candidate_mode
                        ? 1
                        : std::min(size_t(options.batch_size), plans.size() - cursor);
        }
    }
}

void PathSchedule::skipBatch() {
    cursor += width;
    if (hasNext()) {
        width = std::min(size_t(options.batch_size), plans.size() - cursor);
    }
}

bool PathSchedule::prepareNext() {
    frontier_audit = false;
    if (width == 1) {
        const auto decision = policy.explore(plans[cursor]);
        if (decision == ExplorationFrontier::Decision::Suspend) {
            ++result.statistics.frontier_suspended;
            suspended.push_back(std::move(plans[cursor]));
            backoff();
            return false;
        }
        frontier_audit = decision == ExplorationFrontier::Decision::Audit;
    }
    return true;
}

PathBatch PathSchedule::selectBatch(double area_limit) {
    PathBatch batch;
    std::unordered_set<odb::dbNet*> touched_nets;
    std::unordered_set<odb::dbInst*> touched_cone;

    for (size_t i = cursor; i < std::min(cursor + width, plans.size()); ++i) {
        const auto& plan = plans[i];
        const unsigned count = plan.tree ? plan.tree->nodes.size() : 1;
        if (options.fanout_repair &&
            infeasible_topologies.contains(topologyKey(plan))) {
            ++result.statistics.fanout_precheck_rejections;
            continue;
        }
        const bool conflict =
            std::any_of(plan.cone.begin(), plan.cone.end(),
                        [&](auto* inst) { return touched_cone.contains(inst); });
        if (conflict || touched_nets.contains(plan.net) ||
            result.after.area + batch.area + count * detail::area(plan.master) >
                area_limit ||
            (batch.buffers + count > remainingBufferBudget(options.max_inserted_buffers,
                                                           result.inserted_buffers))) {
            continue;
        }
        batch.indices.push_back(i);
        touched_nets.insert(plan.net);
        touched_cone.insert(plan.cone.begin(), plan.cone.end());
        batch.area += count * detail::area(plan.master);
        batch.buffers += count;
    }
    return batch;
}

void PathSchedule::observe(const PathBatch& batch, const PathObservation& trial) {
    // Aggregate gains cannot be attributed to individual candidates.
    if (batch.indices.size() == 1) {
        policy.observe(plans[batch.indices.front()], trial);
    }
}

void PathSchedule::recoverElectrical(const PathBatch& batch,
                                     const PathObservation& trial) {
    if (batch.indices.size() == 1) {
        policy.recoverElectrical(plans[batch.indices.front()], trial, plans,
                                 cursor + 1);
    }
}

void PathSchedule::rejected() {
    backoff();
    // An underestimated probe may reopen earlier suspended siblings, even
    // when the active queue has just become empty.
    reopen();
    rerank();
}

} // namespace ecc::path_detail

namespace ecc::path_detail {
void EndpointSlackProfile::add(double slack) {
    if (std::isfinite(slack)) {
        slacks_.push_back(slack);
    }
}

void EndpointSlackProfile::finish() {
    std::sort(slacks_.begin(), slacks_.end());
    prefix_.assign(slacks_.size() + 1, 0);
    for (size_t i = 0; i < slacks_.size(); ++i) {
        prefix_[i + 1] = prefix_[i] + slacks_[i];
    }
    before_ = negativeSum(0);
}

double EndpointSlackProfile::gain(double delay_gain) const {
    return negativeSum(delay_gain) - before_;
}

size_t EndpointSlackProfile::size() const {
    return slacks_.size();
}

double EndpointSlackProfile::negativeSum(double gain) const {
    const auto count =
        std::lower_bound(slacks_.begin(), slacks_.end(), -gain) - slacks_.begin();
    return prefix_.at(count) + count * gain;
}

unsigned ContextCoverage::attempts(const std::string& family,
                                   const std::string& context) const {
    const auto it = counts_.find(family);
    return it != counts_.end() && it->second.context == context ? it->second.attempts
                                                                : 0;
}

void ContextCoverage::observe(const std::string& family, const std::string& context) {
    auto& count = counts_[family];
    if (count.context != context) {
        count = {context, 0};
    }
    ++count.attempts;
}

void GainCalibration::observe(double predicted, double measured) {
    if (!std::isfinite(predicted) || !std::isfinite(measured)) {
        return;
    }
    xx += predicted * predicted;
    xy += predicted * measured;
    measured_sum += measured;
    ++samples;
}

double GainCalibration::predict(double raw) const {
    return xx > 0 ? raw * xy / xx : samples ? measured_sum / samples : raw;
}

ExplorationFrontier::Decision ExplorationFrontier::decide(Gains predicted,
                                                          Gains floor) const {
    if (!finite(predicted) || !finite(floor)) {
        return Decision::Explore;
    }
    const bool dominated =
        std::any_of(rejected_.begin(), rejected_.end(), [&](Gains p) {
            return p.wns >= predicted.wns && p.tns >= predicted.tns;
        });
    if (!dominated || predicted.wns + error_.wns > floor.wns ||
        predicted.tns + error_.tns > floor.tns) {
        return Decision::Explore;
    }
    return audited_ ? Decision::Suspend : Decision::Audit;
}

void ExplorationFrontier::observe(Gains predicted, Gains actual, bool rejected,
                                  bool audit) {
    if (!finite(predicted) || !finite(actual)) {
        audited_ = false;
        return;
    }
    const Gains next{std::max(error_.wns, actual.wns - predicted.wns),
                     std::max(error_.tns, actual.tns - predicted.tns)};
    const bool expanded = next.wns > error_.wns || next.tns > error_.tns;
    error_ = next;
    if (expanded || !rejected) {
        audited_ = false;
    } else if (audit) {
        audited_ = true;
    }
    if (rejected) {
        // Keep the Pareto frontier rather than every near-identical probe.
        if (std::any_of(rejected_.begin(), rejected_.end(), [&](Gains p) {
                return p.wns >= predicted.wns && p.tns >= predicted.tns;
            })) {
            return;
        }
        rejected_.erase(std::remove_if(rejected_.begin(), rejected_.end(),
                                       [&](Gains p) {
                                           return predicted.wns >= p.wns &&
                                                  predicted.tns >= p.tns;
                                       }),
                        rejected_.end());
        rejected_.push_back(predicted);
    }
}

bool ExplorationFrontier::finite(Gains p) {
    return std::isfinite(p.wns) && std::isfinite(p.tns);
}

unsigned FamilyCoverage::attempts(const std::string& family,
                                  unsigned generation) const {
    const auto it = counts_.find(family);
    return it != counts_.end() && it->second.generation == generation
               ? it->second.attempts
               : 0;
}

void FamilyCoverage::observe(const std::string& family, unsigned generation) {
    auto& count = counts_[family];
    if (count.generation != generation) {
        count = {generation, 0};
    }
    ++count.attempts;
}

Retry FailureHistory::status(const std::string& action, const std::string& context,
                             unsigned generation) const {
    const auto found = records_.find(action);
    if (found == records_.end()) {
        return Retry::Fresh;
    }
    if (found->second.context != context) {
        return Retry::Invalidated;
    }
    if (found->second.generation == generation) {
        return Retry::Duplicate;
    }
    // Matching sampled boundaries do not prove that global STA or an
    // arbitrary placement callback is unchanged. Defer across commits,
    // but require a fresh STA trial before declaring the search exhausted.
    return Retry::Deferred;
}

void FailureHistory::record(std::string action, FailureEvidence evidence) {
    records_[std::move(action)] = std::move(evidence);
}

const FailureEvidence* FailureHistory::find(const std::string& action) const {
    const auto found = records_.find(action);
    return found == records_.end() ? nullptr : &found->second;
}

bool FailureHistory::opportunityChanged(const std::string& action, double wns,
                                        double tns, double wns_floor,
                                        double tns_floor) const {
    const auto* evidence = find(action);
    return evidence && (wns > evidence->predicted_wns_gain + wns_floor ||
                        tns > evidence->predicted_tns_gain + tns_floor);
}

void DependencyFailures::record(const std::string& action, Cone cone, float before,
                                float after) {
    record(action, std::make_shared<const Cone>(std::move(cone)), before, after);
}

void DependencyFailures::record(const std::string& action,
                                std::shared_ptr<const Cone> cone, float before,
                                float after) {
    if (std::isfinite(before) && std::isfinite(after) && after < before) {
        failures_[action] = {std::move(cone), before, after};
    }
}

bool DependencyFailures::rejects(const std::string& action, float current,
                                 double tolerance) const {
    const auto it = failures_.find(action);
    if (it == failures_.end() || !std::isfinite(current)) {
        return false;
    }
    const auto& f = it->second;
    const double loss = double(f.before) - f.after;
    // Whole-design TNS is stored as a float. Allow outward rounding of
    // both recorded totals and both replayed totals, not a tuned margin.
    const auto ulp = [](float value) {
        return double(std::nextafter(std::abs(value),
                                     std::numeric_limits<float>::infinity())) -
               std::abs(value);
    };
    const double uncertainty =
        ulp(f.before) + ulp(f.after) + ulp(current) + ulp(float(current - loss));
    return loss > tolerance + uncertainty;
}

void DependencyFailures::committed(const Cone& changed) {
    std::unordered_map<const Cone*, bool> invalidated;
    for (auto it = failures_.begin(); it != failures_.end();) {
        const auto& cone = *it->second.cone;
        auto [memo, fresh] = invalidated.emplace(&cone, false);
        if (fresh) {
            const auto& smaller = cone.size() < changed.size() ? cone : changed;
            const auto& larger = cone.size() < changed.size() ? changed : cone;
            memo->second =
                std::any_of(smaller.begin(), smaller.end(),
                            [&](unsigned id) { return larger.contains(id); });
        }
        if (memo->second) {
            it = failures_.erase(it);
        } else {
            ++it;
        }
    }
}

void EndpointFailures::record(const std::string& action,
                              std::shared_ptr<const Cone> fanin, float slack) {
    if (std::isfinite(slack)) {
        failures_[action] = {std::move(fanin), slack};
    }
}

bool EndpointFailures::rejects(const std::string& action, float current_wns,
                               double tolerance) const {
    const auto it = failures_.find(action);
    if (it == failures_.end() || !std::isfinite(current_wns)) {
        return false;
    }
    const auto slack = it->second.slack;
    const auto ulp = [](float v) {
        return double(std::nextafter(std::abs(v),
                                     std::numeric_limits<float>::infinity())) -
               std::abs(v);
    };
    return double(current_wns) - slack > tolerance + ulp(current_wns) + ulp(slack);
}

void EndpointFailures::committed(unsigned edited_net) {
    for (auto it = failures_.begin(); it != failures_.end();) {
        if (it->second.fanin->contains(edited_net)) {
            it = failures_.erase(it);
        } else {
            ++it;
        }
    }
}

bool needsFullVerification(const std::vector<std::string>& reasons) {
    return std::all_of(reasons.begin(), reasons.end(), [](const auto& reason) {
        return reason == "slew" || reason == "cap" || reason == "fanout" ||
               reason == "drv_severity";
    });
}

std::string actionFingerprint(const std::string& net,
                              const std::vector<ActionNode>& nodes, bool physical) {
    std::vector<std::string> encoded;
    for (const auto& node : nodes) {
        auto terminals = node.terminals;
        std::sort(terminals.begin(), terminals.end());
        std::vector<std::string> children;
        children.reserve(node.children.size());
        for (auto child : node.children) {
            children.push_back(encoded.at(child));
        }
        std::sort(children.begin(), children.end());
        std::ostringstream out;
        out << '(';
        if (physical) {
            out << node.master.size() << ':' << node.master << '@' << node.x << ','
                << node.y;
        }
        out << '[';
        for (const auto& term : terminals) {
            out << term.size() << ':' << term;
        }
        out << ']';
        for (const auto& child : children) {
            out << child.size() << ':' << child;
        }
        out << ')';
        encoded.push_back(out.str());
    }
    return std::to_string(net.size()) + ':' + net +
           (encoded.empty() ? "()" : encoded.back());
}

bool betterElectricalAlternative(const CandidateQuality& a, const CandidateQuality& b) {
    return a.electrical_excess < b.electrical_excess ||
           (a.electrical_excess == b.electrical_excess && a.path_gain > b.path_gain);
}

ObjectiveForecast forecastObjective(const std::vector<PathForecast>& paths,
                                    const std::map<size_t, double>& endpoint_slacks,
                                    double global_wns, bool local) {
    struct Endpoint {
        double before = 0, after = 0;
        bool initialized = false, affected = false;
    };
    std::map<size_t, Endpoint> endpoints;
    for (const auto& path : paths) {
        auto& end = endpoints[path.endpoint];
        if (!end.initialized) {
            end.before = path.before;
            end.after = path.after;
            end.initialized = true;
        } else {
            end.before = std::min(end.before, path.before);
            end.after = std::min(end.after, path.after);
        }
        end.affected |= path.affected;
    }
    ObjectiveForecast result;
    double affected_before = INFINITY, affected_after = INFINITY;
    for (const auto& path : paths) {
        if (path.affected) {
            affected_before = std::min(affected_before, path.before);
            affected_after = std::min(affected_after, path.after);
        }
    }
    if (std::isfinite(affected_before) && std::isfinite(affected_after)) {
        result.path_gain = affected_after - affected_before;
    }
    double before_wns = 0, after_wns = 0, sampled_wns = 0;
    for (auto& [id, end] : endpoints) {
        const auto found = endpoint_slacks.find(id);
        if (found != endpoint_slacks.end() && found->second < end.before) {
            // Another check/path at this endpoint is already more critical
            // than our samples: preserve that competing slack.
            end.before = found->second;
            end.after = std::min(end.after, found->second);
        }
        sampled_wns = std::min(sampled_wns, end.before);
        if (local && !end.affected) {
            continue;
        }
        before_wns = std::min(before_wns, end.before);
        after_wns = std::min(after_wns, end.after);
        if (end.affected) {
            ++result.affected_endpoints;
            result.tns_gain += std::min(0.0, end.after) - std::min(0.0, end.before);
        }
    }
    if (!local) {
        before_wns = std::min(0.0, global_wns);
        if (global_wns < sampled_wns) {
            after_wns = std::min(after_wns, global_wns);
        }
    }
    result.wns_gain = after_wns - before_wns;
    return result;
}
} // namespace ecc::path_detail
