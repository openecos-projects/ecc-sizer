// SPDX-License-Identifier: Apache-2.0
#include "algorithms/pbbi.h"
#include "design_analysis.h"
#include "BufferedNet.hh"
#include "rsz/Resizer.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "sta/Graph.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace ecc::pbbi {
constexpr double ln9 = 2.1972245773362196;

static bool dominates(const Choice& a, const Choice& b) {
    return a.cap <= b.cap && a.required >= b.required && a.cost <= b.cost &&
           a.fanout <= b.fanout && a.slew_budget >= b.slew_budget;
}

static void prune(std::vector<Choice>& values, std::optional<unsigned> limit,
                  unsigned& truncated) {
    std::stable_sort(values.begin(), values.end(), [](const auto& a, const auto& b) {
        if (a.cost != b.cost) {
            return a.cost < b.cost;
        }
        if (a.cap != b.cap) {
            return a.cap < b.cap;
        }
        return a.required > b.required;
    });
    std::vector<Choice> frontier;
    for (const auto& value : values) {
        if (std::any_of(frontier.begin(), frontier.end(),
                        [&](const auto& other) { return dominates(other, value); })) {
            continue;
        }
        std::erase_if(frontier,
                      [&](const auto& other) { return dominates(value, other); });
        frontier.push_back(value);
    }
    if (limit && frontier.size() > *limit) {
        // Keep the best timing, smallest load, zero-cost alternative and a
        // deterministic spread over the count/delay tradeoff. This is a beam
        // approximation, not an exact Pareto solver when the cap is reached.
        ++truncated;
        std::vector<size_t> keep{0};
        const auto best = std::max_element(
            frontier.begin(), frontier.end(),
            [](const auto& a, const auto& b) { return a.required < b.required; });
        keep.push_back(size_t(best - frontier.begin()));
        const auto smallest = std::min_element(
            frontier.begin(), frontier.end(),
            [](const auto& a, const auto& b) { return a.cap < b.cap; });
        keep.push_back(size_t(smallest - frontier.begin()));
        for (size_t i = 0; i < *limit - 2; ++i) {
            keep.push_back(i * (frontier.size() - 1) / (*limit - 3));
        }
        std::sort(keep.begin(), keep.end());
        keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
        std::vector<Choice> reduced;
        reduced.reserve(keep.size());
        for (auto i : keep) {
            reduced.push_back(frontier[i]);
        }
        frontier = std::move(reduced);
    }
    values = std::move(frontier);
}

Solution solve(const Tree& tree, std::span<const Buffer> buffers, double target,
               const PbbiOptions& options) {
    options.validate();
    Solution result;
    for (size_t index = 0; index < tree.nodes.size(); ++index) {
        const auto& node = tree.nodes[index];
        const auto check_child = [&](int child) {
            if (child < 0 || size_t(child) >= index) {
                throw std::invalid_argument("PBBI requires a postorder tree");
            }
        };
        if (node.kind != NodeKind::Sink) {
            check_child(node.left);
        }
        if (node.kind == NodeKind::Junction) {
            check_child(node.right);
        }
        std::vector<Choice> values;
        if (node.kind == NodeKind::Sink) {
            values.push_back({.cap = node.cap,
                              .required = node.required,
                              .slew_budget = node.slew_limit,
                              .fanout = 1});
        } else {
            const auto& left = result.frontiers[node.left];
            for (size_t i = 0; i < left.size(); ++i) {
                const auto& child = left[i];
                if (node.kind == NodeKind::Junction) {
                    const auto& right = result.frontiers[node.right];
                    for (size_t j = 0; j < right.size(); ++j) {
                        const auto& r = right[j];
                        values.push_back(
                            {.cap = child.cap + r.cap,
                             .required = std::min(child.required, r.required),
                             .slew_budget = std::min(child.slew_budget, r.slew_budget),
                             .cost = child.cost + r.cost,
                             .fanout = child.fanout + r.fanout,
                             .left = int(i),
                             .right = int(j)});
                    }
                } else if (node.kind == NodeKind::Gate) {
                    // Mandatory existing gate: never emit it as a new buffer,
                    // and never merge its output net into its input net.
                    if (child.cap > node.max_cap ||
                        ln9 * node.resistance * child.cap > child.slew_budget) {
                        continue;
                    }
                    values.push_back({.cap = node.cap,
                                      .required = child.required - node.intrinsic -
                                                  node.resistance * child.cap,
                                      .slew_budget = node.slew_limit,
                                      .cost = child.cost,
                                      .fanout = 1,
                                      .left = int(i)});
                } else {
                    const double delay = node.resistance * (child.cap + node.cap / 2);
                    const double propagation = ln9 * delay;
                    if (propagation > child.slew_budget) {
                        continue;
                    }
                    const double budget =
                        std::sqrt(std::max(0.0, child.slew_budget * child.slew_budget -
                                                    propagation * propagation));
                    Choice wire{.cap = child.cap + node.cap,
                                .required = child.required - delay,
                                .slew_budget = budget,
                                .cost = child.cost,
                                .fanout = child.fanout,
                                .left = int(i)};
                    values.push_back(wire);
                    if (!node.can_buffer) {
                        continue;
                    }
                    for (size_t b = 0; b < buffers.size(); ++b) {
                        const auto& buffer = buffers[b];
                        if (wire.cap > buffer.max_cap ||
                            wire.fanout > options.common.max_fanout ||
                            ln9 * buffer.resistance * wire.cap > budget) {
                            continue;
                        }
                        auto buffered = wire;
                        buffered.cap = buffer.cap;
                        buffered.required -=
                            buffer.intrinsic + buffer.resistance * wire.cap;
                        buffered.cost++;
                        buffered.fanout = 1;
                        // Input slew is bounded by the library/SDC snapshot at
                        // this site; output slew was checked above.
                        buffered.slew_budget = node.slew_limit;
                        buffered.buffer = int(b);
                        values.push_back(buffered);
                    }
                }
                if (options.max_frontier &&
                    values.size() > size_t(*options.max_frontier) * 8) {
                    prune(values, options.max_frontier, result.truncated);
                }
            }
        }
        if (options.max_buffers) {
            std::erase_if(values, [&](const auto& value) {
                return value.cost > *options.max_buffers;
            });
        }
        prune(values, options.max_frontier, result.truncated);
        result.frontiers.push_back(std::move(values));
    }
    if (result.frontiers.empty()) {
        return result;
    }
    bool feasible = false;
    unsigned cost = std::numeric_limits<unsigned>::max();
    const auto& root = result.frontiers.back();
    for (size_t i = 0; i < root.size(); ++i) {
        const auto& value = root[i];
        if (std::hypot(tree.driver_slew, ln9 * tree.driver_resistance * value.cap) >
            value.slew_budget) {
            continue;
        }
        const double score = value.required - tree.driver_resistance * value.cap;
        const bool meets = score >= target;
        if ((meets && !feasible) ||
            (meets == feasible && ((meets && value.cost < cost) ||
                                   ((!meets || value.cost == cost) &&
                                    (score > result.score ||
                                     (score == result.score && value.cost < cost)))))) {
            result.selected = int(i);
            result.score = score;
            cost = value.cost;
            feasible = meets;
        }
    }
    return result;
}

std::vector<BufferTreePlan> decode(const Tree& tree, std::span<const Buffer> buffers,
                                   const Solution& solution) {
    std::vector<BufferTreePlan> plans;
    plans.reserve(tree.nets.size());
    for (auto* net : tree.nets) {
        plans.push_back({net, {}});
    }
    if (solution.selected < 0) {
        return {};
    }
    const auto visit = [&](auto&& self, int index, int selected) -> BufferTreeNode {
        const auto& node = tree.nodes.at(index);
        const auto& choice = solution.frontiers.at(index).at(selected);
        if (node.kind == NodeKind::Sink) {
            return {.loads = {node.terminal}};
        }
        auto left = self(self, node.left, choice.left);
        if (node.kind == NodeKind::Gate) {
            return {.loads = {node.terminal}};
        }
        if (node.kind == NodeKind::Junction) {
            auto right = self(self, node.right, choice.right);
            left.loads.insert(left.loads.end(), right.loads.begin(), right.loads.end());
            left.children.insert(left.children.end(), right.children.begin(),
                                 right.children.end());
        }
        if (choice.buffer >= 0) {
            left.master = buffers[choice.buffer].master;
            left.location = node.location;
            auto& plan = plans.at(node.net);
            plan.nodes.push_back(std::move(left));
            return {.children = {plan.nodes.size() - 1}};
        }
        return left;
    };
    visit(visit, int(tree.nodes.size() - 1), solution.selected);
    std::erase_if(plans, [](const auto& plan) { return plan.nodes.empty(); });
    return plans;
}

double adjustedRequired(double required, double endpoint_required,
                        double endpoint_arrival, double root_arrival) {
    const double duration = endpoint_arrival - root_arrival;
    // Equation (7) distributes positive useful slack. For an infeasible path
    // there is no slack resource to spend; do not relax its side constraints.
    if (!std::isfinite(required) || duration <= 0 ||
        endpoint_required <= endpoint_arrival) {
        return required;
    }
    const double fraction =
        std::clamp((endpoint_required - required) / duration, 0.0, 1.0);
    return required - fraction * (endpoint_required - endpoint_arrival);
}

struct Sample {
    std::vector<const sta::Pin*> pins;
    std::vector<double> arrivals;
    double required = 0, slack = 0;
};

static std::vector<Sample> sample(sta::dbSta* sta, unsigned count) {
    sta::StringSeq groups;
    const auto ends =
        sta->findPathEnds(nullptr, nullptr, nullptr, false, sta->scenes(),
                          sta::MinMaxAll::max(), count, 1, true, false, -detail::kInf,
                          0, true, groups, true, false, false, false, false, false);
    std::vector<Sample> paths;
    // Copy everything before any per-pin STA query (which can rebuild groups).
    for (auto* end : ends) {
        Sample path;
        path.slack = double(end->slack(sta));
        sta::PathExpanded expanded(end->path(), sta);
        for (size_t i = expanded.startIndex(); i < expanded.size(); ++i) {
            path.pins.push_back(expanded.path(i)->pin(sta));
            path.arrivals.push_back(double(expanded.path(i)->arrival()));
        }
        if (!path.pins.empty()) {
            path.required = path.arrivals.back() + path.slack;
            paths.push_back(std::move(path));
        }
    }
    return paths;
}

struct Segment {
    const detail::NetState* state;
    const sta::Pin* sink;
    double arrival, sink_arrival;
};
struct Path {
    std::vector<Segment> segments;
    double required = 0;
};

class Snapshot {
  public:
    Snapshot(odb::dbBlock* block, sta::dbSta* sta, rsz::Resizer* resizer,
             const PbbiOptions& options, const DesignConstraints& constraints)
        : sta_(sta), resizer_(resizer),
          analysis_(block, sta, options.common, constraints) {
        resizer_->initBlock();
        for (auto* blockage : block->getBlockages()) {
            if (!blockage->isSoft()) {
                blockages_.push_back(blockage->getBBox()->getBox());
            }
        }
        for (auto* inst : block->getInsts()) {
            if (inst->getMaster()->isBlock()) {
                blockages_.push_back(inst->getBBox()->getBox());
            }
        }
        for (const auto& model : analysis_.bufferChoices()) {
            Buffer buffer{model.master,
                          resizer_->portCapacitance(model.input, sta_->cmdScene()),
                          model.output->driveResistance(),
                          double(model.output->intrinsicDelay(sta_)), model.max_cap};
            if (std::isfinite(buffer.cap) && buffer.cap >= 0 &&
                std::isfinite(buffer.resistance) && buffer.resistance >= 0 &&
                std::isfinite(buffer.intrinsic)) {
                buffers.push_back(buffer);
            }
        }
    }
    std::vector<Buffer> buffers;

    double required(const sta::Pin* pin) {
        auto [it, added] = required_.try_emplace(pin);
        if (added) {
            auto* vertex = sta_->graph()->pinLoadVertex(pin);
            it->second = infinity;
            if (vertex) {
                const auto rat = sta_->required(vertex, sta::RiseFallBoth::riseFall(),
                                                sta_->scenes(), sta::MinMax::max());
                if (!sta::delayInf(rat, sta_)) {
                    it->second = double(rat);
                }
            }
        }
        return it->second;
    }

    Tree build(const Path& path, bool endpoint_only, double estimated_arrival) {
        Tree tree;
        for (const auto& segment : path.segments) {
            tree.nets.push_back(segment.state->net);
        }
        std::function<int(size_t)> append_net;
        std::function<int(const rsz::BufferedNetPtr&, size_t)> append_node;
        append_node = [&](const rsz::BufferedNetPtr& source, size_t net) {
            Node node;
            node.net = unsigned(net);
            node.location = {source->location().x(), source->location().y()};
            const auto& segment = path.segments[net];
            node.slew_limit = segment.state->slew_limit;
            switch (source->type()) {
            case rsz::BufferedNetType::load: {
                node.cap = source->cap();
                node.terminal = sta_->getDbNetwork()->staToDb(source->loadPin());
                if (source->loadPin() == segment.sink) {
                    if (net + 1 < path.segments.size()) {
                        node.kind = NodeKind::Gate;
                        node.left = append_net(net + 1);
                        const auto& next = path.segments[net + 1];
                        auto* port = sta_->getDbNetwork()->libertyPort(
                            sta_->getDbNetwork()->dbToSta(next.state->driver));
                        node.resistance = port ? port->driveResistance() : 0;
                        // Calibrate the linear arc to this exact path's delay,
                        // including its transition and input slew at the snapshot.
                        node.intrinsic = next.arrival - segment.sink_arrival -
                                         node.resistance * next.state->capacitance;
                    } else {
                        node.required = path.required;
                    }
                } else {
                    node.required =
                        endpoint_only
                            ? infinity
                            : adjustedRequired(required(source->loadPin()),
                                               path.required, estimated_arrival,
                                               path.segments.front().arrival);
                }
                break;
            }
            case rsz::BufferedNetType::wire: {
                node.kind = NodeKind::Wire;
                node.left = append_node(source->ref(), net);
                double r = 0, c = 0;
                source->wireRC(sta_->cmdScene(), resizer_,
                               resizer_->getEstimateParasitics(), r, c);
                const double length = resizer_->dbuToMeters(source->length());
                node.cap = c * length;
                node.resistance = r * length;
                node.can_buffer = std::none_of(
                    blockages_.begin(), blockages_.end(), [&](const auto& box) {
                        return box.intersects(source->location());
                    });
                break;
            }
            case rsz::BufferedNetType::junction:
                node.kind = NodeKind::Junction;
                node.left = append_node(source->ref(), net);
                node.right = append_node(source->ref2(), net);
                break;
            default:
                throw std::runtime_error("PBBI expects a placement Steiner tree");
            }
            tree.nodes.push_back(node);
            return int(tree.nodes.size() - 1);
        };
        append_net = [&](size_t net) {
            const auto* state = path.segments[net].state;
            auto [it, added] = trees_.try_emplace(state->net);
            if (added) {
                it->second = resizer_->makeBufferedNetSteiner(
                    sta_->getDbNetwork()->dbToSta(state->driver), sta_->cmdScene());
            }
            if (!it->second) {
                throw std::runtime_error("PBBI missing placement tree");
            }
            return append_node(it->second, net);
        };
        append_net(0);
        auto* port = sta_->getDbNetwork()->libertyPort(
            sta_->getDbNetwork()->dbToSta(path.segments.front().state->driver));
        tree.driver_resistance = port ? port->driveResistance() : 0;
        // Recompute the load-dependent launch slew for each candidate. The
        // measured worst slew on the original net includes its old load/wire
        // delay and cannot be reused after isolating that load with a buffer.
        tree.driver_slew = 0;
        return tree;
    }

  private:
    sta::dbSta* sta_;
    rsz::Resizer* resizer_;
    detail::DesignAnalysis analysis_;
    std::unordered_map<const sta::Pin*, double> required_;
    std::unordered_map<odb::dbNet*, rsz::BufferedNetPtr> trees_;
    std::vector<odb::Rect> blockages_;
};

Proposals generate(odb::dbBlock* block, sta::dbSta* sta, rsz::Resizer* resizer,
                   std::span<const detail::NetState> states, const PbbiOptions& options,
                   const DesignConstraints& constraints) {
    options.validate();
    if (!block || !sta || !resizer || sta->getDbNetwork()->block() != block ||
        resizer->sta() != sta || sta->scenes().size() != 1) {
        throw std::invalid_argument(
            "PBBI requires one live OpenDB/STA scene and resizer");
    }
    Proposals result;
    const auto samples = sample(sta, options.max_paths);
    result.sampled = unsigned(samples.size());
    Snapshot snapshot(block, sta, resizer, options, constraints);
    if (snapshot.buffers.empty()) {
        return result;
    }
    std::unordered_map<const sta::Pin*, const detail::NetState*> drivers;
    for (const auto& state : states) {
        if (state.driver) {
            drivers.emplace(sta->getDbNetwork()->dbToSta(state.driver), &state);
        }
    }
    std::vector<Path> paths;
    for (const auto& sampled : samples) {
        Path current;
        const auto finish = [&]() {
            if (!current.segments.empty()) {
                current.required = snapshot.required(current.segments.back().sink);
                if (current.segments.back().sink == sampled.pins.back()) {
                    current.required = sampled.required;
                }
                paths.push_back(std::move(current));
                current = {};
            }
        };
        for (size_t i = 0; i + 1 < sampled.pins.size(); ++i) {
            const auto found = drivers.find(sampled.pins[i]);
            if (found == drivers.end()) {
                continue;
            }
            const auto* state = found->second;
            auto* object = sta->getDbNetwork()->staToDb(sampled.pins[i + 1]);
            const bool load =
                std::any_of(state->loads.begin(), state->loads.end(),
                            [&](const auto& l) { return l.term == object; });
            if (!load) {
                finish();
                continue;
            }
            if (!current.segments.empty()) {
                auto* previous =
                    sta->getDbNetwork()->staToDb(current.segments.back().sink);
                if (!previous || previous->getObjectType() != odb::dbITermObj ||
                    static_cast<odb::dbITerm*>(previous)->getInst() !=
                        state->driver->getInst()) {
                    finish();
                }
            }
            current.segments.push_back({state, sampled.pins[i + 1], sampled.arrivals[i],
                                        sampled.arrivals[i + 1]});
        }
        finish();
    }
    // Estimate post-buffering endpoint arrival before ordering paths. Side
    // constraints are added in the cost optimization, not in this optimistic
    // endpoint bound. This is sampled-path buffer awareness, not global BSTA.
    struct Ranked {
        Path path;
        double slack;
    };
    std::vector<Ranked> ranked;
    for (auto& path : paths) {
        auto tree = snapshot.build(path, true, path.segments.back().sink_arrival);
        auto relaxed = tree;
        relaxed.driver_slew = 0;
        for (auto& node : relaxed.nodes) {
            node.slew_limit = infinity;
        }
        const auto plain = solve(relaxed, {}, infinity, options);
        const auto buffered = solve(relaxed, snapshot.buffers, infinity, options);
        result.truncated += buffered.truncated;
        if (plain.selected < 0 || buffered.selected < 0) {
            continue;
        }
        const double gain = std::max(0.0, buffered.score - plain.score);
        const double slack = path.required - path.segments.back().sink_arrival + gain;
        ranked.push_back({std::move(path), slack});
    }
    std::stable_sort(ranked.begin(), ranked.end(),
                     [](const auto& a, const auto& b) { return a.slack < b.slack; });
    std::unordered_set<odb::dbNet*> claimed;
    for (const auto& rank : ranked) {
        Path distinct;
        const auto finish = [&]() {
            if (distinct.segments.empty()) {
                return;
            }
            ++result.split_paths;
            distinct.required =
                distinct.segments.back().sink == rank.path.segments.back().sink
                    ? rank.path.required
                    : snapshot.required(distinct.segments.back().sink);
            auto estimate_tree =
                snapshot.build(distinct, true, distinct.segments.back().sink_arrival);
            estimate_tree.driver_slew = 0;
            for (auto& node : estimate_tree.nodes) {
                node.slew_limit = infinity;
            }
            const auto estimate_plain = solve(estimate_tree, {}, infinity, options);
            const auto estimate_best =
                solve(estimate_tree, snapshot.buffers, infinity, options);
            result.truncated += estimate_best.truncated;
            const double estimate =
                distinct.segments.back().sink_arrival -
                std::max(0.0, estimate_best.score - estimate_plain.score);
            auto tree = snapshot.build(distinct, false, estimate);
            // Anchor the root model to observed AT, so linear-model offsets do
            // not manufacture improvement relative to the unchanged design.
            auto plain_tree = tree;
            plain_tree.driver_slew = 0;
            for (auto& node : plain_tree.nodes) {
                node.slew_limit = infinity;
            }
            const auto plain = solve(plain_tree, {}, infinity, options);
            const double target =
                distinct.segments.front().arrival -
                tree.driver_resistance * distinct.segments.front().state->capacitance;
            const auto solution = solve(tree, snapshot.buffers, target, options);
            spdlog::debug(
                "PBBI path: root={} nets={} baseline={:.6f}ns target={:.6f}ns "
                "selected={} score={:.6f}ns buffers={}",
                distinct.segments.front().state->net->getName(),
                distinct.segments.size(), plain.score * 1e9, target * 1e9,
                solution.selected, solution.score * 1e9,
                solution.selected < 0
                    ? 0
                    : solution.frontiers.back()[solution.selected].cost);
            result.truncated += solution.truncated;
            if (solution.selected >= 0 && plain.selected >= 0 &&
                solution.score > plain.score + options.common.tolerances.wns_absolute) {
                auto plans = decode(tree, snapshot.buffers, solution);
                if (!plans.empty()) {
                    result.paths.push_back({std::move(plans),
                                            unsigned(distinct.segments.size()),
                                            solution.score - plain.score});
                }
            }
            distinct = {};
        };
        for (const auto& segment : rank.path.segments) {
            if (!claimed.insert(segment.state->net).second) {
                finish();
            } else {
                distinct.segments.push_back(segment);
            }
        }
        finish();
    }
    spdlog::info(
        "PBBI planning: sampled={} distinct={} proposals={} frontier_truncations={}",
        result.sampled, result.split_paths, result.paths.size(), result.truncated);
    return result;
}
} // namespace ecc::pbbi
