// SPDX-License-Identifier: Apache-2.0
#include "algorithms/gobi.h"
#include "repair_config.h"

#include "BufferedNet.hh"
#include "actions/netlist.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "design_analysis.h"
#include "algorithms/gobi_cuda.h"
#include "rsz/Resizer.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "odb/db.h"

namespace ecc {
namespace gobi_detail {

using detail::NetState;
using Clock = std::chrono::steady_clock;

bool better(const DesignMetrics &before, const DesignMetrics &after,
            const GobiOptions &options) {
    const auto &t = options.common.tolerances;
    if (after.setup_wns + t.wns_absolute < before.setup_wns ||
        after.setup_tns + t.tns_absolute < before.setup_tns)
        return false;
    const bool drv_non_worse =
        after.total_slew_violations <= before.total_slew_violations &&
        after.total_cap_violations <= before.total_cap_violations &&
        after.total_fanout_violations <= before.total_fanout_violations;
    if (options.require_drv_reduction && !drv_non_worse)
        return false;
    return after.total_slew_violations < before.total_slew_violations ||
           after.total_cap_violations < before.total_cap_violations ||
           after.total_fanout_violations < before.total_fanout_violations ||
           after.severity + t.drv_severity < before.severity ||
           after.setup_wns > before.setup_wns + t.wns_absolute ||
           after.setup_tns > before.setup_tns + t.tns_absolute;
}

struct RawNode {
    gobi::HostNode host;
    const rsz::BufferedNet *source = nullptr;
};

struct Forest {
    std::shared_ptr<gobi::HostGraph> graph = std::make_shared<gobi::HostGraph>();
    std::vector<const rsz::BufferedNet *> source;
    std::vector<odb::dbNet *> nets;
    std::vector<rsz::BufferedNetPtr> trees;
    std::vector<sta::LibertyCell *> cells;
    std::vector<double> driver_res;
};

int appendNode(const rsz::BufferedNetPtr &node, int depth, int context,
               rsz::Resizer *resizer, sta::dbSta *sta, const sta::Scene *scene,
               std::vector<RawNode> &raw, int &next_id) {
    if (!node)
        throw std::runtime_error("GOBI encountered a null Steiner node");
    RawNode entry;
    entry.source = node.get();
    entry.host.old_id = next_id++;
    entry.host.depth = depth;
    entry.host.context_id = context;
    entry.host.x = resizer->dbuToMeters(node->location().x());
    entry.host.y = resizer->dbuToMeters(node->location().y());
    switch (node->type()) {
    case rsz::BufferedNetType::load: {
        entry.host.type = 0;
        entry.host.cap = node->cap();
        auto *pin = node->loadPin();
        entry.host.rat_or_res = pin ? sta->slack(pin, sta::RiseFallBoth::riseFall(),
                                                 sta->scenes(), sta::MinMax::max()) *
                                          gobi::scale
                                    : 0.0;
        if (!std::isfinite(entry.host.rat_or_res))
            entry.host.rat_or_res = 0.0;
        break;
    }
    case rsz::BufferedNetType::wire: {
        entry.host.type = 1;
        double r = 0.0, c = 0.0;
        node->wireRC(scene, resizer, resizer->getEstimateParasitics(), r, c);
        const double length = resizer->dbuToMeters(node->length());
        entry.host.cap = length * c;
        entry.host.rat_or_res = length * r;
        const int child = appendNode(node->ref(), depth + 1, context, resizer, sta,
                                     scene, raw, next_id);
        entry.host.left_old = raw[child].host.old_id;
        break;
    }
    case rsz::BufferedNetType::junction: {
        entry.host.type = 2;
        const int left = appendNode(node->ref(), depth + 1, context, resizer, sta,
                                    scene, raw, next_id);
        const int right = appendNode(node->ref2(), depth + 1, context, resizer, sta,
                                     scene, raw, next_id);
        entry.host.left_old = raw[left].host.old_id;
        entry.host.right_old = raw[right].host.old_id;
        break;
    }
    case rsz::BufferedNetType::via:
    case rsz::BufferedNetType::buffer:
        throw std::runtime_error("GOBI input tree contains an unsupported node");
    }
    const int index = static_cast<int>(raw.size());
    raw.push_back(entry);
    return index;
}

void finalizeForest(std::vector<RawNode> &raw, const std::vector<int> &roots_old,
                    Forest &forest) {
    RawNode dummy;
    dummy.host.old_id = 0;
    dummy.host.type = -1;
    dummy.host.level = -1;
    raw.push_back(dummy);
    int maximum = 0;
    std::vector<int> tree_depths(roots_old.size(), 0);
    for (const auto &node : raw)
        if (node.host.context_id >= 0)
            tree_depths.at(node.host.context_id) =
                std::max(tree_depths.at(node.host.context_id), node.host.depth);
    for (auto &node : raw) {
        if (node.host.type >= 0) {
            node.host.level = tree_depths.at(node.host.context_id) - node.host.depth;
            maximum = std::max(maximum, node.host.level);
        }
    }
    std::stable_sort(raw.begin(), raw.end(), [](const RawNode &a, const RawNode &b) {
        if (a.host.level != b.host.level)
            return a.host.level < b.host.level;
        return a.host.type < b.host.type;
    });
    std::unordered_map<int, int> remap;
    for (int i = 0; i < static_cast<int>(raw.size()); ++i)
        remap[raw[i].host.old_id] = i;
    forest.graph->global_max_level = maximum;
    forest.graph->resize(raw.size());
    forest.source.resize(raw.size());
    forest.graph->context_id.resize(raw.size(), -1);
    forest.graph->level_metas.resize(maximum + 1);
    for (int i = 0; i < static_cast<int>(raw.size()); ++i) {
        const auto &n = raw[i].host;
        forest.graph->type[i] = std::max(0, n.type);
        forest.graph->left[i] = remap[n.left_old];
        forest.graph->right[i] = remap[n.right_old];
        forest.graph->cap[i] = n.cap;
        forest.graph->rat_or_res[i] = n.rat_or_res;
        forest.graph->context_id[i] = n.context_id;
        forest.source[i] = raw[i].source;
        if (n.level >= 0) {
            auto &meta = forest.graph->level_metas[n.level];
            if (meta.start_node < 0)
                meta.start_node = i;
            meta.end_node = i + 1;
            if (n.type == 0)
                ++meta.sink_count;
            if (n.type == 1)
                ++meta.wire_count;
            if (n.type == 2)
                ++meta.junc_count;
        }
    }
    for (int old : roots_old) {
        const int id = remap[old];
        forest.graph->root.push_back(id);
        forest.graph->root_level.push_back(raw[id].host.level);
    }
}

BufferTreeNode decodeNode(int id, const gobi::HostGraph &graph,
                          const std::vector<const rsz::BufferedNet *> &source,
                          const std::unordered_map<int, int> &selected,
                          BufferTreePlan &plan, rsz::Resizer *resizer,
                          const std::vector<sta::LibertyCell *> &cells) {
    const int type = graph.type.at(id);
    const auto *node = source.at(id);
    auto *network = resizer->dbNetwork();
    if (type == 0) {
        auto *object = network->staToDb(node->loadPin());
        if (!object)
            throw std::runtime_error("GOBI tree load has no OpenDB terminal");
        return {.loads = {object}};
    }
    if (type == 2) {
        auto left = decodeNode(graph.left.at(id), graph, source, selected, plan,
                               resizer, cells);
        auto right = decodeNode(graph.right.at(id), graph, source, selected, plan,
                                resizer, cells);
        left.loads.insert(left.loads.end(), right.loads.begin(), right.loads.end());
        left.children.insert(left.children.end(), right.children.begin(),
                             right.children.end());
        return left;
    }
    auto loads =
        decodeNode(graph.left.at(id), graph, source, selected, plan, resizer, cells);
    const auto found = selected.find(id);
    if (found == selected.end())
        return loads;
    if (found->second < 0 || found->second >= static_cast<int>(cells.size()))
        throw std::runtime_error("GOBI returned an invalid buffer index");
    if (loads.loads.empty() && loads.children.empty())
        throw std::runtime_error("GOBI selected a buffer without loads");
    loads.master = network->staToDb(cells[found->second]);
    loads.location = {node->location().x(), node->location().y()};
    plan.nodes.push_back(std::move(loads));
    return {.children = {plan.nodes.size() - 1}};
}

} // namespace gobi_detail

void GobiOptions::validate() const { common.validate(); }

GobiSolutions Gobi::solve(odb::dbBlock *block, sta::dbSta *sta,
                          std::span<const detail::NetState> states,
                          rsz::Resizer *resizer, const GobiOptions &options) {
    using namespace gobi_detail;
    options.validate();
    if (!block || !sta || !resizer || sta->getDbNetwork()->block() != block ||
        sta->scenes().size() != 1 || resizer->sta() != sta)
        throw std::invalid_argument(
            "GOBI solve requires block, resizer and one STA scene");
    std::unordered_set<odb::dbNet *> seen;
    for (const auto &state : states) {
        if (!state.net || state.net->getBlock() != block || !state.driver ||
            state.driver->getNet() != state.net || !seen.insert(state.net).second)
            throw std::invalid_argument(
                "GOBI solve requires unique current net snapshots");
    }
    GobiSolutions result;
    if (states.empty())
        return result;
    const auto *scene = sta->cmdScene();
    resizer->initBlock();
    Forest forest;
    std::vector<RawNode> raw;
    std::vector<int> roots;
    int next_id = 1;
    for (const auto &state : states) {
        auto *pin = sta->getDbNetwork()->dbToSta(state.driver);
        auto tree = resizer->makeBufferedNetSteiner(pin, scene);
        auto *port = pin ? sta->getDbNetwork()->libertyPort(pin) : nullptr;
        if (!tree || !port)
            continue;
        forest.nets.push_back(state.net);
        forest.trees.push_back(tree);
        forest.driver_res.push_back(port->driveResistance());
        const int root = appendNode(tree, 0, result.nets_collected, resizer, sta, scene,
                                    raw, next_id);
        roots.push_back(raw[root].host.old_id);
        ++result.nets_collected;
    }
    if (roots.empty())
        return result;
    finalizeForest(raw, roots, forest);
    sta::LibertyCellSeq library;
    resizer->getBufferList(library);
    for (auto *cell : library) {
        if (!cell || !cell->isBuffer())
            continue;
        auto *master = sta->getDbNetwork()->staToDb(cell);
        if (std::find(options.common.excluded_masters.begin(),
                      options.common.excluded_masters.end(),
                      master) != options.common.excluded_masters.end())
            continue;
        forest.cells.push_back(cell);
    }
    // The unchanged CUDA kernels have 32 constant-memory buffer slots.
    if (forest.cells.empty() || forest.cells.size() > 32)
        throw std::runtime_error("GOBI requires between 1 and 32 buffer types, got " +
                                 std::to_string(forest.cells.size()) +
                                 "; restrict excluded_masters / dont_use");
    std::sort(forest.cells.begin(), forest.cells.end(), [&](auto *a, auto *b) {
        sta::LibertyPort *ai = nullptr, *ao = nullptr, *bi = nullptr, *bo = nullptr;
        a->bufferPorts(ai, ao);
        b->bufferPorts(bi, bo);
        const double ac = resizer->portCapacitance(ai, scene);
        const double bc = resizer->portCapacitance(bi, scene);
        return ac != bc ? ac < bc : std::string(a->name()) < b->name();
    });
    auto buffers = std::make_shared<std::vector<gobi::Buffer>>();
    for (auto *cell : forest.cells) {
        sta::LibertyPort *input = nullptr, *output = nullptr;
        cell->bufferPorts(input, output);
        buffers->push_back(
            {output->driveResistance(), resizer->portCapacitance(input, scene),
             static_cast<double>(output->intrinsicDelay(sta)) * gobi::scale});
    }
    gobi::CudaContext context;
    context.buffer_library = buffers;
    context.h_graph = forest.graph;
    context.h_driver_res = forest.driver_res;
    for (size_t i = 0; i < forest.nets.size(); ++i)
        context.results.push_back(std::make_shared<gobi::Result>());
    if (!gobi::gobiCudaAvailable())
        throw std::runtime_error("GOBI CUDA device is unavailable");
    gobi::gobiCudaSolve(context);
    std::vector<std::unordered_map<int, int>> selections(forest.nets.size());
    for (size_t i = 0; i < context.results.size(); ++i) {
        if (context.results[i]->inserted_buffers.empty())
            continue;
        for (const auto &selected : context.results[i]->inserted_buffers) {
            if (selected.node_id <= 0 || selected.node_id >= forest.graph->num_nodes ||
                forest.graph->context_id[selected.node_id] != static_cast<int>(i) ||
                forest.graph->type[selected.node_id] != 1 || selected.buffer_type < 0 ||
                selected.buffer_type >= static_cast<int>(forest.cells.size()) ||
                !selections[i].emplace(selected.node_id, selected.buffer_type).second)
                throw std::runtime_error(
                    "GOBI returned an invalid or duplicate buffer selection");
        }
        BufferTreePlan plan;
        plan.net = forest.nets[i];
        decodeNode(forest.graph->root[i], *forest.graph, forest.source, selections[i],
                   plan, resizer, forest.cells);
        if (plan.nodes.size() != selections[i].size())
            throw std::runtime_error("GOBI selection is not reachable from its root");
        result.plans.push_back(std::move(plan));
    }
    return result;
}

GobiResult Gobi::run(odb::dbBlock *block, sta::dbSta *sta,
                     const std::function<void()> &refresh, rsz::Resizer *resizer,
                     const GobiOptions &options) {
    using namespace gobi_detail;
    GobiResult result;
    std::optional<EcoTransaction> transaction;
    try {
        options.validate();
        if (!block || !sta || !resizer || !refresh ||
            sta->getDbNetwork()->block() != block || sta->scenes().size() != 1)
            throw std::invalid_argument(
                "GOBI run requires block, resizer, one STA scene and refresh");
        detail::DesignAnalysis analysis(block, sta, options.common);
        const auto collect_start = Clock::now();
        std::vector<NetState> states;
        for (auto &state : analysis.analyze(result.before, true)) {
            if (!state.driver || !state.violating())
                continue;
            if (options.max_nets && states.size() >= options.max_nets)
                break;
            states.push_back(std::move(state));
        }
        result.collect_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - collect_start)
                .count();
        result.after = result.before;
        if (!options.common.actions.buffer ||
            remainingBufferBudget(options.max_buffers, 0) == 0) {
            result.ok = true;
            result.message = "no buffer insertion allowed by actions or count limit";
            return result;
        }
        const auto solve_start = Clock::now();
        auto solutions = solve(block, sta, states, resizer, options);
        result.solve_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - solve_start)
                .count();
        result.nets_collected = solutions.nets_collected;
        std::vector<std::size_t> counts;
        for (const auto &plan : solutions.plans)
            counts.push_back(plan.nodes.size());
        const auto selected = selectWholeBufferPlans(counts, options.max_buffers);
        result.plans_collected = selected.size();
        if (selected.empty()) {
            result.ok = true;
            result.message = "GOBI produced no applicable buffer insertion";
            return result;
        }
        transaction.emplace(block);
        const auto apply_start = Clock::now();
        BufferInsertionOptions naming;
        naming.buffer_name_prefix = "sizer_gobi_buf";
        naming.net_name_prefix = "sizer_gobi_net";
        for (auto i : selected)
            result.attempted_buffers +=
                applyBufferTree(solutions.plans[i], naming).insertions.size();
        result.apply_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - apply_start)
                .count();
        refresh();
        ++result.refreshes;
        detail::DesignAnalysis verify(block, sta, options.common);
        verify.analyze(result.after, true);
        if (!better(result.before, result.after, options)) {
            transaction->rollback();
            refresh();
            ++result.refreshes;
            result.after = result.before;
            result.ok = true;
            result.message = "GOBI plan rejected by timing/DRV acceptance";
            return result;
        }
        transaction->commit();
        result.ok = true;
        result.committed = true;
        result.inserted_buffers = result.attempted_buffers;
        result.converged =
            result.after.severity <= options.common.tolerances.drv_severity;
        result.message = "GOBI CUDA plan applied";
    } catch (const std::exception &error) {
        result.message = error.what();
        if (transaction && transaction->active()) {
            transaction->rollback();
            try {
                refresh();
                ++result.refreshes;
            } catch (const std::exception &restore_error) {
                result.message +=
                    std::string("; rollback refresh failed: ") + restore_error.what();
            }
        }
        result.after = result.before;
    }
    return result;
}
} // namespace ecc
