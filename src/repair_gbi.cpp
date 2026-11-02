// SPDX-License-Identifier: Apache-2.0
#include "repair_gbi.h"

#include "BufferedNet.hh"
#include "buffer_inserter.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "repair_backend.h"
#include "repair_gbi_cuda.h"
#include "rsz/Resizer.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "odb/db.h"

namespace ecc {
namespace {

using detail::NetState;
using Clock = std::chrono::steady_clock;

GbiRepairMetrics metrics(const DesignRepairMetrics& m)
{
    return {m.severity,
            m.setup_wns,
            m.setup_tns,
            m.fanout_nets,
            m.capacitance_nets,
            m.slew_nets,
            m.total_slew_violations,
            m.total_cap_violations,
            m.total_fanout_violations};
}

bool better(const GbiRepairMetrics& before,
            const GbiRepairMetrics& after,
            const GbiRepairOptions& options)
{
    const auto& t = options.common.tolerances;
    if(after.setup_wns + t.wns_absolute < before.setup_wns ||
       after.setup_tns + t.tns_absolute < before.setup_tns)
        return false;
    const bool drv_non_worse =
        after.total_slew_violations <= before.total_slew_violations &&
        after.total_cap_violations <= before.total_cap_violations &&
        after.total_fanout_violations <= before.total_fanout_violations;
    if(options.require_drv_reduction && !drv_non_worse)
        return false;
    return after.severity + t.drv_severity < before.severity ||
           after.setup_wns > before.setup_wns + t.wns_absolute ||
           after.setup_tns > before.setup_tns + t.tns_absolute;
}

struct RawNode {
    gbi::HostNode host;
    const rsz::BufferedNet* source = nullptr;
};

struct Forest {
    std::shared_ptr<gbi::HostGraph> graph = std::make_shared<gbi::HostGraph>();
    std::vector<const rsz::BufferedNet*> source;
    std::vector<odb::dbNet*> nets;
    std::vector<rsz::BufferedNetPtr> trees;
    std::vector<sta::LibertyCell*> cells;
    std::vector<double> driver_res;
};

int appendNode(const rsz::BufferedNetPtr& node,
               int depth,
               int context,
               rsz::Resizer* resizer,
               sta::dbSta* sta,
               const sta::Scene* scene,
               std::vector<RawNode>& raw,
               int& next_id)
{
    if(!node)
        throw std::runtime_error("GBI encountered a null Steiner node");
    RawNode entry;
    entry.source = node.get();
    entry.host.old_id = next_id++;
    entry.host.depth = depth;
    entry.host.context_id = context;
    entry.host.x = resizer->dbuToMeters(node->location().x());
    entry.host.y = resizer->dbuToMeters(node->location().y());
    switch(node->type()) {
    case rsz::BufferedNetType::load: {
        entry.host.type = 0;
        entry.host.cap = node->cap();
        auto* pin = node->loadPin();
        entry.host.rat_or_res = pin
                                    ? sta->slack(pin,
                                                sta::RiseFallBoth::riseFall(),
                                                sta->scenes(),
                                                sta::MinMax::max()) *
                                          gbi::scale
                                    : 0.0;
        if(!std::isfinite(entry.host.rat_or_res))
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
        const int child = appendNode(node->ref(), depth + 1, context, resizer,
                                     sta, scene, raw, next_id);
        entry.host.left_old = raw[child].host.old_id;
        break;
    }
    case rsz::BufferedNetType::junction: {
        entry.host.type = 2;
        const int left = appendNode(node->ref(), depth + 1, context, resizer,
                                    sta, scene, raw, next_id);
        const int right = appendNode(node->ref2(), depth + 1, context, resizer,
                                     sta, scene, raw, next_id);
        entry.host.left_old = raw[left].host.old_id;
        entry.host.right_old = raw[right].host.old_id;
        break;
    }
    case rsz::BufferedNetType::via:
    case rsz::BufferedNetType::buffer:
        throw std::runtime_error("GBI input tree contains an unsupported node");
    }
    const int index = static_cast<int>(raw.size());
    raw.push_back(entry);
    return index;
}

void finalizeForest(std::vector<RawNode>& raw,
                    const std::vector<int>& roots_old,
                    Forest& forest)
{
    RawNode dummy;
    dummy.host.old_id = 0;
    dummy.host.type = -1;
    dummy.host.level = -1;
    raw.push_back(dummy);
    int maximum = 0;
    for(auto& node : raw) {
        if(node.host.type >= 0) {
            int tree_depth = 0;
            for(const auto& candidate : raw)
                if(candidate.host.context_id == node.host.context_id)
                    tree_depth = std::max(tree_depth, candidate.host.depth);
            node.host.level = tree_depth - node.host.depth;
            maximum = std::max(maximum, node.host.level);
        }
    }
    std::stable_sort(raw.begin(), raw.end(), [](const RawNode& a, const RawNode& b) {
        if(a.host.level != b.host.level)
            return a.host.level < b.host.level;
        return a.host.type < b.host.type;
    });
    std::unordered_map<int, int> remap;
    for(int i = 0; i < static_cast<int>(raw.size()); ++i)
        remap[raw[i].host.old_id] = i;
    forest.graph->global_max_level = maximum;
    forest.graph->resize(raw.size());
    forest.source.resize(raw.size());
    forest.graph->context_id.resize(raw.size(), -1);
    forest.graph->level_metas.resize(maximum + 1);
    for(int i = 0; i < static_cast<int>(raw.size()); ++i) {
        const auto& n = raw[i].host;
        forest.graph->type[i] = std::max(0, n.type);
        forest.graph->left[i] = remap[n.left_old];
        forest.graph->right[i] = remap[n.right_old];
        forest.graph->cap[i] = n.cap;
        forest.graph->rat_or_res[i] = n.rat_or_res;
        forest.graph->context_id[i] = n.context_id;
        forest.source[i] = raw[i].source;
        if(n.level >= 0) {
            auto& meta = forest.graph->level_metas[n.level];
            if(meta.start_node < 0)
                meta.start_node = i;
            meta.end_node = i + 1;
            if(n.type == 0) ++meta.sink_count;
            if(n.type == 1) ++meta.wire_count;
            if(n.type == 2) ++meta.junc_count;
        }
    }
    for(int old : roots_old) {
        const int id = remap[old];
        forest.graph->root.push_back(id);
        forest.graph->root_level.push_back(raw[id].host.level);
    }
}

std::vector<odb::dbObject*> applyNode(
    int id,
    const gbi::HostGraph& graph,
    const std::vector<const rsz::BufferedNet*>& source,
    const std::unordered_map<int, int>& selected,
    odb::dbNet* net,
    rsz::Resizer* resizer,
    const std::vector<sta::LibertyCell*>& cells)
{
    const int type = graph.type.at(id);
    const auto* node = source.at(id);
    auto* network = resizer->dbNetwork();
    if(type == 0) {
        auto* object = network->staToDb(node->loadPin());
        return object ? std::vector<odb::dbObject*>{object}
                      : std::vector<odb::dbObject*>{};
    }
    if(type == 2) {
        auto left = applyNode(graph.left.at(id), graph, source, selected, net,
                              resizer, cells);
        auto right = applyNode(graph.right.at(id), graph, source, selected, net,
                               resizer, cells);
        left.insert(left.end(), right.begin(), right.end());
        return left;
    }
    auto loads = applyNode(graph.left.at(id), graph, source, selected, net,
                           resizer, cells);
    const auto found = selected.find(id);
    if(found == selected.end())
        return loads;
    if(found->second < 0 || found->second >= static_cast<int>(cells.size()))
        throw std::runtime_error("GBI returned an invalid buffer index");
    if(loads.empty())
        throw std::runtime_error("GBI selected a buffer without loads");
    BufferInsertionOptions insertion;
    insertion.location = {node->location().x(), node->location().y()};
    insertion.buffer_name_prefix = "sizer_gbi_buf";
    insertion.net_name_prefix = "sizer_gbi_net";
    auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
        net, loads, network->staToDb(cells[found->second]), insertion);
    if(!edit)
        throw std::runtime_error(edit.message);
    return {edit.input_pin};
}

}  // namespace

void GbiRepairOptions::validate() const
{
    common.validate();
    if(backend != "cuda")
        throw std::invalid_argument("gbi.backend must be cuda");
}

GbiRepairResult OpenDbGbiRepairer::repair(
    odb::dbBlock* block,
    sta::dbSta* sta,
    const std::function<void()>& refresh,
    rsz::Resizer* resizer,
    const GbiRepairOptions& options)
{
    GbiRepairResult result;
    try {
        options.validate();
    } catch(const std::exception& e) {
        result.message = e.what();
        return result;
    }
    if(!block || !sta || !resizer || !refresh) {
        result.message = "GBI repair requires block, STA, resizer, and refresh";
        return result;
    }
    detail::Repair backend(block, sta, options.common);
    DesignRepairMetrics before_design;
    const auto collect_start = Clock::now();
    const auto states = backend.analyze(before_design, true);
    result.collect_ms = std::chrono::duration<double, std::milli>(Clock::now() - collect_start).count();
    result.before = metrics(before_design);

    const auto* scene = sta->cmdScene();
    resizer->initBlock();
    Forest forest;
    std::vector<RawNode> raw;
    std::vector<int> roots;
    int next_id = 1;
    const auto solve_start = Clock::now();
    for(const auto& state : states) {
        if(!state.driver || !state.violating() || state.loads.size() < 2)
            continue;
        if(options.max_nets && result.nets_collected >= options.max_nets)
            break;
        auto* pin = sta->getDbNetwork()->dbToSta(state.driver);
        auto tree = resizer->makeBufferedNetSteiner(pin, scene);
        auto* port = pin ? sta->getDbNetwork()->libertyPort(pin) : nullptr;
        if(!tree || !port)
            continue;
        forest.nets.push_back(state.net);
        forest.trees.push_back(tree);
        forest.driver_res.push_back(port->driveResistance());
        const int root = appendNode(tree, 0, result.nets_collected, resizer, sta,
                                    scene, raw, next_id);
        roots.push_back(raw[root].host.old_id);
        ++result.nets_collected;
    }
    // The temporary ID counter is kept local to the forest construction.
    // Rebuild if no candidates; this also avoids invoking CUDA for an empty set.
    if(roots.empty()) {
        result.ok = true;
        result.after = result.before;
        result.message = "GBI found no applicable violating net";
        return result;
    }
    finalizeForest(raw, roots, forest);
    sta::LibertyCellSeq library;
    resizer->getBufferList(library);
    for(auto* cell : library) {
        if(!cell || !cell->isBuffer()) continue;
        auto* master = sta->getDbNetwork()->staToDb(cell);
        if(std::find(options.common.excluded_masters.begin(),
                     options.common.excluded_masters.end(), master) !=
           options.common.excluded_masters.end()) continue;
        forest.cells.push_back(cell);
    }
    std::sort(forest.cells.begin(), forest.cells.end(), [](auto* a, auto* b) {
        sta::LibertyPort *ai = nullptr, *ao = nullptr, *bi = nullptr, *bo = nullptr;
        a->bufferPorts(ai, ao); b->bufferPorts(bi, bo);
        return ai->capacitance() < bi->capacitance();
    });
    auto buffers = std::make_shared<std::vector<gbi::Buffer>>();
    for(auto* cell : forest.cells) {
        sta::LibertyPort *input = nullptr, *output = nullptr;
        cell->bufferPorts(input, output);
        buffers->push_back({output->driveResistance(),
                            resizer->portCapacitance(input, scene),
                            static_cast<double>(output->intrinsicDelay(sta)) * gbi::scale});
    }
    gbi::CudaContext context;
    context.buffer_library = buffers;
    context.h_graph = forest.graph;
    context.h_driver_res = forest.driver_res;
    for(size_t i = 0; i < forest.nets.size(); ++i)
        context.results.push_back(std::make_shared<gbi::Result>());
    try {
        if(!gbi::gbiCudaAvailable())
            throw std::runtime_error("GBI CUDA device is unavailable");
        gbi::gbiCudaSolve(context);
    } catch(const std::exception& e) {
        result.message = e.what();
        return result;
    }
    result.solve_ms = std::chrono::duration<double, std::milli>(Clock::now() - solve_start).count();
    std::vector<std::unordered_map<int, int>> selections(forest.nets.size());
    unsigned budget = options.max_buffers;
    unsigned selected_count = 0;
    for(size_t i = 0; i < context.results.size(); ++i) {
        for(const auto& selected : context.results[i]->inserted_buffers) {
            if(options.max_buffers && budget == 0) break;
            selections[i][selected.node_id] = selected.buffer_type;
            ++selected_count;
            if(options.max_buffers) --budget;
        }
    }
    result.plans_collected = std::count_if(selections.begin(), selections.end(),
                                           [](const auto& s) { return !s.empty(); });
    result.attempted_buffers = selected_count;
    if(result.attempted_buffers == 0) {
        result.ok = true;
        result.after = result.before;
        result.message = "GBI produced no buffer insertion";
        return result;
    }
    odb::dbDatabase::beginEco(block);
    bool active = true;
    try {
        const auto apply_start = Clock::now();
        for(size_t i = 0; i < selections.size(); ++i) {
            applyNode(forest.graph->root[i], *forest.graph, forest.source,
                      selections[i], forest.nets[i], resizer, forest.cells);
        }
        result.apply_ms = std::chrono::duration<double, std::milli>(Clock::now() - apply_start).count();
        refresh();
        ++result.refreshes;
        DesignRepairMetrics after_design;
        detail::Repair verify(block, sta, options.common);
        verify.analyze(after_design, true);
        result.after = metrics(after_design);
        if(!better(result.before, result.after, options)) {
            odb::dbDatabase::undoEco(block);
            active = false;
            refresh();
            ++result.refreshes;
            result.after = result.before;
            result.ok = true;
            result.message = "GBI plan rejected by timing/DRV acceptance";
            return result;
        }
        odb::dbDatabase::commitEco(block);
        active = false;
        result.ok = true;
        result.committed = true;
        result.inserted_buffers = result.attempted_buffers;
        result.converged = result.after.severity <= options.common.tolerances.drv_severity;
        result.message = "GBI CUDA plan applied";
        return result;
    } catch(const std::exception& e) {
        if(active) {
            odb::dbDatabase::undoEco(block);
            try { refresh(); ++result.refreshes; } catch(...) {}
        }
        result.message = e.what();
        return result;
    }
}

}  // namespace ecc
