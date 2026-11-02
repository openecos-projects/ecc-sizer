// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_vg.h"
#include "repair_vg_cuda.h"

#include "buffer_inserter.h"
#include "repair_backend.h"

#include "BufferedNet.hh"
#include "est/EstimateParasitics.h"
#include "rsz/Resizer.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"

namespace ecc {
namespace {

using detail::NetState;

struct Plan {
    odb::dbNet* net = nullptr;
    odb::dbMaster* master = nullptr;
    std::vector< odb::dbObject* > loads;
    BufferLocation location;
    float gain = 0;
};

struct VgCandidate {
    rsz::BufferedNetPtr tree;
    double cap = 0;
    double slack = 0;
};

struct VgSolution {
    odb::dbNet* net = nullptr;
    rsz::BufferedNetPtr tree;
    double score = -std::numeric_limits<double>::infinity();
};

struct Timer {
    double* elapsed;
    std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    ~Timer() {
        *elapsed += std::chrono::duration< double, std::milli >(
                        std::chrono::steady_clock::now() - start)
                        .count();
    }
};

VgRepairMetrics metrics(const DesignRepairMetrics& m) {
    return {.severity = m.severity,
            .setup_wns = m.setup_wns,
            .setup_tns = m.setup_tns,
            .fanout_nets = m.fanout_nets,
            .capacitance_nets = m.capacitance_nets,
            .slew_nets = m.slew_nets,
            .total_slew_violations = m.total_slew_violations,
            .total_cap_violations = m.total_cap_violations,
            .total_fanout_violations = m.total_fanout_violations};
}

bool better(const VgRepairMetrics& before, const VgRepairMetrics& after,
            const VgRepairOptions& options) {
    const auto& tolerance = options.common.tolerances;
    const bool no_timing_loss =
        after.setup_wns + tolerance.wns_absolute >= before.setup_wns &&
        after.setup_tns + tolerance.tns_absolute >= before.setup_tns;
    const bool no_drv_loss =
        after.total_slew_violations <= before.total_slew_violations &&
        after.total_cap_violations <= before.total_cap_violations &&
        after.total_fanout_violations <= before.total_fanout_violations;
    if(!no_timing_loss || (options.require_drv_reduction && !no_drv_loss))
        return false;
    const bool strict_drv =
        after.severity + tolerance.drv_severity < before.severity ||
        after.total_slew_violations < before.total_slew_violations ||
        after.total_cap_violations < before.total_cap_violations ||
        after.total_fanout_violations < before.total_fanout_violations;
    const bool strict_timing =
        after.setup_wns > before.setup_wns + tolerance.wns_absolute ||
        after.setup_tns > before.setup_tns + tolerance.tns_absolute;
    return strict_drv || strict_timing;
}

std::vector< Plan > collectPlans(detail::Repair& backend,
                                 const std::vector< NetState >& states,
                                 const VgRepairOptions& options,
                                 unsigned& nets_collected) {
    std::vector< Plan > plans;
    const unsigned group_limit = options.max_group_loads == 0
                                     ? options.common.max_fanout
                                     : options.max_group_loads;
    if(group_limit < 2)
        return plans;
    for(const auto& state : states) {
        if(!state.violating() || state.loads.size() < 2)
            continue;
        if(options.max_nets != 0 && nets_collected >= options.max_nets)
            break;
        ++nets_collected;
        std::vector< detail::Load > loads = state.loads;
        std::stable_sort(loads.begin(), loads.end(),
                         [](const auto& a, const auto& b) {
                             return a.x != b.x ? a.x < b.x : a.y < b.y;
                         });
        const float input_slew =
            std::isfinite(state.slew_limit)
                ? state.slew_limit * options.common.recovered_slew_fraction
                : state.slew;
        const float pin_sum =
            std::accumulate(loads.begin(), loads.end(), 0.0f,
                            [](float total, const detail::Load& load) {
                                return total + load.capacitance;
                            });
        const float extra = std::max(0.0f, state.capacitance - pin_sum) /
                            std::max< size_t >(1, loads.size());
        for(size_t begin = 0; begin < loads.size();) {
            size_t end = begin;
            float cap = 0;
            const detail::Buffer* selected = nullptr;
            while(end < loads.size() && end - begin < group_limit) {
                const float next_cap = cap + loads[end].capacitance + extra;
                const auto* candidate =
                    backend.select(next_cap, state.slew_limit, input_slew);
                if(candidate == nullptr)
                    break;
                cap = next_cap;
                selected = candidate;
                ++end;
            }
            if(selected == nullptr || end - begin < 2) {
                ++begin;
                continue;
            }
            Plan plan;
            plan.net = state.net;
            plan.master = selected->master;
            long long x = 0, y = 0;
            for(size_t i = begin; i < end; ++i) {
                plan.loads.push_back(loads[i].term);
                x += loads[i].x;
                y += loads[i].y;
            }
            plan.location = {int(x / plan.loads.size()),
                             int(y / plan.loads.size())};
            plan.gain = std::max(
                state.fanout_ratio - 1.0f,
                std::max(state.cap_ratio - 1.0f, state.slew_ratio - 1.0f));
            if(plan.gain >= options.min_gain)
                plans.push_back(std::move(plan));
            begin = end;
        }
    }
    std::stable_sort(
        plans.begin(), plans.end(),
        [](const Plan& a, const Plan& b) { return a.gain > b.gain; });
    if(options.max_buffers != 0 && plans.size() > options.max_buffers)
        plans.resize(options.max_buffers);
    return plans;
}

// CPU Van Ginneken dynamic programming over the placement Steiner tree.  The
// tree is supplied by OpenROAD's estimator, while candidates and traceback
// remain owned by this module.
void pruneVg(std::vector<VgCandidate>& options)
{
    auto max_buffer = std::max_element(
        options.begin(), options.end(), [](const auto& a, const auto& b) {
            return a.tree->bufferCount() < b.tree->bufferCount();
        });
    std::shared_ptr<rsz::BufferedNet> preserve;
    if(max_buffer != options.end())
        preserve = max_buffer->tree;
    std::stable_sort(options.begin(), options.end(), [](const auto& a,
                                                        const auto& b) {
        return a.cap != b.cap ? a.cap < b.cap : a.slack > b.slack;
    });
    std::vector<VgCandidate> frontier;
    double best_slack = -std::numeric_limits<double>::infinity();
    for(auto& option : options) {
        if(option.slack > best_slack + 1e-18) {
            best_slack = option.slack;
            frontier.push_back(std::move(option));
        }
    }
    if(preserve && std::none_of(frontier.begin(), frontier.end(),
                                [&](const auto& option) {
                                    return option.tree == preserve;
                                })) {
        for(auto& option : options) {
            if(option.tree == preserve) {
                frontier.push_back(std::move(option));
                break;
            }
        }
    }
    options.swap(frontier);
}

std::vector<VgCandidate> runVg(const rsz::BufferedNetPtr& node,
                               rsz::Resizer* resizer,
                               const sta::Scene* scene,
                               const sta::MinMax* min_max,
                               const VgRepairOptions& options)
{
    if(!node)
        return {};
    switch(node->type()) {
    case rsz::BufferedNetType::load:
        return {{node, node->cap(), 0}};
    case rsz::BufferedNetType::wire: {
        auto child = runVg(node->ref(), resizer, scene, min_max, options);
        std::vector<VgCandidate> result;
        double wire_res = 0, wire_cap = 0;
        node->wireRC(scene, resizer, resizer->getEstimateParasitics(),
                     wire_res, wire_cap);
        const double length = resizer->dbuToMeters(node->length());
        const double total_wire_cap = length * wire_cap;
        sta::LibertyCellSeq buffers;
        resizer->getBufferList(buffers);
        if(buffers.empty()) {
            auto* network = resizer->dbNetwork();
            auto* block = resizer->block();
            for(auto* lib : block->getDataBase()->getLibs()) {
                for(auto* master : lib->getMasters()) {
                    auto* cell = network->libertyCell(network->dbToSta(master));
                    if(cell && cell->isBuffer()) {
                        buffers.push_back(cell);
                    }
                }
            }
        }
        for(const auto& option : child) {
            auto wire = std::make_shared<rsz::BufferedNet>(
                rsz::BufferedNetType::wire, node->location(), node->layer(),
                option.tree, scene, resizer, resizer->getEstimateParasitics());
            const double cap = option.cap + total_wire_cap;
            const double delay = length * wire_res *
                                 (option.cap + total_wire_cap / 2.0);
            result.push_back({wire, cap, option.slack - delay});
            for(auto* buffer : buffers) {
                if(!buffer || !buffer->isBuffer())
                    continue;
                auto* master = resizer->dbNetwork()->staToDb(buffer);
                if(std::find(options.common.excluded_masters.begin(),
                             options.common.excluded_masters.end(),
                             master) != options.common.excluded_masters.end())
                    continue;
                sta::LibertyPort* input = nullptr;
                sta::LibertyPort* output = nullptr;
                buffer->bufferPorts(input, output);
                if(!input || !output)
                    continue;
                const auto intrinsic = output->intrinsicDelay(resizer->sta());
                const double input_cap = resizer->portCapacitance(input, scene);
                const double buffer_delay =
                    static_cast<double>(intrinsic) + output->driveResistance() * cap;
                if(!std::isfinite(buffer_delay) || !std::isfinite(input_cap))
                    continue;
                auto buffered = std::make_shared<rsz::BufferedNet>(
                    rsz::BufferedNetType::buffer, node->location(), buffer,
                    wire, scene, resizer, resizer->getEstimateParasitics());
                result.push_back({buffered,
                                  input_cap,
                                  option.slack - delay - buffer_delay});
            }
        }
        pruneVg(result);
        return result;
    }
    case rsz::BufferedNetType::junction: {
        auto left = runVg(node->ref(), resizer, scene, min_max, options);
        auto right = runVg(node->ref2(), resizer, scene, min_max, options);
        std::vector<VgCandidate> result;
        result.reserve(left.size() * right.size());
        for(const auto& l : left) {
            for(const auto& r : right) {
                auto junction = std::make_shared<rsz::BufferedNet>(
                    rsz::BufferedNetType::junction, node->location(), l.tree,
                    r.tree, resizer);
                result.push_back({junction, l.cap + r.cap,
                                  std::min(l.slack, r.slack)});
            }
        }
        pruneVg(result);
        return result;
    }
    case rsz::BufferedNetType::buffer:
    case rsz::BufferedNetType::via:
        return {};
    }
    return {};
}

// Flat postorder snapshot. Only this adapter knows about OpenROAD types.
struct VgForest {
    std::vector<VgCudaNode> nodes;
    std::vector<rsz::BufferedNetPtr> originals;
    std::vector<VgCudaBuffer> buffers;
    sta::LibertyCellSeq cells;
    std::vector<int> roots;
    std::vector<double> resistance;
};

int flattenVg(const rsz::BufferedNetPtr& node, VgForest& forest,
              rsz::Resizer* resizer, const sta::Scene* scene) {
    if(!node) throw std::runtime_error("VG CUDA null tree node");
    VgCudaNode flat;
    switch(node->type()) {
    case rsz::BufferedNetType::load:
        flat.cap = node->cap();
        break;
    case rsz::BufferedNetType::wire: {
        flat.type = VgCudaNodeType::wire;
        flat.left = flattenVg(node->ref(), forest, resizer, scene);
        double resistance = 0, capacitance = 0;
        node->wireRC(scene, resizer, resizer->getEstimateParasitics(),
                     resistance, capacitance);
        const double length = resizer->dbuToMeters(node->length());
        flat.cap = length * capacitance;
        flat.resistance = length * resistance;
        break;
    }
    case rsz::BufferedNetType::junction:
        flat.type = VgCudaNodeType::junction;
        flat.left = flattenVg(node->ref(), forest, resizer, scene);
        flat.right = flattenVg(node->ref2(), forest, resizer, scene);
        break;
    default:
        throw std::runtime_error("VG CUDA unsupported Steiner node");
    }
    if(forest.nodes.size() >= static_cast<size_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("VG CUDA tree index overflow");
    forest.nodes.push_back(flat);
    forest.originals.push_back(node);
    return static_cast<int>(forest.nodes.size() - 1);
}

void collectCudaBuffers(VgForest& forest, rsz::Resizer* resizer,
                        const sta::Scene* scene,
                        const VgRepairOptions& options) {
    sta::LibertyCellSeq cells;
    resizer->getBufferList(cells);
    if(cells.empty()) {
        auto* network = resizer->dbNetwork();
        for(auto* lib : resizer->block()->getDataBase()->getLibs())
            for(auto* master : lib->getMasters()) {
                auto* cell = network->libertyCell(network->dbToSta(master));
                if(cell && cell->isBuffer()) cells.push_back(cell);
            }
    }
    for(auto* cell : cells) {
        if(!cell || !cell->isBuffer()) continue;
        auto* master = resizer->dbNetwork()->staToDb(cell);
        if(std::find(options.common.excluded_masters.begin(),
                     options.common.excluded_masters.end(), master) !=
           options.common.excluded_masters.end()) continue;
        sta::LibertyPort *input = nullptr, *output = nullptr;
        cell->bufferPorts(input, output);
        if(!input || !output) continue;
        VgCudaBuffer model{resizer->portCapacitance(input, scene),
                             static_cast<double>(output->intrinsicDelay(resizer->sta())),
                             output->driveResistance()};
        if(!std::isfinite(model.cap) || !std::isfinite(model.intrinsic) ||
           !std::isfinite(model.resistance)) continue;
        forest.buffers.push_back(model);
        forest.cells.push_back(cell);
    }
}

rsz::BufferedNetPtr tracebackVg(int id, int selected, const VgForest& forest,
                               const VgCudaForestResult& result,
                               rsz::Resizer* resizer, const sta::Scene* scene) {
    const auto& node = forest.nodes.at(id);
    const auto& candidate = result.frontiers.at(id).at(selected);
    const auto& original = forest.originals.at(id);
    if(node.type == VgCudaNodeType::load) return original;
    auto left = tracebackVg(node.left, candidate.left, forest, result, resizer, scene);
    if(node.type == VgCudaNodeType::junction) {
        auto right = tracebackVg(node.right, candidate.right, forest, result, resizer, scene);
        return std::make_shared<rsz::BufferedNet>(rsz::BufferedNetType::junction,
                                                 original->location(), left, right, resizer);
    }
    auto wire = std::make_shared<rsz::BufferedNet>(rsz::BufferedNetType::wire,
        original->location(), original->layer(), left, scene, resizer,
        resizer->getEstimateParasitics());
    if(candidate.buffer < 0) return wire;
    return std::make_shared<rsz::BufferedNet>(rsz::BufferedNetType::buffer,
        original->location(), forest.cells.at(candidate.buffer), wire, scene,
        resizer, resizer->getEstimateParasitics());
}

std::vector<VgSolution> collectVgSolutions(const std::vector<NetState>& states,
                                           rsz::Resizer* resizer,
                                           sta::dbSta* sta,
                                           const VgRepairOptions& options,
                                           VgRepairResult& report)
{
    std::vector<VgSolution> solutions;
    if(!resizer) return solutions;
    resizer->initBlock();
    const auto* scene = sta->cmdScene();
    struct Input {
        odb::dbNet* net;
        rsz::BufferedNetPtr tree;
        double resistance;
    };
    std::vector<Input> inputs;
    for(const auto& state : states) {
        if(!state.violating() || !state.driver) continue;
        if(options.max_nets != 0 && report.nets_collected >= options.max_nets) break;
        auto* driver_pin = sta->getDbNetwork()->dbToSta(state.driver);
        auto tree = resizer->makeBufferedNetSteiner(driver_pin, scene);
        if(!tree) continue;
        ++report.nets_collected;
        auto* port = sta->getDbNetwork()->libertyPort(driver_pin);
        if(port) inputs.push_back({state.net, tree, port->driveResistance()});
    }
    std::vector<VgSolution> best(inputs.size());
    bool used_cuda = false;
    if(options.backend == "cuda") {
        try {
            VgForest forest;
            collectCudaBuffers(forest, resizer, scene, options);
            for(const auto& input : inputs) {
                forest.roots.push_back(flattenVg(input.tree, forest, resizer, scene));
                forest.resistance.push_back(input.resistance);
            }
            const auto result = vgCudaSolve(forest.nodes, forest.buffers,
                forest.roots, forest.resistance,
                size_t(options.cuda_max_memory_mb) * 1024 * 1024);
            for(size_t i = 0; i < inputs.size(); ++i) {
                if(result.selected[i] < 0) continue;
                const auto& candidate = result.frontiers[forest.roots[i]][result.selected[i]];
                best[i] = {inputs[i].net,
                    tracebackVg(forest.roots[i], result.selected[i], forest, result, resizer, scene),
                    candidate.slack - inputs[i].resistance * candidate.cap};
            }
            used_cuda = true;
            report.backend_used = "cuda";
        } catch(const std::exception& error) {
            report.cuda_fallback = error.what();
            best.assign(inputs.size(), VgSolution{});
        }
    }
    if(!used_cuda) {
        report.backend_used = "cpu";
        for(size_t i = 0; i < inputs.size(); ++i) {
            auto candidates = runVg(inputs[i].tree, resizer, scene, sta::MinMax::max(), options);
            best[i].net = inputs[i].net;
            for(auto& candidate : candidates) {
                if(candidate.tree->bufferCount() == 0) continue;
                const double score = candidate.slack - inputs[i].resistance * candidate.cap;
                if(score > best[i].score) {
                    best[i].score = score;
                    best[i].tree = std::move(candidate.tree);
                }
            }
        }
    }
    unsigned budget = options.max_buffers;
    for(auto& solution : best) {
        const unsigned count = solution.tree ? solution.tree->bufferCount() : 0;
        if(count > 0 && (options.max_buffers == 0 || count <= budget)) {
            solutions.push_back(std::move(solution));
            if(options.max_buffers != 0) budget -= count;
        }
    }
    return solutions;
}

std::vector<odb::dbObject*> applyVgNode(const rsz::BufferedNetPtr& node,
                                        odb::dbNet* net,
                                        rsz::Resizer* resizer)
{
    if(!node)
        return {};
    auto* network = resizer->dbNetwork();
    switch(node->type()) {
    case rsz::BufferedNetType::load: {
        auto* object = network->staToDb(node->loadPin());
        return object ? std::vector<odb::dbObject*>{object}
                      : std::vector<odb::dbObject*>{};
    }
    case rsz::BufferedNetType::wire:
    case rsz::BufferedNetType::via:
        return applyVgNode(node->ref(), net, resizer);
    case rsz::BufferedNetType::junction: {
        auto left = applyVgNode(node->ref(), net, resizer);
        auto right = applyVgNode(node->ref2(), net, resizer);
        left.insert(left.end(), right.begin(), right.end());
        return left;
    }
    case rsz::BufferedNetType::buffer: {
        auto loads = applyVgNode(node->ref(), net, resizer);
        if(loads.empty())
            throw std::runtime_error("VG buffer has no downstream loads");
        BufferInsertionOptions insertion;
        insertion.location = BufferLocation{node->location().x(),
                                            node->location().y()};
        insertion.buffer_name_prefix = "sizer_vg_buf";
        insertion.net_name_prefix = "sizer_vg_net";
        auto* master = network->staToDb(node->bufferCell());
        auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
            net, loads, master, insertion);
        if(!edit)
            throw std::runtime_error(edit.message);
        return {edit.input_pin};
    }
    }
    return {};
}

}  // namespace

void VgRepairOptions::validate() const {
    common.validate();
    if(cuda_max_memory_mb == 0)
        throw std::invalid_argument("vg.cuda_max_memory_mb must be positive");
    if(backend != "cpu" && backend != "cuda")
        throw std::invalid_argument("vg.backend must be cpu or cuda");
    if(max_group_loads != 0 && max_group_loads < 2)
        throw std::invalid_argument("vg.max_group_loads must be at least 2");
    if(!std::isfinite(min_gain) || min_gain < 0)
        throw std::invalid_argument(
            "vg.min_gain must be finite and nonnegative");
}

VgRepairResult OpenDbVgRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh, const VgRepairOptions& options)
{
    return repair(block, sta, refresh, nullptr, options);
}

VgRepairResult OpenDbVgRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh, rsz::Resizer* resizer,
    const VgRepairOptions& options) {
    VgRepairResult result;
    try {
        options.validate();
    }
    catch(const std::exception& error) {
        result.message = error.what();
        return result;
    }
    if(block == nullptr || sta == nullptr || !refresh) {
        result.message = "VG repair requires block, STA, and refresh callback";
        return result;
    }
    if(sta->getDbNetwork()->block() != block) {
        result.message = "VG STA network does not reference the target block";
        return result;
    }
    detail::Repair backend(block, sta, options.common);
    if(!backend.hasCells()) {
        result.message = "VG found no usable combinational cells";
        return result;
    }
    DesignRepairMetrics before_design;
    std::vector< NetState > states;
    {
        Timer timer{&result.collect_ms};
        states = backend.analyze(before_design, true);
    }
    result.before = metrics(before_design);
    if(resizer) {
        Timer timer{&result.solve_ms};
        auto solutions = collectVgSolutions(states, resizer, sta, options,
                                            result);
        result.plans_collected = solutions.size();
        if(solutions.empty()) {
            result.ok = true;
            result.after = result.before;
            result.message = "VG found no applicable buffer plan";
            return result;
        }
        odb::dbDatabase::beginEco(block);
        bool active = true;
        try {
            Timer apply_timer{&result.apply_ms};
            for(const auto& solution : solutions) {
                applyVgNode(solution.tree, solution.net, resizer);
                result.attempted_buffers += solution.tree->bufferCount();
            }
            refresh();
            ++result.refreshes;
            DesignRepairMetrics after_design;
            detail::Repair verify_backend(block, sta, options.common);
            verify_backend.analyze(after_design, true);
            result.after = metrics(after_design);
            if(!better(result.before, result.after, options)) {
                odb::dbDatabase::undoEco(block);
                active = false;
                refresh();
                ++result.refreshes;
                result.after = result.before;
                result.ok = true;
                result.message = "VG plan rejected by timing/DRV acceptance";
                return result;
            }
            odb::dbDatabase::commitEco(block);
            active = false;
            result.ok = true;
            result.committed = true;
            result.inserted_buffers = result.attempted_buffers;
            result.converged = result.after.severity <=
                               options.common.tolerances.drv_severity;
            result.message = "VG one-shot plan applied";
            return result;
        }
        catch(const std::exception& error) {
            if(active) {
                odb::dbDatabase::undoEco(block);
                try {
                    refresh();
                    ++result.refreshes;
                }
                catch(const std::exception& restore_error) {
                    result.message = std::string(error.what()) +
                                     "; rollback refresh failed: " +
                                     restore_error.what();
                    return result;
                }
            }
            result.message = error.what();
            return result;
        }
    }
    std::vector< Plan > plans;
    {
        Timer timer{&result.solve_ms};
        plans = collectPlans(backend, states, options, result.nets_collected);
    }
    result.plans_collected = plans.size();
    if(plans.empty()) {
        result.ok = true;
        result.converged =
            result.before.severity <= options.common.tolerances.drv_severity;
        result.after = result.before;
        result.message = "VG found no applicable one-level buffer plan";
        return result;
    }

    odb::dbDatabase::beginEco(block);
    bool active = true;
    std::unordered_set< odb::dbObject* > seen_loads;
    try {
        Timer timer{&result.apply_ms};
        for(const auto& plan : plans) {
            std::vector< odb::dbObject* > loads;
            loads.reserve(plan.loads.size());
            for(auto* load : plan.loads) {
                if(!seen_loads.insert(load).second)
                    continue;
                loads.push_back(load);
            }
            if(loads.size() < 2)
                continue;
            BufferInsertionOptions insertion;
            insertion.location = plan.location;
            insertion.buffer_name_prefix = "sizer_vg_buf";
            insertion.net_name_prefix = "sizer_vg_net";
            const auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
                plan.net, loads, plan.master, insertion);
            if(!edit)
                throw std::runtime_error(edit.message);
            ++result.attempted_buffers;
        }
        if(result.attempted_buffers == 0) {
            odb::dbDatabase::undoEco(block);
            active = false;
            result.ok = true;
            result.after = result.before;
            result.message = "VG plans became stale before apply";
            return result;
        }
        refresh();
        ++result.refreshes;
        DesignRepairMetrics after_design;
        backend.analyze(after_design, true);
        result.after = metrics(after_design);
        if(!better(result.before, result.after, options)) {
            odb::dbDatabase::undoEco(block);
            active = false;
            refresh();
            ++result.refreshes;
            result.after = result.before;
            result.ok = true;
            result.message = "VG plan rejected by timing/DRV acceptance";
            return result;
        }
        odb::dbDatabase::commitEco(block);
        active = false;
        result.ok = true;
        result.committed = true;
        result.inserted_buffers = result.attempted_buffers;
        result.converged =
            result.after.severity <= options.common.tolerances.drv_severity;
        result.message = "VG one-shot plan applied";
        return result;
    }
    catch(const std::exception& error) {
        if(active) {
            odb::dbDatabase::undoEco(block);
            try {
                refresh();
                ++result.refreshes;
            }
            catch(const std::exception& restore_error) {
                result.message =
                    std::string(error.what()) +
                    "; rollback refresh failed: " + restore_error.what();
                return result;
            }
        }
        result.message = error.what();
        return result;
    }
}

}  // namespace ecc
