// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "algorithms/van_ginneken.h"
#include "repair/config.h"
#include "algorithms/vg_cuda.h"

#include "actions/netlist.h"
#include "design_analysis.h"

#include "BufferedNet.hh"
#include "est/EstimateParasitics.h"
#include "rsz/Resizer.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"

namespace ecc {
namespace vg_detail {

using detail::NetState;

struct Plan {
    odb::dbNet* net = nullptr;
    odb::dbMaster* master = nullptr;
    std::vector<odb::dbObject*> loads;
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

// Score the unchanged RC tree with the same linear model as VG. Keep this
// baseline separately: cap/RAT pruning need not retain its traceback.
std::pair<double, double> unchangedVg(const rsz::BufferedNetPtr& node,
                                      rsz::Resizer* resizer, const sta::Scene* scene) {
    if (node->type() == rsz::BufferedNetType::load) {
        return {node->cap(), 0};
    }
    auto left = unchangedVg(node->ref(), resizer, scene);
    if (node->type() == rsz::BufferedNetType::junction) {
        auto right = unchangedVg(node->ref2(), resizer, scene);
        return {left.first + right.first, std::min(left.second, right.second)};
    }
    if (node->type() != rsz::BufferedNetType::wire) {
        throw std::invalid_argument("VG unchanged score requires a placement RC tree");
    }
    double resistance = 0, capacitance = 0;
    node->wireRC(scene, resizer, resizer->getEstimateParasitics(), resistance,
                 capacitance);
    const double length = resizer->dbuToMeters(node->length());
    return {left.first + length * capacitance,
            left.second -
                length * resistance * (left.first + length * capacitance / 2)};
}

std::vector<Plan> collectPlans(detail::DesignAnalysis& analysis,
                               std::span<const NetState> states,
                               const VgOptions& options, unsigned& nets_collected) {
    std::vector<Plan> plans;
    const unsigned group_limit = options.max_group_loads == 0
                                     ? options.common.max_fanout
                                     : options.max_group_loads;
    if (group_limit < 2) {
        return plans;
    }
    for (const auto& state : states) {
        if (state.loads.size() < 2) {
            continue;
        }
        ++nets_collected;
        std::vector<detail::Load> loads = state.loads;
        std::stable_sort(loads.begin(), loads.end(), [](const auto& a, const auto& b) {
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
                            std::max<size_t>(1, loads.size());
        for (size_t begin = 0; begin < loads.size();) {
            size_t end = begin;
            float cap = 0;
            const detail::CellModel* selected = nullptr;
            while (end < loads.size() && end - begin < group_limit) {
                const float next_cap = cap + loads[end].capacitance + extra;
                const auto* candidate =
                    analysis.select(next_cap, state.slew_limit, input_slew);
                if (candidate == nullptr) {
                    break;
                }
                cap = next_cap;
                selected = candidate;
                ++end;
            }
            if (selected == nullptr || end - begin < 2) {
                ++begin;
                continue;
            }
            Plan plan;
            plan.net = state.net;
            plan.master = selected->master;
            long long x = 0, y = 0;
            for (size_t i = begin; i < end; ++i) {
                plan.loads.push_back(loads[i].term);
                x += loads[i].x;
                y += loads[i].y;
            }
            plan.location = {int(x / plan.loads.size()), int(y / plan.loads.size())};
            plan.gain =
                std::max(state.fanout_ratio - 1.0f,
                         std::max(state.cap_ratio - 1.0f, state.slew_ratio - 1.0f));
            if (plan.gain >= options.min_gain) {
                plans.push_back(std::move(plan));
            }
            begin = end;
        }
    }
    std::stable_sort(plans.begin(), plans.end(),
                     [](const Plan& a, const Plan& b) { return a.gain > b.gain; });
    return plans;
}

// CPU Van Ginneken dynamic programming over the placement Steiner tree.  The
// tree is supplied by OpenROAD's estimator, while candidates and traceback
// remain owned by this module.
void pruneVg(std::vector<VgCandidate>& options) {
    auto max_buffer = std::max_element(
        options.begin(), options.end(), [](const auto& a, const auto& b) {
            return a.tree->bufferCount() < b.tree->bufferCount();
        });
    std::shared_ptr<rsz::BufferedNet> preserve;
    if (max_buffer != options.end()) {
        preserve = max_buffer->tree;
    }
    std::stable_sort(options.begin(), options.end(), [](const auto& a, const auto& b) {
        return a.cap != b.cap ? a.cap < b.cap : a.slack > b.slack;
    });
    std::vector<VgCandidate> frontier;
    double best_slack = -std::numeric_limits<double>::infinity();
    for (auto& option : options) {
        if (option.slack > best_slack + 1e-18) {
            best_slack = option.slack;
            frontier.push_back(std::move(option));
        }
    }
    if (preserve &&
        std::none_of(frontier.begin(), frontier.end(),
                     [&](const auto& option) { return option.tree == preserve; })) {
        for (auto& option : options) {
            if (option.tree == preserve) {
                frontier.push_back(std::move(option));
                break;
            }
        }
    }
    options.swap(frontier);
}

std::vector<VgCandidate> runVg(const rsz::BufferedNetPtr& node, rsz::Resizer* resizer,
                               const sta::Scene* scene, const sta::MinMax* min_max,
                               const VgOptions& options,
                               const DesignConstraints& constraints) {
    if (!node) {
        return {};
    }
    switch (node->type()) {
    case rsz::BufferedNetType::load:
        return {{node, node->cap(), 0}};
    case rsz::BufferedNetType::wire: {
        auto child = runVg(node->ref(), resizer, scene, min_max, options, constraints);
        std::vector<VgCandidate> result;
        double wire_res = 0, wire_cap = 0;
        node->wireRC(scene, resizer, resizer->getEstimateParasitics(), wire_res,
                     wire_cap);
        const double length = resizer->dbuToMeters(node->length());
        const double total_wire_cap = length * wire_cap;
        sta::LibertyCellSeq buffers;
        resizer->getBufferList(buffers);
        if (buffers.empty()) {
            auto* network = resizer->dbNetwork();
            auto* block = resizer->block();
            for (auto* lib : block->getDataBase()->getLibs()) {
                for (auto* master : lib->getMasters()) {
                    auto* cell = network->libertyCell(network->dbToSta(master));
                    if (cell && cell->isBuffer()) {
                        buffers.push_back(cell);
                    }
                }
            }
        }
        for (const auto& option : child) {
            auto wire = std::make_shared<rsz::BufferedNet>(
                rsz::BufferedNetType::wire, node->location(), node->layer(),
                option.tree, scene, resizer, resizer->getEstimateParasitics());
            const double cap = option.cap + total_wire_cap;
            const double delay =
                length * wire_res * (option.cap + total_wire_cap / 2.0);
            result.push_back({wire, cap, option.slack - delay});
            for (auto* buffer : buffers) {
                if (!buffer || !buffer->isBuffer()) {
                    continue;
                }
                auto* master = resizer->dbNetwork()->staToDb(buffer);
                if (std::find(constraints.excluded_masters.begin(),
                              constraints.excluded_masters.end(),
                              master) != constraints.excluded_masters.end()) {
                    continue;
                }
                sta::LibertyPort* input = nullptr;
                sta::LibertyPort* output = nullptr;
                buffer->bufferPorts(input, output);
                if (!input || !output) {
                    continue;
                }
                const auto intrinsic = output->intrinsicDelay(resizer->sta());
                const double input_cap = resizer->portCapacitance(input, scene);
                const double buffer_delay =
                    static_cast<double>(intrinsic) + output->driveResistance() * cap;
                if (!std::isfinite(buffer_delay) || !std::isfinite(input_cap)) {
                    continue;
                }
                auto buffered = std::make_shared<rsz::BufferedNet>(
                    rsz::BufferedNetType::buffer, node->location(), buffer, wire, scene,
                    resizer, resizer->getEstimateParasitics());
                result.push_back(
                    {buffered, input_cap, option.slack - delay - buffer_delay});
            }
        }
        pruneVg(result);
        return result;
    }
    case rsz::BufferedNetType::junction: {
        auto left = runVg(node->ref(), resizer, scene, min_max, options, constraints);
        auto right = runVg(node->ref2(), resizer, scene, min_max, options, constraints);
        std::vector<VgCandidate> result;
        result.reserve(left.size() * right.size());
        for (const auto& l : left) {
            for (const auto& r : right) {
                auto junction = std::make_shared<rsz::BufferedNet>(
                    rsz::BufferedNetType::junction, node->location(), l.tree, r.tree,
                    resizer);
                result.push_back({junction, l.cap + r.cap, std::min(l.slack, r.slack)});
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

int flattenVg(const rsz::BufferedNetPtr& node, VgForest& forest, rsz::Resizer* resizer,
              const sta::Scene* scene) {
    if (!node) {
        throw std::runtime_error("VG CUDA null tree node");
    }
    VgCudaNode flat;
    switch (node->type()) {
    case rsz::BufferedNetType::load:
        flat.cap = node->cap();
        break;
    case rsz::BufferedNetType::wire: {
        flat.type = VgCudaNodeType::wire;
        flat.left = flattenVg(node->ref(), forest, resizer, scene);
        double resistance = 0, capacitance = 0;
        node->wireRC(scene, resizer, resizer->getEstimateParasitics(), resistance,
                     capacitance);
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
    if (forest.nodes.size() >= static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("VG CUDA tree index overflow");
    }
    forest.nodes.push_back(flat);
    forest.originals.push_back(node);
    return static_cast<int>(forest.nodes.size() - 1);
}

void collectCudaBuffers(VgForest& forest, rsz::Resizer* resizer,
                        const sta::Scene* scene, const VgOptions& options,
                        const DesignConstraints& constraints) {
    sta::LibertyCellSeq cells;
    resizer->getBufferList(cells);
    if (cells.empty()) {
        auto* network = resizer->dbNetwork();
        for (auto* lib : resizer->block()->getDataBase()->getLibs()) {
            for (auto* master : lib->getMasters()) {
                auto* cell = network->libertyCell(network->dbToSta(master));
                if (cell && cell->isBuffer()) {
                    cells.push_back(cell);
                }
            }
        }
    }
    for (auto* cell : cells) {
        if (!cell || !cell->isBuffer()) {
            continue;
        }
        auto* master = resizer->dbNetwork()->staToDb(cell);
        if (std::find(constraints.excluded_masters.begin(),
                      constraints.excluded_masters.end(),
                      master) != constraints.excluded_masters.end()) {
            continue;
        }
        sta::LibertyPort *input = nullptr, *output = nullptr;
        cell->bufferPorts(input, output);
        if (!input || !output) {
            continue;
        }
        VgCudaBuffer model{resizer->portCapacitance(input, scene),
                           static_cast<double>(output->intrinsicDelay(resizer->sta())),
                           output->driveResistance()};
        if (!std::isfinite(model.cap) || !std::isfinite(model.intrinsic) ||
            !std::isfinite(model.resistance)) {
            continue;
        }
        forest.buffers.push_back(model);
        forest.cells.push_back(cell);
    }
}

rsz::BufferedNetPtr tracebackVg(int id, int selected, const VgForest& forest,
                                const VgCudaForestResult& result, rsz::Resizer* resizer,
                                const sta::Scene* scene) {
    const auto& node = forest.nodes.at(id);
    const auto& candidate = result.frontiers.at(id).at(selected);
    const auto& original = forest.originals.at(id);
    if (node.type == VgCudaNodeType::load) {
        return original;
    }
    auto left = tracebackVg(node.left, candidate.left, forest, result, resizer, scene);
    if (node.type == VgCudaNodeType::junction) {
        auto right =
            tracebackVg(node.right, candidate.right, forest, result, resizer, scene);
        return std::make_shared<rsz::BufferedNet>(
            rsz::BufferedNetType::junction, original->location(), left, right, resizer);
    }
    auto wire = std::make_shared<rsz::BufferedNet>(
        rsz::BufferedNetType::wire, original->location(), original->layer(), left,
        scene, resizer, resizer->getEstimateParasitics());
    if (candidate.buffer < 0) {
        return wire;
    }
    return std::make_shared<rsz::BufferedNet>(
        rsz::BufferedNetType::buffer, original->location(),
        forest.cells.at(candidate.buffer), wire, scene, resizer,
        resizer->getEstimateParasitics());
}

std::vector<VgSolution> collectVgSolutions(std::span<const NetState> states,
                                           rsz::Resizer* resizer, sta::dbSta* sta,
                                           const VgOptions& options,
                                           VgSolutions& report,
                                           const DesignConstraints& constraints) {
    std::vector<VgSolution> solutions;
    if (!resizer) {
        return solutions;
    }
    resizer->initBlock();
    const auto* scene = sta->cmdScene();
    struct Input {
        odb::dbNet* net;
        rsz::BufferedNetPtr tree;
        double resistance;
        double unchanged_score;
    };
    std::vector<Input> inputs;
    for (const auto& state : states) {
        auto* driver_pin = sta->getDbNetwork()->dbToSta(state.driver);
        auto tree = resizer->makeBufferedNetSteiner(driver_pin, scene);
        if (!tree) {
            continue;
        }
        ++report.nets_collected;
        auto* port = sta->getDbNetwork()->libertyPort(driver_pin);
        if (port) {
            const auto original = unchangedVg(tree, resizer, scene);
            const double resistance = port->driveResistance();
            inputs.push_back({state.net, tree, resistance,
                              original.second - resistance * original.first});
        }
    }
    std::vector<VgSolution> best(inputs.size());
    bool used_cuda = false;
    if (options.backend == ecc::VgBackend::Cuda) {
        try {
            VgForest forest;
            collectCudaBuffers(forest, resizer, scene, options, constraints);
            for (const auto& input : inputs) {
                forest.roots.push_back(flattenVg(input.tree, forest, resizer, scene));
                forest.resistance.push_back(input.resistance);
            }
            const auto result = vgCudaSolve(
                forest.nodes, forest.buffers, forest.roots, forest.resistance,
                size_t(options.cuda_max_memory_mb) * 1024 * 1024);
            for (size_t i = 0; i < inputs.size(); ++i) {
                int selected = result.selected[i];
                if (options.scope == ecc::VgScope::Setup) {
                    selected = -1;
                    double best_score = inputs[i].unchanged_score;
                    const auto& frontier = result.frontiers[forest.roots[i]];
                    for (size_t j = 0; j < frontier.size(); ++j) {
                        const auto& c = frontier[j];
                        const double score = c.slack - inputs[i].resistance * c.cap;
                        if (c.buffers && score > best_score) {
                            best_score = score;
                            selected = int(j);
                        }
                    }
                }
                if (selected < 0) {
                    continue;
                }
                const auto& candidate = result.frontiers[forest.roots[i]][selected];
                best[i] = {inputs[i].net,
                           tracebackVg(forest.roots[i], selected, forest, result,
                                       resizer, scene),
                           candidate.slack - inputs[i].resistance * candidate.cap};
            }
            used_cuda = true;
            report.backend_used = "cuda";
        } catch (const std::exception& error) {
            report.cuda_fallback = error.what();
            best.assign(inputs.size(), VgSolution{});
        }
    }
    if (!used_cuda) {
        report.backend_used = "cpu";
        for (size_t i = 0; i < inputs.size(); ++i) {
            auto candidates = runVg(inputs[i].tree, resizer, scene, sta::MinMax::max(),
                                    options, constraints);
            best[i].net = inputs[i].net;
            if (options.scope == ecc::VgScope::Setup) {
                best[i].score = inputs[i].unchanged_score;
            }
            for (auto& candidate : candidates) {
                if (candidate.tree->bufferCount() == 0) {
                    continue;
                }
                const double score =
                    candidate.slack - inputs[i].resistance * candidate.cap;
                if (score > best[i].score) {
                    best[i].score = score;
                    best[i].tree = std::move(candidate.tree);
                }
            }
        }
    }
    for (auto& solution : best) {
        if (solution.tree && solution.tree->bufferCount()) {
            solutions.push_back(std::move(solution));
        }
    }
    return solutions;
}

// Decode solver topology without touching OpenDB. A returned bundle denotes
// direct terminals and already-decoded child buffers still on the source net.
BufferTreeNode decodeVgNode(const rsz::BufferedNetPtr& node, BufferTreePlan& plan,
                            sta::dbNetwork* network) {
    if (!node) {
        throw std::runtime_error("VG returned a null tree node");
    }
    switch (node->type()) {
    case rsz::BufferedNetType::load: {
        auto* object = network->staToDb(node->loadPin());
        if (!object) {
            throw std::runtime_error("VG tree load has no OpenDB terminal");
        }
        return {.loads = {object}};
    }
    case rsz::BufferedNetType::wire:
    case rsz::BufferedNetType::via:
        return decodeVgNode(node->ref(), plan, network);
    case rsz::BufferedNetType::junction: {
        auto left = decodeVgNode(node->ref(), plan, network);
        auto right = decodeVgNode(node->ref2(), plan, network);
        left.loads.insert(left.loads.end(), right.loads.begin(), right.loads.end());
        left.children.insert(left.children.end(), right.children.begin(),
                             right.children.end());
        return left;
    }
    case rsz::BufferedNetType::buffer: {
        auto buffer = decodeVgNode(node->ref(), plan, network);
        buffer.master = network->staToDb(node->bufferCell());
        buffer.location = {node->location().x(), node->location().y()};
        plan.nodes.push_back(std::move(buffer));
        return {.children = {plan.nodes.size() - 1}};
    }
    }
    throw std::runtime_error("VG returned an unsupported node");
}

} // namespace vg_detail

VgSolutions VanGinneken::solve(odb::dbBlock* block, sta::dbSta* sta,
                               std::span<const detail::NetState> states,
                               rsz::Resizer* resizer, const VgOptions& options,
                               const DesignConstraints& constraints) {
    using namespace vg_detail;
    options.validate();
    if (!block || !sta || sta->getDbNetwork()->block() != block ||
        sta->scenes().size() != 1 || (resizer && resizer->sta() != sta)) {
        throw std::invalid_argument("VG solve requires one live OpenDB/STA scene");
    }
    if (options.scope == VgScope::Setup && !resizer) {
        throw std::invalid_argument("VG setup scope requires the Steiner-tree solver");
    }
    std::unordered_set<odb::dbNet*> seen;
    for (const auto& state : states) {
        if (!state.net || state.net->getBlock() != block || !state.driver ||
            state.driver->getNet() != state.net || !seen.insert(state.net).second) {
            throw std::invalid_argument(
                "VG solve requires unique current net snapshots");
        }
    }
    VgSolutions result;
    if (states.empty()) {
        return result;
    }
    if (resizer) {
        auto solutions =
            collectVgSolutions(states, resizer, sta, options, result, constraints);
        for (const auto& solution : solutions) {
            BufferTreePlan plan;
            plan.net = solution.net;
            decodeVgNode(solution.tree, plan, sta->getDbNetwork());
            result.plans.push_back(std::move(plan));
        }
    } else {
        detail::DesignAnalysis analysis(block, sta, options.common, constraints);
        for (auto& candidate :
             collectPlans(analysis, states, options, result.nets_collected)) {
            BufferTreePlan plan;
            plan.net = candidate.net;
            plan.nodes.push_back(
                {candidate.master, candidate.location, std::move(candidate.loads), {}});
            result.plans.push_back(std::move(plan));
        }
    }
    return result;
}

} // namespace ecc
