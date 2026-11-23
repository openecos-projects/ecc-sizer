// SPDX-License-Identifier: Apache-2.0
#include "repair/buffer.h"

#include "actions/netlist.h"
#include "algorithms/gobi.h"
#include "algorithms/van_ginneken.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"

#include <chrono>
#include <optional>
#include <stdexcept>
#include <type_traits>

namespace ecc {
namespace buffer_detail {
using detail::NetState;

bool eligible(const NetState& state, sta::dbSta* sta, const VgOptions& options) {
    if (!state.driver) {
        return false;
    }
    if (state.violating()) {
        return true;
    }
    if (options.scope != ecc::VgScope::Setup) {
        return false;
    }
    auto* pin = sta->getDbNetwork()->dbToSta(state.driver);
    return sta->slack(pin, sta::RiseFallBoth::riseFall(),
                      sta::SceneSeq{sta->cmdScene()}, sta::MinMax::max()) < 0;
}

struct Timer {
    double* elapsed;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    ~Timer() {
        *elapsed += std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start)
                        .count();
    }
};

bool better(const DesignMetrics& before, const DesignMetrics& after,
            const RepairTolerances& tolerance, bool require_drv_reduction) {
    const bool no_timing_loss =
        after.setup_wns + tolerance.wns_absolute >= before.setup_wns &&
        after.setup_tns + tolerance.tns_absolute >= before.setup_tns;
    const bool no_drv_loss =
        after.total_slew_violations <= before.total_slew_violations &&
        after.total_cap_violations <= before.total_cap_violations &&
        after.total_fanout_violations <= before.total_fanout_violations;
    if (!no_timing_loss || (require_drv_reduction && !no_drv_loss)) {
        return false;
    }
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

bool eligible(const NetState& state, sta::dbSta*, const GobiOptions&) {
    return state.driver && state.violating();
}

template <typename Options, typename Solve>
BufferRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                       const std::function<void()>& refresh, rsz::Resizer* resizer,
                       const Options& options, const RepairRuntime& runtime,
                       const std::string& name, Solve solve) {
    BufferRepairResult result;
    if constexpr (std::is_same_v<Options, GobiOptions>) {
        result.backend_used = "cuda";
    }
    std::optional<EcoTransaction> transaction;
    try {
        options.validate();
        if (!block || !sta || !refresh || sta->getDbNetwork()->block() != block ||
            sta->scenes().size() != 1) {
            throw std::invalid_argument(name +
                                        " requires block, one STA scene and refresh");
        }
        if constexpr (std::is_same_v<Options, VgOptions>) {
            if (options.scope == VgScope::Setup && !resizer) {
                throw std::invalid_argument(
                    "VG setup scope requires the Steiner-tree solver");
            }
        } else if (!resizer) {
            throw std::invalid_argument("GOBI requires a resizer");
        }
        detail::DesignAnalysis analysis(block, sta, options.common,
                                        runtime.constraints);
        if (!analysis.hasCells()) {
            throw std::runtime_error(name + " found no usable combinational cells");
        }
        std::vector<NetState> states;
        {
            Timer timer{&result.collect_ms};
            for (auto& state : analysis.analyze(result.before, true)) {
                if (!eligible(state, sta, options)) {
                    continue;
                }
                if (!resizer && state.loads.size() < 2) {
                    continue;
                }
                if (options.max_nets && states.size() >= options.max_nets) {
                    break;
                }
                states.push_back(std::move(state));
            }
        }
        result.after = result.before;
        if (!options.common.actions.buffer ||
            remainingBufferBudget(options.max_buffers, 0) == 0) {
            result.ok = true;
            result.message = "no buffer insertion allowed by actions or count limit";
            return result;
        }
        decltype(solve(block, sta, states, resizer, options,
                       runtime.constraints)) solutions;
        {
            Timer timer{&result.solve_ms};
            solutions =
                solve(block, sta, states, resizer, options, runtime.constraints);
        }
        result.nets_collected = solutions.nets_collected;
        if constexpr (std::is_same_v<Options, VgOptions>) {
            result.backend_used = solutions.backend_used;
            result.cuda_fallback = std::move(solutions.cuda_fallback);
        }
        std::vector<std::size_t> counts;
        counts.reserve(solutions.plans.size());
        for (const auto& plan : solutions.plans) {
            counts.push_back(plan.nodes.size());
        }
        const auto selected = selectWholeBufferPlans(counts, options.max_buffers);
        result.plans_collected = selected.size();
        if (selected.empty()) {
            result.ok = true;
            result.converged =
                result.before.severity <= options.common.tolerances.drv_severity;
            result.message = name + " found no applicable buffer plan";
            return result;
        }
        transaction.emplace(block);
        {
            Timer timer{&result.apply_ms};
            BufferInsertionOptions naming;
            naming.buffer_name_prefix = "sizer_" + name + "_buf";
            naming.net_name_prefix = "sizer_" + name + "_net";
            for (auto i : selected) {
                result.attempted_buffers +=
                    applyBufferTree(solutions.plans[i], naming).insertions.size();
            }
        }
        refresh();
        ++result.refreshes;
        detail::DesignAnalysis verify(block, sta, options.common, runtime.constraints);
        verify.analyze(result.after, true);
        if (!better(result.before, result.after, options.common.tolerances,
                    options.require_drv_reduction)) {
            transaction->rollback();
            refresh();
            ++result.refreshes;
            result.after = result.before;
            result.ok = true;
            result.message = name + " plan rejected by timing/DRV acceptance";
            return result;
        }
        transaction->commit();
        result.ok = true;
        result.committed = true;
        result.inserted_buffers = result.attempted_buffers;
        result.converged =
            result.after.severity <= options.common.tolerances.drv_severity;
        result.message = name + " one-shot plan applied";
    } catch (const std::exception& error) {
        result.message = error.what();
        if (transaction && transaction->active()) {
            transaction->rollback();
            try {
                refresh();
                ++result.refreshes;
            } catch (const std::exception& restore_error) {
                result.message +=
                    std::string("; rollback refresh failed: ") + restore_error.what();
            }
        }
        result.after = result.before;
    }
    return result;
}
} // namespace buffer_detail

BufferRepairResult VgRepairer::run(odb::dbBlock* block, sta::dbSta* sta,
                                   const std::function<void()>& refresh,
                                   const VgOptions& options,
                                   const RepairRuntime& runtime) {
    return run(block, sta, refresh, nullptr, options, runtime);
}

BufferRepairResult VgRepairer::run(odb::dbBlock* block, sta::dbSta* sta,
                                   const std::function<void()>& refresh,
                                   rsz::Resizer* resizer, const VgOptions& options,
                                   const RepairRuntime& runtime) {
    return buffer_detail::run(block, sta, refresh, resizer, options, runtime, "vg",
                              VanGinneken::solve);
}

BufferRepairResult GobiRepairer::run(odb::dbBlock* block, sta::dbSta* sta,
                                     const std::function<void()>& refresh,
                                     rsz::Resizer* resizer, const GobiOptions& options,
                                     const RepairRuntime& runtime) {
    return buffer_detail::run(block, sta, refresh, resizer, options, runtime, "gobi",
                              Gobi::solve);
}
} // namespace ecc
