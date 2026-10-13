// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_design.h"
#include "repair_backend.h"
#include "repair_load.h"
#include "repair_acceptance.h"

namespace ecc {
using detail::NetState;
using detail::Repair;

DesignRepairResult OpenDbDesignRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh,
    const DesignRepairOptions& options) {
    DesignRepairResult result;
    try {
        options.validate();
    }
    catch(const std::exception& error) {
        result.message = error.what();
        return result;
    }
    if(!block || !sta || !refresh || sta->scenes().size() != 1 ||
       sta->getDbNetwork()->block() != block) {
        result.message = "invalid repair backend (one STA scene required)";
        return result;
    }
    Repair repair(block, sta, options);
    if(!repair.hasCells()) {
        result.message = "no usable Liberty cell models";
        return result;
    }
    const bool collect_all =
        options.allow_load_downsize && options.max_load_downsizes_per_pass > 0;
    const auto analyze = [&](DesignRepairMetrics& metrics) {
        return repair.analyze(metrics, collect_all);
    };
    const auto violating = [](const DesignRepairMetrics& metrics) {
        return metrics.fanout_nets || metrics.capacitance_nets ||
               metrics.slew_nets;
    };
    auto states = analyze(result.before);
    result.after = result.before;
    bool allow_resize = true;
    bool allow_load_downsize = collect_all;
    for(unsigned pass = 0; pass < options.max_passes && violating(result.after);
        ++pass) {
        const bool overloaded = std::any_of(
            states.begin(), states.end(), [](const NetState& state) {
                return state.fanout_ratio > 1 || state.cap_ratio > 1;
            });
        std::vector< detail::LoadDownsizeCandidate > load_plans;
        if(allow_load_downsize) {
            detail::LoadDownsizePlanner planner(repair, sta, states, options);
            std::unordered_set< odb::dbInst* > selected;
            for(const auto& state : states) {
                if(state.fanout_ratio > 1 ||
                   (overloaded
                        ? state.cap_ratio <= 1
                        : state.slew_ratio <= 1 || state.propagated_slew))
                    continue;
                for(const auto& candidate : planner.candidates(state)) {
                    if(load_plans.size() >= options.max_load_downsizes_per_pass)
                        break;
                    if(selected.insert(candidate.inst).second)
                        load_plans.push_back(candidate);
                }
                if(load_plans.size() >= options.max_load_downsizes_per_pass)
                    break;
            }
        }
        odb::dbDatabase::beginEco(block);
        bool eco_active = true;
        unsigned inserted = 0, resized = 0, resized_gates = 0, downsized = 0,
                 vt_swaps = 0;
        try {
            // Try relief at slack-rich loads before adding drive strength or
            // buffers. All candidates are measured before any ECO mutation.
            if(!load_plans.empty()) {
                for(const auto& plan : load_plans)
                    if(plan.inst->swapMaster(plan.master))
                        ++downsized;
            }
            else {
                for(auto& state : states) {
                    if(!state.violating())
                        continue;
                    const bool load_violation =
                        state.fanout_ratio > 1 || state.cap_ratio > 1;
                    if(overloaded ? !load_violation : state.propagated_slew)
                        continue;
                    repair.edit(
                        state, inserted, resized, resized_gates,
                        options.max_inserted_buffers - result.inserted_buffers,
                        allow_resize, vt_swaps);
                }
            }
            if(inserted == 0 && resized == 0 && resized_gates == 0 &&
               downsized == 0) {
                odb::dbDatabase::undoEco(block);
                eco_active = false;
                result.message = "no feasible edit for remaining violations";
                break;
            }
            if(options.prepare)
                options.prepare();
            refresh();
            DesignRepairMetrics after;
            auto next_states = analyze(after);
            // Accept only measured DRV improvement without setup/hold WNS
            // regression beyond configured tolerance. Never claim convergence
            // just because a batch or insertion budget has been exhausted.
            if(after.severity + options.tolerances.drv_severity <
                   result.after.severity &&
               detail::timingNonRegressing(result.after, after,
                                           options.tolerances) &&
               (!(downsized || vt_swaps) ||
                detail::noNewDrv(result.after, after))) {
                odb::dbDatabase::commitEco(block);
                eco_active = false;
                result.inserted_buffers += inserted;
                result.resized_buffers += resized;
                result.resized_gates += resized_gates;
                result.vt_swaps += vt_swaps;
                result.downsized_loads += downsized;
                allow_load_downsize = collect_all;
                ++result.accepted_passes;
                result.after = after;
                states = std::move(next_states);
            }
            else {
                odb::dbDatabase::undoEco(block);
                eco_active = false;
                refresh();
                states = analyze(result.after);
                ++result.rejected_passes;
                if(downsized) {
                    ++result.rejected_load_batches;
                    allow_load_downsize = false;
                    continue;  // Preserve existing source resize/buffer
                               // fallback.
                }
                if(allow_resize && (resized || resized_gates)) {
                    allow_resize = false;
                    continue;  // Retry with buffering if upsizing hurt upstream
                               // timing.
                }
                result.message =
                    "batch rejected: DRV severity or WNS did not improve";
                break;
            }
        }
        catch(const std::exception& error) {
            result.message = error.what();
            if(eco_active) {
                odb::dbDatabase::undoEco(block);
                try {
                    refresh();
                    analyze(result.after);
                }
                catch(const std::exception& restore_error) {
                    result.message +=
                        std::string(
                            "; timing refresh after rollback failed: ") +
                        restore_error.what();
                }
            }
            return result;
        }
    }
    result.ok = true;
    result.converged = !violating(result.after);
    if(result.message.empty())
        result.message = result.converged
                             ? "eligible nets repaired"
                             : "pass limit reached; violations remain";
    return result;
}
}  // namespace ecc
