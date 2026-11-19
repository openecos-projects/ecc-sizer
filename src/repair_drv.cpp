// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_drv.h"
#include "repair_backend.h"
#include <chrono>
#include <unordered_map>
#include "db_sta/dbNetwork.hh"
#include "sta/Graph.hh"
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <utility>
#include <vector>
#include "odb/db.h"
#include "odb/dbTypes.h"
#include "sta/PowerClass.hh"
#include "power/Power.hh"
#include <set>

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
                    repair.edit(state, inserted, resized, resized_gates,
                                detail::remainingBufferBudget(
                                    options.max_inserted_buffers,
                                    result.inserted_buffers),
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

OrdDesignRepairResult OpenDbOrdDesignRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh,
    const DesignRepairOptions& options) {
    OrdDesignRepairResult result;
    try {
        options.validate();
    }
    catch(const std::exception& error) {
        result.message = error.what();
        return result;
    }
    if(!block || !sta || !refresh || sta->scenes().size() != 1 ||
       sta->getDbNetwork()->block() != block) {
        result.message = "invalid OpenROAD-style repair backend";
        return result;
    }

    detail::Repair backend(block, sta, options);
    if(!backend.hasCells()) {
        result.message = "no usable Liberty cell models";
        return result;
    }

    const auto analyze = [&](DesignRepairMetrics& metrics) {
        return backend.analyze(metrics, true);
    };
    auto states = analyze(result.before);
    result.after = result.before;
    const auto* network = sta->getDbNetwork();

    for(unsigned pass = 0;
        pass < options.max_passes && result.after.severity > 0; ++pass) {
        bool pass_progress = false;
        unsigned batch_limit = options.ord_driver_batch_size;
        std::unordered_set< odb::dbNet* > processed;
        while(true) {
            // RepairDesign walks levelized driver vertices from sinks back to
            // sources. Rebuild the map after each accepted ECO because buffer
            // insertion changes the OpenSTA graph and net ownership.
            std::unordered_map< odb::dbNet*, detail::NetState* > by_net;
            by_net.reserve(states.size());
            for(auto& state : states)
                by_net.emplace(state.net, &state);

            std::vector< odb::dbNet* > selected_nets;
            const auto& drivers = sta->levelizedDrvrVertices();
            for(int index = int(drivers.size()) - 1;
                index >= 0 && selected_nets.size() < batch_limit; --index) {
                auto* vertex = drivers[index];
                if(!vertex || !vertex->pin())
                    continue;
                auto* object = network->staToDb(vertex->pin());
                if(!object || object->getObjectType() != odb::dbITermObj)
                    continue;
                auto* driver = static_cast< odb::dbITerm* >(object);
                auto* net = driver->getNet();
                const auto found = by_net.find(net);
                if(found == by_net.end() || !found->second->violating() ||
                   processed.count(net))
                    continue;
                processed.insert(net);
                selected_nets.push_back(net);
            }
            if(selected_nets.empty())
                break;

            odb::dbDatabase::beginEco(block);
            bool active = true;
            unsigned inserted = 0, resized_buffers = 0, resized_gates = 0,
                     vt_swaps = 0, repaired = 0;
            try {
                for(auto* net : selected_nets) {
                    const auto found = by_net.find(net);
                    if(found == by_net.end())
                        continue;
                    if(backend.edit(*found->second, inserted, resized_buffers,
                                    resized_gates,
                                    detail::remainingBufferBudget(
                                        options.max_inserted_buffers,
                                        result.inserted_buffers),
                                    true, vt_swaps))
                        ++repaired;
                }
                if(repaired == 0) {
                    odb::dbDatabase::undoEco(block);
                    active = false;
                    for(auto* net : selected_nets)
                        processed.erase(net);
                    break;
                }
                if(options.prepare)
                    options.prepare();
                refresh();
                DesignRepairMetrics after;
                auto next_states = analyze(after);
                // OpenROAD repair_design commits a DRV-improving driver edit;
                // setup/hold protection belongs to repair_timing.  Keeping
                // this acceptance rule separate is essential for a faithful
                // A/B reference and is the main difference from Native's
                // timing-preserving transaction policy.
                if(after.severity + options.tolerances.drv_severity <
                   result.after.severity) {
                    odb::dbDatabase::commitEco(block);
                    active = false;
                    result.after = after;
                    states = std::move(next_states);
                    result.repaired_nets += repaired;
                    result.inserted_buffers += inserted;
                    result.resized_buffers += resized_buffers;
                    result.resized_gates += resized_gates;
                    result.vt_swaps += vt_swaps;
                    pass_progress = true;
                    continue;
                }
                odb::dbDatabase::undoEco(block);
                active = false;
                refresh();
                states = analyze(result.after);
                for(auto* net : selected_nets)
                    processed.erase(net);
                ++result.rejected_passes;
                if(batch_limit > 1) {
                    batch_limit = std::max(1u, batch_limit / 2);
                    continue;
                }
                result.message =
                    "OpenROAD-style batch rejected: DRV or timing regressed";
                break;
            }
            catch(const std::exception& error) {
                result.message = error.what();
                if(active) {
                    odb::dbDatabase::undoEco(block);
                    for(auto* net : selected_nets)
                        processed.erase(net);
                    try {
                        refresh();
                        states = analyze(result.after);
                    }
                    catch(const std::exception& restore_error) {
                        result.message +=
                            std::string("; refresh after rollback failed: ") +
                            restore_error.what();
                    }
                }
                return result;
            }
            break;
        }
        if(pass_progress)
            ++result.passes;
        else if(result.message.empty())
            result.message = "no feasible OpenROAD-style edit";
        if(!pass_progress)
            break;
    }

    result.ok = true;
    result.converged = result.after.severity <= options.tolerances.drv_severity;
    if(result.message.empty())
        result.message =
            result.converged
                ? "OpenROAD-style DRV repair converged"
                : "OpenROAD-style pass limit reached; violations remain";
    return result;
}

static FanoutRepairResult failure(FanoutRepairStatus status,
                                  std::string message) {
    FanoutRepairResult result;
    result.status = status;
    result.message = std::move(message);
    return result;
}

static bool isLoad(odb::dbObject* term) {
    if(term == nullptr) {
        return false;
    }
    if(term->getObjectType() == odb::dbITermObj) {
        const odb::dbIoType io_type =
            static_cast< odb::dbITerm* >(term)->getIoType();
        return io_type == odb::dbIoType::INPUT ||
               io_type == odb::dbIoType::INOUT;
    }
    if(term->getObjectType() == odb::dbBTermObj) {
        const odb::dbIoType io_type =
            static_cast< odb::dbBTerm* >(term)->getIoType();
        return io_type == odb::dbIoType::OUTPUT ||
               io_type == odb::dbIoType::INOUT;
    }
    return false;
}

static std::vector< odb::dbObject* > loads(odb::dbNet* net) {
    std::vector< odb::dbObject* > result;
    if(net == nullptr) {
        return result;
    }
    result.reserve(net->getITerms().size() + net->getBTerms().size());
    for(odb::dbITerm* term : net->getITerms()) {
        if(isLoad(term)) {
            result.push_back(term);
        }
    }
    for(odb::dbBTerm* term : net->getBTerms()) {
        if(isLoad(term)) {
            result.push_back(term);
        }
    }
    return result;
}

static bool shouldRepair(
    odb::dbNet* net, bool repair_clock_nets,
    const std::unordered_set< odb::dbNet* >* excluded_nets = nullptr) {
    if(net == nullptr || net->isDoNotTouch() || net->isSpecial() ||
       (excluded_nets != nullptr && excluded_nets->count(net) > 0)) {
        return false;
    }
    const odb::dbSigType signal_type = net->getSigType();
    if(signal_type == odb::dbSigType::POWER ||
       signal_type == odb::dbSigType::GROUND) {
        return false;
    }
    return repair_clock_nets || signal_type != odb::dbSigType::CLOCK;
}

static unsigned maxFanoutWithPolicy(
    odb::dbBlock* block, bool include_clock_nets,
    const std::unordered_set< odb::dbNet* >* excluded_nets) {
    unsigned max_fanout = 0;
    if(block == nullptr) {
        return max_fanout;
    }
    for(odb::dbNet* net : block->getNets()) {
        if(shouldRepair(net, include_clock_nets, excluded_nets)) {
            max_fanout =
                std::max(max_fanout, OpenDbFanoutRepairer::loadCount(net));
        }
    }
    return max_fanout;
}

static bool isSimpleBuffer(odb::dbMaster* master) {
    if(master == nullptr || !master->isFrozen()) {
        return false;
    }
    unsigned input_count = 0;
    unsigned output_count = 0;
    for(odb::dbMTerm* term : master->getMTerms()) {
        if(term->getIoType() == odb::dbIoType::INPUT) {
            ++input_count;
        }
        else if(term->getIoType() == odb::dbIoType::OUTPUT) {
            ++output_count;
        }
    }
    return input_count == 1 && output_count == 1;
}

static void addAffectedNet(FanoutRepairResult& result,
                           std::unordered_set< odb::dbNet* >& seen,
                           odb::dbNet* net) {
    if(net != nullptr && seen.insert(net).second) {
        result.affected_nets.push_back(net);
    }
}

unsigned OpenDbFanoutRepairer::loadCount(odb::dbNet* net) {
    return loads(net).size();
}

unsigned OpenDbFanoutRepairer::maxFanout(odb::dbBlock* block,
                                         bool include_clock_nets) {
    return maxFanoutWithPolicy(block, include_clock_nets, nullptr);
}

FanoutRepairResult OpenDbFanoutRepairer::repair(
    odb::dbBlock* block, odb::dbMaster* buffer_master,
    const FanoutRepairOptions& options) {
    if(block == nullptr) {
        return failure(FanoutRepairStatus::kInvalidBlock,
                       "fanout repair requires a block");
    }
    if(!isSimpleBuffer(buffer_master)) {
        return failure(FanoutRepairStatus::kInvalidBufferMaster,
                       "fanout repair requires a frozen buffer master with "
                       "one input and one output");
    }
    if(options.max_fanout < 2) {
        return failure(FanoutRepairStatus::kInvalidMaxFanout,
                       "max fanout must be at least two");
    }

    FanoutRepairResult result;
    result.status = FanoutRepairStatus::kSuccess;
    const std::unordered_set< odb::dbNet* > excluded_nets(
        options.excluded_nets.begin(), options.excluded_nets.end());
    result.before_max_fanout =
        maxFanoutWithPolicy(block, options.repair_clock_nets, &excluded_nets);
    std::unordered_set< odb::dbNet* > seen_affected_nets;

    // Keep the tree construction atomic. OpenDB supports nested ECO journals,
    // so this also composes with a caller-owned transaction.
    odb::dbDatabase::beginEco(block);
    const auto rollback = [&](FanoutRepairStatus status,
                              const std::string& message) {
        odb::dbDatabase::undoEco(block);
        result.status = status;
        result.message = message;
        result.inserted_buffers.clear();
        result.affected_nets.clear();
        result.after_max_fanout = maxFanoutWithPolicy(
            block, options.repair_clock_nets, &excluded_nets);
        return result;
    };

    std::vector< odb::dbNet* > candidate_nets;
    candidate_nets.reserve(block->getNets().size());
    for(odb::dbNet* net : block->getNets()) {
        if(shouldRepair(net, options.repair_clock_nets, &excluded_nets)) {
            candidate_nets.push_back(net);
        }
    }

    for(odb::dbNet* net : candidate_nets) {
        ++result.nets_examined;
        std::vector< odb::dbObject* > net_loads = loads(net);
        if(net_loads.size() <= options.max_fanout) {
            continue;
        }
        ++result.violating_nets;

        struct TreeNode {
            odb::dbObject* term;
            unsigned depth;
        };
        std::deque< TreeNode > root_loads;
        for(odb::dbObject* load : net_loads) {
            root_loads.push_back({load, 0});
        }

        // Each insertion reduces the root fanout by at most K - 1. Always
        // take that full reduction except for the last insertion, which
        // reaches K exactly. Taking the shallowest nodes first also keeps the
        // resulting minimum-size tree balanced.
        while(root_loads.size() > options.max_fanout) {
            const size_t excess = root_loads.size() - options.max_fanout;
            const size_t group_size =
                std::min(static_cast< size_t >(options.max_fanout), excess + 1);
            std::vector< odb::dbObject* > group;
            group.reserve(group_size);
            unsigned group_depth = 0;
            for(size_t index = 0; index < group_size; ++index) {
                group.push_back(root_loads.front().term);
                group_depth = std::max(group_depth, root_loads.front().depth);
                root_loads.pop_front();
            }
            const unsigned buffer_depth = group_depth + 1;
            if(buffer_depth > options.max_levels) {
                return rollback(
                    FanoutRepairStatus::kMaxLevelsExceeded,
                    "fanout repair exceeded the maximum tree depth");
            }

            BufferInsertionResult insertion =
                OpenDbBufferInserter::insertBufferBeforeLoads(
                    net, group, buffer_master, options.insertion);
            if(!insertion) {
                return rollback(FanoutRepairStatus::kInsertionFailed,
                                insertion.message);
            }
            result.inserted_buffers.push_back(insertion.buffer);
            for(odb::dbNet* affected : insertion.affected_nets) {
                addAffectedNet(result, seen_affected_nets, affected);
            }
            root_loads.push_back({insertion.input_pin, buffer_depth});
        }
    }

    result.after_max_fanout =
        maxFanoutWithPolicy(block, options.repair_clock_nets, &excluded_nets);
    if(result.after_max_fanout > options.max_fanout) {
        return rollback(FanoutRepairStatus::kInsertionFailed,
                        "fanout repair finished with an unresolved violation");
    }
    odb::dbDatabase::commitEco(block);
    return result;
}

struct PowerCandidate {
    odb::dbInst* inst;
    odb::dbMaster* master;
    double saving;
    float slack;
    bool vt;
};
// Characterized leakage is only a ranking estimate. Acceptance uses
// whole-design OpenSTA power including conditional leakage/activity effects.
static float rankLeakage(sta::LibertyCell* cell) {
    // Conditional tables (e.g. ASAP7) often have no cell_leakage_power.
    // Use the sum of per-supply maxima as a cheap, activity-independent rank.
    // This is not reported as measured leakage or used for acceptance.
    if(!cell->leakagePowers().empty()) {
        std::unordered_map< const sta::LibertyPort*, float > peaks;
        for(const auto& entry : cell->leakagePowers()) {
            if(!std::isfinite(entry.power()) || entry.power() < 0)
                return -1;
            auto& peak = peaks[entry.relatedPgPort()];
            peak = std::max(peak, entry.power());
        }
        float sum = 0;
        for(const auto& entry : peaks)
            sum += entry.second;
        return sum;
    }
    float value = 0;
    bool exists = false;
    cell->leakagePower(value, exists);
    return exists && std::isfinite(value) && value >= 0 ? value : -1;
}
static std::vector< PowerCandidate > powerCandidates(
    detail::Repair& backend, sta::dbSta* sta,
    const std::vector< detail::NetState >& states,
    const PowerRepairOptions& options) {
    std::vector< PowerCandidate > plans;
    for(const auto& state : states) {
        auto* inst = state.driver->getInst();
        if(backend.protectedInputs(inst))
            continue;
        const auto original = backend.driverModel(state);
        if(!original.cell)
            continue;
        const float old_leakage = rankLeakage(original.cell);
        if(old_leakage <= 0)
            continue;
        const float slack = float(sta->slack(
            sta->getDbNetwork()->dbToSta(state.driver),
            sta::RiseFallBoth::riseFall(), sta->scenes(), sta::MinMax::max()));
        // Unconstrained outputs have no verified timing budget.
        if(!std::isfinite(slack) || slack >= sta::INF ||
           slack < options.slack_margin)
            continue;
        const float old_delay = backend.predictedDelay(
            original, state.capacitance, state.input_slew);
        auto replacements = backend.resizeCandidates(state, true);
        const auto vt = backend.resizeCandidates(state, false, true);
        replacements.insert(replacements.end(), vt.begin(), vt.end());
        for(const auto* replacement : replacements) {
            const float new_leakage = rankLeakage(replacement->cell);
            if(new_leakage < 0 || new_leakage >= old_leakage)
                continue;
            const float delay = backend.predictedDelay(
                *replacement, state.capacitance, state.input_slew);
            if(!std::isfinite(old_delay) || !std::isfinite(delay) ||
               slack < options.slack_margin + std::max(0.0f, delay - old_delay))
                continue;
            plans.push_back(
                {inst, replacement->master, old_leakage - new_leakage, slack,
                 backend.isVtSwap(inst->getMaster(), replacement->master)});
        }
    }
    std::sort(plans.begin(), plans.end(), [](const auto& a, const auto& b) {
        if(a.saving != b.saving)
            return a.saving > b.saving;
        if(a.slack != b.slack)
            return a.slack > b.slack;
        if(a.inst->getName() != b.inst->getName())
            return a.inst->getName() < b.inst->getName();
        return a.master->getName() < b.master->getName();
    });
    if(plans.size() > options.max_candidates)
        plans.resize(options.max_candidates);
    return plans;
}
static std::vector< detail::NetState > measurePower(
    detail::Repair& backend, sta::dbSta* sta, PowerRepairMetrics& metrics) {
    auto states = backend.analyze(metrics.design, true);
    sta::PowerResult total, seq, comb, clock, macro, pad;
    // This vendored STA's powerInvalid() clears the instance map without
    // invalidating its validity flag. activitiesInvalid() invalidates both
    // activity and instance power caches and preserves user annotations.
    sta->power()->activitiesInvalid();
    sta->power(*sta->scenes().begin(), total, seq, comb, clock, macro, pad);
    metrics.leakage = total.leakage();
    metrics.internal = total.internal();
    metrics.switching = total.switching();
    metrics.total = total.total();
    for(const double value :
        {metrics.leakage, metrics.internal, metrics.switching, metrics.total})
        if(!std::isfinite(value) || value < 0)
            throw std::runtime_error("invalid STA power result");
    return states;
}
static bool acceptable(const PowerRepairMetrics& before,
                       const PowerRepairMetrics& after,
                       const PowerRepairOptions& options) {
    const double old_power =
        options.objective == "total" ? before.total : before.leakage;
    const double new_power =
        options.objective == "total" ? after.total : after.leakage;
    const double minimum =
        std::max(options.min_absolute_improvement,
                 old_power * options.min_relative_improvement);
    return new_power < old_power - minimum &&
           after.design.area <= before.design.area &&
           detail::timingNonRegressing(before.design, after.design,
                                       options.common.tolerances) &&
           detail::noNewDrv(before.design, after.design) &&
           after.design.severity <=
               before.design.severity + options.common.tolerances.drv_severity;
}
PowerRepairResult OpenDbPowerRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta,
    const std::function< void() >& refresh, const PowerRepairOptions& options) {
    PowerRepairResult result;
    const auto start = std::chrono::steady_clock::now();
    bool active = false;
    try {
        options.validate();
        if(!block || !sta || !refresh ||
           sta->getDbNetwork()->block() != block || sta->scenes().size() != 1)
            throw std::invalid_argument(
                "invalid power backend (one STA scene required)");
        detail::Repair backend(block, sta, options.common);
        auto states = measurePower(backend, sta, result.before);
        result.after = result.before;
        const auto update = [&]() {
            ++result.refreshes;
            refresh();
        };
        for(unsigned pass = 0; pass < options.max_passes &&
                               result.attempted_batches < options.max_trials;
            ++pass) {
            const auto plans = powerCandidates(backend, sta, states, options);
            size_t cursor = 0,
                   width = std::min(size_t(options.batch_size), plans.size());
            std::set< std::vector< size_t > > rejected;
            bool accepted = false;
            const auto backoff = [&]() {
                if(width > 1)
                    width = std::min(
                        width - 1,
                        std::max(size_t(1),
                                 size_t(width * options.batch_backoff)));
                else {
                    ++cursor;
                    width = std::min(size_t(options.batch_size),
                                     plans.size() - cursor);
                }
            };
            while(cursor < plans.size() &&
                  result.attempted_batches < options.max_trials) {
                std::vector< size_t > selected;
                std::unordered_set< odb::dbInst* > touched;
                for(size_t i = cursor;
                    i < std::min(cursor + width, plans.size()); ++i)
                    if(touched.insert(plans[i].inst).second)
                        selected.push_back(i);
                if(rejected.count(selected)) {
                    backoff();
                    continue;
                }
                odb::dbDatabase::beginEco(block);
                active = true;
                unsigned vt_swaps = 0, downsizes = 0;
                for(const auto i : selected) {
                    const auto& plan = plans[i];
                    if(!plan.inst->swapMaster(plan.master))
                        throw std::runtime_error("power cell swap failed");
                    if(plan.vt)
                        ++vt_swaps;
                    else
                        ++downsizes;
                }
                ++result.attempted_batches;
                if(options.common.prepare)
                    options.common.prepare();
                update();
                PowerRepairMetrics after;
                auto next = measurePower(backend, sta, after);
                bool local_slack_ok = true;
                for(const auto i : selected) {
                    for(auto* pin : plans[i].inst->getITerms()) {
                        if(pin->getSigType() != odb::dbSigType::SIGNAL ||
                           pin->getIoType() != odb::dbIoType::OUTPUT)
                            continue;
                        const float slack = float(
                            sta->slack(sta->getDbNetwork()->dbToSta(pin),
                                       sta::RiseFallBoth::riseFall(),
                                       sta->scenes(), sta::MinMax::max()));
                        local_slack_ok &= std::isfinite(slack) &&
                                          slack < sta::INF &&
                                          slack >= options.slack_margin;
                    }
                }
                if(local_slack_ok && acceptable(result.after, after, options)) {
                    odb::dbDatabase::commitEco(block);
                    active = false;
                    result.after = after;
                    states = std::move(next);
                    ++result.accepted_batches;
                    result.vt_swaps += vt_swaps;
                    result.downsized_cells += downsizes;
                    accepted = true;
                    break;
                }
                odb::dbDatabase::undoEco(block);
                active = false;
                update();  // Always restore current timing/power before
                           // returning or replanning.
                states = measurePower(backend, sta, result.after);
                ++result.rejected_batches;
                rejected.insert(selected);
                backoff();
            }
            if(!accepted)
                break;
        }
        result.ok = true;
        result.message = result.attempted_batches >= options.max_trials
                             ? "power trial budget exhausted"
                         : result.accepted_batches >= options.max_passes
                             ? "power pass budget exhausted"
                             : "no further acceptable power candidate";
    }
    catch(const std::exception& error) {
        result.message = error.what();
        if(active) {
            odb::dbDatabase::undoEco(block);
            try {
                ++result.refreshes;
                refresh();
                sta->power()->activitiesInvalid();
            }
            catch(const std::exception& restore) {
                result.message +=
                    std::string("; rollback refresh failed: ") + restore.what();
            }
        }
    }
    result.elapsed_ms = std::chrono::duration< double, std::milli >(
                            std::chrono::steady_clock::now() - start)
                            .count();
    return result;
}
}  // namespace ecc

namespace ecc::detail {
LoadDownsizePlanner::LoadDownsizePlanner(Repair& backend, sta::dbSta* sta,
                                         const std::vector< NetState >& states,
                                         const RepairCommonOptions& options)
    : backend_(backend), sta_(sta), options_(options) {
    if(options.allow_load_downsize)
        for(const auto& state : states)
            outputs_.emplace(state.driver->getInst(), &state);
}

std::vector< LoadDownsizeCandidate > LoadDownsizePlanner::candidates(
    const NetState& source,
    const std::unordered_set< odb::dbObject* >& protected_loads) {
    std::vector< LoadDownsizeCandidate > result;
    if(!options_.allow_load_downsize)
        return result;
    auto* network = sta_->getDbNetwork();
    std::unordered_set< odb::dbInst* > protected_instances;
    for(auto* pin : protected_loads)
        if(pin->getObjectType() == odb::dbITermObj)
            protected_instances.insert(
                static_cast< odb::dbITerm* >(pin)->getInst());
    std::unordered_set< odb::dbInst* > seen;
    const auto driver_model = backend_.driverModel(source);
    const float source_delay = backend_.predictedDelay(
        driver_model, source.capacitance, source.input_slew);
    for(const auto& load : source.loads) {
        if(load.term->getObjectType() != odb::dbITermObj)
            continue;
        auto* inst = static_cast< odb::dbITerm* >(load.term)->getInst();
        if(inst == source.driver->getInst() ||
           protected_instances.count(inst) || !seen.insert(inst).second)
            continue;
        const auto found = outputs_.find(inst);
        if(found == outputs_.end())
            continue;  // Includes protected/fixed-function/undriven output
                       // nets.
        const auto& output = *found->second;
        bool protected_net = false;
        for(auto* pin : inst->getITerms()) {
            if(pin->getSigType() == odb::dbSigType::POWER ||
               pin->getSigType() == odb::dbSigType::GROUND)
                continue;
            auto* net = pin->getNet();
            if(!net || pin->getIoType() == odb::dbIoType::INOUT ||
               net->isDoNotTouch() || net->isSpecial() ||
               sta_->isClock(network->dbToSta(net), sta_->cmdMode()))
                protected_net = true;
        }
        if(protected_net)
            continue;
        const auto replacements = backend_.resizeCandidates(output, true);
        if(replacements.empty())
            continue;
        const float slack = float(sta_->slack(
            network->dbToSta(output.driver), sta::RiseFallBoth::riseFall(),
            sta_->scenes(), sta::MinMax::max()));
        if(std::isnan(slack) || slack < options_.load_downsize_slack_margin)
            continue;
        const auto original = backend_.driverModel(output);
        const float old_delay = backend_.predictedDelay(
            original, output.capacitance, output.input_slew);
        if(!std::isfinite(old_delay))
            continue;
        for(const auto* replacement : replacements) {
            float reduction = 0;
            bool increases_input_cap = false;
            // Sum all pins of the same cell on this source, avoiding double
            // counting tied inputs. Do not increase another input net's load.
            for(auto* pin : inst->getITerms()) {
                if(pin->getIoType() != odb::dbIoType::INPUT)
                    continue;
                const auto* old_port =
                    network->libertyPort(network->dbToSta(pin));
                const auto* new_port = replacement->cell->findLibertyPort(
                    pin->getMTerm()->getConstName());
                if(!old_port || !new_port) {
                    increases_input_cap = true;
                    break;
                }
                const float delta = old_port->capacitance(sta::MinMax::max()) -
                                    new_port->capacitance(sta::MinMax::max());
                increases_input_cap |= delta < 0;
                if(pin->getNet() == source.net)
                    reduction += delta;
            }
            if(increases_input_cap || reduction <= 0)
                continue;
            const float new_delay = backend_.predictedDelay(
                *replacement, output.capacitance, output.input_slew);
            const float penalty = std::max(0.0f, new_delay - old_delay);
            if(!std::isfinite(new_delay) ||
               slack < penalty + options_.load_downsize_slack_margin)
                continue;
            const float after = backend_.predictedDelay(
                driver_model, std::max(0.0f, source.capacitance - reduction),
                source.input_slew);
            const float gain = source_delay - after;
            if(!std::isfinite(gain) || gain <= 0)
                continue;
            result.push_back({inst, replacement->master, reduction, gain,
                              slack - penalty,
                              replacement->area - original.area});
        }
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        if(a.cap_reduction != b.cap_reduction)
            return a.cap_reduction > b.cap_reduction;
        if(a.slack != b.slack)
            return a.slack > b.slack;
        if(a.added_area != b.added_area)
            return a.added_area < b.added_area;
        if(a.inst->getName() != b.inst->getName())
            return a.inst->getName() < b.inst->getName();
        return a.master->getName() < b.master->getName();
    });
    return result;
}
}  // namespace ecc::detail
