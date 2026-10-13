// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_power.h"
#include "repair_backend.h"
#include "repair_acceptance.h"
#include "sta/PowerClass.hh"
#include "power/Power.hh"
#include <chrono>
#include <set>

namespace ecc {
namespace {
struct Candidate {
    odb::dbInst* inst;
    odb::dbMaster* master;
    double saving;
    float slack;
    bool vt;
};
// Characterized leakage is only a ranking estimate. Acceptance uses
// whole-design OpenSTA power including conditional leakage/activity effects.
float leakage(sta::LibertyCell* cell) {
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
std::vector< Candidate > candidates(
    detail::Repair& backend, sta::dbSta* sta,
    const std::vector< detail::NetState >& states,
    const PowerRepairOptions& options) {
    std::vector< Candidate > plans;
    for(const auto& state : states) {
        auto* inst = state.driver->getInst();
        if(backend.protectedInputs(inst))
            continue;
        const auto original = backend.driverModel(state);
        if(!original.cell)
            continue;
        const float old_leakage = leakage(original.cell);
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
            const float new_leakage = leakage(replacement->cell);
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
std::vector< detail::NetState > measure(detail::Repair& backend,
                                        sta::dbSta* sta,
                                        PowerRepairMetrics& metrics) {
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
bool acceptable(const PowerRepairMetrics& before,
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
}  // namespace
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
        auto states = measure(backend, sta, result.before);
        result.after = result.before;
        const auto update = [&]() {
            ++result.refreshes;
            refresh();
        };
        for(unsigned pass = 0; options.enabled && pass < options.max_passes &&
                               result.attempted_batches < options.max_trials;
            ++pass) {
            const auto plans = candidates(backend, sta, states, options);
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
                auto next = measure(backend, sta, after);
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
                states = measure(backend, sta, result.after);
                ++result.rejected_batches;
                rejected.insert(selected);
                backoff();
            }
            if(!accepted)
                break;
        }
        result.ok = true;
        result.message = !options.enabled ? "power recovery disabled"
                         : result.attempted_batches >= options.max_trials
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
