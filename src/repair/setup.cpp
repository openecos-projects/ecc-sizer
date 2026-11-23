// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair/setup.h"
#include "design_analysis.h"
#include "algorithms/cell_candidates.h"
#include "actions/netlist.h"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "odb/db.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"

#include <chrono>
#include <algorithm>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "sta/Path.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"

namespace ecc {
using detail::CellModel;
using detail::DesignAnalysis;
using detail::NetState;

// Wall-clock phase accounting includes exceptions and excludes logging.
struct SetupPhaseTimer {
    double& elapsed;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    ~SetupPhaseTimer() {
        elapsed += std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - start)
                       .count();
    }
};

struct CriticalNet {
    float slack = 0;
    std::unordered_set<odb::dbObject*> loads;
};
struct SetupCandidate {
    odb::dbNet* net;
    odb::dbInst* driver;
    odb::dbMaster* master;
    std::vector<odb::dbObject*> loads; // Empty means resize the driver.
    BufferLocation location;
    double added_area;
    float gain;
    float slack;
    bool downsize_load = false;
};

static std::vector<SetupCandidate> setupCandidates(DesignAnalysis& analysis,
                                                   sta::dbSta* sta,
                                                   const std::vector<NetState>& states,
                                                   const SetupRepairOptions& options) {
    detail::CellCandidates cells(analysis, sta, options.common);
    auto* network = sta->getDbNetwork();
    sta::StringSeq groups;
    const auto ends = sta->findPathEnds(
        nullptr, nullptr, nullptr, false, sta->scenes(), sta::MinMaxAll::max(),
        options.max_path_ends, options.paths_per_endpoint, true, false, -detail::kInf,
        0, true, groups, true, false, false, false, false, false);
    std::unordered_map<odb::dbNet*, CriticalNet> critical;
    unsigned used_ends = 0;
    for (auto* end : ends) {
        if (used_ends++ >= options.max_path_ends) {
            break;
        }
        const float slack = float(end->slack(sta));
        if (slack >= 0) {
            continue;
        }
        sta::PathExpanded expanded(end->path(), sta);
        for (size_t i = expanded.startIndex(); i + 1 < expanded.size(); ++i) {
            auto* object = network->staToDb(expanded.path(i)->pin(sta));
            if (!object || object->getObjectType() != odb::dbITermObj) {
                continue;
            }
            auto* output = static_cast<odb::dbITerm*>(object);
            if (output->getIoType() != odb::dbIoType::OUTPUT || !output->getNet()) {
                continue;
            }
            auto* load_pin = expanded.path(i + 1)->pin(sta);
            auto* load = network->staToDb(load_pin);
            if (!load) {
                continue;
            }
            // Top-level output pins use a terminal connection in STA; compare
            // their actual OpenDB net, just as for internal instance loads.
            auto* load_net = load->getObjectType() == odb::dbBTermObj
                                 ? static_cast<odb::dbBTerm*>(load)->getNet()
                             : load->getObjectType() == odb::dbITermObj
                                 ? static_cast<odb::dbITerm*>(load)->getNet()
                                 : nullptr;
            if (load_net != output->getNet()) {
                continue;
            }
            auto& entry = critical[output->getNet()];
            entry.slack = std::min(entry.slack, slack);
            entry.loads.insert(load);
        }
    }
    std::vector<SetupCandidate> result;
    detail::LoadDownsizePlanner load_planner(analysis, sta, states, options.common);
    for (const auto& state : states) {
        const auto found = critical.find(state.net);
        if (found == critical.end()) {
            continue;
        }
        const auto original = analysis.driverModel(state);
        if (!original.cell || original.cell->hasSequentials() ||
            original.cell->isClockGate() || original.cell->isIsolationCell() ||
            original.cell->isLevelShifter()) {
            continue;
        }
        const float before =
            analysis.predictedDelay(original, state.capacitance, state.input_slew);
        if (!std::isfinite(before)) {
            continue;
        }
        bool protected_input = false;
        for (auto* pin : state.driver->getInst()->getITerms()) {
            if (pin->getIoType() == odb::dbIoType::INPUT && pin->getNet() &&
                (pin->getNet()->isDoNotTouch() || pin->getNet()->isSpecial() ||
                 sta->isClock(network->dbToSta(pin->getNet()), sta->cmdMode()))) {
                protected_input = true;
            }
        }
        if (!protected_input && options.common.actions.resize) {
            auto replacements = cells.resize(state);
            const auto vt = cells.resize(state, false, true);
            replacements.insert(replacements.end(), vt.begin(), vt.end());
            for (const auto* replacement : replacements) {
                const float after = analysis.predictedDelay(
                    *replacement, state.capacitance, state.input_slew);
                if (before - after > options.min_improvement) {
                    result.push_back({state.net,
                                      state.driver->getInst(),
                                      replacement->master,
                                      {},
                                      {0, 0},
                                      replacement->area - original.area,
                                      before - after,
                                      found->second.slack});
                }
            }
        }
        // Recover upstream drive by shrinking slack-rich side loads. The
        // modified instance is the load, so it shares resize conflict handling.
        for (const auto& load : load_planner.candidates(state, found->second.loads)) {
            if (load.driver_gain > options.min_improvement) {
                result.push_back({state.net,
                                  load.inst,
                                  load.master,
                                  {},
                                  {0, 0},
                                  load.added_area,
                                  load.driver_gain,
                                  found->second.slack,
                                  true});
            }
        }
        if (!options.common.actions.buffer) {
            continue;
        }
        // Isolate non-critical branches; keep the sampled critical loads on
        // the original driver to avoid adding a gate to the critical path.
        std::vector<odb::dbObject*> side;
        float pin_cap = 0, total_pin_cap = 0;
        long long x = 0, y = 0;
        for (const auto& load : state.loads) {
            total_pin_cap += load.capacitance;
            if (!found->second.loads.contains(load.term)) {
                side.push_back(load.term);
                pin_cap += load.capacitance;
                x += load.x;
                y += load.y;
            }
        }
        if (side.size() < options.min_side_loads ||
            side.size() > options.common.max_fanout) {
            continue;
        }
        const float side_cap =
            pin_cap + std::max(0.0f, state.capacitance - total_pin_cap) *
                          float(side.size()) / state.loads.size();
        const auto* buffer = analysis.select(side_cap, state.slew_limit, state.slew);
        if (!buffer) {
            continue;
        }
        const float reduced_cap =
            std::max(0.0f, state.capacitance - side_cap) + buffer->input->capacitance();
        const float after =
            analysis.predictedDelay(original, reduced_cap, state.input_slew);
        if (before - after > options.min_improvement) {
            const BufferLocation location{int(x / side.size()), int(y / side.size())};
            result.push_back({state.net, state.driver->getInst(), buffer->master,
                              std::move(side), location, buffer->area, before - after,
                              found->second.slack});
        }
    }
    std::sort(result.begin(), result.end(),
              [](const SetupCandidate& a, const SetupCandidate& b) {
                  if (a.slack != b.slack) {
                      return a.slack < b.slack;
                  }
                  if (a.gain != b.gain) {
                      return a.gain > b.gain;
                  }
                  if (a.added_area != b.added_area) {
                      return a.added_area < b.added_area;
                  }
                  if (a.driver->getName() != b.driver->getName()) {
                      return a.driver->getName() < b.driver->getName();
                  }
                  if (a.master->getName() != b.master->getName()) {
                      return a.master->getName() < b.master->getName();
                  }
                  return a.loads.size() < b.loads.size();
              });
    if (result.size() > options.max_candidates) {
        result.resize(options.max_candidates);
    }
    return result;
}

static bool acceptableSetup(const DesignMetrics& before, const DesignMetrics& after,
                            double area_limit, const SetupRepairOptions& options) {
    const auto& tolerance = options.common.tolerances;
    const float improvement = options.min_improvement;
    const float tns_tolerance = detail::tnsTolerance(before.setup_tns, tolerance);
    return after.area <= area_limit &&
           detail::timingNonRegressing(before, after, tolerance) &&
           (after.setup_wns > before.setup_wns + improvement ||
            after.setup_tns >
                before.setup_tns + std::max(improvement, tns_tolerance)) &&
           detail::noNewDrv(before, after) &&
           after.severity <= before.severity + tolerance.drv_severity;
}

SetupRepairResult SetupRepairer::run(odb::dbBlock* block, sta::dbSta* sta,
                                     const std::function<void()>& refresh,
                                     const SetupRepairOptions& options,
                                     const RepairRuntime& runtime) {
    SetupRepairResult result;
    try {
        options.validate();
    } catch (const std::exception& error) {
        result.message = error.what();
        return result;
    }
    if (!block || !sta || !refresh || sta->getDbNetwork()->block() != block ||
        sta->scenes().size() != 1) {
        result.message = "invalid setup analysis (one STA scene required)";
        return result;
    }
    DesignAnalysis analysis(block, sta, options.common, runtime.constraints);
    bool needs_refresh = false;
    const auto update = [&](bool restoring) {
        SetupPhaseTimer timer{result.refresh_ms};
        ++result.refreshes;
        if (restoring) {
            ++result.rollback_refreshes;
        }
        refresh();
        needs_refresh = false;
    };
    const auto inspect = [&](DesignMetrics& metrics) {
        SetupPhaseTimer timer{result.measure_ms};
        return analysis.analyze(metrics, true);
    };
    auto states = inspect(result.before);
    result.after = result.before;
    const double area_limit = options.max_area_growth
                                  ? result.before.area * (1 + *options.max_area_growth)
                                  : std::numeric_limits<double>::infinity();
    const auto trials_available = [&]() {
        return !options.max_trials || result.attempted_batches < *options.max_trials;
    };
    for (unsigned pass = 0;
         pass < options.max_passes && result.after.setup_wns < 0 && trials_available();
         ++pass) {
        std::vector<SetupCandidate> plans;
        {
            SetupPhaseTimer timer{result.candidate_ms};
            plans = setupCandidates(analysis, sta, states, options);
        }
        // A rejection applies only to this committed design. Clear on every
        // accepted pass, because loads/timing/placement may then have changed.
        std::set<std::vector<size_t>> rejected_edits;
        size_t cursor = 0, width = std::min(size_t(options.batch_size), plans.size());
        bool accepted = false;
        const auto backoff = [&]() {
            if (width > 1) {
                width = std::min(
                    width - 1,
                    std::max(size_t(1),
                             size_t(std::floor(width * options.batch_backoff))));
            } else {
                ++cursor;
                width = std::min(size_t(options.batch_size), plans.size() - cursor);
            }
        };
        while (cursor < plans.size() && trials_available()) {
            // Batch width counts candidates, but alternatives for the same
            // driver collapse to one edit. Compare the selected edit set before
            // touching OpenDB, so duplicate batches never trigger RC/STA work.
            std::vector<size_t> selected;
            std::unordered_set<odb::dbInst*> touched;
            double planned_area = 0;
            unsigned planned_buffers = 0;
            for (size_t i = cursor; i < std::min(cursor + width, plans.size()); ++i) {
                const auto& plan = plans[i];
                if (touched.contains(plan.driver) ||
                    result.after.area + planned_area + plan.added_area > area_limit ||
                    (!plan.loads.empty() &&
                     planned_buffers >=
                         remainingBufferBudget(options.max_inserted_buffers,
                                               result.inserted_buffers))) {
                    continue;
                }
                selected.push_back(i);
                touched.insert(plan.driver);
                planned_area += plan.added_area;
                planned_buffers += !plan.loads.empty();
            }
            if (selected.empty()) {
                cursor += width;
                width = std::min(size_t(options.batch_size), plans.size() - cursor);
                continue;
            }
            if (options.skip_duplicate_trials && rejected_edits.contains(selected)) {
                ++result.skipped_duplicate_batches;
                backoff();
                continue;
            }
            EcoTransaction transaction(block);
            unsigned inserted = 0, resized = 0, downsized = 0, vt_swaps = 0;
            try {
                for (const auto i : selected) {
                    const auto& plan = plans[i];
                    if (plan.loads.empty()) {
                        const bool vt =
                            analysis.isVtSwap(plan.driver->getMaster(), plan.master);
                        if (!replaceCell({plan.driver, plan.master})) {
                            continue;
                        }
                        ++resized;
                        vt_swaps += vt;
                        downsized += plan.downsize_load;
                    } else {
                        BufferInsertionOptions insertion;
                        insertion.location = plan.location;
                        insertion.buffer_name_prefix = "sizer_setup_buf";
                        insertion.net_name_prefix = "sizer_setup_net";
                        const auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
                            plan.net, plan.loads, plan.master, insertion);
                        if (!edit) {
                            throw std::runtime_error(edit.message);
                        }
                        ++inserted;
                    }
                }
                if (!inserted && !resized) {
                    transaction.rollback();
                    cursor += width;
                    width = std::min(size_t(options.batch_size), plans.size() - cursor);
                    continue;
                }
                ++result.attempted_batches;
                if (runtime.prepare) {
                    SetupPhaseTimer timer{result.prepare_ms};
                    runtime.prepare();
                }
                update(false);
                DesignMetrics after;
                auto next_states = inspect(after);
                if (acceptableSetup(result.after, after, area_limit, options)) {
                    transaction.commit();
                    ++result.accepted_batches;
                    result.inserted_buffers += inserted;
                    result.resized_cells += resized;
                    result.downsized_loads += downsized;
                    result.vt_swaps += vt_swaps;
                    result.after = after;
                    states = std::move(next_states);
                    accepted = true;
                    break;
                }
                transaction.rollback();
                needs_refresh = true;
                if (!options.defer_rollback_refresh) {
                    update(true);
                    states = inspect(result.after);
                }
                rejected_edits.insert(selected);
                ++result.rejected_batches;
                backoff();
            } catch (const std::exception& error) {
                result.message = error.what();
                if (transaction.active()) {
                    transaction.rollback();
                    try {
                        update(true);
                        states = inspect(result.after);
                    } catch (const std::exception& restore_error) {
                        result.message +=
                            std::string("; timing refresh after rollback failed: ") +
                            restore_error.what();
                    }
                }
                return result;
            }
        }
        if (!accepted) {
            result.message = !trials_available()
                                 ? "setup trial budget exhausted; violations remain"
                                 : "no acceptable setup candidate; violations remain";
            break;
        }
    }
    // Even if the last candidate was rejected/skipped or a budget expired,
    // callers must receive current timing for the restored committed design.
    if (needs_refresh) {
        try {
            update(true);
            states = inspect(result.after);
        } catch (const std::exception& error) {
            result.message =
                std::string("timing refresh after rollback failed: ") + error.what();
            return result;
        }
    }
    result.ok = true;
    result.converged = result.after.setup_wns >= 0;
    if (result.message.empty()) {
        result.message = result.converged
                             ? "setup timing met"
                             : "setup pass/trial budget exhausted; violations remain";
    }
    return result;
}
} // namespace ecc
