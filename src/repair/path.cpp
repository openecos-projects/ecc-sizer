// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair/path.h"

#include "actions/netlist.h"
#include "design_analysis.h"
#include "sta/Graph.hh"
#include "sta/Sdc.hh"
#include "sta/Mode.hh"
#include "sta/Liberty.hh"
#include "algorithms/path.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "sta/PathExpanded.hh"
#include "sta/PathEnd.hh"
#include "sta/TimingRole.hh"

namespace ecc {
namespace path_detail {
using detail::NetState;

struct PhaseTimer {
    double& elapsed;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    ~PhaseTimer() {
        elapsed += std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - start)
                       .count();
    }
};

std::vector<PathFanoutCheck> fanoutChecks(sta::dbSta* sta, odb::dbITerm* driver,
                                          const char* phase) {
    std::vector<PathFanoutCheck> result;
    auto* network = sta->getDbNetwork();
    auto* pin = network->dbToSta(driver);
    for (auto* mode : sta->modes()) {
        PathFanoutCheck check;
        check.phase = phase;
        check.net = driver->getNet()->getName();
        check.pin = network->pathName(pin);
        check.mode = mode->name();
        sta->checkFanout(pin, mode, sta::MinMax::max(), check.load, check.limit,
                         check.slack);
        result.push_back(std::move(check));
    }
    return result;
}

// Runs entirely inside the caller's ECO, before placement/extraction/timing.
// Only complete a tree when the original single-buffer proposal increases
// the violation count. Existing violations may remain, as in STA acceptance.
// Every added level strictly reduces root fanout; no arbitrary depth budget.
bool completeFanoutTrees(sta::dbSta* sta,
                         const std::vector<BufferInsertionResult>& roots,
                         const std::vector<odb::dbITerm*>& original_drivers,
                         unsigned baseline_violations, unsigned buffer_allowance,
                         double area_allowance, const PathSearchOptions& options,
                         unsigned& buffer_count, double initial_added_area,
                         const PathRuntime& runtime) {
    auto* network = sta->getDbNetwork();
    sta->checkFanoutPreamble();
    const auto violation_count = [&]() {
        return sta->fanoutViolationCount(sta::MinMax::max(), sta->modes());
    };
    if (violation_count() <= baseline_violations) {
        return true;
    }
    const auto report = [&](odb::dbITerm* driver, const char* phase) {
        if (runtime.fanout_observer) {
            for (const auto& check : fanoutChecks(sta, driver, phase)) {
                if (check.slack < 0) {
                    runtime.fanout_observer(check);
                }
            }
        }
    };
    for (auto* driver : original_drivers) {
        report(driver, "candidate_driver");
    }
    for (const auto& root : roots) {
        report(root.output_pin, "candidate_buffer");
    }
    double added_area = initial_added_area;
    for (const auto& root : roots) {
        auto* master = root.buffer->getMaster();
        for (;;) {
            float limit = std::numeric_limits<float>::infinity();
            float previous_load = 0;
            for (const auto& check : fanoutChecks(sta, root.output_pin, "tree")) {
                if (check.slack < 0) {
                    limit = std::min(limit, check.limit);
                    previous_load = std::max(previous_load, check.load);
                }
            }
            if (!std::isfinite(limit)) {
                break;
            }
            if (!(limit > 0)) {
                return false;
            }
            struct WeightedLoad {
                odb::dbObject* term;
                float weight;
                int x, y;
            };
            std::vector<WeightedLoad> loads;
            for (auto* term : root.output_net->getITerms()) {
                if (!network->isLoad(network->dbToSta(term))) {
                    continue;
                }
                int x, y;
                term->getInst()->getLocation(x, y);
                loads.push_back({term, fanoutWeight(network, term), x, y});
            }
            // Top-level output ports carry zero fanout and can stay at root.
            if (loads.empty()) {
                return false;
            }
            int min_x = loads.front().x, max_x = min_x;
            int min_y = loads.front().y, max_y = min_y;
            for (const auto& load : loads) {
                min_x = std::min(min_x, load.x);
                max_x = std::max(max_x, load.x);
                min_y = std::min(min_y, load.y);
                max_y = std::max(max_y, load.y);
            }
            const bool axis_x = double(max_x) - min_x >= double(max_y) - min_y;
            std::sort(loads.begin(), loads.end(), [&](const auto& a, const auto& b) {
                return std::make_tuple(axis_x ? a.x : a.y, axis_x ? a.y : a.x,
                                       a.term->getId()) <
                       std::make_tuple(axis_x ? b.x : b.y, axis_x ? b.y : b.x,
                                       b.term->getId());
            });
            std::vector<std::vector<WeightedLoad>> groups(1);
            float sum = 0;
            for (const auto& load : loads) {
                if (!std::isfinite(load.weight) || load.weight < 0 ||
                    load.weight > limit) {
                    return false;
                }
                if (sum + load.weight > limit && !groups.back().empty()) {
                    groups.emplace_back();
                    sum = 0;
                }
                groups.back().push_back(load);
                sum += load.weight;
            }
            // An additional level must decrease the root's weighted load.
            const float input_weight = fanoutWeight(network, root.input_pin);
            if (!(groups.size() * double(input_weight) < previous_load) ||
                buffer_count + groups.size() > buffer_allowance ||
                added_area + groups.size() * detail::area(master) > area_allowance) {
                return false;
            }
            std::vector<odb::dbITerm*> children;
            for (const auto& group : groups) {
                std::vector<odb::dbObject*> terms;
                long long x = 0, y = 0;
                for (const auto& load : group) {
                    terms.push_back(load.term);
                    x += load.x;
                    y += load.y;
                }
                BufferInsertionOptions insertion;
                insertion.location =
                    BufferLocation{int(x / group.size()), int(y / group.size())};
                insertion.buffer_name_prefix = "sizer_path_buf";
                insertion.net_name_prefix = "sizer_path_net";
                const auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(
                    root.output_net, terms, master, insertion);
                if (!edit) {
                    throw std::runtime_error(edit.message);
                }
                children.push_back(edit.output_pin);
                ++buffer_count;
                added_area += detail::area(master);
            }
            sta->checkFanoutPreamble();
            // STA is authoritative, including exceptions and all active modes.
            for (auto* child : children) {
                for (const auto& check : fanoutChecks(sta, child, "tree_buffer")) {
                    if (check.slack < 0) {
                        if (runtime.fanout_observer) {
                            runtime.fanout_observer(check);
                        }
                        return false;
                    }
                }
            }
        }
    }
    return violation_count() <= baseline_violations;
}

std::vector<NetState> analyze(detail::DesignAnalysis& analysis,
                              DesignMetrics& metrics) {
    return analysis.analyze(metrics, true);
}

std::vector<std::string> rejectionReasons(const DesignMetrics& before,
                                          const DesignMetrics& after, double area_limit,
                                          const DesignMetrics& initial,
                                          const PathSearchOptions& options,
                                          const PathTrialResult& trial) {
    const auto& tolerance = options.common.tolerances;
    const float improvement = options.min_improvement;
    const bool local = options.gain_scope == ecc::PathGainScope::Endpoints;
    const auto& reference = globalGainReference(options, initial, before);
    const double objective_wns_before =
        local ? trial.objective_wns_before : before.setup_wns;
    const double objective_tns_before =
        local ? trial.objective_tns_before : before.setup_tns;
    const double objective_wns_after =
        local ? trial.objective_wns_after : after.setup_wns;
    const double objective_tns_after =
        local ? trial.objective_tns_after : after.setup_tns;
    const double wns_improvement =
        std::max(double(improvement),
                 std::abs(local ? objective_wns_before : double(reference.setup_wns)) *
                     options.min_relative_improvement);
    const double tns_improvement =
        std::max(double(improvement),
                 std::abs(local ? objective_tns_before : double(reference.setup_tns)) *
                     options.min_relative_improvement);
    const float tns_tolerance = detail::tnsTolerance(before.setup_tns, tolerance);
    std::vector<std::string> reasons;
    if (!(after.area <= area_limit)) {
        reasons.emplace_back("area");
    }
    if (!(after.setup_wns + tolerance.wns_absolute >= before.setup_wns)) {
        reasons.emplace_back("setup_wns");
    }
    if (!(after.setup_tns + tns_tolerance >= before.setup_tns)) {
        reasons.emplace_back("setup_tns");
    }
    if (!(after.hold_wns + tolerance.wns_absolute >= before.hold_wns)) {
        reasons.emplace_back("hold_wns");
    }
    if (!(after.hold_tns + detail::tnsTolerance(before.hold_tns, tolerance) >=
          before.hold_tns)) {
        reasons.emplace_back("hold_tns");
    }
    const double objective_tns_tolerance =
        local ? detail::tnsTolerance(float(objective_tns_before), tolerance)
              : tns_tolerance;
    if (!(objective_wns_after > objective_wns_before + wns_improvement ||
          objective_tns_after >
              objective_tns_before +
                  std::max(tns_improvement, objective_tns_tolerance))) {
        reasons.emplace_back("no_setup_gain");
    }
    // Local gains cannot justify a globally neutral/numerically insignificant
    // ECO. Keep full-design monotonic progress in addition to DRV/hold guards.
    if (local &&
        !(after.setup_wns > before.setup_wns + improvement ||
          after.setup_tns > before.setup_tns + std::max(improvement, tns_tolerance))) {
        reasons.emplace_back("no_global_gain");
    }
    if (after.total_slew_violations > before.total_slew_violations) {
        reasons.emplace_back("slew");
    }
    if (after.total_cap_violations > before.total_cap_violations) {
        reasons.emplace_back("cap");
    }
    if (after.total_fanout_violations > before.total_fanout_violations) {
        reasons.emplace_back("fanout");
    }
    if (!(after.severity <= before.severity + tolerance.drv_severity)) {
        reasons.emplace_back("drv_severity");
    }
    return reasons;
}

PathObservation observation(const PathTrialResult& trial) {
    return {double(trial.after.setup_wns) - trial.before.setup_wns,
            double(trial.after.setup_tns) - trial.before.setup_tns,
            trial.objective_wns_after - trial.objective_wns_before,
            trial.objective_tns_after - trial.objective_tns_before,
            trial.rejection_reasons,
            trial.metrics_complete,
            trial.frontier_audit};
}

// A trial owns reporting and objective measurement. The repair loop owns
// the ECO lifetime; the algorithm schedule owns candidate/retry decisions.
struct TrialMeasurement {
    PathTrialResult trial;
    std::unordered_map<const sta::Pin*, float> objective_pins;
    bool repeated = false, deferred = false;
    double prepare_start = 0, refresh_start = 0, measure_start = 0;

    TrialMeasurement(const std::vector<PathBufferCandidate>& pending,
                     const std::vector<size_t>& selected, const PathPolicy& policy,
                     const PathRepairResult& result, bool frontier_audit) {
        trial.frontier_audit = frontier_audit;
        trial.before = result.after;
        for (const auto index : selected) {
            const auto& plan = pending[index];
            repeated |= policy.hasFailure(plan);
            deferred |= plan.deferred_retry;
            trial.candidates.push_back(
                {.net = plan.net->getName(),
                 .master = plan.master->getName(),
                 .loads = unsigned(plan.loads.size()),
                 .predicted_driver_gain = plan.predicted_gain,
                 .predicted_wns_gain = plan.predicted_wns_gain,
                 .predicted_tns_gain = plan.predicted_tns_gain,
                 .affected_endpoints = plan.affected_endpoints,
                 .x = plan.location.x,
                 .y = plan.location.y,
                 .recovery = plan.recovery,
                 .planned_buffers = unsigned(plan.tree ? plan.tree->nodes.size() : 1),
                 .fingerprint = actionKey(plan),
                 .ranked_wns_gain = plan.ranked_wns_gain,
                 .ranked_tns_gain = plan.ranked_tns_gain,
                 .deferred_retry = plan.deferred_retry,
                 .covered_tns_gain = plan.covered_tns_gain,
                 .covered_endpoints = plan.covered_endpoints,
                 .family_attempts = plan.family_attempts,
                 .generation = policy.generationIndex(),
                 .load_group = plan.load_group_key,
                 .predicted_path_gain = plan.quality.path_gain,
                 .predicted_electrical_excess = plan.quality.electrical_excess,
                 .electrical_retry = plan.electrical_retry});
        }
        for (const auto index : selected) {
            for (const auto& [pin, slack] : pending[index].endpoints) {
                objective_pins.emplace(pin, slack);
            }
        }
        trial.objective_endpoints = objective_pins.size();
        for (const auto& [pin, slack] : objective_pins) {
            trial.objective_wns_before =
                std::min(trial.objective_wns_before, double(slack));
            trial.objective_tns_before += slack;
        }
        prepare_start = result.prepare_ms;
        refresh_start = result.refresh_ms;
        measure_start = result.measure_ms;
    }

    template <typename Inspect>
    std::vector<NetState> measure(sta::dbSta* sta, const PathSearchOptions& options,
                                  const PathRuntime& runtime, PathRepairResult& result,
                                  double area_limit, Inspect inspect,
                                  const std::function<void()>& verify) {
        DesignMetrics after;
        std::vector<NetState> next_states;
        const auto objective = [&]() {
            trial.objective_wns_after = trial.objective_tns_after = 0;
            for (const auto& [pin, old_slack] : objective_pins) {
                const double slack = std::min(
                    0.0, double(sta->slack(pin, sta::RiseFallBoth::riseFall(),
                                           sta->scenes(), sta::MinMax::max())));
                trial.objective_wns_after = std::min(trial.objective_wns_after, slack);
                trial.objective_tns_after += slack;
            }
        };
        if (options.timing_precheck) {
            PhaseTimer timer{result.measure_ms};
            sta->findRequireds();
            // Placeholder electrical fields are used only by the
            // timing rejection predicate and never exported as
            // measured values or used to accept an ECO.
            after = result.after;
            after.setup_wns =
                std::min(0.0f, float(sta->worstSlack(sta::MinMax::max())));
            after.setup_tns = float(sta->totalNegativeSlack(sta::MinMax::max()));
            after.hold_wns = std::min(0.0f, float(sta->worstSlack(sta::MinMax::min())));
            after.hold_tns = float(sta->totalNegativeSlack(sta::MinMax::min()));
            objective();
            trial.rejection_reasons = rejectionReasons(result.after, after, area_limit,
                                                       result.before, options, trial);
        }
        // Survivors are refreshed and inspected below (full RC or
        // the accumulated timing cone). Do not scan electrical
        // states twice before the same acceptance decision.
        const bool verify_next =
            options.timing_precheck && verify && trial.rejection_reasons.empty();
        if (trial.rejection_reasons.empty() && !verify_next) {
            next_states = inspect(after);
            objective();
            trial.rejection_reasons = rejectionReasons(result.after, after, area_limit,
                                                       result.before, options, trial);
        } else if (!trial.rejection_reasons.empty()) {
            trial.metrics_complete = false;
            ++result.timing_precheck_rejections;
        } else {
            trial.metrics_complete = false;
        }
        trial.after = after;
        if (verify && path_detail::needsFullVerification(trial.rejection_reasons)) {
            {
                PhaseTimer timer{result.refresh_ms};
                ++result.refreshes;
                verify();
            }
            next_states = inspect(after);
            trial.metrics_complete = true;
            trial.after = after;
            objective();
            trial.rejection_reasons = rejectionReasons(result.after, after, area_limit,
                                                       result.before, options, trial);
            trial.rechecked_full_rc = bool(runtime.verification_refresh);
        }
        return next_states;
    }

    void record(PathRepairResult& result, const PathRuntime& runtime) {
        trial.prepare_ms = result.prepare_ms - prepare_start;
        trial.refresh_ms = result.refresh_ms - refresh_start;
        trial.measure_ms = result.measure_ms - measure_start;
        result.trials.push_back(std::move(trial));
        if (runtime.trial_observer) {
            runtime.trial_observer(result.attempted_trials, result.trials.back());
        }
    }
};

void reportEvent(const PathRuntime& runtime, PathEvent::Kind kind,
                 const PathRepairResult& result, bool provisional = false,
                 unsigned wave_candidates = 0) {
    if (runtime.event_observer) {
        runtime.event_observer({kind, result.attempted_trials, result.rejected_trials,
                                result.inserted_buffers, result.candidate_count,
                                wave_candidates, result.after.setup_wns,
                                result.after.setup_tns, provisional});
    }
}

// Keep all provisional ECOs in one parent journal. Child commits merge into
// it, so final legalization (including moves of existing cells) can be undone
// together with every inserted buffer and reconnected load.
PathRepairResult repairWithFinalPlacement(odb::dbBlock* block, sta::dbSta* sta,
                                          const std::function<void()>& refresh,
                                          const PathSearchOptions& options,
                                          const PathRuntime& runtime) {
    PathRepairResult result;
    if (!runtime.prepare) {
        result.message = "final path placement requires a legalization callback";
        return result;
    }
    detail::DesignAnalysis analysis(block, sta, options.common, runtime.constraints);
    analyze(analysis, result.before);
    result.after = result.before;
    const auto authoritative_refresh = [&]() {
        PhaseTimer timer{result.refresh_ms};
        ++result.refreshes;
        if (runtime.verification_refresh) {
            runtime.verification_refresh();
        } else {
            refresh();
        }
    };
    std::optional<EcoTransaction> parent, final;
    const auto restore = [&]() {
        if (final) {
            final->rollback();
        }
        if (parent) {
            parent->rollback();
        }
        result.final_placement.rolled_back = true;
        result.inserted_buffers = result.accepted_trials = 0;
        ++result.rollback_refreshes;
        authoritative_refresh();
        PhaseTimer timer{result.measure_ms};
        analyze(analysis, result.after);
        result.converged = result.after.setup_wns >= 0;
        reportEvent(runtime, PathEvent::Kind::FinalPlacementRolledBack, result);
    };
    try {
        parent.emplace(block);
        parent->suspend();
        auto search_options = options;
        auto search_runtime = runtime;
        search_options.placement = ecc::PathPlacement::Trial;
        search_runtime.prepare = {};
        if (options.verification == ecc::PathVerification::Final) {
            // Provisional ECOs still undergo global timing and electrical
            // acceptance. The parent journal makes the final full extraction
            // authoritative for the entire phase, including all child commits.
            search_options.verification = ecc::PathVerification::Trial;
            search_runtime.verification_refresh = {};
        }
        search_runtime.trial_observer = [&](unsigned index,
                                            const PathTrialResult& trial) {
            if (runtime.trial_observer) {
                auto provisional = trial;
                provisional.placement_provisional = true;
                runtime.trial_observer(index, provisional);
            }
        };
        search_runtime.event_observer = [&](const PathEvent& event) {
            if (runtime.event_observer) {
                auto provisional = event;
                provisional.placement_provisional = true;
                runtime.event_observer(provisional);
            }
        };
        result = PathRepairer::run(block, sta, refresh, search_options, search_runtime);
        for (auto& trial : result.trials) {
            trial.placement_provisional = true;
        }
        result.final_placement.provisional_buffers = result.inserted_buffers;
        result.final_placement.before = result.after;
        if (!result.ok) {
            restore();
            return result;
        }
        if (!result.inserted_buffers) {
            if (options.verification == ecc::PathVerification::Final) {
                authoritative_refresh();
                PhaseTimer timer{result.measure_ms};
                analyze(analysis, result.after);
            }
            parent->commit();
            return result;
        }
        result.final_placement.attempted = true;
        reportEvent(runtime, PathEvent::Kind::FinalPlacementStarted, result, true);
        PhaseTimer finalize_timer{result.final_placement.elapsed_ms};
        final.emplace(block);
        {
            PhaseTimer timer{result.prepare_ms};
            runtime.prepare();
        }
        authoritative_refresh();
        {
            PhaseTimer timer{result.measure_ms};
            analyze(analysis, result.final_placement.after);
        }
        auto acceptance = options;
        // Final acceptance compares the legal result with the legal phase
        // input, even if trial ranking used sampled endpoint objectives.
        acceptance.gain_scope = ecc::PathGainScope::Global;
        const double area_limit =
            options.max_area_growth
                ? result.before.area * (1 + *options.max_area_growth)
                : std::numeric_limits<double>::infinity();
        result.final_placement.rejection_reasons =
            rejectionReasons(result.before, result.final_placement.after, area_limit,
                             result.before, acceptance, {});
        if (result.final_placement.rejection_reasons.empty()) {
            final->commit();
            parent->commit();
            result.after = result.final_placement.after;
            result.final_placement.accepted = true;
            result.converged = result.after.setup_wns >= 0;
            result.message += "; final placement and global STA accepted";
            reportEvent(runtime, PathEvent::Kind::FinalPlacementAccepted, result);
        } else {
            restore();
            result.message = "final path placement rejected; restored phase input";
            for (const auto& reason : result.final_placement.rejection_reasons) {
                result.message += " " + reason;
            }
        }
    } catch (const std::exception& error) {
        result.ok = false;
        result.message = std::string("final path placement failed: ") + error.what();
        if ((parent && parent->active()) || (final && final->active())) {
            try {
                restore();
            } catch (const std::exception& failure) {
                result.message += std::string("; restore failed: ") + failure.what();
            }
        }
    }
    return result;
}

} // namespace path_detail

PathRepairResult PathRepairer::run(odb::dbBlock* block, sta::dbSta* sta,
                                   const std::function<void()>& refresh,
                                   const PathSearchOptions& options,
                                   const PathRuntime& runtime) {
    using namespace path_detail;
    PathRepairResult result;
    try {
        options.validate();
    } catch (const std::exception& error) {
        result.message = error.what();
        return result;
    }
    if (!block || !sta || !refresh || sta->getDbNetwork()->block() != block ||
        sta->scenes().size() != 1) {
        result.message = "invalid path repair analysis (one STA scene required)";
        return result;
    }
    if (options.verification == ecc::PathVerification::Final &&
        (options.placement != ecc::PathPlacement::Final ||
         !runtime.verification_refresh)) {
        result.message =
            "final path verification requires final placement and a full RC callback";
        return result;
    }
    if (options.placement == ecc::PathPlacement::Final) {
        return repairWithFinalPlacement(block, sta, refresh, options, runtime);
    }

    const auto start = std::chrono::steady_clock::now();
    const auto& verify = runtime.verification_refresh
                             ? runtime.verification_refresh
                             : runtime.incremental_verification;
    bool needs_refresh = false;
    detail::DesignAnalysis analysis(block, sta, options.common, runtime.constraints);
    const auto update = [&](bool restoring) {
        PhaseTimer timer{result.refresh_ms};
        ++result.refreshes;
        if (restoring) {
            ++result.rollback_refreshes;
        }
        if (restoring && runtime.verification_refresh) {
            runtime.verification_refresh();
        } else {
            refresh();
        }
        needs_refresh = false;
    };
    const auto inspect = [&](DesignMetrics& metrics) {
        PhaseTimer timer{result.measure_ms};
        if (options.incremental_analysis && runtime.analysis_changes &&
            !runtime.verification_refresh && runtime.incremental_verification &&
            !runtime.prepare) {
            auto states =
                analysis.analyzeIncremental(metrics, runtime.analysis_changes());
            const auto& stats = analysis.analysisStatistics();
            result.electrical_full_scans = stats.full_scans;
            result.electrical_net_checks = stats.net_checks;
            result.electrical_instance_checks = stats.instance_checks;
            return states;
        }
        ++result.electrical_full_scans;
        return analyze(analysis, metrics);
    };
    auto states = inspect(result.before);
    result.after = result.before;
    const double area_limit = options.max_area_growth
                                  ? result.before.area * (1 + *options.max_area_growth)
                                  : std::numeric_limits<double>::infinity();
    const auto trials_available = [&]() {
        return !options.max_trials || result.attempted_trials < *options.max_trials;
    };
    reportEvent(runtime, PathEvent::Kind::SearchStarted, result);
    PathPolicy policy{options, result};
    path_detail::DependencyFailures dependency_failures;
    path_detail::EndpointFailures endpoint_failures;
    const auto endpoint_fanin = [&](sta::Vertex* endpoint) {
        path_detail::DependencyFailures::Cone nets;
        std::unordered_set<sta::Vertex*> visited{endpoint};
        std::vector<sta::Vertex*> queue{endpoint};
        auto* graph = sta->graph();
        auto* network = sta->getDbNetwork();
        for (size_t i = 0; i < queue.size(); ++i) {
            if (auto* net = network->net(queue[i]->pin())) {
                if (auto* db_net = network->staToDb(net)) {
                    nets.insert(db_net->getId());
                }
            }
            // The input-pin slew also depends on its cell's output loads
            // through the delay model, even for side outputs outside fanin.
            auto* object = network->staToDb(queue[i]->pin());
            if (object && object->getObjectType() == odb::dbITermObj) {
                auto* inst = static_cast<odb::dbITerm*>(object)->getInst();
                for (auto* pin : inst->getITerms()) {
                    if (pin->getNet()) {
                        nets.insert(pin->getNet()->getId());
                    }
                }
            }
            // Include timing checks: capture-clock/constraint changes must
            // invalidate a slack witness, not only its data arrival cone.
            sta::VertexInEdgeIterator edges(queue[i], graph);
            while (edges.hasNext()) {
                auto* vertex = edges.next()->from(graph);
                if (visited.insert(vertex).second) {
                    queue.push_back(vertex);
                }
            }
        }
        auto result = std::make_shared<const path_detail::DependencyFailures::Cone>(
            std::move(nets));
        return result;
    };

    // No intermediate DP or full RC: only the inserted tree changes physical
    // dependencies. Final placement still receives authoritative global STA.
    const bool reuse_rejections =
        policy.reuseRejections() && sta->scenes().size() == 1 && !runtime.prepare &&
        !runtime.verification_refresh && runtime.incremental_verification;
    std::unordered_map<odb::dbNet*,
                       std::shared_ptr<const path_detail::DependencyFailures::Cone>>
        dependency_cones;
    const auto dependency_cone = [&](odb::dbNet* net) {
        if (const auto it = dependency_cones.find(net); it != dependency_cones.end()) {
            return it->second;
        }
        path_detail::DependencyFailures::Cone cone{net->getId()};
        for (auto* vertex : timingCone(sta, {net})) {
            auto* sta_net = sta->getDbNetwork()->net(vertex->pin());
            if (sta_net) {
                auto* db_net = sta->getDbNetwork()->staToDb(sta_net);
                if (db_net) {
                    cone.insert(db_net->getId());
                }
            }
        }
        auto shared = std::make_shared<const path_detail::DependencyFailures::Cone>(
            std::move(cone));
        dependency_cones.emplace(net, shared);
        return shared;
    };

    const auto budget_available = [&]() {
        if (options.max_consecutive_rejections == 0) {
            return false;
        }
        // Reserve a final refresh for a rejected ECO. Never return stale STA.
        if (options.max_refreshes &&
            result.refreshes + (verify ? 3 : 2) > *options.max_refreshes) {
            return false;
        }
        if (options.max_seconds) {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                    .count();
            if (elapsed >= *options.max_seconds) {
                return false;
            }
        }
        return true;
    };
    try {
        while (options.common.actions.buffer && budget_available() &&
               trials_available() && result.after.setup_wns < 0 &&
               remainingBufferBudget(options.max_inserted_buffers,
                                     result.inserted_buffers) > 0) {
            CandidateCollection collection;
            {
                PhaseTimer timer{result.candidate_ms};
                collection = policy.collect(analysis, sta, states, result.after);
                policy.screen(collection.candidates);
            }
            result.candidate_count += collection.candidates.size();
            result.duplicate_candidates += collection.duplicate_candidates;
            reportEvent(runtime, PathEvent::Kind::CandidatesCollected, result, false,
                        collection.candidates.size());
            auto plans = std::move(collection.candidates);
            if (plans.empty()) {
                break;
            }
            bool accepted = false;
            PathSchedule schedule(std::move(plans), options, result, policy);
            const auto& pending = schedule.candidates();
            while (schedule.hasNext() && budget_available() && trials_available()) {
                if (!schedule.prepareNext()) {
                    continue;
                }
                const bool frontier_audit = schedule.frontierAudit();
                const auto batch = schedule.selectBatch(area_limit);
                const auto& selected = batch.indices;
                double planned_area = batch.area;
                if (selected.empty()) {
                    schedule.skipBatch();
                    continue;
                }
                if (reuse_rejections && selected.size() == 1 &&
                    (dependency_failures.rejects(
                         pending[selected.front()].feedback_key, result.after.setup_tns,
                         detail::tnsTolerance(result.after.setup_tns,
                                              options.common.tolerances)) ||
                     endpoint_failures.rejects(
                         pending[selected.front()].feedback_key, result.after.setup_wns,
                         options.common.tolerances.wns_absolute))) {
                    ++result.failure_cache_hits;
                    ++result.reused_rejections;
                    ++result.consecutive_rejections;
                    // A dependency-certified rejection completes this family's
                    // turn just like a measured rejection. Advance coverage,
                    // but do not train calibration/frontiers using an old
                    // global delta: other commits may have changed that delta.
                    policy.covered(pending[selected.front()]);
                    if (options.max_consecutive_rejections &&
                        result.consecutive_rejections >=
                            *options.max_consecutive_rejections) {
                        result.stopped_on_rejections = true;
                        break;
                    }
                    schedule.backoff();
                    schedule.rerank();
                    continue;
                }
                std::shared_ptr<const path_detail::DependencyFailures::Cone>
                    dependencies;
                if (reuse_rejections && selected.size() == 1) {
                    dependencies = dependency_cone(pending[selected.front()].net);
                }
                TrialMeasurement measurement(pending, selected, policy, result,
                                             frontier_audit);
                auto& trial = measurement.trial;
                std::vector<odb::dbITerm*> original_drivers;
                for (const auto index : selected) {
                    for (auto* term : pending[index].net->getITerms()) {
                        if (term->getIoType() == odb::dbIoType::OUTPUT) {
                            original_drivers.push_back(term);
                        }
                    }
                }
                EcoTransaction transaction(block);
                try {
                    std::vector<BufferInsertionResult> roots;
                    for (const auto index : selected) {
                        const auto& plan = pending[index];
                        BufferInsertionOptions insertion;
                        insertion.location = plan.location;
                        insertion.buffer_name_prefix = "sizer_path_buf";
                        insertion.net_name_prefix = "sizer_path_net";
                        const auto edits = applyBufferTree(bufferPlan(plan), insertion);
                        for (auto root : edits.roots) {
                            roots.push_back(edits.insertions[root]);
                        }
                        trial.buffers += edits.insertions.size();
                    }
                    if (options.fanout_repair) {
                        PhaseTimer timer{result.candidate_ms};
                        const auto allowance = remainingBufferBudget(
                            options.max_inserted_buffers, result.inserted_buffers);
                        if (!completeFanoutTrees(
                                sta, roots, original_drivers,
                                result.after.total_fanout_violations, allowance,
                                area_limit - result.after.area, options, trial.buffers,
                                planned_area, runtime)) {
                            transaction.rollback();
                            needs_refresh = true;
                            ++result.fanout_precheck_rejections;
                            if (selected.size() == 1) {
                                schedule.fanoutRejected(pending[selected.front()]);
                            }
                            if (selected.size() > 1) {
                                reportEvent(runtime, PathEvent::Kind::BatchBackoff,
                                            result);
                            }
                            schedule.backoff();
                            continue;
                        }
                    }
                    if (trial.buffers > roots.size()) {
                        ++result.fanout_tree_trials;
                    }
                    ++result.attempted_trials;
                    result.frontier_audits += frontier_audit;
                    result.repeated_trials += measurement.repeated;
                    result.deferred_trials += measurement.deferred;
                    if (runtime.prepare) {
                        PhaseTimer timer{result.prepare_ms};
                        runtime.prepare();
                    }
                    update(false);
                    auto next_states = measurement.measure(
                        sta, options, runtime, result, area_limit, inspect, verify);
                    const auto& after = trial.after;
                    schedule.observe(batch, observation(trial));
                    if (trial.rejection_reasons.empty()) {
                        transaction.commit();
                        ++result.accepted_trials;
                        result.inserted_buffers += trial.buffers;
                        result.consecutive_rejections = 0;
                        result.after = after;
                        states = std::move(next_states);
                        if (reuse_rejections) {
                            dependency_failures.committed(*dependencies);
                            endpoint_failures.committed(
                                pending[selected.front()].net->getId());
                            dependency_cones.clear();
                        }
                        policy.committed();
                        accepted = true;
                        measurement.record(result, runtime);
                        break;
                    }
                    if (reuse_rejections && selected.size() == 1 &&
                        std::find(trial.rejection_reasons.begin(),
                                  trial.rejection_reasons.end(),
                                  "setup_tns") != trial.rejection_reasons.end()) {
                        dependency_failures.record(
                            pending[selected.front()].feedback_key,
                            std::move(dependencies), trial.before.setup_tns,
                            trial.after.setup_tns);
                    }
                    if (reuse_rejections && selected.size() == 1 &&
                        std::find(trial.rejection_reasons.begin(),
                                  trial.rejection_reasons.end(),
                                  "setup_wns") != trial.rejection_reasons.end()) {
                        sta::Slack slack;
                        sta::Vertex* endpoint = nullptr;
                        sta->worstSlack(sta::MinMax::max(), slack, endpoint);
                        if (endpoint) {
                            endpoint_failures.record(
                                pending[selected.front()].feedback_key,
                                endpoint_fanin(endpoint), float(slack));
                        }
                    }
                    transaction.rollback();
                    ++result.rejected_trials;
                    // Do not stop after a failed aggregate: its members must
                    // remain eligible for individual trials during backoff.
                    if (selected.size() == 1) {
                        ++result.consecutive_rejections;
                    }
                    needs_refresh = true;
                    if (!options.defer_rollback_refresh) {
                        update(true);
                        states = inspect(result.after);
                    }
                    schedule.recoverElectrical(batch, observation(trial));
                    measurement.record(result, runtime);
                    if (options.max_consecutive_rejections &&
                        result.consecutive_rejections >=
                            *options.max_consecutive_rejections) {
                        result.stopped_on_rejections = true;
                        break; // The common exit path restores RC/STA first.
                    }
                    if (selected.size() > 1) {
                        reportEvent(runtime, PathEvent::Kind::BatchBackoff, result);
                    }
                    schedule.rejected();
                } catch (...) {
                    if (transaction.active()) {
                        transaction.rollback();
                        needs_refresh = true;
                    }
                    throw;
                }
            }
            if (!accepted) {
                break;
            }
        }
    } catch (const std::exception& error) {
        result.message = error.what();
        if (needs_refresh) {
            try {
                update(true);
                states = inspect(result.after);
            } catch (const std::exception& restore) {
                result.message +=
                    std::string("; rollback refresh failed: ") + restore.what();
            }
        }
        reportEvent(runtime, PathEvent::Kind::SearchFinished, result);
        return result;
    }
    if (needs_refresh) {
        try {
            update(true);
            states = inspect(result.after);
        } catch (const std::exception& error) {
            result.message = error.what();
            reportEvent(runtime, PathEvent::Kind::SearchFinished, result);
            return result;
        }
    }

    result.ok = true;
    result.converged = result.after.setup_wns >= 0;
    result.message =
        result.converged ? "path buffer setup timing met"
        : remainingBufferBudget(options.max_inserted_buffers,
                                result.inserted_buffers) == 0
            ? "path buffer insertion budget exhausted; violations remain"
        : result.stopped_on_rejections
            ? "path buffer stopped after consecutive rejections; search not exhausted"
        : !budget_available()
            ? "path buffer execution budget exhausted; violations remain"
        : !trials_available() ? "path buffer trial budget exhausted; violations remain"
        : result.frontier_suspended
            ? "audited empirical path frontier exhausted; violations remain"
            : "no acceptable path buffer candidate; violations remain";
    reportEvent(runtime, PathEvent::Kind::SearchFinished, result);
    return result;
}
} // namespace ecc
