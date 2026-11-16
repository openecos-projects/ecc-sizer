// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_config.h"
#include "repair_design.h"
#include <vector>
#include <map>

namespace rsz {
class Resizer;
}
namespace ecc {
struct PbbiPinViolation {
    std::string pin, net, master, kind, relation;
    unsigned transition = 0;
    bool had_before = false;
    double before = 0, after = 0, limit = 0;
    bool has_prediction = false;
    double predicted = 0;
    unsigned moved_net_instances = 0;
    double net_max_displacement_dbu = 0;
};
struct PbbiTrial {
    bool accepted = false;
    bool area_recovery = false, target_before = false, target_after = false;
    unsigned buffers = 0, resized_cells = 0, path_nets = 0;
    double predicted_gain = 0, predicted_cost = 0;
    double prepare_ms = 0, refresh_ms = 0, measure_ms = 0;
    unsigned moved_instances = 0, moved_unsized_instances = 0;
    double max_displacement_dbu = 0;
    unsigned protected_instances = 0;
    bool placement_failed = false;
    std::string fingerprint;
    std::vector< PbbiPinViolation > violations;
    unsigned predicted_slew_violations = 0, predicted_cap_violations = 0;
    double predicted_severity = 0, predicted_wns_delta = 0,
           predicted_tns_delta = 0;
    double prediction_baseline_wns = 0, prediction_baseline_tns = 0;
    double graph_baseline_wns = 0, graph_baseline_tns = 0;
    unsigned prediction_endpoints = 0, graph_endpoints = 0;
    unsigned path_rank = 0;
    unsigned source_paths = 1, pool_candidates = 0, pool_paths = 0;
    unsigned pool_nonregressing_setup = 0, bounded_offpath_loads = 0;
    unsigned affected_nets = 0, affected_gates = 0, affected_endpoints = 0;
    std::vector< std::string > rejection_reasons;
    DesignRepairMetrics before, after;
};
struct PbbiRepairResult {
    bool ok = false, converged = false;
    std::string message;
    unsigned attempted = 0, accepted = 0, rejected = 0;
    unsigned area_reductions = 0;
    bool target_met = false;
    double target_wns = 0, target_tns = 0;
    unsigned inserted_buffers = 0, resized_cells = 0;
    std::map< std::string, unsigned > skip_reasons;
    unsigned paths = 0, solved_paths = 0, skipped_paths = 0,
             exhausted_paths = 0;
    unsigned collected_problems = 0, duplicate_problems = 0,
             max_problem_nets = 0;
    unsigned refreshes = 0, rollback_refreshes = 0, duplicate_plans = 0;
    size_t baseline_model_violations = 0;
    size_t slew_cache_hits = 0, slew_cache_misses = 0;
    size_t slew_rejected = 0, response_evaluations = 0;
    size_t timing_nets = 0, timing_buffered_nets = 0, timing_fallback_nets = 0,
           timing_exhausted_nets = 0;
    size_t timing_vertices = 0, timing_edges = 0, timing_fallback_arcs = 0,
           timing_expansions = 0, timing_truncated = 0;
    size_t timing_readonly_arcs = 0;
    unsigned forecast_paths = 0;
    unsigned electrical_evaluations = 0, electrical_unsupported = 0;
    unsigned feedback_reorders = 0;
    unsigned scheduler_decisions = 0, scheduler_cross_path = 0;
    unsigned scheduler_peak_candidates = 0, scheduler_overlap_admissions = 0;
    unsigned cancelled_paths = 0, admission_deferred = 0;
    unsigned forecast_cache_hits = 0, forecast_cache_misses = 0;
    unsigned forecast_cache_invalidations = 0;
    unsigned path_cache_hits = 0, path_cache_misses = 0, path_cache_invalidations = 0;
    double electrical_ms = 0;
    double timing_ms = 0;
    size_t expansions = 0, peak_states = 0, truncated_states = 0;
    double collect_ms = 0, solve_ms = 0, prepare_ms = 0, refresh_ms = 0,
           measure_ms = 0;
    DesignRepairMetrics before, after;
    std::vector< PbbiTrial > trials;
};
// Paper-inspired path VGDP with gate/sink sizing. No Resizer repair calls:
// Resizer supplies placement Steiner trees/RC only. Real edits use OpenDB ECO,
// and complete multi-net plans must pass global STA before commit.
class OpenDbPbbiRepairer {
   public:
    static PbbiRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                   rsz::Resizer* resizer,
                                   const std::function< void() >& refresh,
                                   const PbbiRepairOptions& options = {});
};
}  // namespace ecc
