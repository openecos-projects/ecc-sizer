// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_design.h"
#include "repair_path.h"

namespace ecc {
struct SetupRepairMetrics {
    DesignRepairMetrics design;
    float setup_tns = 0;
    float hold_tns = 0;
    unsigned slew_violations = 0;
    unsigned cap_violations = 0;
    unsigned fanout_violations = 0;
};

struct SetupRepairResult {
    bool ok = false;
    bool converged = false;
    std::string message;
    unsigned accepted_batches = 0;
    unsigned rejected_batches = 0;
    unsigned attempted_batches = 0;
    unsigned skipped_duplicate_batches = 0;
    unsigned refreshes = 0;
    unsigned rollback_refreshes = 0;
    double candidate_ms = 0;
    double prepare_ms = 0;
    double refresh_ms = 0;
    double measure_ms = 0;
    unsigned inserted_buffers = 0;
    unsigned path_buffer_attempted = 0;
    unsigned path_buffer_accepted = 0;
    unsigned path_buffer_rejected = 0;
    unsigned path_buffer_consecutive_rejections = 0;
    bool path_buffer_stopped_on_rejections = false;
    unsigned path_buffer_inserted_buffers = 0;
    unsigned path_buffer_candidates = 0;
    unsigned path_buffer_duplicate_candidates = 0;
    unsigned path_buffer_screened_candidates = 0;
    unsigned path_buffer_failure_cache_hits = 0;
    unsigned path_buffer_reused_rejections = 0;
    size_t path_buffer_electrical_full_scans = 0;
    size_t path_buffer_electrical_net_checks = 0;
    size_t path_buffer_electrical_instance_checks = 0;

    unsigned path_buffer_deferred_candidates = 0;
    unsigned path_buffer_invalidated_failures = 0;
    unsigned path_buffer_deferred_trials = 0;
    unsigned path_buffer_repeated_trials = 0;
    unsigned path_buffer_calibrated_candidates = 0;
    unsigned path_buffer_feedback_updates = 0;
    unsigned path_buffer_frontier_suspended = 0;
    unsigned path_buffer_frontier_audits = 0;
    unsigned path_buffer_frontier_reopened = 0;
    unsigned path_buffer_timing_precheck_rejections = 0;
    unsigned path_buffer_fanout_precheck_rejections = 0;
    unsigned path_buffer_fanout_tree_trials = 0;
    std::vector< PathBufferTrial > path_buffer_trials;
    PathFinalPlacement path_final_placement;
    unsigned resized_cells = 0;
    unsigned downsized_loads = 0;
    unsigned vt_swaps = 0;
    SetupRepairMetrics before;
    SetupRepairMetrics after;
};

// Setup-only native strategy. Shares OpenDB/STA models with design repair.
// Input must have current RC/STA; common.prepare legalizes each candidate ECO.
// Failed trials restore topology and placement immediately. With deferred
// refresh, STA is refreshed after the next edit or before returning; baseline
// metrics/plans are reused only while the committed design stays unchanged.
// No Resizer repair calls.  When path_buffer_enabled is set, a verified
// path-level side-load buffer phase runs before the native candidate phase;
// path_buffer_only selects that phase as the complete setup repair.
class OpenDbSetupRepairer {
   public:
    static SetupRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                    const std::function< void() >& refresh,
                                    const SetupRepairOptions& options = {});
};
}  // namespace ecc
