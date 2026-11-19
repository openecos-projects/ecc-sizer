// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_config.h"
#include "repair_drv.h"

#include <functional>
#include <string>
#include <vector>

namespace odb {
class dbBlock;
}
namespace sta {
class dbSta;
}

namespace ecc {

struct PathFanoutCheck {
    std::string phase, net, pin, mode;
    float load = 0, limit = 0, slack = 0;
};

struct PathBufferTrial {
    // STA acceptance at proposed coordinates; final DP may undo this phase.
    bool placement_provisional = false;
    bool full_verified = false;
    // False: after contains timing only; electrical/area fields are not valid.
    bool metrics_complete = true;
    bool frontier_audit = false;
    unsigned buffers = 0; // Includes every branch in a compound ECO.
    unsigned objective_endpoints = 0;
    double objective_wns_before = 0, objective_wns_after = 0;
    double objective_tns_before = 0, objective_tns_after = 0;
    struct Candidate {
        std::string net;
        std::string master;
        unsigned loads = 0;
        // Driver-only estimate, not a prediction of global WNS improvement.
        float predicted_driver_gain = 0;
        float predicted_wns_gain = 0;
        float predicted_tns_gain = 0;
        unsigned affected_endpoints = 0;
        int x = 0;
        int y = 0;
        bool recovery = false;
        unsigned planned_buffers = 1;
        std::string fingerprint;
        double ranked_wns_gain = 0, ranked_tns_gain = 0;
        bool deferred_retry = false;
        double covered_tns_gain = 0;
        unsigned covered_endpoints = 0, family_attempts = 0, generation = 0;
        std::string load_group;
        double predicted_path_gain = 0, predicted_electrical_excess = 0;
        bool electrical_retry = false;
    };
    std::vector< Candidate > candidates;
    std::vector< std::string > rejection_reasons;
    DesignRepairMetrics before;
    DesignRepairMetrics after;
    double prepare_ms = 0;
    double refresh_ms = 0;
    double measure_ms = 0;
};

struct PathFinalPlacement {
    bool attempted = false, accepted = false, rolled_back = false;
    unsigned provisional_buffers = 0;
    double elapsed_ms = 0;
    DesignRepairMetrics before, after;
    std::vector<std::string> rejection_reasons;
};

struct PathBufferRepairResult {
    bool ok = false;
    bool converged = false;
    std::string message;
    // attempted/rejected count STA-verified ECO batches; accepted counts
    // buffers committed by those batches.
    unsigned attempted_trials = 0;
    unsigned accepted_trials = 0;
    unsigned rejected_trials = 0;
    unsigned consecutive_rejections = 0;
    bool stopped_on_rejections = false;
    unsigned inserted_buffers = 0;
    unsigned candidate_count = 0;
    unsigned duplicate_candidates = 0;
    unsigned screened_candidates = 0;
    unsigned failure_cache_hits = 0;
    unsigned reused_rejections = 0;
    size_t electrical_full_scans = 0;
    size_t electrical_net_checks = 0;
    size_t electrical_instance_checks = 0;

    unsigned deferred_candidates = 0;
    unsigned invalidated_failures = 0;
    unsigned deferred_trials = 0;
    unsigned repeated_trials = 0;
    unsigned calibrated_candidates = 0;
    unsigned feedback_updates = 0;
    unsigned frontier_suspended = 0;
    unsigned frontier_audits = 0;
    unsigned frontier_reopened = 0;
    unsigned timing_precheck_rejections = 0;
    unsigned fanout_precheck_rejections = 0;
    unsigned fanout_tree_trials = 0;
    unsigned refreshes = 0;
    unsigned rollback_refreshes = 0;
    double candidate_ms = 0;
    double prepare_ms = 0;
    double refresh_ms = 0;
    double measure_ms = 0;
    std::vector< PathBufferTrial > trials;
    DesignRepairMetrics before;
    DesignRepairMetrics after;
    PathFinalPlacement final_placement;
};

// Independent path-level buffer repair.  Candidates are collected from the
// current STA paths and every topology edit is globally STA-verified before
// it is committed.  The caller supplies refresh so the path module remains
// independent of Sizer's placement/parasitic implementation.
class OpenDbPathRepairer {
   public:
    static PathBufferRepairResult repair(
        odb::dbBlock* block, sta::dbSta* sta,
        const std::function< void() >& refresh,
        const PathRepairOptions& options = {});
};

}  // namespace ecc
