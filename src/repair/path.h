// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "algorithms/path.h"
#include "design_analysis.h"
#include "actions/netlist.h"

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

struct PathEvent {
    enum class Kind {
        SearchStarted,
        CandidatesCollected,
        BatchBackoff,
        SearchFinished,
        FinalPlacementStarted,
        FinalPlacementAccepted,
        FinalPlacementRolledBack
    };
    Kind kind;
    unsigned attempted_trials = 0, rejected_trials = 0, inserted_buffers = 0;
    unsigned candidates = 0, wave_candidates = 0;
    double setup_wns = 0, setup_tns = 0; // STA seconds, last accepted design.
    bool placement_provisional = false;
};

struct PathFanoutCheck {
    std::string phase, net, pin, mode;
    float load = 0, limit = 0, slack = 0;
};

struct PathTrialResult {
    // STA acceptance at proposed coordinates; final DP may undo this phase.
    bool placement_provisional = false;
    bool rechecked_full_rc = false;
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
    std::vector<Candidate> candidates;
    std::vector<std::string> rejection_reasons;
    DesignMetrics before;
    DesignMetrics after;
    double prepare_ms = 0;
    double refresh_ms = 0;
    double measure_ms = 0;
};

struct PathFinalPlacement {
    bool attempted = false, accepted = false, rolled_back = false;
    unsigned provisional_buffers = 0;
    double elapsed_ms = 0;
    DesignMetrics before, after;
    std::vector<std::string> rejection_reasons;
};

struct PathRepairResult : PathSearchState {
    bool ok = false;
    bool converged = false;
    std::string message;
    // Trial counters count ECO batches; inserted_buffers counts their edits.
    // Final-placement rollback clears accepted_trials and inserted_buffers;
    // individual provisional measurements remain in trials.
    unsigned attempted_trials = 0;
    unsigned accepted_trials = 0;
    unsigned rejected_trials = 0;
    unsigned consecutive_rejections = 0;
    bool stopped_on_rejections = false;
    size_t electrical_full_scans = 0;
    size_t electrical_net_checks = 0;
    size_t electrical_instance_checks = 0;

    unsigned timing_precheck_rejections = 0;
    unsigned fanout_tree_trials = 0;
    unsigned refreshes = 0;
    unsigned rollback_refreshes = 0;
    double candidate_ms = 0;
    double prepare_ms = 0;
    double refresh_ms = 0;
    double measure_ms = 0;
    std::vector<PathTrialResult> trials;
    PathFinalPlacement final_placement;
};

// Applies Path candidates through ECOs, STA acceptance and optional final placement.
class PathRepairer {
  public:
    static PathRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                                const std::function<void()>& refresh,
                                const PathSearchOptions& options = {},
                                const PathRuntime& runtime = {});
};

} // namespace ecc
