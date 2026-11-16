// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_config.h"
#include "repair_design.h"

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
    };
    std::vector< Candidate > candidates;
    std::vector< std::string > rejection_reasons;
    DesignRepairMetrics before;
    DesignRepairMetrics after;
    double prepare_ms = 0;
    double refresh_ms = 0;
    double measure_ms = 0;
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
    unsigned inserted_buffers = 0;
    unsigned candidate_count = 0;
    unsigned duplicate_candidates = 0;
    unsigned screened_candidates = 0;
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
        const SetupRepairOptions& options = {});
};

}  // namespace ecc
