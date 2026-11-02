// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_config.h"

#include <functional>
#include <string>
#include <vector>

namespace odb {
class dbBlock;
class dbInst;
class dbMaster;
class dbNet;
}
namespace sta {
class dbSta;
}
namespace rsz {
class Resizer;
}

namespace ecc {

// One-shot CPU/CUDA Van Ginneken repair over placement Steiner trees.
// Both solvers share the same OpenDB apply/rollback path.
struct VgRepairOptions {
    RepairCommonOptions common;
    std::string backend = "cpu";
    unsigned cuda_max_memory_mb = 512;
    unsigned max_buffers = 64;
    unsigned max_nets = 0;  // zero means all eligible violating nets.
    unsigned max_group_loads = 0;  // zero means common.max_fanout.
    float min_gain = 0;
    bool require_drv_reduction = true;

    void validate() const;
};

struct VgRepairMetrics {
    double severity = 0;
    float setup_wns = 0;
    float setup_tns = 0;
    unsigned fanout_nets = 0;
    unsigned capacitance_nets = 0;
    unsigned slew_nets = 0;
    unsigned total_slew_violations = 0;
    unsigned total_cap_violations = 0;
    unsigned total_fanout_violations = 0;
};

struct VgRepairResult {
    bool ok = false;
    bool committed = false;
    bool converged = false;
    std::string message;
    std::string backend_used = "cpu";
    std::string cuda_fallback;
    unsigned nets_collected = 0;
    unsigned plans_collected = 0;
    unsigned attempted_buffers = 0;
    unsigned inserted_buffers = 0;
    unsigned refreshes = 0;
    double collect_ms = 0;
    double solve_ms = 0;
    double apply_ms = 0;
    VgRepairMetrics before;
    VgRepairMetrics after;
};

// Collects all candidates from the current OpenDB/STA snapshot, solves the
// independent net problems, and applies the accepted plan in one ECO.  The
// refresh callback must rebuild placement parasitics and update STA.  A failed
// acceptance check undoes the complete ECO and refreshes the restored design.
class OpenDbVgRepairer {
   public:
    static VgRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                  const std::function<void()>& refresh,
                                  const VgRepairOptions& options = {});
    static VgRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                  const std::function<void()>& refresh,
                                  rsz::Resizer* resizer,
                                  const VgRepairOptions& options = {});
};

}  // namespace ecc
