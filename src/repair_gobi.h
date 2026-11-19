// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_config.h"

#include <functional>
#include <string>

namespace odb {
class dbBlock;
}
namespace sta {
class dbSta;
}
namespace rsz {
class Resizer;
}

namespace ecc {

// One-shot OpenDB adapter for the migrated OpenROAD GOBI CUDA kernels.  The
// graph is collected from the current placement Steiner trees, solved as one
// CUDA forest, then applied and checked as one ECO.
struct GobiRepairOptions {
    RepairCommonOptions common;
    std::string backend = "cuda";
    unsigned max_buffers = 64;
    unsigned max_nets = 0;
    bool require_drv_reduction = true;

    void validate() const;
};

struct GobiRepairMetrics {
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

struct GobiRepairResult {
    bool ok = false;
    bool committed = false;
    bool converged = false;
    std::string message;
    std::string backend_used = "cuda";
    unsigned nets_collected = 0;
    unsigned plans_collected = 0;
    unsigned attempted_buffers = 0;
    unsigned inserted_buffers = 0;
    unsigned refreshes = 0;
    double collect_ms = 0;
    double solve_ms = 0;
    double apply_ms = 0;
    GobiRepairMetrics before;
    GobiRepairMetrics after;
};


class OpenDbGobiRepairer {
   public:
    static GobiRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                  const std::function<void()>& refresh,
                                  rsz::Resizer* resizer,
                                  const GobiRepairOptions& options = {});
};

}  // namespace ecc
