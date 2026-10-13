// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#ifndef ECC_SIZER_REPAIR_DESIGN_H_
#define ECC_SIZER_REPAIR_DESIGN_H_

#include "repair_config.h"

#include <functional>
#include <string>
#include <vector>

namespace odb {
class dbBlock;
class dbMaster;
class dbInst;
}  // namespace odb
namespace sta {
class dbSta;
}
namespace ecc {

struct DesignRepairMetrics {
    unsigned fanout_nets = 0;
    unsigned capacitance_nets = 0;
    unsigned slew_nets = 0;
    unsigned skipped_nets = 0;
    double severity = 0.0;
    double area = 0.0;       // OpenDB area units (DBU squared).
    float setup_wns = 0.0f;  // STA internal seconds.
    float hold_wns = 0.0f;
    float setup_tns = 0.0f;
    float hold_tns = 0.0f;
    unsigned total_slew_violations = 0;
    unsigned total_cap_violations = 0;
    unsigned total_fanout_violations = 0;
};

struct DesignRepairResult {
    bool ok = false;
    bool converged = false;
    std::string message;
    unsigned accepted_passes = 0;
    unsigned rejected_passes = 0;
    unsigned inserted_buffers = 0;
    unsigned resized_buffers = 0;
    unsigned resized_gates = 0;
    unsigned vt_swaps = 0;
    unsigned downsized_loads = 0;
    unsigned rejected_load_batches = 0;
    DesignRepairMetrics before;
    DesignRepairMetrics after;
};

// Native DRV decisions on an OpenDB/dbSta backend. No Resizer repair/move
// calls. refresh must re-estimate placement parasitics and update STA after
// both edits and rollback. Single STA scene; invoke at a topology
// synchronization boundary.
class OpenDbDesignRepairer {
   public:
    static DesignRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                     const std::function< void() >& refresh,
                                     const DesignRepairOptions& options = {});
};

}  // namespace ecc
#endif
