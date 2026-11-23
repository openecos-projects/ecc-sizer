// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_config.h"
#include "design_analysis.h"
#include "actions/netlist.h"
#include <span>

#include <functional>
#include <string>
#include <vector>

namespace odb {
class dbBlock;
class dbInst;
class dbMaster;
class dbNet;
} // namespace odb
namespace sta {
class dbSta;
}
namespace rsz {
class Resizer;
}

namespace ecc {

// One-shot CPU/CUDA Van Ginneken repair over placement Steiner trees.
// Both solvers share the same OpenDB apply/rollback path.
struct VgOptions : VgSettings {
    RepairCommonOptions common;

    void validate() const;
};

struct VgSolutions {
    std::vector<BufferTreePlan> plans;
    unsigned nets_collected = 0;
    std::string backend_used = "cpu";
    std::string cuda_fallback;
};

struct VgResult {
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
    DesignMetrics before;
    DesignMetrics after;
};

// Collects all candidates from the current OpenDB/STA snapshot, solves the
// independent net problems, and applies the accepted plan in one ECO.  The
// refresh callback must rebuild placement parasitics and update STA.  A failed
// acceptance check undoes the complete ECO and refreshes the restored design.
class VanGinneken {
  public:
    // Solve exactly the supplied net snapshots; goal scheduling belongs to run
    // or its caller. Does not edit OpenDB, open an ECO, refresh STA or commit.
    // max_buffers is an execution limit and is deliberately not applied here.
    // Throws on invalid inputs/solver failures. Plans expire after design edits.
    static VgSolutions solve(odb::dbBlock *block, sta::dbSta *sta,
                             std::span<const detail::NetState> states,
                             rsz::Resizer *resizer, const VgOptions &options = {});

    static VgResult run(odb::dbBlock *block, sta::dbSta *sta,
                        const std::function<void()> &refresh,
                        const VgOptions &options = {});
    static VgResult run(odb::dbBlock *block, sta::dbSta *sta,
                        const std::function<void()> &refresh, rsz::Resizer *resizer,
                        const VgOptions &options = {});
};

} // namespace ecc
