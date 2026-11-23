// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair/config.h"
#include "design_analysis.h"
#include "actions/netlist.h"
#include <span>

#include <functional>
#include <string>
#include <vector>

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
struct GobiSolutions {
    std::vector<BufferTreePlan> plans;
    unsigned nets_collected = 0;
};

struct GobiResult {
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
    DesignMetrics before;
    DesignMetrics after;
};

class Gobi {
  public:
    // Solve exactly the supplied net snapshots; goal scheduling belongs to run
    // or its caller. Does not edit OpenDB, open an ECO, refresh STA or commit.
    // max_buffers is an execution limit and is deliberately not applied here.
    // Throws on invalid inputs/solver failures. Plans expire after design edits.
    static GobiSolutions solve(odb::dbBlock *block, sta::dbSta *sta,
                               std::span<const detail::NetState> states,
                               rsz::Resizer *resizer, const GobiOptions &options = {},
                               const DesignConstraints &constraints = {});

    static GobiResult run(odb::dbBlock *block, sta::dbSta *sta,
                          const std::function<void()> &refresh, rsz::Resizer *resizer,
                          const GobiOptions &options = {},
                          const RepairRuntime &runtime = {});
};

} // namespace ecc
