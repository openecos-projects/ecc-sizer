// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair/config.h"
#include "design_analysis.h"
#include "actions/netlist.h"
#include <span>

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

// CPU/CUDA Van Ginneken candidates over placement Steiner trees.
struct VgSolutions {
    std::vector<BufferTreePlan> plans;
    unsigned nets_collected = 0;
    std::string backend_used = "cpu";
    std::string cuda_fallback;
};

class VanGinneken {
  public:
    // Solve exactly the supplied net snapshots; goal scheduling belongs to the repair
    // layer. Does not edit OpenDB, open an ECO, refresh STA or commit. max_buffers is
    // an execution limit and is deliberately not applied here. Throws on invalid
    // inputs/solver failures. Plans expire after design edits.
    static VgSolutions solve(odb::dbBlock* block, sta::dbSta* sta,
                             std::span<const detail::NetState> states,
                             rsz::Resizer* resizer, const VgOptions& options = {},
                             const DesignConstraints& constraints = {});
};

} // namespace ecc
