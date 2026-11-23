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
}
namespace sta {
class dbSta;
}
namespace rsz {
class Resizer;
}

namespace ecc {

// GOBI CUDA candidates over placement Steiner trees, solved as one forest.
struct GobiSolutions {
    std::vector<BufferTreePlan> plans;
    unsigned nets_collected = 0;
};

class Gobi {
  public:
    // Solve exactly the supplied net snapshots; goal scheduling belongs to the repair
    // layer. Does not edit OpenDB, open an ECO, refresh STA or commit. max_buffers is
    // an execution limit and is deliberately not applied here. Throws on invalid
    // inputs/solver failures. Plans expire after design edits.
    static GobiSolutions solve(odb::dbBlock* block, sta::dbSta* sta,
                               std::span<const detail::NetState> states,
                               rsz::Resizer* resizer, const GobiOptions& options = {},
                               const DesignConstraints& constraints = {});
};

} // namespace ecc
