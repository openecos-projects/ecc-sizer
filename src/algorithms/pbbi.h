// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "actions/netlist.h"
#include "repair/config.h"
#include "repair/runtime.h"

#include <limits>
#include <span>
#include <vector>

namespace sta {
class dbSta;
}
namespace rsz {
class Resizer;
}
namespace ecc::detail {
struct NetState;
}

namespace ecc::pbbi {
// DAC 2005, Sze et al., section 4.2: nets are joined by mandatory fixed
// gates. A gate resets capacitance, but carries downstream RAT and cost.
// Postorder indices and value snapshots deliberately own no STA Path objects.
enum class NodeKind { Sink, Wire, Junction, Gate };
constexpr double infinity = std::numeric_limits<double>::infinity();
struct Node {
    NodeKind kind = NodeKind::Sink;
    int left = -1, right = -1;
    double cap = 0, resistance = 0, intrinsic = 0;
    double required = 0, slew_limit = infinity, max_cap = infinity;
    bool can_buffer = true;
    unsigned net = 0;
    odb::dbObject* terminal = nullptr;
    BufferLocation location;
};
struct Buffer {
    odb::dbMaster* master = nullptr;
    double cap = 0, resistance = 0, intrinsic = 0, max_cap = infinity;
};
struct Tree {
    std::vector<Node> nodes;
    std::vector<odb::dbNet*> nets;
    double driver_resistance = 0, driver_slew = 0;
};
struct Choice {
    double cap = 0, required = 0, slew_budget = infinity;
    unsigned cost = 0, fanout = 0;
    int left = -1, right = -1, buffer = -1;
};
struct Solution {
    std::vector<std::vector<Choice>> frontiers;
    int selected = -1;
    double score = -infinity;
    unsigned truncated = 0;
};
// Target is arrival time at the root before its output resistance. Feasible
// solutions minimize buffer count, otherwise maximize RAT, then minimize cost.
Solution solve(const Tree& tree, std::span<const Buffer> buffers, double target,
               const PbbiOptions& options);
std::vector<BufferTreePlan> decode(const Tree& tree, std::span<const Buffer> buffers,
                                   const Solution& solution);
double adjustedRequired(double required, double endpoint_required,
                        double endpoint_arrival, double root_arrival);

struct Proposal {
    std::vector<BufferTreePlan> plans;
    unsigned nets = 0;
    double predicted_gain = 0;
};
struct Proposals {
    std::vector<Proposal> paths;
    unsigned sampled = 0, split_paths = 0, truncated = 0;
};
// Read-only planning. Buffer-aware estimates are local to the sampled paths;
// live OpenSTA remains the authority for all acceptance decisions.
Proposals generate(odb::dbBlock* block, sta::dbSta* sta, rsz::Resizer* resizer,
                   std::span<const detail::NetState> states, const PbbiOptions& options,
                   const DesignConstraints& constraints);
} // namespace ecc::pbbi
