// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <array>
#include <functional>
#include <cmath>

namespace ecc::pbbi {
// All times are seconds, capacitances farads, resistance ohms. Cost is area.
// Nodes are postordered. A gate is mandatory; its original choice has zero
// incremental cost and edit=false. It never represents a new buffer instance.
enum class Kind { load, wire, junction, gate };
struct ArcResponse {
    // One input transition can reach either/both output transitions.
    std::array< double, 2 > delay{{-INFINITY, -INFINITY}};
    std::array< double, 2 > slew{{0, 0}};
};
using GateResponse = std::function< ArcResponse(double, double, unsigned) >;
struct Choice {
    double cap = 0, intrinsic = 0, resistance = 0, cost = 0;
    double max_cap = std::numeric_limits< double >::infinity();
    bool edit = false;
    GateResponse response;
    double input_slew_limit = std::numeric_limits< double >::infinity();
    double output_slew_limit = std::numeric_limits< double >::infinity();
    // Sink sizing: RAT as a function of the arriving transition and slew.
    std::function< double(double, unsigned) > sink_required;
};
struct Node {
    Kind kind = Kind::load;
    int left = -1, right = -1;
    double cap = 0, resistance = 0, required = 0;
    std::vector< Choice > choices;
    std::array< double, 2 > wire_slew_factor{{std::log(9.0), std::log(9.0)}};
    double slew_limit = std::numeric_limits< double >::infinity();
    std::array< double, 2 > required_rf{{INFINITY, INFINITY}};
    std::array< double, 2 > reference_slew{{-1, -1}};
    double requiredFor(unsigned rf) const {
        return required_rf[0] != INFINITY || required_rf[1] != INFINITY
                   ? required_rf[rf]
                   : required;
    }
};
struct Trace {
    int node = -1, choice = -1;
    std::shared_ptr< const Trace > left, right;
    double cap = 0;
    size_t serial = 0;
};
struct SlewCache;
struct State {
    double cap = 0, required = 0, cost = 0;
    unsigned buffers = 0, resizes = 0;
    std::shared_ptr< const Trace > trace;
    // Permissible arrival vs input slew, separately for rise/fall. -inf means
    // electrically infeasible. Shared ownership keeps trace expansion cheap.
    std::shared_ptr< const std::vector< std::array< double, 2 > > >
        required_curve;
    std::shared_ptr< const std::vector< Node > > models;
    std::shared_ptr< SlewCache > exact_cache;
    double ranking_required = 0;
};
struct Limits {
    // Checked between DP expansion groups and dominance comparisons. A
    // cancelled solve returns no partial frontier and is never cacheable.
    std::function< bool() > stop_requested;
    size_t max_slew_cache_entries = 262144;  // zero disables exact memoization
    size_t max_states = 128;    // zero: exact frontier, no truncation
    size_t max_expansions = 0;  // zero: unlimited per solve
    unsigned max_buffers = 0;   // zero: unlimited
    double max_cost = std::numeric_limits< double >::infinity();
    bool edits = true;
    bool operating_point_pruning = true;
    std::vector< double > slew_grid;  // empty selects the linear RC model
    std::array< double, 2 > root_slew{{0, 0}};
};
struct Result {
    std::vector< State > frontier;
    size_t expansions = 0, peak_states = 0, truncated = 0;
    bool exhausted = false;
    bool cancelled = false;
    size_t slew_rejected = 0, response_evaluations = 0;
    size_t slew_cache_hits = 0, slew_cache_misses = 0;
};
double requiredAt(const State& state, const std::vector< double >& grid,
                  double slew, unsigned rf);
struct LeafTiming {
    int node;
    std::array< double, 2 > arrival, slew;
};
struct Simulation {
    bool feasible = true;
    double required = INFINITY;
    std::vector< LeafTiming > leaves;
};
Simulation simulate(const std::vector< Node >& nodes,
                    const std::shared_ptr< const Trace >& trace,
                    std::array< double, 2 > slew, bool enforce_limits = true);
Result solve(const std::vector< Node >& nodes, int root,
             const Limits& limits = {});
}  // namespace ecc::pbbi
