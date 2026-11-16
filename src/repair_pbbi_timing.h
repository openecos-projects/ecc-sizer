// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_pbbi_solver.h"
#include <string>
#include <algorithm>
#include <cmath>

namespace ecc::pbbi {
// A future buffered branch must not spend slack that the committed branch
// does not have. Existing violations are allowed to retain their boundary;
// apply this independently to rise/fall, never to the worst of both edges.
inline double offPathRequired(double forecast_required, double current_required,
                              double current_slack) {
    return std::isfinite(current_slack)
               ? std::min(forecast_required, current_required) -
                     std::min(0.0, current_slack)
               : forecast_required;
}

// A combinational timing projection between real STA boundaries. Reference
// arrivals/requireds calibrate the collapsed (untagged) graph to OpenSTA. No
// prediction is written into STA or used as a commit criterion.
struct TimingVertex {
    std::array< double, 2 > arrival{{-INFINITY, -INFINITY}};
    std::array< double, 2 > required{{INFINITY, INFINITY}};
    std::array< double, 2 > slew{{0, 0}};
};
struct TimingEdge {
    unsigned from = 0, to = 0;
    std::array< ArcResponse, 2 > reference;
    std::function< ArcResponse(double, unsigned) > predict;
};
struct TimingProjection {
    std::vector< TimingVertex > vertices;
    std::vector< std::array< int, 2 > > predecessor;
    std::vector< std::array< unsigned, 2 > > predecessor_rf;
    std::vector< unsigned > order;
    std::vector< std::vector< unsigned > > incoming, outgoing;
};
TimingProjection propagate(const std::vector< TimingVertex >& reference,
                           const std::vector< TimingEdge >& edges,
                           const std::vector< TimingVertex >& launch = {});

// Repeated candidate scoring needs arrival/slew, not a new RAT assignment.
// Own the immutable reference graph and build its adjacency/order once.
// The evaluator supplies candidate-dependent arc responses without allocating
// a closure and copying Liberty models for every edge of every candidate.
class TimingGraph {
   public:
    using Evaluator = std::function< ArcResponse(unsigned, double, unsigned) >;
    TimingGraph(std::vector< TimingVertex > reference,
                std::vector< TimingEdge > edges);
    std::vector< TimingVertex > forward(
        const std::vector< TimingVertex >& launch, const Evaluator& evaluate) const;
   private:
    std::vector< TimingVertex > reference_;
    std::vector< TimingEdge > edges_;
    std::vector< unsigned > order_;
    std::vector< std::vector< unsigned > > incoming_;
};
}  // namespace ecc::pbbi
