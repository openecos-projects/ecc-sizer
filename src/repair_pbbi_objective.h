// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <string>
#include <vector>

namespace ecc::pbbi {
struct SetupPoint {
    double wns = 0, tns = 0, area = 0;
};
struct TimingTarget {
    double wns = 0, tns = 0;
    double wns_tolerance = 0, tns_tolerance = 0;
    bool met(double candidate_wns, double candidate_tns) const {
        return std::isfinite(candidate_wns) && std::isfinite(candidate_tns) &&
               candidate_wns + wns_tolerance >= wns &&
               candidate_tns + tns_tolerance >= tns;
    }
};

// Setup/area part of acceptance only; caller must also enforce area budget,
// hold, DRV and placement. Targets are fixed and clamped to the input before
// constructing this policy. A direct transition into the feasible region may
// consume surplus in either setup metric, even if the previous point was not
// yet feasible in the other metric.
inline std::vector< std::string > setupRejections(
    const SetupPoint& before, const SetupPoint& after,
    const TimingTarget& target, bool minimize_area, double min_gain,
    double current_tns_tolerance) {
    const bool recovering = minimize_area && target.met(before.wns, before.tns);
    const bool reaching = minimize_area && target.met(after.wns, after.tns);
    const bool bounded = recovering || reaching;
    std::vector< std::string > reasons;
    if(!(after.wns + target.wns_tolerance >= (bounded ? target.wns : before.wns)))
        reasons.push_back("setup_wns");
    if(!(after.tns + (bounded ? target.tns_tolerance : current_tns_tolerance) >=
         (bounded ? target.tns : before.tns)))
        reasons.push_back("setup_tns");
    if(recovering) {
        if(!reaching) reasons.push_back("setup_target");
        if(!(after.area < before.area)) reasons.push_back("no_area_gain");
    }
    else if(!reaching && !(after.wns > before.wns + min_gain ||
                           after.tns > before.tns + min_gain))
        reasons.push_back("no_setup_gain");
    return reasons;
}
}  // namespace ecc::pbbi
