// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_pbbi_objective.h"
#include <set>
#include <string>
#include <algorithm>

namespace ecc::pbbi {
// A footprint includes all inputs/outputs of sized gates, not only edited
// nets. Endpoint names describe timing coupling across otherwise distinct nets.
struct Footprint {
    std::set< std::string > nets, gates, endpoints;
    bool overlaps(const Footprint& other) const {
        auto intersects = [](const auto& a, const auto& b) {
            for(const auto& x : a)
                if(b.count(x))
                    return true;
            return false;
        };
        return intersects(nets, other.nets) || intersects(gates, other.gates) ||
               intersects(endpoints, other.endpoints);
    }
};

// A ranking preference, never a reason to discard a candidate. Reuse the
// same WNS/TNS tolerances as STA acceptance; do not invent a score weight.
inline bool predictedSetupRegresses(double wns_gain, double tns_gain,
                                    double wns_tolerance, double tns_tolerance) {
    return !(wns_gain >= -wns_tolerance && tns_gain >= -tns_tolerance);
}

struct Priority {
    double severity = 0, wns_gain = 0, tns_gain = 0, cost = 0;
    unsigned failed_actions = 0, actions = 0, path_rank = 0;
    std::string key;
    bool target_met = false;
};

// Deterministic ranking: never mix time, area and violation severity through
// arbitrary weighted sums. Runtime budgets govern admission; measured wall
// time is deliberately not a tie-breaker (shared-host load must not alter PPA).
inline bool preferred(const Priority& a, const Priority& b,
                      double wns_tolerance, double tns_tolerance,
                      bool minimize_area = false) {
    if(a.severity != b.severity)
        return a.severity < b.severity;
    if(minimize_area) {
        if(a.target_met != b.target_met)
            return a.target_met;
        if(a.target_met && a.cost != b.cost)
            return a.cost < b.cost;
    }
    auto regresses = [&](const auto& p) {
        return predictedSetupRegresses(p.wns_gain, p.tns_gain,
                                       wns_tolerance, tns_tolerance);
    };
    if(regresses(a) != regresses(b))
        return !regresses(a);
    if(a.failed_actions != b.failed_actions)
        return a.failed_actions < b.failed_actions;
    if(a.wns_gain != b.wns_gain)
        return a.wns_gain > b.wns_gain;
    if(a.tns_gain != b.tns_gain)
        return a.tns_gain > b.tns_gain;
    if(a.cost != b.cost)
        return a.cost < b.cost;
    if(a.actions != b.actions)
        return a.actions < b.actions;
    if(a.path_rank != b.path_rank)
        return a.path_rank < b.path_rank;
    return a.key < b.key;
}
}  // namespace ecc::pbbi
