// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_design.h"
#include <algorithm>
#include <cmath>

namespace ecc::detail {
inline float tnsTolerance(float baseline, const RepairTolerances& tolerances) {
    return std::max(tolerances.tns_absolute,
                    std::abs(baseline) * tolerances.tns_relative);
}
inline bool timingNonRegressing(const DesignRepairMetrics& before,
                                const DesignRepairMetrics& after,
                                const RepairTolerances& tolerances) {
    return after.setup_wns + tolerances.wns_absolute >= before.setup_wns &&
           after.hold_wns + tolerances.wns_absolute >= before.hold_wns &&
           after.setup_tns + tnsTolerance(before.setup_tns, tolerances) >=
               before.setup_tns &&
           after.hold_tns + tnsTolerance(before.hold_tns, tolerances) >=
               before.hold_tns;
}
inline bool noNewDrv(const DesignRepairMetrics& before,
                     const DesignRepairMetrics& after) {
    return after.total_slew_violations <= before.total_slew_violations &&
           after.total_cap_violations <= before.total_cap_violations &&
           after.total_fanout_violations <= before.total_fanout_violations;
}
}  // namespace ecc::detail
