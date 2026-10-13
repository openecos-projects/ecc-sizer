// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_design.h"

namespace ecc {
struct PowerRepairMetrics {
    DesignRepairMetrics design;
    double leakage = 0, internal = 0, switching = 0, total = 0;  // watts
};
struct PowerRepairResult {
    bool ok = false;
    std::string message;
    unsigned accepted_batches = 0, rejected_batches = 0, attempted_batches = 0;
    unsigned downsized_cells = 0, vt_swaps = 0, refreshes = 0;
    double elapsed_ms = 0;
    PowerRepairMetrics before, after;
};
// Native slack-budgeted leakage/total-power recovery after DRV/setup repair.
// Uses the caller's STA activity assumptions. No Resizer strategy calls.
class OpenDbPowerRepairer {
   public:
    static PowerRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                    const std::function< void() >& refresh,
                                    const PowerRepairOptions& options = {});
};
}  // namespace ecc
