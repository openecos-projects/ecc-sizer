// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_drv.h"
#include "repair_path.h"
#include "repair_vg.h"
#include "repair_gobi.h"

namespace ord {
class Design;
}

namespace ecc {
enum class ParasiticsMode;
struct SetupRepairMetrics {
    DesignRepairMetrics design;
    float setup_tns = 0;
    float hold_tns = 0;
    unsigned slew_violations = 0;
    unsigned cap_violations = 0;
    unsigned fanout_violations = 0;
};

struct SetupRepairResult {
    bool ok = false;
    bool converged = false;
    std::string message;
    unsigned accepted_batches = 0;
    unsigned rejected_batches = 0;
    unsigned attempted_batches = 0;
    unsigned skipped_duplicate_batches = 0;
    unsigned refreshes = 0;
    unsigned rollback_refreshes = 0;
    double candidate_ms = 0;
    double prepare_ms = 0;
    double refresh_ms = 0;
    double measure_ms = 0;
    unsigned inserted_buffers = 0;
    unsigned resized_cells = 0;
    unsigned downsized_loads = 0;
    unsigned vt_swaps = 0;
    SetupRepairMetrics before;
    SetupRepairMetrics after;
};

// Setup-only native strategy. Shares OpenDB/STA models with design repair.
// Input must have current RC/STA; common.prepare legalizes each candidate ECO.
// Failed trials restore topology and placement immediately. With deferred
// refresh, STA is refreshed after the next edit or before returning; baseline
// metrics/plans are reused only while the committed design stays unchanged.
// Path/VG/GOBI are independent timing strategies selected by the application.
class OpenDbSetupRepairer {
   public:
    static SetupRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                    const std::function< void() >& refresh,
                                    const SetupRepairOptions& options = {});
};
// Existing cell-sizing search on the caller's loaded OpenROAD design.
void runCellSizing(ord::Design& design, const RepairConfig& config,
                   const std::function< void() >& refresh,
                   ParasiticsMode parasitics_mode);
}  // namespace ecc
