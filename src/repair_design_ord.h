// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "repair_design.h"

namespace ecc {

// OpenROAD-style DRV repair: drivers are visited in the reverse levelized
// order used by RepairDesign::repairDriver().  The implementation keeps the
// Sizer transaction/acceptance boundary and OpenDB inserter, while exposing a
// separate strategy for A/B evaluation against Native repair.
struct OrdDesignRepairResult {
    bool ok = false;
    bool converged = false;
    std::string message;
    unsigned passes = 0;
    unsigned repaired_nets = 0;
    unsigned inserted_buffers = 0;
    unsigned resized_buffers = 0;
    unsigned resized_gates = 0;
    unsigned vt_swaps = 0;
    unsigned rejected_passes = 0;
    DesignRepairMetrics before;
    DesignRepairMetrics after;
};

class OpenDbOrdDesignRepairer {
   public:
    static OrdDesignRepairResult repair(
        odb::dbBlock* block, sta::dbSta* sta,
        const std::function< void() >& refresh,
        const DesignRepairOptions& options = {});
};

}  // namespace ecc
