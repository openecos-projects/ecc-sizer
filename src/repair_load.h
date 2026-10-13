// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_backend.h"

namespace ecc::detail {
struct LoadDownsizeCandidate {
    odb::dbInst* inst;
    odb::dbMaster* master;
    float cap_reduction;
    float driver_gain;
    float slack;
    double added_area;
};

// Read-only candidate generation shared by DRV/setup. Valid only for the
// measured committed design. No Sizer snapshot and no edits inside this class.
class LoadDownsizePlanner {
   public:
    LoadDownsizePlanner(Repair& backend, sta::dbSta* sta,
                        const std::vector< NetState >& states,
                        const RepairCommonOptions& options);
    std::vector< LoadDownsizeCandidate > candidates(
        const NetState& source,
        const std::unordered_set< odb::dbObject* >& protected_loads = {});

   private:
    Repair& backend_;
    sta::dbSta* sta_;
    const RepairCommonOptions& options_;
    std::unordered_map< odb::dbInst*, const NetState* > outputs_;
};
}  // namespace ecc::detail
