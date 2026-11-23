// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "design_analysis.h"

namespace ecc::detail {
// Candidate generation only; measured models belong to DesignAnalysis.
class CellCandidates {
  public:
    CellCandidates(DesignAnalysis& analysis, sta::dbSta* sta,
                   const RepairCommonOptions& options);
    std::vector<const CellModel*> resize(const NetState& state, bool downsize = false,
                                         bool vt_only = false, bool check_load = true);

  private:
    DesignAnalysis& analysis_;
    sta::dbNetwork* network_;
    const RepairCommonOptions& opts_;
    std::unordered_map<sta::LibertyCell*, std::vector<CellModel>> equivalents_;
};
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
    LoadDownsizePlanner(DesignAnalysis& analysis, sta::dbSta* sta,
                        const std::vector<NetState>& states,
                        const RepairCommonOptions& options);
    std::vector<LoadDownsizeCandidate>
    candidates(const NetState& source,
               const std::unordered_set<odb::dbObject*>& protected_loads = {});

  private:
    DesignAnalysis& analysis_;
    CellCandidates cells_;
    sta::dbSta* sta_;
    const RepairCommonOptions& options_;
    std::unordered_map<odb::dbInst*, const NetState*> outputs_;
};
} // namespace ecc::detail
