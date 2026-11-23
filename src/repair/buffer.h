// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "design_analysis.h"
#include "repair/config.h"
#include "repair/runtime.h"

namespace rsz {
class Resizer;
}
namespace ecc {
// One ECO for independent net plans. Failed global timing/DRV acceptance
// rolls back every edit and refreshes the restored design.
struct BufferRepairResult {
    bool ok = false;
    bool committed = false;
    bool converged = false;
    std::string message;
    std::string backend_used = "cpu";
    std::string cuda_fallback;
    unsigned nets_collected = 0;
    unsigned plans_collected = 0;
    unsigned attempted_buffers = 0;
    unsigned inserted_buffers = 0;
    unsigned refreshes = 0;
    double collect_ms = 0;
    double solve_ms = 0;
    double apply_ms = 0;
    DesignMetrics before;
    DesignMetrics after;
};

class VgRepairer {
  public:
    static BufferRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                                  const std::function<void()>& refresh,
                                  const VgOptions& options = {},
                                  const RepairRuntime& runtime = {});
    static BufferRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                                  const std::function<void()>& refresh,
                                  rsz::Resizer* resizer, const VgOptions& options = {},
                                  const RepairRuntime& runtime = {});
};

class GobiRepairer {
  public:
    static BufferRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                                  const std::function<void()>& refresh,
                                  rsz::Resizer* resizer,
                                  const GobiOptions& options = {},
                                  const RepairRuntime& runtime = {});
};
} // namespace ecc
