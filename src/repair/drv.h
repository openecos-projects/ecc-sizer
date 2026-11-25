// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair/config.h"
#include "actions/netlist.h"
#include "design_analysis.h"
#include <functional>
#include <string>
#include <vector>
namespace odb {
class dbBlock;
}
namespace sta {
class dbSta;
}
namespace ecc {
namespace detail {
class CellCandidates;
}
// Alternatives planned against one measured design snapshot.
struct DrvNetPlan {
    std::optional<CellReplacement> replacement;
    std::vector<BufferInsertion> buffers;
};
DrvNetPlan planDrvNetRepair(detail::DesignAnalysis& analysis,
                            detail::CellCandidates& cells, sta::dbSta* sta,
                            const detail::NetState& input, unsigned available,
                            bool allow_resize, const RepairCommonOptions& options);

struct DrvRepairResult {
    bool ok = false;
    bool converged = false;
    std::string message;
    unsigned accepted_passes = 0;
    unsigned rejected_passes = 0;
    unsigned inserted_buffers = 0;
    unsigned resized_buffers = 0;
    unsigned resized_gates = 0;
    unsigned vt_swaps = 0;
    unsigned downsized_loads = 0;
    unsigned rejected_load_batches = 0;
    DesignMetrics before;
    DesignMetrics after;
};

// DRV decisions on an OpenDB/dbSta backend. No Resizer repair/move
// calls. refresh must re-estimate placement parasitics and update STA after
// both edits and rollback. Single STA scene; invoke at a topology
// synchronization boundary.
class DrvRepairer {
  public:
    static DrvRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                               const std::function<void()>& refresh,
                               const DrvRepairOptions& options = {},
                               const RepairRuntime& runtime = {});
};

// OpenROAD-style DRV repair: drivers are visited in the reverse levelized
// order used by RepairDesign::repairDriver().  The implementation keeps the
// Sizer transaction/acceptance boundary and OpenDB inserter, while exposing a
// separate traversal policy for comparison with DrvRepairer.
struct OrdDrvRepairResult {
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
    DesignMetrics before;
    DesignMetrics after;
};

class OrdDrvRepairer {
  public:
    static OrdDrvRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                                  const std::function<void()>& refresh,
                                  const DrvRepairOptions& options = {},
                                  const RepairRuntime& runtime = {});
};

struct FanoutRepairOptions {
    unsigned max_fanout = 64;
    unsigned max_levels = 32;
    bool repair_clock_nets = false;
    std::vector<odb::dbNet*> excluded_nets;
    BufferInsertionOptions insertion;
};

enum class FanoutRepairStatus {
    kSuccess,
    kInvalidBlock,
    kInvalidBufferMaster,
    kInvalidMaxFanout,
    kInsertionFailed,
    kMaxLevelsExceeded,
};

struct FanoutRepairResult {
    FanoutRepairStatus status = FanoutRepairStatus::kInsertionFailed;
    std::string message;
    unsigned nets_examined = 0;
    unsigned violating_nets = 0;
    unsigned before_max_fanout = 0;
    unsigned after_max_fanout = 0;
    std::vector<odb::dbInst*> inserted_buffers;
    std::vector<odb::dbNet*> affected_nets;

    bool ok() const { return status == FanoutRepairStatus::kSuccess; }
    explicit operator bool() const { return ok(); }
};

class FanoutRepairer {
  public:
    static FanoutRepairResult run(odb::dbBlock* block, odb::dbMaster* buffer_master,
                                  const FanoutRepairOptions& options = {});

    static unsigned loadCount(odb::dbNet* net);
    static unsigned maxFanout(odb::dbBlock* block, bool include_clock_nets = false);
};

struct PowerRepairMetrics {
    DesignMetrics design;
    double leakage = 0, internal = 0, switching = 0, total = 0; // watts
};
struct PowerRepairResult {
    bool ok = false;
    std::string message;
    unsigned accepted_batches = 0, rejected_batches = 0;
    // Total STA trials: power proposals plus nested DRV/setup trials.
    unsigned attempted_trials = 0;
    unsigned downsized_cells = 0, vt_swaps = 0, refreshes = 0;
    unsigned recovery_attempts = 0;
    double elapsed_ms = 0;
    PowerRepairMetrics before, after;
};
// Leakage/total-power search with bounded DRV/setup recovery.
// Power and recovery trials share max_trials; every compound edit must reduce
// power without worsening area, setup/hold or DRV versus the committed design.
// Uses the caller's STA activity assumptions. No Resizer strategy calls.
class PowerRepairer {
  public:
    static PowerRepairResult run(odb::dbBlock* block, sta::dbSta* sta,
                                 const std::function<void()>& refresh,
                                 const PowerRepairOptions& options = {},
                                 const RepairRuntime& runtime = {});
};
} // namespace ecc
