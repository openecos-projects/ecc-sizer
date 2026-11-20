// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_config.h"
#include "design_edit.h"
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
struct DesignRepairMetrics {
    unsigned fanout_nets = 0;
    unsigned capacitance_nets = 0;
    unsigned slew_nets = 0;
    unsigned skipped_nets = 0;
    double severity = 0.0;
    double area = 0.0;       // OpenDB area units (DBU squared).
    float setup_wns = 0.0f;  // STA internal seconds.
    float hold_wns = 0.0f;
    float setup_tns = 0.0f;
    float hold_tns = 0.0f;
    unsigned total_slew_violations = 0;
    unsigned total_cap_violations = 0;
    unsigned total_fanout_violations = 0;
};

struct DesignRepairResult {
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
    DesignRepairMetrics before;
    DesignRepairMetrics after;
};

// Native DRV decisions on an OpenDB/dbSta backend. No Resizer repair/move
// calls. refresh must re-estimate placement parasitics and update STA after
// both edits and rollback. Single STA scene; invoke at a topology
// synchronization boundary.
class OpenDbDesignRepairer {
   public:
    static DesignRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                     const std::function< void() >& refresh,
                                     const DesignRepairOptions& options = {});
};

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

struct FanoutRepairOptions {
    unsigned max_fanout = 64;
    unsigned max_levels = 32;
    bool repair_clock_nets = false;
    std::vector< odb::dbNet* > excluded_nets;
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
    std::vector< odb::dbInst* > inserted_buffers;
    std::vector< odb::dbNet* > affected_nets;

    bool ok() const {
        return status == FanoutRepairStatus::kSuccess;
    }
    explicit operator bool() const {
        return ok();
    }
};

class OpenDbFanoutRepairer {
   public:
    static FanoutRepairResult repair(odb::dbBlock* block,
                                     odb::dbMaster* buffer_master,
                                     const FanoutRepairOptions& options = {});

    static unsigned loadCount(odb::dbNet* net);
    static unsigned maxFanout(odb::dbBlock* block,
                              bool include_clock_nets = false);
};

struct PowerRepairMetrics {
    DesignRepairMetrics design;
    double leakage = 0, internal = 0, switching = 0, total = 0;  // watts
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
// Native leakage/total-power search with bounded DRV/setup recovery.
// Power and recovery trials share max_trials; every compound edit must reduce
// power without worsening area, setup/hold or DRV versus the committed design.
// Uses the caller's STA activity assumptions. No Resizer strategy calls.
class OpenDbPowerRepairer {
   public:
    static PowerRepairResult repair(odb::dbBlock* block, sta::dbSta* sta,
                                    const std::function< void() >& refresh,
                                    const PowerRepairOptions& options = {});
};
}  // namespace ecc
