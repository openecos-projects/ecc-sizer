// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace odb {
class dbMaster;
class dbInst;
}  // namespace odb
namespace ecc {
struct PathBufferTrial;
struct PathFanoutCheck;
struct RepairAnalysisChanges;
// All time values in the C++ API are seconds; text configuration uses *_ns.
struct RepairTolerances {
    float wns_absolute = 1e-15f;
    float tns_absolute = 1e-15f;
    float tns_relative = 1e-6f;
    double drv_severity = 1e-6;
};
struct RepairCommonOptions {
    unsigned max_fanout = 64;
    float margin = 0.1f;
    float recovered_slew_fraction = 0.25f;
    double overload_group_fraction = 0.5;
    bool allow_gate_resize = true;
    bool allow_load_downsize = true;
    // Explicit same-drive VT suffixes, e.g. {"_R", "_L", "_SL"}.
    // Empty disables VT moves; no technology naming is guessed.
    std::vector< std::string > vt_suffixes;
    float load_downsize_slack_margin = 0;
    RepairTolerances tolerances;
    // Runtime hooks/constraints are not serialized as text configuration.
    std::function< void() > prepare;
    std::vector< odb::dbMaster* > excluded_masters;
    std::vector< odb::dbInst* > excluded_instances;
    void validate() const;
};
struct DesignRepairOptions : RepairCommonOptions {
    // Zero means unlimited. A positive value is an optional total buffer
    // insertion cap used to bound runtime and memory during experiments.
    unsigned max_passes = 8;
    unsigned max_inserted_buffers = 0;
    unsigned max_load_downsizes_per_pass = 64;
    // OpenROAD-style repair commits driver edits in bounded traversal batches.
    unsigned ord_driver_batch_size = 10000;
    void validate() const;
};
struct SetupRepairSettings {
    unsigned max_passes = 6;
    unsigned max_path_ends = 16;
    unsigned paths_per_endpoint = 1;
    unsigned max_candidates = 32;
    // Native setup repair uses this as the maximum number of independent ECOs
    // probed in one STA-verified batch.
    unsigned batch_size = 8;
    double batch_backoff = 0.5;
    unsigned max_trials = 24;
    // Both optimizations operate only while the committed design is unchanged.
    bool skip_duplicate_trials = true;
    bool defer_rollback_refresh = true;
    unsigned max_inserted_buffers = 64;
    unsigned min_side_loads = 2;
    double max_area_growth = 0.02;
    float min_improvement = 1e-13f;
    void validate() const;
};
struct SetupRepairOptions : SetupRepairSettings {
    RepairCommonOptions common;
    void validate() const;
};

struct PathRepairSettings {
    unsigned paths_per_endpoint = 1;
    float min_improvement = 1e-13f;
    bool defer_rollback_refresh = true;
    // Sequential is the measured reference; path_gain ranks multi-path ECOs.
    std::string strategy = "path_gain";
    // Feedback ablations; none changes the global STA acceptance tolerances.
    bool failure_cache = true;
    bool calibrate = true;
    bool diversify = true;
    // Opt-in scheduling ablations; acceptance still uses global STA.
    bool endpoint_coverage = false;
    bool group_schedule = false;
    // Opt-in empirical family exhaustion, audited on each committed design.
    bool adaptive_search = false;
    // Reject with global timing first; collect full electrical states only
    // for survivors. Does not change acceptance thresholds or allow commits.
    bool timing_precheck = true;
    // Placement RC only; global STA acceptance is unchanged.
    bool incremental_parasitics = false;
    // Reuse electrical checks outside edited timing cones in final verification
    // mode.
    bool incremental_analysis = false;
    // Single-scene failure reuse: TNS cone or a WNS endpoint fanin witness.
    bool reuse_rejections = false;
    // "trial": legalize every trial; "final": search at proposed locations,
    // then legalize and verify the whole phase, restoring its input on failure.
    std::string placement = "trial";
    // "trial": full RC before each acceptance. "final": incremental RC/STA
    // and full electrical checks during search, with authoritative full RC
    // after final placement. Requires the phase-wide rollback journal.
    std::string verification = "trial";
    // Complete high-fanout candidates inside one ECO before RC/STA.
    bool fanout_repair = true;
    // Search setup and recovery separately so one check type cannot starve
    // the other. max_paths is a per-check-type sampling limit.
    bool recovery = true;
    // Plan whole-net trees with cap/fanout limits before candidate screening.
    bool tree_candidates = true;
    // Relative gain is measured on fixed affected sampled endpoints for each
    // ECO. "global" uses initial full-design scaling; "current_global"
    // uses the committed remaining full-design violation, recomputed after
    // each acceptance. Neither changes global hold/DRV/non-regression guards.
    std::string gain_scope = "endpoints";
    // Relative to the selected gain scope.
    // Zero preserves the existing absolute improvement requirement.
    // Default search only screens with the zero-negative-slack upper bound.
    // Optional adaptive search additionally uses audited empirical evidence;
    // its stopping condition is heuristic, not a proof of global optimality.
    double min_relative_improvement = 0;
    // Optional wall time / actual refresh limits. Zero means unlimited.
    // An in-flight STA/rollback always completes before returning.
    double max_seconds = 0;
    unsigned max_refreshes = 0;
    unsigned max_paths = 16;
    unsigned max_candidates = 8;
    // Counts STA-verified ECO batches, including a batch that is rolled back.
    // Zero means unlimited; stop at timing closure or no acceptable candidate.
    unsigned max_trials = 8;
    // Optional early exit after this many rejected single-candidate STA
    // trials (including reused rejection verdicts) without an acceptance.
    // Zero disables it. Batch failures and
    // cheap prechecks do not count; final legalization/STA still runs.
    unsigned max_consecutive_rejections = 0;
    // Default one preserves sequential greedy behavior; larger values enable
    // buffer-only ECO batching for throughput experiments.
    unsigned batch_size = 1;
    // Zero means unlimited, subject to the area/trial budgets.
    unsigned max_inserted_buffers = 0;
    // -1 disables the area budget; zero permits no area growth.
    double max_area_growth = 0.02;
    void validate() const;
};
struct PathRepairOptions : PathRepairSettings {
    RepairCommonOptions common;
    // Called after commit/rollback; rejected trials can have deferred STA.
    std::function< void(unsigned, const PathBufferTrial&) > path_trial_observer;
    std::function< void(const PathFanoutCheck&) > path_fanout_observer;
    // Authoritative refresh for trial verification or final phase validation.
    // Final mode uses it after legalization, before committing the parent ECO.
    std::function< void() > path_verification_refresh;
    // Refresh accumulated edited timing cones for a promising incremental
    // trial. Final-phase mode still requires authoritative final RC/STA.
    std::function< void() > path_incremental_verification;
    std::function< RepairAnalysisChanges() > path_analysis_changes;
    void validate() const;
};

struct PowerRepairSettings {
    // "leakage" or "total"; both measured by OpenSTA in watts.
    std::string objective = "leakage";
    unsigned max_passes = 6;
    unsigned max_trials = 24;
    unsigned max_candidates = 64;
    unsigned batch_size = 8;
    double batch_backoff = 0.5;
    float slack_margin = 0;
    double min_relative_improvement = 1e-6;
    double min_absolute_improvement = 0;
    void validate() const;
};
struct VgRepairSettings {
    // setup also includes electrically clean nets on negative-slack paths.
    std::string scope = "drv";
    // VG solver backend. CUDA runs propagation, merge, pruning and selection;
    // falls back to CPU if unavailable or over the GPU memory budget.
    std::string backend = "cpu";
    unsigned cuda_max_memory_mb = 512;
    unsigned max_buffers = 64;
    unsigned max_nets = 0;
    unsigned max_group_loads = 0;
    float min_gain = 0;
    bool require_drv_reduction = true;
    void validate() const;
};
struct GobiRepairSettings {
    // The migrated GOBI implementation is GPU-only.  Keeping the backend in
    // the configuration makes the execution policy explicit and avoids a
    // silent fallback to the different CPU VG implementation.
    std::string backend = "cuda";
    unsigned max_buffers = 64;
    unsigned max_nets = 0;
    bool require_drv_reduction = true;
    void validate() const;
};
struct PowerRepairOptions : PowerRepairSettings {
    RepairCommonOptions common;
    void validate() const;
};

// A single source of effective policy for the native DRV -> setup pipeline.
// Algorithms also accept their own default-initialized options independently.
struct RepairConfig {
    DesignRepairOptions design;
    SetupRepairSettings setup;
    PathRepairSettings path;
    PowerRepairSettings power;
    VgRepairSettings vg;
    GobiRepairSettings gobi;
    unsigned sta_threads = 32;

    SetupRepairOptions setupOptions() const;
    PathRepairOptions pathOptions() const;
    PowerRepairOptions powerOptions() const;
    void validate() const;
    // Strict key/value parsing. Unknown keys, malformed values and NaN fail.
    void set(const std::string& key, const std::string& value);
    // Atomic load: a malformed file never partially changes this object.
    void read(std::istream& input, const std::string& source = "repair config");
    void readFile(const std::string& path);
    void write(std::ostream& output) const;
};

}  // namespace ecc
