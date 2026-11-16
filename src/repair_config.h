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
    // Experimental path-level buffer repair.  Each accepted edit is
    // re-extracted from STA and verified with a global timing update.
    bool path_buffer_enabled = false;
    bool path_buffer_only = false;
    // Sequential is the measured reference; path_gain ranks multi-path ECOs.
    std::string path_buffer_strategy = "path_gain";
    // Complete high-fanout candidates inside one ECO before RC/STA.
    bool path_buffer_fanout_repair = true;
    // Search setup and recovery separately so one check type cannot starve
    // the other. max_paths is a per-check-type sampling limit.
    bool path_buffer_recovery = true;
    // Plan whole-net trees with cap/fanout limits before candidate screening.
    bool path_buffer_tree_candidates = true;
    // Relative gain is measured on fixed affected sampled endpoints for each
    // ECO. "global" keeps initial full-design scaling for controlled ablation.
    std::string path_buffer_gain_scope = "endpoints";
    // Relative to the selected gain scope.
    // Zero preserves the existing absolute improvement requirement.
    // In feedback mode a positive value also screens predicted improvements;
    // this is approximate and can exclude gains underestimated by the model.
    double path_buffer_min_relative_improvement = 0;
    // Optional wall time / actual refresh limits. Zero means unlimited.
    // An in-flight STA/rollback always completes before returning.
    double path_buffer_max_seconds = 0;
    unsigned path_buffer_max_refreshes = 0;
    unsigned path_buffer_max_paths = 16;
    unsigned path_buffer_max_candidates = 8;
    // Counts STA-verified ECO batches, including a batch that is rolled back.
    // Zero means unlimited; stop at timing closure or no acceptable candidate.
    unsigned path_buffer_max_trials = 8;
    // Default one preserves sequential greedy behavior; larger values enable
    // buffer-only ECO batching for throughput experiments.
    unsigned path_buffer_batch_size = 1;
    // Zero means unlimited, subject to the area/trial budgets.
    unsigned path_buffer_max_inserted_buffers = 0;
    // -1 disables the area budget; zero permits no area growth.
    double path_buffer_max_area_growth = 0.02;
    void validate() const;
};
struct SetupRepairOptions : SetupRepairSettings {
    RepairCommonOptions common;
    // Called after commit/rollback; rejected trials can have deferred STA.
    std::function< void(unsigned, const PathBufferTrial&) > path_trial_observer;
    std::function< void(const PathFanoutCheck&) > path_fanout_observer;
    void validate() const;
};

struct PowerRepairSettings {
    bool enabled = false;
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
struct GbiRepairSettings {
    // The migrated GBI implementation is GPU-only.  Keeping the backend in
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

struct PbbiRepairSettings {
    // net is a same-model ablation: solve the selected paths one net at a
    // time, retaining driver/sink sizing and the full STA acceptance policy.
    std::string scope = "path";
    // area first reaches explicit setup targets, then reduces area while
    // retaining those targets and the input timing floor. Times are seconds.
    std::string objective = "timing";
    double target_wns = 0;
    std::optional< double > target_tns;
    bool incremental_placement = false;
    bool placement_feedback = false;
    // adaptive admits one path on demand and rotates after rejected trials.
    std::string scheduler = "sequential";
    unsigned scheduler_window = 2; // paths admitted before first global trial
    bool forecast_cache = true;
    bool path_cache = true;
    bool pin_diagnostics = true;
    bool candidate_electrical_check = true;
    bool operating_point_pruning = true;
    bool failure_feedback = true;
    bool buffer_aware_timing = true;
    bool slew_propagation = true;
    unsigned slew_samples = 12;
    unsigned slew_cache_entries = 262144; // zero disables memoization
    unsigned timing_max_frontier_states = 16;
    unsigned timing_max_expansions = 0;
    bool enabled = false;
    bool gate_sizing = true;
    bool sink_sizing = true;
    unsigned max_paths = 16;
    unsigned paths_per_endpoint = 1;
    unsigned max_trials = 8;
    unsigned max_frontier_states = 128; // zero: exact, potentially expensive
    unsigned max_expansions = 0;  // zero: unlimited per path solve
    unsigned max_inserted_buffers = 0;
    unsigned max_refreshes = 0;
    double max_seconds = 0;
    double max_area_growth = 0.02;
    float min_improvement = 1e-13f;
    bool defer_rollback_refresh = true;
    void validate() const;
};
struct PbbiRepairOptions : PbbiRepairSettings {
    RepairCommonOptions common;
    void validate() const;
};

// A single source of effective policy for the native DRV -> setup pipeline.
// Algorithms also accept their own default-initialized options independently.
struct RepairConfig {
    DesignRepairOptions design;
    SetupRepairSettings setup;
    PowerRepairSettings power;
    VgRepairSettings vg;
    GbiRepairSettings gbi;
    PbbiRepairSettings pbbi;
    unsigned sta_threads = 32;

    SetupRepairOptions setupOptions() const;
    PowerRepairOptions powerOptions() const;
    PbbiRepairOptions pbbiOptions() const;
    void validate() const;
    // Strict key/value parsing. Unknown keys, malformed values and NaN fail.
    void set(const std::string& key, const std::string& value);
    // Atomic load: a malformed file never partially changes this object.
    void read(std::istream& input, const std::string& source = "repair config");
    void readFile(const std::string& path);
    void write(std::ostream& output) const;
};

// Load -sizer_repair_config files first; explicit CLI overrides win regardless
// of their position relative to a config-file option in the command file.
void loadRepairConfigFiles(RepairConfig& config,
                           const std::string& command_file);
bool applyRepairConfigOverride(RepairConfig& config, const std::string& line);
}  // namespace ecc
