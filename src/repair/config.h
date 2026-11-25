// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <iosfwd>
#include <optional>
#include <span>
#include <cstddef>
#include <string>
#include <vector>

namespace ecc {
enum class PathPlacement { Trial, Final };
enum class PathVerification { Trial, Final };
enum class PathGainScope { Global, CurrentGlobal, Endpoints };
enum class PowerObjective { Leakage, Total };
enum class VgScope { Drv, Setup };
enum class VgBackend { Cpu, Cuda };

// All time values in the C++ API are seconds; text configuration uses *_ns.
struct RepairTolerances {
    float wns_absolute = 1e-15f;
    float tns_absolute = 1e-15f;
    float tns_relative = 1e-6f;
    double drv_severity = 1e-6;
};
// An absent limit is unbounded. Zero is an actual count, never an enable switch.
unsigned remainingBufferBudget(std::optional<unsigned> limit, unsigned used);
std::vector<std::size_t> selectWholeBufferPlans(std::span<const std::size_t> counts,
                                                std::optional<unsigned> limit);

struct RepairActions {
    bool resize = true;
    bool buffer = true;
    bool load_downsize = true;
    bool operator==(const RepairActions&) const = default;
};
struct RepairCommonOptions {
    unsigned max_fanout = 64;
    float margin = 0.1f;
    float recovered_slew_fraction = 0.25f;
    double overload_group_fraction = 0.5;
    RepairActions actions;
    // Explicit same-drive VT suffixes, e.g. {"_R", "_L", "_SL"}.
    // Empty disables VT moves; no technology naming is guessed.
    std::vector<std::string> vt_suffixes;
    float load_downsize_slack_margin = 0;
    RepairTolerances tolerances;
    void validate() const;
};
struct DrvRepairSettings {
    unsigned max_passes = 8;
    std::optional<unsigned> max_inserted_buffers;
    unsigned max_load_downsizes_per_pass = 64;
    // OpenROAD-style repair commits driver edits in bounded traversal batches.
    unsigned ord_driver_batch_size = 10000;
    void validate() const;
};
struct DrvRepairOptions : DrvRepairSettings {
    RepairCommonOptions common;
    void validate() const;
};
struct SetupRepairSettings {
    unsigned max_passes = 6;
    unsigned max_path_ends = 16;
    unsigned paths_per_endpoint = 1;
    unsigned max_candidates = 32;
    // Setup repair uses this as the maximum number of independent ECOs
    // probed in one STA-verified batch.
    unsigned batch_size = 8;
    double batch_backoff = 0.5;
    std::optional<unsigned> max_trials = 24;
    // Both optimizations operate only while the committed design is unchanged.
    bool skip_duplicate_trials = true;
    bool defer_rollback_refresh = true;
    std::optional<unsigned> max_inserted_buffers;
    unsigned min_side_loads = 2;
    std::optional<double> max_area_growth = 0.02;
    float min_improvement = 1e-13f;
    void validate() const;
};
struct SetupRepairOptions : SetupRepairSettings {
    RepairCommonOptions common;
    void validate() const;
};

struct PathSearchSettings {
    unsigned paths_per_endpoint = 1;
    // Optional acceptance floor, in addition to STA numerical tolerances.
    float min_improvement = 0;
    bool defer_rollback_refresh = true;
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
    PathPlacement placement = PathPlacement::Trial;
    // "trial": full RC before each acceptance. "final": incremental RC/STA
    // and full electrical checks during search, with authoritative full RC
    // after final placement. Requires the phase-wide rollback journal.
    PathVerification verification = PathVerification::Trial;
    // Complete high-fanout candidates inside one ECO before RC/STA.
    bool fanout_repair = true;
    // Search setup and recovery separately so one check type cannot starve
    // the other. max_paths is a per-check-type sampling limit.
    bool recovery = true;
    // Plan whole-net trees with cap/fanout limits before candidate screening.
    bool tree_candidates = true;
    // "global" and "current_global" measure full-design gains. When a relative
    // threshold is enabled, they scale it by initial or committed remaining
    // violations, respectively. "endpoints" measures fixed sampled endpoints.
    // All scopes preserve global hold/DRV/non-regression guards.
    PathGainScope gain_scope = PathGainScope::Global;
    // Disabled by default: accept positive STA gains above the absolute floor
    // and TNS numerical tolerance. Positive values additionally require a
    // fraction of the selected reference and enable zero-slack-bound screening.
    double min_relative_improvement = 0;
    // Optional actual refresh limit. Absence means unlimited.
    // An in-flight STA/rollback always completes before returning.
    std::optional<unsigned> max_refreshes;
    unsigned max_paths = 16;
    unsigned max_candidates = 8;
    // Counts STA-verified ECO batches, including a batch that is rolled back.
    // Absence means unlimited; zero permits no trials.
    std::optional<unsigned> max_trials;
    // Each trial spends one unit; measured progress earns fractional units
    // back, capped at this initial budget. Failed batches are split first.
    // Absence disables this heuristic, not the execution/resource limits.
    std::optional<unsigned> search_patience = 32;
    // Fraction of best remaining WNS or TNS that earns one extra trial.
    // This controls continuation only; it never rejects an individual ECO.
    double progress_fraction = 0.05;
    // Default one validates each candidate independently; larger values enable
    // buffer-only ECO batching for throughput experiments.
    unsigned batch_size = 1;
    // Absence disables the area budget; zero permits no area growth.
    std::optional<double> max_area_growth = 0.02;
    void validate() const;
};
struct PathSearchOptions : PathSearchSettings {
    RepairCommonOptions common;
    std::optional<unsigned> max_inserted_buffers;
    void validate() const;
};

struct PowerRepairSettings {
    // "leakage" or "total"; both measured by OpenSTA in watts.
    PowerObjective objective = PowerObjective::Leakage;
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
struct VgSettings {
    // setup also includes electrically clean nets on negative-slack paths.
    VgScope scope = VgScope::Drv;
    // VG solver backend. CUDA runs propagation, merge, pruning and selection;
    // falls back to CPU if unavailable or over the GPU memory budget.
    VgBackend backend = VgBackend::Cpu;
    unsigned cuda_max_memory_mb = 512;
    std::optional<unsigned> max_nets;
    unsigned max_group_loads = 0;
    float min_gain = 0;
    bool require_drv_non_regression = true;
    void validate() const;
};
struct VgOptions : VgSettings {
    RepairCommonOptions common;
    std::optional<unsigned> max_buffers;
    void validate() const;
};
struct GobiSettings {
    // GOBI uses its CUDA solver; backend availability is checked at runtime.
    std::optional<unsigned> max_nets;
    bool require_drv_non_regression = true;
};
struct GobiOptions : GobiSettings {
    RepairCommonOptions common;
    std::optional<unsigned> max_buffers;
    void validate() const;
};
struct PbbiSettings {
    unsigned max_paths = 64;
    // Bounded Pareto approximation; unlimited retains the exact frontier.
    std::optional<unsigned> max_frontier = 128;
    void validate() const;
};
struct PbbiOptions : PbbiSettings {
    RepairCommonOptions common;
    std::optional<unsigned> max_buffers;
    void validate() const;
};
struct PowerRepairOptions : PowerRepairSettings {
    RepairCommonOptions common;
    void validate() const;
};

struct BufferSettings {
    std::optional<unsigned> max_inserted_buffers;
    PathSearchSettings path;
    VgSettings vg;
    GobiSettings gobi;
    PbbiSettings pbbi;
};

// Serializable policy shared by independently ordered repair stages.
// Algorithms also accept their own default-initialized options independently.
struct RepairConfig {
    RepairCommonOptions common;
    DrvRepairSettings drv;
    SetupRepairSettings setup;
    BufferSettings buffer;
    PowerRepairSettings power;
    unsigned sta_threads = 32;

    DrvRepairOptions drvOptions() const;
    SetupRepairOptions setupOptions() const;
    PathSearchOptions pathOptions() const;
    PowerRepairOptions powerOptions() const;
    VgOptions vgOptions() const;
    GobiOptions gobiOptions() const;
    PbbiOptions pbbiOptions() const;
    void validate() const;
    // Strict key/value parsing. Unknown keys, malformed values and NaN fail.
    void set(const std::string& key, const std::string& value);
    // Atomic load: a malformed file never partially changes this object.
    void read(std::istream& input, const std::string& source = "repair config");
    void readFile(const std::string& path);
    void write(std::ostream& output) const;
};

} // namespace ecc
