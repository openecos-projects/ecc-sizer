// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

namespace odb {
class dbMaster;
class dbInst;
}  // namespace odb
namespace ecc {
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
    // Zero disables the corresponding work budget; it never means unlimited.
    unsigned max_passes = 8;
    unsigned max_inserted_buffers = 10000;
    unsigned max_load_downsizes_per_pass = 64;
};
struct SetupRepairSettings {
    unsigned max_passes = 6;
    unsigned max_path_ends = 16;
    unsigned paths_per_endpoint = 1;
    unsigned max_candidates = 32;
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
struct GbiRepairSettings {
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
struct PowerRepairOptions : PowerRepairSettings {
    RepairCommonOptions common;
    void validate() const;
};

// A single source of effective policy for the native DRV -> setup pipeline.
// Algorithms also accept their own default-initialized options independently.
struct RepairConfig {
    DesignRepairOptions design;
    SetupRepairSettings setup;
    PowerRepairSettings power;
    GbiRepairSettings gbi;
    unsigned sta_threads = 32;

    SetupRepairOptions setupOptions() const;
    PowerRepairOptions powerOptions() const;
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
