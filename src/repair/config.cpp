// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair/config.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <string_view>
#include <type_traits>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ecc {
unsigned remainingBufferBudget(std::optional<unsigned> limit, unsigned used) {
    if (!limit) {
        return std::numeric_limits<unsigned>::max();
    }
    return used >= *limit ? 0 : *limit - used;
}

// Select complete plans in priority order; skip oversized plans and allow
// smaller later plans. This limits application, never a solver's frontier.
std::vector<std::size_t> selectWholeBufferPlans(std::span<const std::size_t> counts,
                                                std::optional<unsigned> limit) {
    std::vector<std::size_t> selected;
    auto remaining = remainingBufferBudget(limit, 0);
    for (std::size_t i = 0; i < counts.size(); ++i) {
        if (counts[i] && (!limit || counts[i] <= remaining)) {
            selected.push_back(i);
            if (limit) {
                remaining -= static_cast<unsigned>(counts[i]);
            }
        }
    }
    return selected;
}

static std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    return text.substr(begin, text.find_last_not_of(" \t\r\n") - begin + 1);
}
static void require(bool valid, const std::string& key) {
    if (!valid) {
        throw std::invalid_argument("invalid repair configuration: " + key);
    }
}
static bool boolean(const std::string& value) {
    require(value == "true" || value == "false", "expected true/false: " + value);
    return value == "true";
}
static unsigned count(const std::string& text) {
    unsigned value = 0;
    const auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), value);
    require(error == std::errc{} && end == text.data() + text.size() &&
                value <= unsigned(std::numeric_limits<int>::max()),
            text);
    return value;
}
static double real(const std::string& text) {
    double value = 0;
    const auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), value);
    require(error == std::errc{} && end == text.data() + text.size() &&
                std::isfinite(value),
            text);
    return value;
}
static std::pair<std::string, std::string> assignment(const std::string& text) {
    const auto split = text.find('=');
    require(split != std::string::npos, "expected key=value: " + text);
    const auto key = trim(text.substr(0, split)), value = trim(text.substr(split + 1));
    require(!key.empty() && !value.empty(), "expected key=value: " + text);
    return {key, value};
}

static constexpr auto enumValues(PathStrategy) {
    using enum PathStrategy;
    return std::array{std::pair{std::string_view{"sequential"}, Sequential},
                      std::pair{std::string_view{"path_gain"}, PathGain},
                      std::pair{std::string_view{"feedback"}, Feedback}};
}
static constexpr auto enumValues(PathPlacement) {
    using enum PathPlacement;
    return std::array{std::pair{std::string_view{"trial"}, Trial},
                      std::pair{std::string_view{"final"}, Final}};
}
static constexpr auto enumValues(PathVerification) {
    using enum PathVerification;
    return std::array{std::pair{std::string_view{"trial"}, Trial},
                      std::pair{std::string_view{"final"}, Final}};
}
static constexpr auto enumValues(PathGainScope) {
    using enum PathGainScope;
    return std::array{std::pair{std::string_view{"global"}, Global},
                      std::pair{std::string_view{"current_global"}, CurrentGlobal},
                      std::pair{std::string_view{"endpoints"}, Endpoints}};
}
static constexpr auto enumValues(PowerObjective) {
    using enum PowerObjective;
    return std::array{std::pair{std::string_view{"leakage"}, Leakage},
                      std::pair{std::string_view{"total"}, Total}};
}
static constexpr auto enumValues(VgScope) {
    using enum VgScope;
    return std::array{std::pair{std::string_view{"drv"}, Drv},
                      std::pair{std::string_view{"setup"}, Setup}};
}
static constexpr auto enumValues(VgBackend) {
    using enum VgBackend;
    return std::array{std::pair{std::string_view{"cpu"}, Cpu},
                      std::pair{std::string_view{"cuda"}, Cuda}};
}

template <class Enum> static std::string_view enumName(Enum value) {
    for (const auto& [name, candidate] : enumValues(value)) {
        if (candidate == value) {
            return name;
        }
    }
    throw std::invalid_argument("invalid repair policy enum");
}

template <class Value>
static void parseSetting(Value& target, const std::string& text, double scale) {
    if constexpr (std::is_same_v<Value, std::optional<unsigned>> ||
                  std::is_same_v<Value, std::optional<double>>) {
        if (text == "unlimited") {
            target.reset();
        } else {
            typename Value::value_type value{};
            parseSetting(value, text, scale);
            target = value;
        }
    } else if constexpr (std::is_same_v<Value, RepairActions>) {
        target = {false, false, false};
        if (text == "none") {
            return;
        }
        require(!text.empty() && text.back() != ',', "expected action list or none");
        std::istringstream input(text);
        std::string action;
        while (std::getline(input, action, ',')) {
            action = trim(action);
            bool* enabled = action == "resize"          ? &target.resize
                            : action == "buffer"        ? &target.buffer
                            : action == "load_downsize" ? &target.load_downsize
                                                        : nullptr;
            require(enabled && !*enabled, "unknown or duplicate action: " + action);
            *enabled = true;
        }
    } else if constexpr (std::is_enum_v<Value>) {
        for (const auto& [name, value] : enumValues(target)) {
            if (name == text) {
                target = value;
                return;
            }
        }
        throw std::invalid_argument("unknown policy: " + text);
    } else if constexpr (std::is_same_v<Value, bool>) {
        target = boolean(text);
    } else if constexpr (std::is_integral_v<Value>) {
        target = count(text);
    } else if constexpr (std::is_floating_point_v<Value>) {
        const double value = real(text) / scale;
        require(std::abs(value) <= std::numeric_limits<Value>::max(), text);
        target = static_cast<Value>(value);
    } else {
        static_assert(std::is_same_v<Value, std::vector<std::string>>);
        require(!text.empty(), "expected suffixes or none");
        target.clear();
        if (text != "none") {
            std::istringstream input(text);
            std::string suffix;
            while (std::getline(input, suffix, ',')) {
                target.push_back(trim(suffix));
            }
            require(text.back() != ',', "empty trailing suffix");
        }
    }
}

template <class Value>
static void writeSetting(std::ostream& out, const Value& value, double scale) {
    if constexpr (std::is_same_v<Value, std::optional<unsigned>> ||
                  std::is_same_v<Value, std::optional<double>>) {
        if (value) {
            writeSetting(out, *value, scale);
        } else {
            out << "unlimited";
        }
    } else if constexpr (std::is_same_v<Value, RepairActions>) {
        const char* separator = "";
        for (const auto& [name, enabled] : {std::pair{"resize", value.resize},
                                            {"buffer", value.buffer},
                                            {"load_downsize", value.load_downsize}}) {
            if (enabled) {
                out << separator << name;
                separator = ",";
            }
        }
        if (!*separator) {
            out << "none";
        }
    } else if constexpr (std::is_enum_v<Value>) {
        out << enumName(value);
    } else if constexpr (std::is_same_v<Value, bool>) {
        out << (value ? "true" : "false");
    } else if constexpr (std::is_floating_point_v<Value>) {
        out << double(value) * scale;
    } else if constexpr (std::is_integral_v<Value>) {
        out << value;
    } else {
        if (value.empty()) {
            out << "none";
        }
        for (size_t i = 0; i < value.size(); ++i) {
            out << (i ? "," : "") << value[i];
        }
    }
}

// Each configuration key, field and external unit is declared once. The same
// visitor serves parsing, serialization and section recognition.
template <class Config, class Visitor>
static void visitSettings(Config& c, Visitor&& visit) {
    visit("common.max_fanout", c.common.max_fanout, 1.0);
    visit("common.margin", c.common.margin, 1.0);
    visit("common.recovered_slew_fraction", c.common.recovered_slew_fraction, 1.0);
    visit("common.overload_group_fraction", c.common.overload_group_fraction, 1.0);
    visit("power.objective", c.power.objective, 1.0);
    visit("power.max_passes", c.power.max_passes, 1.0);
    visit("power.max_trials", c.power.max_trials, 1.0);
    visit("power.max_candidates", c.power.max_candidates, 1.0);
    visit("power.batch_size", c.power.batch_size, 1.0);
    visit("power.batch_backoff", c.power.batch_backoff, 1.0);
    visit("power.slack_margin_ns", c.power.slack_margin, 1e9);
    visit("power.min_relative_improvement", c.power.min_relative_improvement, 1.0);
    visit("power.min_absolute_improvement_w", c.power.min_absolute_improvement, 1.0);
    visit("vg.backend", c.vg.backend, 1.0);
    visit("vg.scope", c.vg.scope, 1.0);
    visit("vg.cuda_max_memory_mb", c.vg.cuda_max_memory_mb, 1.0);
    visit("vg.max_buffers", c.vg.max_buffers, 1.0);
    visit("vg.max_nets", c.vg.max_nets, 1.0);
    visit("vg.max_group_loads", c.vg.max_group_loads, 1.0);
    visit("vg.min_gain", c.vg.min_gain, 1.0);
    visit("vg.require_drv_non_regression", c.vg.require_drv_non_regression, 1.0);
    visit("gobi.max_buffers", c.gobi.max_buffers, 1.0);
    visit("gobi.max_nets", c.gobi.max_nets, 1.0);
    visit("gobi.require_drv_non_regression", c.gobi.require_drv_non_regression, 1.0);
    visit("common.actions", c.common.actions, 1.0);
    visit("common.load_downsize_slack_margin_ns", c.common.load_downsize_slack_margin,
          1e9);
    visit("drv.max_load_downsizes_per_pass", c.drv.max_load_downsizes_per_pass, 1.0);
    visit("drv.max_passes", c.drv.max_passes, 1.0);
    visit("drv.max_inserted_buffers", c.drv.max_inserted_buffers, 1.0);
    visit("drv.ord_driver_batch_size", c.drv.ord_driver_batch_size, 1.0);
    visit("path.paths_per_endpoint", c.path.paths_per_endpoint, 1.0);
    visit("path.min_improvement_ns", c.path.min_improvement, 1e9);
    visit("path.defer_rollback_refresh", c.path.defer_rollback_refresh, 1.0);
    visit("setup.max_passes", c.setup.max_passes, 1.0);
    visit("setup.skip_duplicate_trials", c.setup.skip_duplicate_trials, 1.0);
    visit("setup.defer_rollback_refresh", c.setup.defer_rollback_refresh, 1.0);
    visit("setup.max_trials", c.setup.max_trials, 1.0);
    visit("setup.max_path_ends", c.setup.max_path_ends, 1.0);
    visit("setup.paths_per_endpoint", c.setup.paths_per_endpoint, 1.0);
    visit("setup.max_candidates", c.setup.max_candidates, 1.0);
    visit("setup.batch_size", c.setup.batch_size, 1.0);
    visit("setup.batch_backoff", c.setup.batch_backoff, 1.0);
    visit("setup.max_inserted_buffers", c.setup.max_inserted_buffers, 1.0);
    visit("setup.min_side_loads", c.setup.min_side_loads, 1.0);
    visit("setup.max_area_growth", c.setup.max_area_growth, 1.0);
    visit("setup.min_improvement_ns", c.setup.min_improvement, 1e9);
    visit("path.recovery", c.path.recovery, 1.0);
    visit("path.tree_candidates", c.path.tree_candidates, 1.0);
    visit("path.gain_scope", c.path.gain_scope, 1.0);
    visit("path.fanout_repair", c.path.fanout_repair, 1.0);
    visit("path.strategy", c.path.strategy, 1.0);
    visit("path.failure_cache", c.path.failure_cache, 1.0);
    visit("path.adaptive_search", c.path.adaptive_search, 1.0);
    visit("path.timing_precheck", c.path.timing_precheck, 1.0);
    visit("path.calibrate", c.path.calibrate, 1.0);
    visit("path.diversify", c.path.diversify, 1.0);
    visit("path.endpoint_coverage", c.path.endpoint_coverage, 1.0);
    visit("path.group_schedule", c.path.group_schedule, 1.0);
    visit("path.incremental_parasitics", c.path.incremental_parasitics, 1.0);
    visit("path.incremental_analysis", c.path.incremental_analysis, 1.0);
    visit("path.reuse_rejections", c.path.reuse_rejections, 1.0);
    visit("path.placement", c.path.placement, 1.0);
    visit("path.verification", c.path.verification, 1.0);
    visit("path.min_relative_improvement", c.path.min_relative_improvement, 1.0);
    visit("path.max_seconds", c.path.max_seconds, 1.0);
    visit("path.max_refreshes", c.path.max_refreshes, 1.0);
    visit("path.max_paths", c.path.max_paths, 1.0);
    visit("path.max_candidates", c.path.max_candidates, 1.0);
    visit("path.max_trials", c.path.max_trials, 1.0);
    visit("path.max_consecutive_rejections", c.path.max_consecutive_rejections, 1.0);
    visit("path.batch_size", c.path.batch_size, 1.0);
    visit("path.max_inserted_buffers", c.path.max_inserted_buffers, 1.0);
    visit("path.max_area_growth", c.path.max_area_growth, 1.0);
    visit("tolerances.wns_absolute_ns", c.common.tolerances.wns_absolute, 1e9);
    visit("tolerances.tns_absolute_ns", c.common.tolerances.tns_absolute, 1e9);
    visit("tolerances.tns_relative", c.common.tolerances.tns_relative, 1.0);
    visit("tolerances.drv_severity", c.common.tolerances.drv_severity, 1.0);
    visit("runtime.sta_threads", c.sta_threads, 1.0);
    visit("common.vt_suffixes", c.common.vt_suffixes, 1.0);
}

void RepairCommonOptions::validate() const {
    require(vt_suffixes.empty() || vt_suffixes.size() >= 2,
            "common.vt_suffixes requires at least two suffixes");
    for (size_t i = 0; i < vt_suffixes.size(); ++i) {
        const auto& a = vt_suffixes[i];
        require(!a.empty() &&
                    a.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQR"
                                        "STUVWXYZ0123456789_") == std::string::npos,
                "common.vt_suffixes: use nonempty alphanumeric/underscore "
                "suffixes");
        for (size_t j = 0; j < i; ++j) {
            const auto& b = vt_suffixes[j];
            require(!(a.size() >= b.size() &&
                      a.compare(a.size() - b.size(), b.size(), b) == 0) &&
                        !(b.size() >= a.size() &&
                          b.compare(b.size() - a.size(), a.size(), a) == 0),
                    "common.vt_suffixes: duplicate or ambiguous suffixes");
        }
    }
    require(std::isfinite(load_downsize_slack_margin) &&
                load_downsize_slack_margin >= 0,
            "common.load_downsize_slack_margin_ns >= 0");
    require(max_fanout >= 2, "common.max_fanout >= 2");
    require(std::isfinite(margin) && margin >= 0 && margin < 1,
            "common.margin in [0,1)");
    require(std::isfinite(recovered_slew_fraction) && recovered_slew_fraction > 0 &&
                recovered_slew_fraction <= 1,
            "common.recovered_slew_fraction in (0,1]");
    require(std::isfinite(overload_group_fraction) && overload_group_fraction > 0 &&
                overload_group_fraction < 1,
            "common.overload_group_fraction in (0,1)");
    require(std::isfinite(tolerances.wns_absolute) && tolerances.wns_absolute >= 0,
            "tolerances.wns_absolute_ns >= 0");
    require(std::isfinite(tolerances.tns_absolute) && tolerances.tns_absolute >= 0,
            "tolerances.tns_absolute_ns >= 0");
    require(std::isfinite(tolerances.tns_relative) && tolerances.tns_relative >= 0 &&
                tolerances.tns_relative < 1,
            "tolerances.tns_relative in [0,1)");
    require(std::isfinite(tolerances.drv_severity) && tolerances.drv_severity >= 0,
            "tolerances.drv_severity >= 0");
}
void DrvRepairSettings::validate() const {
    require(ord_driver_batch_size > 0, "drv.ord_driver_batch_size > 0");
}
void DrvRepairOptions::validate() const {
    common.validate();
    DrvRepairSettings::validate();
}
DrvRepairOptions RepairConfig::drvOptions() const {
    return {drv, common};
}
void SetupRepairSettings::validate() const {
    require(max_path_ends > 0 &&
                max_path_ends <= unsigned(std::numeric_limits<int>::max()),
            "setup.max_path_ends > 0");
    require(paths_per_endpoint > 0 &&
                paths_per_endpoint <= unsigned(std::numeric_limits<int>::max()),
            "setup.paths_per_endpoint > 0");
    require(max_candidates > 0, "setup.max_candidates > 0");
    require(batch_size > 0, "setup.batch_size > 0");
    require(std::isfinite(batch_backoff) && batch_backoff > 0 && batch_backoff < 1,
            "setup.batch_backoff in (0,1)");
    require(min_side_loads > 0, "setup.min_side_loads > 0");
    require(!max_area_growth ||
                (std::isfinite(*max_area_growth) && *max_area_growth >= 0),
            "setup.max_area_growth >= 0");
    require(std::isfinite(min_improvement) && min_improvement > 0,
            "setup.min_improvement_ns > 0");
}
void PathSearchSettings::validate() const {
    require(paths_per_endpoint > 0, "path.paths_per_endpoint > 0");
    require(std::isfinite(min_improvement) && min_improvement > 0,
            "path.min_improvement_ns > 0");
    require(max_paths > 0, "path.max_paths > 0");
    require(max_candidates > 0, "path.max_candidates > 0");
    require(batch_size > 0, "path.batch_size > 0");
    (void)enumName(strategy);
    (void)enumName(gain_scope);
    (void)enumName(placement);
    (void)enumName(verification);
    require(std::isfinite(min_relative_improvement) && min_relative_improvement >= 0 &&
                min_relative_improvement <= 1,
            "path.min_relative_improvement in [0,1]");
    require(!max_seconds || (std::isfinite(*max_seconds) && *max_seconds >= 0),
            "path.max_seconds >= 0 or unlimited");
    require(!max_area_growth ||
                (std::isfinite(*max_area_growth) && *max_area_growth >= 0),
            "path.max_area_growth >= 0 or unlimited");
}
void SetupRepairOptions::validate() const {
    common.validate();
    SetupRepairSettings::validate();
}
SetupRepairOptions RepairConfig::setupOptions() const {
    return {setup, common};
}
void PathSearchOptions::validate() const {
    common.validate();
    PathSearchSettings::validate();
}
PathSearchOptions RepairConfig::pathOptions() const {
    return {path, common};
}
void PowerRepairSettings::validate() const {
    (void)enumName(objective);
    require(max_candidates > 0 && batch_size > 0, "power candidates/batch size > 0");
    require(std::isfinite(batch_backoff) && batch_backoff > 0 && batch_backoff < 1,
            "power.batch_backoff in (0,1)");
    require(std::isfinite(slack_margin) && slack_margin >= 0,
            "power.slack_margin_ns >= 0");
    require(std::isfinite(min_relative_improvement) && min_relative_improvement >= 0 &&
                min_relative_improvement < 1,
            "power.min_relative_improvement in [0,1)");
    require(std::isfinite(min_absolute_improvement) && min_absolute_improvement >= 0,
            "power.min_absolute_improvement_w >= 0");
}
void PowerRepairOptions::validate() const {
    common.validate();
    PowerRepairSettings::validate();
}
void VgSettings::validate() const {
    (void)enumName(scope);
    require(cuda_max_memory_mb > 0, "vg.cuda_max_memory_mb > 0");
    (void)enumName(backend);
    require(std::isfinite(min_gain) && min_gain >= 0, "vg.min_gain >= 0");
    require(max_group_loads == 0 || max_group_loads >= 2,
            "vg.max_group_loads is zero or >= 2");
}
PowerRepairOptions RepairConfig::powerOptions() const {
    return {power, common};
}
void VgOptions::validate() const {
    common.validate();
    VgSettings::validate();
}
void GobiOptions::validate() const {
    common.validate();
}
VgOptions RepairConfig::vgOptions() const {
    return {vg, common};
}
GobiOptions RepairConfig::gobiOptions() const {
    return {gobi, common};
}
void RepairConfig::validate() const {
    common.validate();
    drv.validate();
    setup.validate();
    path.validate();
    power.validate();
    vg.validate();
    require(sta_threads > 0 && sta_threads <= unsigned(std::numeric_limits<int>::max()),
            "runtime.sta_threads > 0");
}

void RepairConfig::set(const std::string& key, const std::string& value) {
    auto next = *this;
    bool found = false;
    visitSettings(next, [&](std::string_view name, auto& field, double scale) {
        if (name == key) {
            parseSetting(field, value, scale);
            found = true;
        }
    });
    require(found, "unknown repair configuration key: " + key);
    next.validate();
    *this = std::move(next);
}
void RepairConfig::read(std::istream& input, const std::string& source) {
    auto next = *this;
    std::string line, section;
    unsigned number = 0;
    while (std::getline(input, line)) {
        ++number;
        try {
            line = trim(line.substr(0, line.find('#')));
            if (line.empty()) {
                continue;
            }
            if (line.front() == '[' && line.back() == ']') {
                section = trim(line.substr(1, line.size() - 2));
                bool known = false;
                visitSettings(next, [&](std::string_view key, const auto&, double) {
                    known |= key.substr(0, key.find('.')) == section;
                });
                require(known, "unknown section " + section);
                continue;
            }
            auto [key, value] = assignment(line);
            if (key.find('.') == std::string::npos && !section.empty()) {
                key.insert(0, 1, '.');
                key.insert(0, section);
            }
            next.set(key, value);
        } catch (const std::exception& error) {
            throw std::invalid_argument(source + ":" + std::to_string(number) + ": " +
                                        error.what());
        }
    }
    if (input.bad()) {
        throw std::runtime_error("cannot read " + source);
    }
    *this = std::move(next);
}
void RepairConfig::readFile(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open repair configuration: " + path);
    }
    read(file, path);
}
void RepairConfig::write(std::ostream& output) const {
    validate();
    std::ostringstream text;
    text << std::setprecision(std::numeric_limits<double>::max_digits10);
    visitSettings(*this, [&](std::string_view key, const auto& field, double scale) {
        text << key << '=';
        writeSetting(text, field, scale);
        text << '\n';
    });
    output << text.str();
}
} // namespace ecc
