// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_config.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ecc {
namespace {
std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if(begin == std::string::npos)
        return {};
    return text.substr(begin, text.find_last_not_of(" \t\r\n") - begin + 1);
}
void require(bool valid, const std::string& key) {
    if(!valid)
        throw std::invalid_argument("invalid repair configuration: " + key);
}
bool boolean(const std::string& value) {
    require(value == "true" || value == "false",
            "expected true/false: " + value);
    return value == "true";
}
unsigned count(const std::string& value) {
    require(!value.empty() &&
                value.find_first_not_of("0123456789") == std::string::npos,
            value);
    const auto number = std::stoull(value);
    require(number <= unsigned(std::numeric_limits< int >::max()), value);
    return static_cast< unsigned >(number);
}
double real(const std::string& value) {
    size_t used = 0;
    const double number = std::stod(value, &used);
    require(used == value.size() && std::isfinite(number), value);
    return number;
}
std::pair< std::string, std::string > assignment(const std::string& text) {
    const auto split = text.find('=');
    require(split != std::string::npos, "expected key=value: " + text);
    const auto key = trim(text.substr(0, split)),
               value = trim(text.substr(split + 1));
    require(!key.empty() && !value.empty(), "expected key=value: " + text);
    return {key, value};
}
bool noExtra(std::istringstream& input) {
    std::string extra;
    return !(input >> extra) || extra.front() == '#';
}
}  // namespace

void RepairCommonOptions::validate() const {
    require(vt_suffixes.empty() || vt_suffixes.size() >= 2,
            "common.vt_suffixes requires at least two suffixes");
    for(size_t i = 0; i < vt_suffixes.size(); ++i) {
        const auto& a = vt_suffixes[i];
        require(!a.empty() && a.find_first_not_of(
                                  "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQR"
                                  "STUVWXYZ0123456789_") == std::string::npos,
                "common.vt_suffixes: use nonempty alphanumeric/underscore "
                "suffixes");
        for(size_t j = 0; j < i; ++j) {
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
    require(std::isfinite(recovered_slew_fraction) &&
                recovered_slew_fraction > 0 && recovered_slew_fraction <= 1,
            "common.recovered_slew_fraction in (0,1]");
    require(std::isfinite(overload_group_fraction) &&
                overload_group_fraction > 0 && overload_group_fraction < 1,
            "common.overload_group_fraction in (0,1)");
    require(
        std::isfinite(tolerances.wns_absolute) && tolerances.wns_absolute >= 0,
        "tolerances.wns_absolute_ns >= 0");
    require(
        std::isfinite(tolerances.tns_absolute) && tolerances.tns_absolute >= 0,
        "tolerances.tns_absolute_ns >= 0");
    require(std::isfinite(tolerances.tns_relative) &&
                tolerances.tns_relative >= 0 && tolerances.tns_relative < 1,
            "tolerances.tns_relative in [0,1)");
    require(
        std::isfinite(tolerances.drv_severity) && tolerances.drv_severity >= 0,
        "tolerances.drv_severity >= 0");
}
void DesignRepairOptions::validate() const {
    RepairCommonOptions::validate();
    require(ord_driver_batch_size > 0,
            "design.ord_driver_batch_size > 0");
}
void SetupRepairSettings::validate() const {
    require(max_path_ends > 0 &&
                max_path_ends <= unsigned(std::numeric_limits< int >::max()),
            "setup.max_path_ends > 0");
    require(
        paths_per_endpoint > 0 &&
            paths_per_endpoint <= unsigned(std::numeric_limits< int >::max()),
        "setup.paths_per_endpoint > 0");
    require(max_candidates > 0, "setup.max_candidates > 0");
    require(batch_size > 0, "setup.batch_size > 0");
    require(
        std::isfinite(batch_backoff) && batch_backoff > 0 && batch_backoff < 1,
        "setup.batch_backoff in (0,1)");
    require(min_side_loads > 0, "setup.min_side_loads > 0");
    require(std::isfinite(max_area_growth) && max_area_growth >= 0,
            "setup.max_area_growth >= 0");
    require(std::isfinite(min_improvement) && min_improvement > 0,
            "setup.min_improvement_ns > 0");
}
void SetupRepairOptions::validate() const {
    common.validate();
    SetupRepairSettings::validate();
}
SetupRepairOptions RepairConfig::setupOptions() const {
    SetupRepairOptions result;
    static_cast< SetupRepairSettings& >(result) = setup;
    result.common = design;
    return result;
}
void PowerRepairSettings::validate() const {
    require(objective == "leakage" || objective == "total",
            "power.objective: leakage or total");
    require(max_candidates > 0 && batch_size > 0,
            "power candidates/batch size > 0");
    require(
        std::isfinite(batch_backoff) && batch_backoff > 0 && batch_backoff < 1,
        "power.batch_backoff in (0,1)");
    require(std::isfinite(slack_margin) && slack_margin >= 0,
            "power.slack_margin_ns >= 0");
    require(std::isfinite(min_relative_improvement) &&
                min_relative_improvement >= 0 && min_relative_improvement < 1,
            "power.min_relative_improvement in [0,1)");
    require(std::isfinite(min_absolute_improvement) &&
                min_absolute_improvement >= 0,
            "power.min_absolute_improvement_w >= 0");
}
void PowerRepairOptions::validate() const {
    common.validate();
    PowerRepairSettings::validate();
}
void VgRepairSettings::validate() const {
    require(cuda_max_memory_mb > 0, "vg.cuda_max_memory_mb > 0");
    require(backend == "cpu" || backend == "cuda",
            "vg.backend: cpu or cuda");
    require(std::isfinite(min_gain) && min_gain >= 0,
            "vg.min_gain >= 0");
    require(max_group_loads == 0 || max_group_loads >= 2,
            "vg.max_group_loads is zero or >= 2");
}
void GbiRepairSettings::validate() const {
    require(backend == "cuda", "gbi.backend: cuda only");
}
PowerRepairOptions RepairConfig::powerOptions() const {
    PowerRepairOptions result;
    static_cast< PowerRepairSettings& >(result) = power;
    result.common = design;
    return result;
}
void RepairConfig::validate() const {
    design.validate();
    setup.validate();
    power.validate();
    vg.validate();
    gbi.validate();
    require(sta_threads > 0 &&
                sta_threads <= unsigned(std::numeric_limits< int >::max()),
            "runtime.sta_threads > 0");
}

void RepairConfig::set(const std::string& key, const std::string& value) {
    auto next = *this;
    auto& d = next.design;
    auto& s = next.setup;
    auto& p = next.power;
    auto& g = next.gbi;
    auto& t = d.tolerances;
    if(key == "common.max_fanout")
        d.max_fanout = count(value);
    else if(key == "common.margin")
        d.margin = real(value);
    else if(key == "common.recovered_slew_fraction")
        d.recovered_slew_fraction = real(value);
    else if(key == "common.overload_group_fraction")
        d.overload_group_fraction = real(value);
    else if(key == "common.allow_gate_resize") {
        require(value == "true" || value == "false",
                key + " expects true/false");
        d.allow_gate_resize = value == "true";
    }
    else if(key == "common.vt_suffixes") {
        require(!value.empty(),
                "common.vt_suffixes: expected suffixes or none");
        d.vt_suffixes.clear();
        if(value != "none") {
            std::istringstream input(value);
            std::string suffix;
            while(std::getline(input, suffix, ','))
                d.vt_suffixes.push_back(trim(suffix));
            require(value.back() != ',',
                    "common.vt_suffixes: empty trailing suffix");
        }
    }
    else if(key == "power.enabled")
        p.enabled = boolean(value);
    else if(key == "power.objective")
        p.objective = value;
    else if(key == "power.max_passes")
        p.max_passes = count(value);
    else if(key == "power.max_trials")
        p.max_trials = count(value);
    else if(key == "power.max_candidates")
        p.max_candidates = count(value);
    else if(key == "power.batch_size")
        p.batch_size = count(value);
    else if(key == "power.batch_backoff")
        p.batch_backoff = real(value);
    else if(key == "power.slack_margin_ns")
        p.slack_margin = real(value) * 1e-9;
    else if(key == "power.min_relative_improvement")
        p.min_relative_improvement = real(value);
    else if(key == "power.min_absolute_improvement_w")
        p.min_absolute_improvement = real(value);
    else if(key == "vg.backend")
        next.vg.backend = value;
    else if(key == "vg.cuda_max_memory_mb")
        next.vg.cuda_max_memory_mb = count(value);
    else if(key == "vg.max_buffers")
        next.vg.max_buffers = count(value);
    else if(key == "vg.max_nets")
        next.vg.max_nets = count(value);
    else if(key == "vg.max_group_loads")
        next.vg.max_group_loads = count(value);
    else if(key == "vg.min_gain")
        next.vg.min_gain = real(value);
    else if(key == "vg.require_drv_reduction")
        next.vg.require_drv_reduction = boolean(value);
    else if(key == "gbi.backend")
        g.backend = value;
    else if(key == "gbi.max_buffers")
        g.max_buffers = count(value);
    else if(key == "gbi.max_nets")
        g.max_nets = count(value);
    else if(key == "gbi.require_drv_reduction")
        g.require_drv_reduction = boolean(value);
    else if(key == "common.allow_load_downsize")
        d.allow_load_downsize = boolean(value);
    else if(key == "common.load_downsize_slack_margin_ns")
        d.load_downsize_slack_margin = real(value) * 1e-9;
    else if(key == "design.max_load_downsizes_per_pass")
        d.max_load_downsizes_per_pass = count(value);
    else if(key == "design.max_passes")
        d.max_passes = count(value);
    else if(key == "design.max_inserted_buffers")
        d.max_inserted_buffers = count(value);
    else if(key == "design.ord_driver_batch_size")
        d.ord_driver_batch_size = count(value);
    else if(key == "setup.max_passes")
        s.max_passes = count(value);
    else if(key == "setup.skip_duplicate_trials")
        s.skip_duplicate_trials = boolean(value);
    else if(key == "setup.defer_rollback_refresh")
        s.defer_rollback_refresh = boolean(value);
    else if(key == "setup.max_trials")
        s.max_trials = count(value);
    else if(key == "setup.max_path_ends")
        s.max_path_ends = count(value);
    else if(key == "setup.paths_per_endpoint")
        s.paths_per_endpoint = count(value);
    else if(key == "setup.max_candidates")
        s.max_candidates = count(value);
    else if(key == "setup.batch_size")
        s.batch_size = count(value);
    else if(key == "setup.batch_backoff")
        s.batch_backoff = real(value);
    else if(key == "setup.max_inserted_buffers")
        s.max_inserted_buffers = count(value);
    else if(key == "setup.min_side_loads")
        s.min_side_loads = count(value);
    else if(key == "setup.max_area_growth")
        s.max_area_growth = real(value);
    else if(key == "setup.min_improvement_ns")
        s.min_improvement = real(value) * 1e-9;
    else if(key == "tolerances.wns_absolute_ns")
        t.wns_absolute = real(value) * 1e-9;
    else if(key == "tolerances.tns_absolute_ns")
        t.tns_absolute = real(value) * 1e-9;
    else if(key == "tolerances.tns_relative")
        t.tns_relative = real(value);
    else if(key == "tolerances.drv_severity")
        t.drv_severity = real(value);
    else if(key == "runtime.sta_threads")
        next.sta_threads = count(value);
    else
        throw std::invalid_argument("unknown repair configuration key: " + key);
    next.validate();
    *this = std::move(next);
}
void RepairConfig::read(std::istream& input, const std::string& source) {
    auto next = *this;
    std::string line, section;
    unsigned number = 0;
    while(std::getline(input, line)) {
        ++number;
        try {
            line = trim(line.substr(0, line.find('#')));
            if(line.empty())
                continue;
            if(line.front() == '[' && line.back() == ']') {
                section = trim(line.substr(1, line.size() - 2));
                require(section == "common" || section == "design" ||
                            section == "setup" || section == "power" ||
                            section == "vg" || section == "gbi" || section == "tolerances" ||
                            section == "runtime",
                        "unknown section " + section);
                continue;
            }
            auto [key, value] = assignment(line);
            if(key.find('.') == std::string::npos && !section.empty())
                key = section + "." + key;
            next.set(key, value);
        }
        catch(const std::exception& error) {
            throw std::invalid_argument(source + ":" + std::to_string(number) +
                                        ": " + error.what());
        }
    }
    if(input.bad())
        throw std::runtime_error("cannot read " + source);
    *this = std::move(next);
}
void RepairConfig::readFile(const std::string& path) {
    std::ifstream file(path);
    if(!file)
        throw std::runtime_error("cannot open repair configuration: " + path);
    read(file, path);
}
void RepairConfig::write(std::ostream& output) const {
    // Serialize sufficient precision for a lossless default/override round
    // trip.
    std::ostringstream text;
    text << std::setprecision(std::numeric_limits< double >::max_digits10);
    const auto& d = design;
    const auto& s = setup;
    const auto& t = d.tolerances;
    text << "common.max_fanout=" << d.max_fanout << '\n'
         << "common.margin=" << d.margin << '\n'
         << "common.recovered_slew_fraction=" << d.recovered_slew_fraction
         << '\n'
         << "common.overload_group_fraction=" << d.overload_group_fraction
         << '\n'
         << "common.allow_gate_resize="
         << (d.allow_gate_resize ? "true" : "false") << '\n'
         << "common.allow_load_downsize="
         << (d.allow_load_downsize ? "true" : "false") << '\n'
         << "common.load_downsize_slack_margin_ns="
         << double(d.load_downsize_slack_margin) * 1e9 << '\n'
         << "design.max_load_downsizes_per_pass="
         << d.max_load_downsizes_per_pass << '\n'
         << "design.max_passes=" << d.max_passes << '\n'
         << "design.max_inserted_buffers=" << d.max_inserted_buffers << '\n'
         << "design.ord_driver_batch_size=" << d.ord_driver_batch_size << '\n'
         << "setup.max_passes=" << s.max_passes << '\n'
         << "setup.skip_duplicate_trials="
         << (s.skip_duplicate_trials ? "true" : "false") << '\n'
         << "setup.defer_rollback_refresh="
         << (s.defer_rollback_refresh ? "true" : "false") << '\n'
         << "setup.max_trials=" << s.max_trials << '\n'
         << "setup.max_path_ends=" << s.max_path_ends << '\n'
         << "setup.paths_per_endpoint=" << s.paths_per_endpoint << '\n'
         << "setup.max_candidates=" << s.max_candidates << '\n'
         << "setup.batch_size=" << s.batch_size << '\n'
         << "setup.batch_backoff=" << s.batch_backoff << '\n'
         << "setup.max_inserted_buffers=" << s.max_inserted_buffers << '\n'
         << "setup.min_side_loads=" << s.min_side_loads << '\n'
         << "setup.max_area_growth=" << s.max_area_growth << '\n'
         << "setup.min_improvement_ns=" << double(s.min_improvement) * 1e9
         << '\n'
         << "tolerances.wns_absolute_ns=" << double(t.wns_absolute) * 1e9
         << '\n'
         << "tolerances.tns_absolute_ns=" << double(t.tns_absolute) * 1e9
         << '\n'
         << "tolerances.tns_relative=" << t.tns_relative << '\n'
         << "tolerances.drv_severity=" << t.drv_severity << '\n'
         << "runtime.sta_threads=" << sta_threads << '\n';
    text << "common.vt_suffixes=";
    if(d.vt_suffixes.empty())
        text << "none";
    for(size_t i = 0; i < d.vt_suffixes.size(); ++i)
        text << (i ? "," : "") << d.vt_suffixes[i];
    text << '\n'
         << "power.enabled=" << (power.enabled ? "true" : "false") << '\n'
         << "power.objective=" << power.objective << '\n'
         << "power.max_passes=" << power.max_passes << '\n'
         << "power.max_trials=" << power.max_trials << '\n'
         << "power.max_candidates=" << power.max_candidates << '\n'
         << "power.batch_size=" << power.batch_size << '\n'
         << "power.batch_backoff=" << power.batch_backoff << '\n'
         << "power.slack_margin_ns=" << double(power.slack_margin) * 1e9 << '\n'
         << "power.min_relative_improvement=" << power.min_relative_improvement
         << '\n'
         << "power.min_absolute_improvement_w="
         << power.min_absolute_improvement << '\n';
    text << "vg.backend=" << vg.backend << '\n'
         << "vg.cuda_max_memory_mb=" << vg.cuda_max_memory_mb << '\n'
         << "vg.max_buffers=" << vg.max_buffers << '\n'
         << "vg.max_nets=" << vg.max_nets << '\n'
         << "vg.max_group_loads=" << vg.max_group_loads << '\n'
         << "vg.min_gain=" << vg.min_gain << '\n'
         << "vg.require_drv_reduction="
         << (vg.require_drv_reduction ? "true" : "false") << '\n'
         << "gbi.backend=" << gbi.backend << '\n'
         << "gbi.max_buffers=" << gbi.max_buffers << '\n'
         << "gbi.max_nets=" << gbi.max_nets << '\n'
         << "gbi.require_drv_reduction="
         << (gbi.require_drv_reduction ? "true" : "false") << '\n';
    output << text.str();
}
void loadRepairConfigFiles(RepairConfig& config,
                           const std::string& command_file) {
    std::ifstream commands(command_file);
    if(!commands)
        throw std::runtime_error("cannot open command file: " + command_file);
    std::string line;
    while(std::getline(commands, line)) {
        std::istringstream input(line);
        std::string flag, path;
        input >> flag;
        if(flag == "-sizer_repair_config") {
            require(bool(input >> std::quoted(path)) && noExtra(input),
                    flag + " expects one file path");
            config.readFile(path);
        }
    }
}
bool applyRepairConfigOverride(RepairConfig& config, const std::string& line) {
    std::istringstream input(line);
    std::string flag, value, key;
    input >> flag;
    if(flag == "-sizer_repair_set") {
        require(bool(input >> value) && noExtra(input),
                flag + " expects key=value");
        auto entry = assignment(value);
        config.set(entry.first, entry.second);
        return true;
    }
    if(flag == "-sizer_setup_trials")
        key = "setup.max_trials";
    else if(flag == "-sizer_setup_passes")
        key = "setup.max_passes";
    else if(flag == "-sizer_setup_area_growth")
        key = "setup.max_area_growth";
    else if(flag == "-sizer_repair_no_gate_resize") {
        require(noExtra(input), flag + " takes no value");
        config.set("common.allow_gate_resize", "false");
        return true;
    }
    else
        return false;
    require(bool(input >> value) && noExtra(input),
            flag + " expects one value");
    config.set(key, value);
    return true;
}
}  // namespace ecc
