// SPDX-License-Identifier: Apache-2.0
#include "design_io.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "ord/Design.h"
#include "repair_config.h"
#include "repair_design.h"
#include "repair_design_ord.h"
#include "repair_fanout.h"
#include "repair_gobi.h"
#include "repair_gobi_cuda.h"
#include "repair_parasitics.h"
#include "repair_path.h"
#include "repair_placement.h"
#include "repair_power.h"
#include "repair_timing.h"
#include "repair_vg.h"
#include "rsz/Resizer.hh"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Sdc.hh"

// Retained sizing/preplace/hold/postroute functionality uses its existing
// implementation until migrated. It cannot initialize a second OpenROAD
// session inside the new flow.
int runLegacySizer(int argc, char** argv);

namespace ecc {
namespace {
enum class Stage { Native, Design, Setup, Path, Vg, Gobi, Ord, Power, Fanout, Legalize };
struct StageName { std::string_view name; Stage stage; };
constexpr StageName stage_names[] = {
    {"native", Stage::Native}, {"design", Stage::Design}, {"setup", Stage::Setup},
    {"path", Stage::Path}, {"vg", Stage::Vg}, {"gobi", Stage::Gobi},
    {"ord", Stage::Ord}, {"power", Stage::Power}, {"fanout", Stage::Fanout},
    {"legalize", Stage::Legalize}};

struct Options {
    DesignInputs inputs;
    RepairConfig repair;
    std::vector<Stage> stages{Stage::Native};
    std::string output;
    std::string log;
    spdlog::level::level_enum level = spdlog::level::info;
    bool help = false;
};

constexpr auto help = R"(Usage:
  Sizer design.tcl [--flow native|vg|path|gobi] [--output PREFIX]
  Sizer --def design.def --lef tech.lef --lef cells.lef --lib cells.lib --sdc design.sdc
  Sizer --odb design.odb --lib cells.lib --sdc design.sdc

The default flow is native (electrical repair followed by setup repair).
--flow accepts comma-separated stages, e.g. vg,native,path,power.
Stages: native, design, setup, path, vg, gobi, ord, power, fanout, legalize.
Inputs are loaded once; every stage sees the preceding committed design.

Options:
  --lef FILE          Repeat for LEFs, technology first
  --lib FILE          Repeat for Liberty libraries
  --sdc FILE          Constraints (required for direct DEF/ODB input)
  --rc FILE           Optional OpenROAD RC Tcl; otherwise derive RC from LEF
  --output PREFIX     Write PREFIX.def/.v/.odb (default: <design>.sized)
  --threads N         STA worker count (default: available hardware threads)
  --config FILE       Optional advanced repair key=value configuration
  --set KEY=VALUE     Advanced override; overrides configuration files
  --log FILE          Also write Sizer messages to a file
  --log-level LEVEL   Sizer logging: debug, info, warn, error, off
  --help              Show this help

Input Tcl uses standard OpenROAD read_* commands; paths are relative to the
working directory. Tcl may also specify dont_use/dont_touch and wire RC.
Native/VG/Path/GOBI use placement RC and one STA scene. GOBI requires CUDA.
Other existing flows remain available via: Sizer legacy -env FILE -f FILE
PBBI is not part of the new flow. OpenROAD diagnostics retain their own controls.
)";

std::vector<Stage> parseStages(const std::string& value) {
    std::vector<Stage> stages;
    size_t begin = 0;
    do {
        const auto end = value.find(',', begin);
        const auto name = value.substr(begin, end - begin);
        const auto found = std::find_if(std::begin(stage_names), std::end(stage_names),
            [&](const auto& item) { return item.name == name; });
        if(found == std::end(stage_names))
            throw std::invalid_argument("unknown flow stage: " + name);
        stages.push_back(found->stage);
        if(end == std::string::npos) break;
        begin = end + 1;
    } while(true);
    return stages;
}

Options parseOptions(int argc, char** argv) {
    Options result;
    result.repair.sta_threads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::string> configs, overrides;
    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string {
            if(i + 1 >= argc || std::string_view(argv[i + 1]).starts_with("--"))
                throw std::invalid_argument("missing value for " + arg);
            return argv[++i];
        };
        if(arg == "--help" || arg == "-h") result.help = true;
        else if(arg == "--def") result.inputs.def = value();
        else if(arg == "--odb") result.inputs.odb = value();
        else if(arg == "--lef") result.inputs.lef.push_back(value());
        else if(arg == "--lib") result.inputs.liberty.push_back(value());
        else if(arg == "--sdc") result.inputs.sdc = value();
        else if(arg == "--rc") result.inputs.rc = value();
        else if(arg == "--flow") result.stages = parseStages(value());
        else if(arg == "--output") result.output = value();
        else if(arg == "--threads") overrides.push_back("runtime.sta_threads=" + value());
        else if(arg == "--config") configs.push_back(value());
        else if(arg == "--set") overrides.push_back(value());
        else if(arg == "--log") result.log = value();
        else if(arg == "--log-level") {
            const auto level = value();
            if(level != "debug" && level != "info" && level != "warn" && level != "error" && level != "off")
                throw std::invalid_argument("invalid log level: " + level);
            result.level = spdlog::level::from_str(level);
        } else if(arg.starts_with('-')) {
            throw std::invalid_argument("unknown option: " + arg);
        } else {
            if(!result.inputs.script.empty())
                throw std::invalid_argument("supply only one design Tcl script");
            result.inputs.script = arg;
        }
    }
    if(result.help) return result;
    const auto& in = result.inputs;
    if(unsigned(!in.script.empty()) + unsigned(!in.def.empty()) + unsigned(!in.odb.empty()) != 1)
        throw std::invalid_argument("supply one design Tcl script, --def, or --odb");
    if(!in.script.empty() && (!in.lef.empty() || !in.liberty.empty()))
        throw std::invalid_argument("read LEF/Liberty in the design script or use direct DEF/ODB input");
    if(in.script.empty() && (in.liberty.empty() || in.sdc.empty()))
        throw std::invalid_argument("direct DEF/ODB input requires --lib and --sdc");
    if(!in.def.empty() && in.lef.empty())
        throw std::invalid_argument("DEF input requires --lef (technology first)");
    for(const auto& path : configs) result.repair.readFile(path);
    for(const auto& override : overrides) {
        const auto split = override.find('=');
        if(split == std::string::npos)
            throw std::invalid_argument("expected --set KEY=VALUE: " + override);
        result.repair.set(override.substr(0, split), override.substr(split + 1));
    }
    result.repair.validate();
    if(result.repair.pbbi.enabled)
        throw std::invalid_argument("PBBI is outside the new flow");
    if(result.repair.setup.path_buffer_enabled || result.repair.setup.path_buffer_only)
        throw std::invalid_argument("select Path with --flow path or --flow path,setup");
    if(result.repair.power.enabled)
        throw std::invalid_argument("select power repair with --flow native,power");
    if(result.output.empty()) {
        const auto& input = !in.script.empty() ? in.script : !in.odb.empty() ? in.odb : in.def;
        result.output = std::filesystem::path(input).stem().string() + ".sized";
    }
    // Do not overwrite an input ODB/DEF through an output-prefix alias.
    for(const auto& input : {in.odb, in.def}) {
        if(input.empty()) continue;
        const auto output = result.output + std::filesystem::path(input).extension().string();
        if(std::filesystem::weakly_canonical(input) == std::filesystem::weakly_canonical(output))
            throw std::invalid_argument("output prefix would overwrite input: " + input);
    }
    return result;
}

template<class Result>
void report(const std::string& stage, const Result& result) {
    if(!result.ok) throw std::runtime_error(stage + ": " + result.message);
    spdlog::info("{}: {}", stage, result.message);
}

// Application orchestration owns placement, RC refresh and output. Algorithms
// choose candidates; buffer_inserter and OpenDB ECOs own topology edits/undo.
void run(Options& options) {
    if(std::find(options.stages.begin(), options.stages.end(), Stage::Gobi) != options.stages.end()
       && !gobi::gobiCudaAvailable())
        throw std::runtime_error("GOBI requires a CUDA build and a usable CUDA device");
    DesignSession session(options.inputs);
    auto* block = session.block();
    auto* sta = session.timing();
    if(sta->scenes().size() != 1)
        throw std::runtime_error("repair currently requires one STA scene");
    sta->setThreadCount(static_cast<int>(options.repair.sta_threads));
    auto common = options.repair.design;
    for(auto* inst : block->getInsts())
        if(inst->isDoNotTouch()) common.excluded_instances.push_back(inst);
    common.prepare = [&]() {
        legalizeRepairPlacement(block, session.design().getOpendp(), common.excluded_instances);
    };
    auto* top = sta->network()->cell(sta->network()->topInstance());
    float limit = 0;
    bool has_limit = false;
    sta->cmdSdc()->fanoutLimit(top, sta::MinMax::max(), limit, has_limit);
    // A convenience default must never relax an input SDC constraint.
    sta->setFanoutLimit(top, sta::MinMax::max(),
                        has_limit ? std::min(limit, float(common.max_fanout)) : common.max_fanout,
                        sta->cmdSdc());
    const auto refresh = [&]() { session.refresh(); };
    const auto placeAndRefresh = [&]() { common.prepare(); refresh(); };
    session.initializePlacementRC();
    placeAndRefresh();

    for(const auto stage : options.stages) {
        const auto begin = std::chrono::steady_clock::now();
        const auto name = std::find_if(std::begin(stage_names), std::end(stage_names),
                                      [&](const auto& item) { return item.stage == stage; })->name;
        spdlog::info("Starting {}", name);
        switch(stage) {
        case Stage::Native:
        case Stage::Design:
            report("design", OpenDbDesignRepairer::repair(block, sta, refresh, common));
            if(stage == Stage::Design) break;
            [[fallthrough]];
        case Stage::Setup: {
            auto setup = options.repair.setupOptions();
            setup.common = common;
            report("setup", OpenDbSetupRepairer::repair(block, sta, refresh, setup));
            break;
        }
        case Stage::Path: {
            auto path = options.repair.setupOptions();
            path.common = common;
            path.path_trial_observer = [](unsigned index, const PathBufferTrial& trial) {
                spdlog::debug("Path trial {}: accepted={} buffers={} provisional={} verified={}",
                    index, trial.rejection_reasons.empty(), trial.buffers,
                    trial.placement_provisional, trial.full_verified);
            };
            if(path.path_buffer_incremental_parasitics) {
                PlacementParasiticUpdater incremental(block, sta,
                    session.design().getResizer()->getEstimateParasitics());
                path.path_verification_refresh = refresh;
                path.path_incremental_verification = [&]() { incremental.verifyTiming(); };
                path.path_analysis_changes = [&]() { return incremental.takeAnalysisChanges(); };
                report("path", OpenDbPathRepairer::repair(block, sta, [&]() {
                    session.reconnectSupplies();
                    incremental.update();
                }, path));
            } else {
                report("path", OpenDbPathRepairer::repair(block, sta, refresh, path));
            }
            break;
        }
        case Stage::Vg: {
            VgRepairOptions vg;
            static_cast<VgRepairSettings&>(vg) = options.repair.vg;
            vg.common = common;
            report("vg", OpenDbVgRepairer::repair(block, sta, placeAndRefresh,
                session.design().getResizer(), vg));
            break;
        }
        case Stage::Gobi: {
            GobiRepairOptions gobi;
            static_cast<GobiRepairSettings&>(gobi) = options.repair.gobi;
            gobi.common = common;
            report("gobi", OpenDbGobiRepairer::repair(block, sta, placeAndRefresh,
                session.design().getResizer(), gobi));
            break;
        }
        case Stage::Ord:
            report("ord", OpenDbOrdDesignRepairer::repair(block, sta, refresh, common));
            break;
        case Stage::Power: {
            auto power = options.repair.powerOptions();
            power.common = common;
            report("power", OpenDbPowerRepairer::repair(block, sta, refresh, power));
            break;
        }
        case Stage::Fanout: {
            odb::dbMaster* buffer = nullptr;
            for(auto* lib : block->getDataBase()->getLibs())
                for(auto* master : lib->getMasters()) {
                    auto* cell = sta->getDbNetwork()->libertyCell(sta->getDbNetwork()->dbToSta(master));
                    if(!master->isFrozen() || !master->isCore() || !cell || cell->dontUse() || !cell->isBuffer()) continue;
                    const auto area = [](auto* m) { return uint64_t(m->getWidth()) * m->getHeight(); };
                    if(!buffer || area(master) < area(buffer)
                       || (area(master) == area(buffer) && master->getName() < buffer->getName())) buffer = master;
                }
            if(!buffer) throw std::runtime_error("no usable buffer master for fanout repair");
            FanoutRepairOptions fanout;
            fanout.max_fanout = common.max_fanout;
            if(has_limit) fanout.max_fanout = std::min(fanout.max_fanout, unsigned(std::max(0.0f, limit)));
            for(auto* net : block->getNets())
                if(sta->isClock(sta->getDbNetwork()->dbToSta(net), sta->cmdMode())) fanout.excluded_nets.push_back(net);
            const auto result = OpenDbFanoutRepairer::repair(block, buffer, fanout);
            if(!result) throw std::runtime_error("fanout: " + result.message);
            placeAndRefresh();
            spdlog::info("Fanout repair inserted {} buffers", result.inserted_buffers.size());
            break;
        }
        case Stage::Legalize:
            placeAndRefresh();
            break;
        }
        session.command({"check_placement"});
        sta->checkFanoutPreamble();
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        spdlog::info("{} finished in {:.3f}s; setup WNS={:.6f}ns TNS={:.6f}ns; slew={} cap={} fanout={}",
            name, elapsed, double(sta->worstSlack(sta::MinMax::max())) * 1e9,
            double(sta->totalNegativeSlack(sta::MinMax::max())) * 1e9,
            sta->maxSlewViolationCount(), sta->maxCapacitanceViolationCount(),
            sta->fanoutViolationCount(sta::MinMax::max(), sta->modes()));
    }
    session.write(options.output);
}
}  // namespace
}  // namespace ecc

int main(int argc, char** argv) {
    // Legacy dispatch happens before creating the new application's session.
    if(argc > 1 && std::string_view(argv[1]) == "legacy")
        return runLegacySizer(argc - 1, argv + 1);
    try {
        auto options = ecc::parseOptions(argc, argv);
        if(options.help) {
            spdlog::set_pattern("%v");
            spdlog::info("{}", ecc::help);
            return 0;
        }
        std::vector<spdlog::sink_ptr> sinks{std::make_shared<spdlog::sinks::stderr_color_sink_mt>()};
        if(!options.log.empty()) sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(options.log, true));
        auto logger = std::make_shared<spdlog::logger>("sizer", sinks.begin(), sinks.end());
        logger->set_pattern("[%l] %v");
        logger->set_level(options.level);
        logger->flush_on(spdlog::level::info);
        spdlog::set_default_logger(logger);
        ecc::run(options);
        return 0;
    } catch(const std::exception& error) {
        // Errors remain visible even if normal application logging is disabled.
        spdlog::default_logger()->set_level(spdlog::level::err);
        spdlog::error("{}", error.what());
        return 1;
    }
}
