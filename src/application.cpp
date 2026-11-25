// SPDX-License-Identifier: Apache-2.0
#include "repair/buffer.h"
#include "design_session.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "ord/Design.h"
#include "est/EstimateParasitics.h"
#include "repair/config.h"
#include "repair/drv.h"
#include "algorithms/gobi.h"
#include "algorithms/gobi_cuda.h"
#include "parasitics.h"
#include "actions/netlist.h"
#include "actions/placement.h"
#include "repair/setup.h"
#include "algorithms/van_ginneken.h"
#include "rsz/Resizer.hh"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <fmt/ranges.h>
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Sdc.hh"

namespace ecc {
enum class Stage {
    Drv,
    Setup,
    Path,
    Pbbi,
    Vg,
    Gobi,
    Ord,
    Power,
    Fanout,
    Legalize,
    Hold,
    Preplace,
    PreplaceGain
};
struct StageName {
    std::string_view name;
    Stage stage;
};
constexpr StageName stage_names[] = {
    {"drv", Stage::Drv},           {"setup", Stage::Setup},
    {"path", Stage::Path},         {"vg", Stage::Vg},
    {"pbbi", Stage::Pbbi},
    {"gobi", Stage::Gobi},         {"ord", Stage::Ord},
    {"power", Stage::Power},       {"fanout", Stage::Fanout},
    {"legalize", Stage::Legalize}, {"hold", Stage::Hold},
    {"preplace", Stage::Preplace}, {"preplace-gain", Stage::PreplaceGain}};

struct Options {
    DesignInputs inputs;
    PlacementLegalizer legalizer = PlacementLegalizer::OpenDP;
    RepairConfig repair;
    std::vector<Stage> stages{Stage::Drv, Stage::Setup};
    std::string output;
    std::string log;
    spdlog::level::level_enum level = spdlog::level::info;
    bool help = false;
};

constexpr auto help = R"(Usage:
  Sizer design.tcl [--flow drv,setup|vg|path|gobi|pbbi] [--output PREFIX]
  Sizer --def design.def --lef tech.lef --lef cells.lef --lib cells.lib --sdc design.sdc
  Sizer --odb design.odb --lib cells.lib --sdc design.sdc

The default flow is drv,setup (electrical repair followed by setup repair).
--flow accepts comma-separated stages, e.g. path,drv,setup,power.
Stages: drv, setup, path, vg, gobi, pbbi, ord, power, fanout, legalize,
        hold, preplace, preplace-gain.
Inputs are loaded once; every stage sees the preceding committed design.

Options:
  --lef FILE          Repeat for LEFs, technology first
  --lib FILE          Repeat for Liberty libraries
  --sdc FILE          Constraints (required for direct DEF/ODB input)
  --rc FILE           Optional OpenROAD RC Tcl; otherwise derive RC from LEF
  --legalizer NAME    opendp (default) or abacus, with OpenDP fallback
  --spef FILE         Use extracted parasitics (postroute)
  --global-routing    Estimate RC from global routing instead of placement
  --output PREFIX     Write PREFIX.def/.v/.odb (default: <design>.sized)
  --threads N         STA worker count (default: available hardware threads)
  --config FILE       Optional advanced repair key=value configuration
  --set KEY=VALUE     Advanced override; overrides configuration files
  --log FILE          Also write Sizer messages to a file
  --log-level LEVEL   Sizer logging: debug, info, warn, error, off
  --help              Show this help

Input Tcl uses standard OpenROAD read_* commands; paths are relative to the
working directory. Tcl may also specify dont_use/dont_touch and wire RC.
VG/Path/GOBI/PBBI use placement RC. All repair uses one STA scene. GOBI requires CUDA.
OpenROAD diagnostics retain their own controls.
)";

static std::vector<Stage> parseStages(const std::string& value) {
    std::vector<Stage> stages;
    size_t begin = 0;
    do {
        const auto end = value.find(',', begin);
        const auto name = value.substr(begin, end - begin);
        const auto found =
            std::find_if(std::begin(stage_names), std::end(stage_names),
                         [&](const auto& item) { return item.name == name; });
        if (found == std::end(stage_names)) {
            throw std::invalid_argument("unknown flow stage: " + name);
        }
        stages.push_back(found->stage);
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    } while (true);
    return stages;
}

static bool isPreplacement(Stage stage) {
    return stage == Stage::Preplace || stage == Stage::PreplaceGain;
}

static bool requiresPlacementRc(Stage stage) {
    return stage == Stage::Path || stage == Stage::Vg || stage == Stage::Gobi || stage == Stage::Pbbi;
}

static void validateFlow(const Options& options) {
    const bool preplace =
        std::any_of(options.stages.begin(), options.stages.end(), isPreplacement);
    if (preplace && options.stages.size() != 1) {
        throw std::invalid_argument(
            "preplace is an independent flow on an unplaced design");
    }
    if (preplace && (!options.inputs.spef.empty() || options.inputs.global_routing)) {
        throw std::invalid_argument(
            "preplace uses zero wire RC, without SPEF or global routing");
    }
    const bool placement_only =
        std::any_of(options.stages.begin(), options.stages.end(), requiresPlacementRc);
    if (placement_only &&
        (!options.inputs.spef.empty() || options.inputs.global_routing)) {
        throw std::invalid_argument(
            "buffer requires placement RC; use drv, setup, power or hold for "
            "postroute/global-routing repair");
    }
}

static Options parseOptions(int argc, char** argv) {
    Options result;
    result.repair.sta_threads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::string> configs, overrides;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= argc || std::string_view(argv[i + 1]).starts_with("--")) {
                throw std::invalid_argument("missing value for " + arg);
            }
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h") {
            result.help = true;
        } else if (arg == "--def") {
            result.inputs.def = value();
        } else if (arg == "--odb") {
            result.inputs.odb = value();
        } else if (arg == "--lef") {
            result.inputs.lef.push_back(value());
        } else if (arg == "--lib") {
            result.inputs.liberty.push_back(value());
        } else if (arg == "--sdc") {
            result.inputs.sdc = value();
        } else if (arg == "--rc") {
            result.inputs.rc = value();
        } else if (arg == "--spef") {
            result.inputs.spef = value();
        } else if (arg == "--global-routing") {
            result.inputs.global_routing = true;
        } else if (arg == "--legalizer") {
            const auto name = value();
            if (name == "opendp") {
                result.legalizer = PlacementLegalizer::OpenDP;
            } else if (name == "abacus") {
                result.legalizer = PlacementLegalizer::Abacus;
            } else {
                throw std::invalid_argument("unknown legalizer: " + name);
            }
        } else if (arg == "--flow") {
            result.stages = parseStages(value());
        } else if (arg == "--output") {
            result.output = value();
        } else if (arg == "--threads") {
            overrides.push_back("runtime.sta_threads=" + value());
        } else if (arg == "--config") {
            configs.push_back(value());
        } else if (arg == "--set") {
            overrides.push_back(value());
        } else if (arg == "--log") {
            result.log = value();
        } else if (arg == "--log-level") {
            const auto level = value();
            if (level != "debug" && level != "info" && level != "warn" &&
                level != "error" && level != "off") {
                throw std::invalid_argument("invalid log level: " + level);
            }
            result.level = spdlog::level::from_str(level);
        } else if (arg.starts_with('-')) {
            throw std::invalid_argument("unknown option: " + arg);
        } else {
            if (!result.inputs.script.empty()) {
                throw std::invalid_argument("supply only one design Tcl script");
            }
            result.inputs.script = arg;
        }
    }
    if (result.help) {
        return result;
    }
    const auto& in = result.inputs;
    if (unsigned(!in.script.empty()) + unsigned(!in.def.empty()) +
            unsigned(!in.odb.empty()) !=
        1) {
        throw std::invalid_argument("supply one design Tcl script, --def, or --odb");
    }
    if (!in.script.empty() && (!in.lef.empty() || !in.liberty.empty())) {
        throw std::invalid_argument(
            "read LEF/Liberty in the design script or use direct DEF/ODB "
            "input");
    }
    if (in.script.empty() && (in.liberty.empty() || in.sdc.empty())) {
        throw std::invalid_argument("direct DEF/ODB input requires --lib and --sdc");
    }
    if (!in.def.empty() && in.lef.empty()) {
        throw std::invalid_argument("DEF input requires --lef (technology first)");
    }
    for (const auto& path : configs) {
        result.repair.readFile(path);
    }
    for (const auto& override : overrides) {
        const auto split = override.find('=');
        if (split == std::string::npos) {
            throw std::invalid_argument("expected --set KEY=VALUE: " + override);
        }
        result.repair.set(override.substr(0, split), override.substr(split + 1));
    }
    result.repair.validate();
    validateFlow(result);
    if (result.output.empty()) {
        const auto& input = !in.script.empty() ? in.script
                            : !in.odb.empty()  ? in.odb
                                               : in.def;
        result.output = std::filesystem::path(input).stem().string() + ".sized";
    }
    // Do not overwrite an input ODB/DEF through an output-prefix alias.
    for (const auto& input : {in.odb, in.def}) {
        if (input.empty()) {
            continue;
        }
        const auto output =
            result.output + std::filesystem::path(input).extension().string();
        if (std::filesystem::weakly_canonical(input) ==
            std::filesystem::weakly_canonical(output)) {
            throw std::invalid_argument("output prefix would overwrite input: " +
                                        input);
        }
    }
    return result;
}

template <class Result> void report(const std::string& stage, const Result& result) {
    if (!result.ok) {
        throw std::runtime_error(stage + ": " + result.message);
    }
    spdlog::info("{}: {}", stage, result.message);
}

void report(const std::string& stage, const BufferRepairResult& result) {
    report<BufferRepairResult>(stage, result);
    if (stage != "path") {
        spdlog::info("Buffer solve: strategy={} backend={} nets={} plans={} "
                     "buffers={} collect={:.6f}s solve={:.6f}s apply={:.6f}s "
                     "fallback=\"{}\"",
                     stage, result.backend_used, result.nets_collected,
                     result.plans_collected, result.inserted_buffers,
                     result.collect_ms / 1000, result.solve_ms / 1000,
                     result.apply_ms / 1000, result.cuda_fallback);
    }
}

static void logBufferEvent(const BufferEvent& event) {
    using Kind = BufferEvent::Kind;
    switch (event.kind) {
    case Kind::SearchStarted:
        spdlog::info("Path search started: WNS={:.6f}ns TNS={:.6f}ns provisional={}",
                     event.setup_wns * 1e9, event.setup_tns * 1e9,
                     event.placement_provisional);
        break;
    case Kind::CandidatesCollected:
        spdlog::debug("Path candidates: wave={} total={}", event.wave_candidates,
                      event.candidates);
        break;
    case Kind::BatchBackoff:
        spdlog::debug("Path batch rejected: retrying candidates individually");
        break;
    case Kind::SearchFinished:
        spdlog::info("Path search finished: trials={} rejected={} buffers={} "
                     "WNS={:.6f}ns TNS={:.6f}ns provisional={}",
                     event.attempted_trials, event.rejected_trials,
                     event.inserted_buffers, event.setup_wns * 1e9,
                     event.setup_tns * 1e9, event.placement_provisional);
        break;
    case Kind::FinalPlacementStarted:
        spdlog::info("Path final placement started: provisional_buffers={}",
                     event.inserted_buffers);
        break;
    case Kind::FinalPlacementAccepted:
    case Kind::FinalPlacementRolledBack:
        spdlog::info(
            "Path final placement {}: buffers={} WNS={:.6f}ns TNS={:.6f}ns",
            event.kind == Kind::FinalPlacementAccepted ? "accepted" : "rolled back",
            event.inserted_buffers, event.setup_wns * 1e9, event.setup_tns * 1e9);
        break;
    }
}

// Application owns the session and stage services. The callbacks capture this
// stable object and are consumed synchronously by repair/algorithm calls.
class ApplicationFlow {
  public:
    explicit ApplicationFlow(const Options& policy)
        : options(policy), session(options.inputs), block(session.block()),
          sta(session.timing()), preplace(isPreplacement(options.stages.front())),
          refresh([this]() { session.refresh(); }), placeAndRefresh([this]() {
              runtime.prepare();
              refresh();
          }) {}
    ApplicationFlow(const ApplicationFlow&) = delete;
    ApplicationFlow& operator=(const ApplicationFlow&) = delete;

    void run() {
        initialize();
        // Keep callbacks alive across edits AND ECO undo. SPEF nets retain
        // extracted RC; edited topology is re-estimated by OpenROAD.
        std::unique_ptr<est::IncrementalParasiticsGuard> extracted_updates;
        if (session.hasExtractedParasitics()) {
            extracted_updates = std::make_unique<est::IncrementalParasiticsGuard>(
                session.design().getResizer()->getEstimateParasitics());
        }
        for (const auto stage : options.stages) {
            const auto begin = std::chrono::steady_clock::now();
            const auto name =
                std::find_if(std::begin(stage_names), std::end(stage_names),
                             [&](const auto& item) { return item.stage == stage; })
                    ->name;
            spdlog::info("Starting {}", name);
            execute(stage);
            reportStage(name, begin);
        }
        session.write(options.output);
    }

  private:
    void initialize() {
        if (sta->scenes().size() != 1) {
            throw std::runtime_error("repair currently requires one STA scene");
        }
        sta->setThreadCount(static_cast<int>(options.repair.sta_threads));
        const bool placement_only = std::any_of(
            options.stages.begin(), options.stages.end(), requiresPlacementRc);
        if (placement_only &&
            (!options.inputs.spef.empty() || options.inputs.global_routing ||
             session.hasExtractedParasitics())) {
            throw std::invalid_argument(
                "buffer requires placement RC; use drv, setup, power or hold "
                "for postroute/global-routing repair");
        }
        const auto& common = options.repair.common;
        // Tcl set_dont_use updates Resizer's set, not LibertyCell::dontUse().
        // Carry it into Sizer-owned searches without duplicating parsing or
        // changing the Liberty model. The input script has finished here.
        auto* network = sta->getDbNetwork();
        auto* resizer = session.design().getResizer();
        for (auto* lib : block->getDataBase()->getLibs()) {
            for (auto* master : lib->getMasters()) {
                auto* cell = network->libertyCell(network->dbToSta(master));
                if (cell && resizer->dontUse(cell)) {
                    runtime.constraints.excluded_masters.push_back(master);
                }
            }
        }
        for (auto* inst : block->getInsts()) {
            if (inst->isDoNotTouch()) {
                runtime.constraints.excluded_instances.push_back(inst);
            }
        }
        runtime.prepare = [this]() {
            legalizeRepairPlacement(block, session.design().getOpendp(),
                                    runtime.constraints.excluded_instances, false,
                                    options.legalizer);
        };
        auto* top = sta->network()->cell(sta->network()->topInstance());
        sta->cmdSdc()->fanoutLimit(top, sta::MinMax::max(), limit, has_limit);
        // A convenience default must never relax an input SDC constraint.
        sta->setFanoutLimit(top, sta::MinMax::max(),
                            has_limit ? std::min(limit, float(common.max_fanout))
                                      : common.max_fanout,
                            sta->cmdSdc());
        if (!preplace) {
            session.initializeParasitics();
            if (session.hasExtractedParasitics()) {
                refresh();
            } else {
                placeAndRefresh();
            }
        } else if (session.hasExtractedParasitics()) {
            throw std::invalid_argument(
                "preplace uses zero wire RC, without SPEF or global routing");
        }
    }

    void runBuffer(Stage strategy) {
        BufferRuntime buffer_runtime;
        static_cast<RepairRuntime&>(buffer_runtime) = runtime;
        if (strategy == Stage::Pbbi) {
            report("pbbi", BufferRepairer::run(
                               block, sta, refresh, session.design().getResizer(),
                               options.repair.pbbiOptions(), buffer_runtime));
            return;
        }
        if (strategy == Stage::Vg) {
            report("vg", BufferRepairer::run(
                             block, sta, refresh, session.design().getResizer(),
                             options.repair.vgOptions(), buffer_runtime));
            return;
        }
        if (strategy == Stage::Gobi) {
            report("gobi", BufferRepairer::run(
                               block, sta, refresh, session.design().getResizer(),
                               options.repair.gobiOptions(), buffer_runtime));
            return;
        }
        auto path = options.repair.pathOptions();
        const auto report_path = [&](const BufferRepairResult& result) {
            report("path", result);
            spdlog::debug("Path work: candidates={:.3f}s prepare={:.3f}s "
                          "refresh={:.3f}s measure={:.3f}s final={:.3f}s "
                          "refreshes={} rollback_refreshes={}",
                          result.candidate_ms / 1000, result.prepare_ms / 1000,
                          result.refresh_ms / 1000, result.measure_ms / 1000,
                          result.final_placement.elapsed_ms / 1000, result.refreshes,
                          result.rollback_refreshes);
            spdlog::debug("Path candidates breakdown: path_sampling={:.3f}s "
                          "tree_planning_prediction={:.3f}s",
                          result.search.sampling_ms / 1000, result.search.tree_ms / 1000);
            if (path.search_patience) {
                spdlog::debug("Path search patience: remaining={:.3f}/{} exhausted={}",
                              result.remaining_patience, *path.search_patience,
                              result.stopped_on_low_progress);
            }
        };
        buffer_runtime.trial_observer = [](unsigned index,
                                           const BufferTrialResult& trial) {
            if (trial.rejection_reasons.empty()) {
                spdlog::info("Path commit: trial={} buffers=+{} WNS={:.6f}ns "
                             "TNS={:.6f}ns provisional={}",
                             index, trial.buffers, double(trial.after.setup_wns) * 1e9,
                             double(trial.after.setup_tns) * 1e9,
                             trial.placement_provisional);
            }
            if (!spdlog::should_log(spdlog::level::debug)) {
                return;
            }
            spdlog::debug(
                "Path trial {}: accepted={} buffers={} provisional={} "
                "full_rc_rechecked={} metrics_complete={} reasons=[{}]; global gain "
                "WNS={:+.6f}ns "
                "TNS={:+.6f}ns; objective gain WNS={:+.6f}ns TNS={:+.6f}ns; "
                "required gain WNS={:.6f}ns TNS={:.6f}ns; "
                "prepare={:.1f}ms refresh={:.1f}ms measure={:.1f}ms",
                index, trial.rejection_reasons.empty(), trial.buffers,
                trial.placement_provisional, trial.rechecked_full_rc,
                trial.metrics_complete, fmt::join(trial.rejection_reasons, ","),
                (double(trial.after.setup_wns) - trial.before.setup_wns) * 1e9,
                (double(trial.after.setup_tns) - trial.before.setup_tns) * 1e9,
                (trial.objective_wns_after - trial.objective_wns_before) * 1e9,
                (trial.objective_tns_after - trial.objective_tns_before) * 1e9,
                trial.required_wns_gain * 1e9, trial.required_tns_gain * 1e9,
                trial.prepare_ms, trial.refresh_ms, trial.measure_ms);
            for (const auto& candidate : trial.candidates) {
                spdlog::debug("Path candidate: trial={} net={} master={} loads={} "
                              "planned_buffers={} position=({}, {}) generation={}",
                              index, candidate.net, candidate.master, candidate.loads,
                              candidate.planned_buffers, candidate.x, candidate.y,
                              candidate.generation);
            }
        };
        buffer_runtime.event_observer = logBufferEvent;
        if (path.incremental_parasitics) {
            PlacementParasiticUpdater incremental(
                block, sta, session.design().getResizer()->getEstimateParasitics());
            buffer_runtime.verification_refresh = refresh;
            buffer_runtime.incremental_verification = [&]() {
                incremental.verifyTiming();
            };
            buffer_runtime.analysis_changes = [&]() {
                return incremental.takeAnalysisChanges();
            };
            report_path(BufferRepairer::run(
                               block, sta,
                               [&]() {
                                   session.reconnectSupplies();
                                   incremental.updateVerified();
                               },
                               path, buffer_runtime));
            const auto& stats = incremental.statistics();
            spdlog::debug("Path incremental update: geometry={:.3f}s RC={:.3f}s "
                          "STA={:.3f}s updates={} verifications={}",
                          stats.geometry_ms / 1000, stats.rc_ms / 1000,
                          stats.sta_ms / 1000, stats.updates, stats.verifications);
        } else {
            report_path(BufferRepairer::run(block, sta, refresh, path, buffer_runtime));
        }
    }

    void runFanout() {
        odb::dbMaster* buffer = nullptr;
        for (auto* lib : block->getDataBase()->getLibs()) {
            for (auto* master : lib->getMasters()) {
                auto* cell = sta->getDbNetwork()->libertyCell(
                    sta->getDbNetwork()->dbToSta(master));
                if (!master->isFrozen() || !master->isCore() || !cell ||
                    cell->dontUse() || !cell->isBuffer() ||
                    std::find(runtime.constraints.excluded_masters.begin(),
                              runtime.constraints.excluded_masters.end(),
                              master) != runtime.constraints.excluded_masters.end()) {
                    continue;
                }
                const auto area = [](auto* m) {
                    return uint64_t(m->getWidth()) * m->getHeight();
                };
                if (!buffer || area(master) < area(buffer) ||
                    (area(master) == area(buffer) &&
                     master->getName() < buffer->getName())) {
                    buffer = master;
                }
            }
        }
        if (!buffer) {
            throw std::runtime_error("no usable buffer master for fanout repair");
        }
        FanoutRepairOptions fanout;
        fanout.max_fanout = options.repair.common.max_fanout;
        if (has_limit) {
            fanout.max_fanout =
                std::min(fanout.max_fanout, unsigned(std::max(0.0f, limit)));
        }
        for (auto* net : block->getNets()) {
            if (sta->isClock(sta->getDbNetwork()->dbToSta(net), sta->cmdMode())) {
                fanout.excluded_nets.push_back(net);
            }
        }
        const auto result = FanoutRepairer::run(block, buffer, fanout);
        if (!result) {
            throw std::runtime_error("fanout: " + result.message);
        }
        placeAndRefresh();
        spdlog::info("Fanout repair inserted {} buffers",
                     result.inserted_buffers.size());
    }

    void execute(Stage stage) {
        switch (stage) {
        case Stage::Drv:
            report("drv", DrvRepairer::run(block, sta, refresh,
                                           options.repair.drvOptions(), runtime));
            break;
        case Stage::Setup:
            report("setup", SetupRepairer::run(block, sta, refresh,
                                               options.repair.setupOptions(), runtime));
            break;
        case Stage::Path:
        case Stage::Pbbi:
        case Stage::Vg:
        case Stage::Gobi:
            runBuffer(stage);
            break;
        case Stage::Ord:
            report("ord", OrdDrvRepairer::run(block, sta, refresh,
                                              options.repair.drvOptions(), runtime));
            break;
        case Stage::Power:
            report("power", PowerRepairer::run(block, sta, refresh,
                                               options.repair.powerOptions(), runtime));
            break;
        case Stage::Fanout:
            runFanout();
            break;
        case Stage::Hold:
            // Preserve repair -> legalize -> refresh -> repair.
            for (unsigned pass = 0; pass < 2; ++pass) {
                session.command({"repair_timing", "-hold", "-skip_pin_swap"});
                placeAndRefresh();
            }
            break;
        case Stage::Preplace:
        case Stage::PreplaceGain:
            session.command({"set_wire_rc"});
            session.command(
                {"unset_propagated_clock", session.command({"all_clocks"})});
            if (stage == Stage::PreplaceGain) {
                session.command({"repair_design", "-pre_placement"});
            } else {
                session.command({"repair_design"});
            }
            session.reconnectSupplies();
            break;
        case Stage::Legalize:
            placeAndRefresh();
            break;
        }
    }

    void reportStage(std::string_view name,
                     std::chrono::steady_clock::time_point begin) {
        if (!preplace) {
            session.command({"check_placement"});
        }
        sta->checkFanoutPreamble();
        const auto elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
                .count();
        spdlog::info("{} finished in {:.3f}s; setup WNS={:.6f}ns TNS={:.6f}ns; slew={} "
                     "cap={} fanout={}",
                     name, elapsed, double(sta->worstSlack(sta::MinMax::max())) * 1e9,
                     double(sta->totalNegativeSlack(sta::MinMax::max())) * 1e9,
                     sta->maxSlewViolationCount(), sta->maxCapacitanceViolationCount(),
                     sta->fanoutViolationCount(sta::MinMax::max(), sta->modes()));
    }

    const Options& options;
    DesignSession session;
    odb::dbBlock* block;
    sta::dbSta* sta;
    RepairRuntime runtime;
    float limit = 0;
    bool has_limit = false;
    const bool preplace;
    const std::function<void()> refresh, placeAndRefresh;
};

static void run(const Options& options) {
    if (std::find(options.stages.begin(), options.stages.end(), Stage::Gobi) !=
            options.stages.end() &&
        !gobi::gobiCudaAvailable()) {
        throw std::runtime_error("GOBI requires a CUDA build and a usable CUDA device");
    }
    ApplicationFlow(options).run();
}
} // namespace ecc

int main(int argc, char** argv) {
    try {
        auto options = ecc::parseOptions(argc, argv);
        if (options.help) {
            spdlog::set_pattern("%v");
            spdlog::info("{}", ecc::help);
            return 0;
        }
        std::vector<spdlog::sink_ptr> sinks{
            std::make_shared<spdlog::sinks::stderr_color_sink_mt>()};
        if (!options.log.empty()) {
            sinks.push_back(
                std::make_shared<spdlog::sinks::basic_file_sink_mt>(options.log, true));
        }
        auto logger =
            std::make_shared<spdlog::logger>("sizer", sinks.begin(), sinks.end());
        logger->set_pattern("[%l] %v");
        logger->set_level(options.level);
        logger->flush_on(spdlog::level::info);
        spdlog::set_default_logger(logger);
        ecc::run(options);
        return 0;
    } catch (const std::exception& error) {
        // Errors remain visible even if normal application logging is disabled.
        spdlog::default_logger()->set_level(spdlog::level::err);
        spdlog::error("{}", error.what());
        return 1;
    }
}
