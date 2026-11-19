// SPDX-License-Identifier: Apache-2.0
#include "design_io.h"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "lef_layer_rc.h"
#include "odb/db.h"
#include "ord/Design.h"
#include "ord/OpenRoad.hh"
#include "ord/Tech.h"
#include "rsz/Resizer.hh"
#include "special_net.h"
#include "spdlog/spdlog.h"
#include "tcl.h"

namespace ecc {
namespace {
void requireFile(const std::string& path) {
    if(!std::filesystem::is_regular_file(path))
        throw std::invalid_argument("input file does not exist: " + path);
}

unsigned missingSupplies(odb::dbBlock* block) {
    unsigned count = 0;
    for(auto* inst : block->getInsts())
        for(auto* pin : inst->getITerms()) {
            const auto type = pin->getSigType();
            if((type == odb::dbSigType::POWER || type == odb::dbSigType::GROUND)
               && (!pin->getNet() || pin->getNet()->getSigType() != type))
                ++count;
        }
    return count;
}
}  // namespace

DesignSession::DesignSession(const DesignInputs& inputs) {
    for(const auto& path : inputs.lef) requireFile(path);
    for(const auto& path : inputs.liberty) requireFile(path);
    for(const auto& path : {inputs.script, inputs.odb, inputs.def, inputs.sdc, inputs.rc})
        if(!path.empty()) requireFile(path);

    tech_ = std::make_unique<ord::Tech>();
    design_ = std::make_unique<ord::Design>(tech_.get());
    // Bind OpenROAD/STA's Tcl context once; subsequent calls check Tcl status.
    design_->evalTclString("");
    if(!inputs.script.empty()) {
        command({"source", inputs.script});
    } else {
        for(const auto& path : inputs.lef) command({"read_lef", path});
        for(const auto& path : inputs.liberty) command({"read_liberty", path});
        if(!inputs.odb.empty()) design_->readDb(inputs.odb);
        else design_->readDef(inputs.def);
    }
    if(!block()) throw std::runtime_error("input did not load an OpenDB design");
    if(!timing()->getDbNetwork()->defaultLibertyLibrary())
        throw std::runtime_error("no Liberty library loaded; supply --lib or read_liberty in the input script");
    if(!inputs.sdc.empty()) command({"read_sdc", inputs.sdc});
    if(!inputs.rc.empty()) command({"source", inputs.rc});
    markExternalPadOnlyNetsDontTouch(block());
    block()->globalConnect(false, false);
    connectMissingWildcardSpecialNetPins(block());
    baseline_missing_supplies_ = missingSupplies(block());
    design_->getResizer()->setBlockageAwareSteiner(true);
}

DesignSession::~DesignSession() = default;
ord::Design& DesignSession::design() const { return *design_; }
odb::dbBlock* DesignSession::block() const { return design_->getBlock(); }
sta::dbSta* DesignSession::timing() const { return tech_->getSta(); }

std::string DesignSession::command(std::initializer_list<std::string> words) const {
    auto* interp = design_->getOpenRoad()->tclInterp();
    struct Arguments {
        std::vector<Tcl_Obj*> values;
        ~Arguments() { for(auto* obj : values) Tcl_DecrRefCount(obj); }
    } args;
    args.values.reserve(words.size());
    for(const auto& word : words) {
        auto* obj = Tcl_NewStringObj(word.data(), static_cast<int>(word.size()));
        Tcl_IncrRefCount(obj);
        args.values.push_back(obj);
    }
    const int code = Tcl_EvalObjv(interp, static_cast<int>(args.values.size()),
                                  args.values.data(), TCL_EVAL_GLOBAL);
    const std::string result = Tcl_GetStringResult(interp);
    if(code != TCL_OK)
        throw std::runtime_error((words.size() ? *words.begin() : "Tcl") + ": " + result);
    return result;
}

void DesignSession::initializePlacementRC() {
    auto* estimator = design_->getOpenRoad()->getEstimateParasitics();
    auto* tech = tech_->getTech();
    if(!tech) throw std::runtime_error("no technology loaded");
    // Keep explicitly configured RC. Otherwise derive signal RC from the
    // lowest routing layer with physical RC in the LEF, without platform names.
    for(auto* scene : timing()->scenes()) {
        const double resistance = estimator->wireSignalResistance(scene);
        const double capacitance = estimator->wireSignalCapacitance(scene);
        if(std::isfinite(resistance) && std::isfinite(capacitance)
           && resistance > 0 && capacitance > 0) continue;
        if(resistance != 0 || capacitance != 0)
            throw std::runtime_error("incomplete or invalid signal RC; configure set_wire_rc in --rc or the input script");
        odb::dbTechLayer* selected = nullptr;
        std::optional<LefLayerRC> selected_rc;
        for(auto* layer : tech->getLayers()) {
            if(layer->getType() != odb::dbTechLayerType::ROUTING) continue;
            const auto rc = lefLayerRC(
                double(layer->getWidth()) / tech->getDbUnitsPerMicron(),
                layer->getResistance(), layer->getCapacitance(), layer->getEdgeCapacitance());
            if(rc && rc->resistance_per_meter > 0 && rc->capacitance_per_meter > 0
               && (!selected || layer->getRoutingLevel() < selected->getRoutingLevel())) {
                selected = layer;
                selected_rc = rc;
            }
        }
        if(!selected_rc)
            throw std::runtime_error("LEF has no usable wire RC; provide --rc with set_wire_rc");
        estimator->setHWireSignalRC(scene, selected_rc->resistance_per_meter,
                                    selected_rc->capacitance_per_meter);
        estimator->setVWireSignalRC(scene, selected_rc->resistance_per_meter,
                                    selected_rc->capacitance_per_meter);
        spdlog::info("Placement RC from LEF layer {}", selected->getName());
    }
}

void DesignSession::reconnectSupplies() {
    block()->globalConnect(false, false);
    connectMissingWildcardSpecialNetPins(block());
    if(missingSupplies(block()) > baseline_missing_supplies_)
        throw std::runtime_error("repair introduced disconnected or mismatched supply pins");
}

void DesignSession::refresh() {
    reconnectSupplies();
    command({"estimate_parasitics", "-placement"});
    timing()->findRequireds();
}

void DesignSession::write(const std::string& prefix) const {
    const auto parent = std::filesystem::path(prefix).parent_path();
    if(!parent.empty()) std::filesystem::create_directories(parent);
    design_->writeDef(prefix + ".def");
    command({"write_verilog", prefix + ".v"});
    design_->writeDb(prefix + ".odb");
    spdlog::info("Wrote {}.def, {}.v and {}.odb", prefix, prefix, prefix);
}
}  // namespace ecc
