// SPDX-License-Identifier: Apache-2.0
#include "design_io.h"

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "lef_layer_rc.h"
#include "grt/GlobalRouter.h"
#include "odb/db.h"
#include "ord/Design.h"
#include "ord/OpenRoad.hh"
#include "ord/Tech.h"
#include "rsz/Resizer.hh"
#include "sta/Parasitics.hh"
#include "sta/Scene.hh"
#include "special_net.h"
#include "spdlog/spdlog.h"
#include "tcl.h"

namespace ecc {
static void requireFile(const std::string& path) {
    if(!std::filesystem::is_regular_file(path))
        throw std::invalid_argument("input file does not exist: " + path);
}

static unsigned missingSupplies(odb::dbBlock* block) {
    // Netlists before power planning can legitimately have no supply nets.
    const bool has_supplies =
        std::any_of(block->getNets().begin(), block->getNets().end(),
                    [](auto* net) { return net->getSigType().isSupply(); });
    if(!has_supplies)
        return 0;
    unsigned count = 0;
    for(auto* inst : block->getInsts())
        for(auto* pin : inst->getITerms()) {
            const auto type = pin->getSigType();
            if((type == odb::dbSigType::POWER ||
                type == odb::dbSigType::GROUND) &&
               (!pin->getNet() || pin->getNet()->getSigType() != type))
                ++count;
        }
    return count;
}

DesignSession::DesignSession(const DesignInputs& inputs) : inputs_(inputs) {
    for(const auto& path : inputs.lef)
        requireFile(path);
    for(const auto& path : inputs.liberty)
        requireFile(path);
    for(const auto& path : {inputs.script, inputs.odb, inputs.def, inputs.sdc,
                            inputs.rc, inputs.spef})
        if(!path.empty())
            requireFile(path);

    tech_ = std::make_unique< ord::Tech >();
    design_ = std::make_unique< ord::Design >(tech_.get());
    // Bind OpenROAD/STA's Tcl context once; subsequent calls check Tcl status.
    design_->evalTclString("");
    if(!inputs.script.empty()) {
        command({"source", inputs.script});
    }
    else {
        for(const auto& path : inputs.lef)
            command({"read_lef", path});
        for(const auto& path : inputs.liberty)
            command({"read_liberty", path});
        if(!inputs.odb.empty())
            design_->readDb(inputs.odb);
        else
            design_->readDef(inputs.def);
    }
    if(!block())
        throw std::runtime_error("input did not load an OpenDB design");
    if(!timing()->getDbNetwork()->defaultLibertyLibrary())
        throw std::runtime_error(
            "no Liberty library loaded; supply --lib or read_liberty in the "
            "input script");
    if(!inputs.sdc.empty())
        command({"read_sdc", inputs.sdc});
    if(!inputs.rc.empty())
        command({"source", inputs.rc});
    // OpenSTA read_spef does not register its networks with the placement
    // estimator. Detect those loaded by input Tcl before any refresh can
    // replace them. Explicitly estimated placement/GR RC is not SPEF.
    auto* estimator = design_->getOpenRoad()->getEstimateParasitics();
    if(estimator->getParasiticsSrc() == est::ParasiticsSrc::kNone) {
        for(auto* scene : timing()->scenes()) {
            auto* parasitics = scene->parasitics(sta::MinMax::max());
            script_spef_ |= parasitics && parasitics->haveParasitics();
        }
    }
    markExternalPadOnlyNetsDontTouch(block());
    block()->globalConnect(false, false);
    connectMissingWildcardSpecialNetPins(block());
    baseline_missing_supplies_ = missingSupplies(block());
    design_->getResizer()->setBlockageAwareSteiner(true);
}

DesignSession::~DesignSession() = default;
ord::Design& DesignSession::design() const {
    return *design_;
}
odb::dbBlock* DesignSession::block() const {
    return design_->getBlock();
}
sta::dbSta* DesignSession::timing() const {
    return tech_->getSta();
}

std::string DesignSession::command(
    std::initializer_list< std::string > words) const {
    auto* interp = design_->getOpenRoad()->tclInterp();
    struct Arguments {
        std::vector< Tcl_Obj* > values;
        ~Arguments() {
            for(auto* obj : values)
                Tcl_DecrRefCount(obj);
        }
    } args;
    args.values.reserve(words.size());
    for(const auto& word : words) {
        auto* obj =
            Tcl_NewStringObj(word.data(), static_cast< int >(word.size()));
        Tcl_IncrRefCount(obj);
        args.values.push_back(obj);
    }
    const int code =
        Tcl_EvalObjv(interp, static_cast< int >(args.values.size()),
                     args.values.data(), TCL_EVAL_GLOBAL);
    const std::string result = Tcl_GetStringResult(interp);
    if(code != TCL_OK)
        throw std::runtime_error((words.size() ? *words.begin() : "Tcl") +
                                 ": " + result);
    return result;
}

void DesignSession::initializePlacementRC() {
    auto* estimator = design_->getOpenRoad()->getEstimateParasitics();
    auto* tech = tech_->getTech();
    if(!tech)
        throw std::runtime_error("no technology loaded");
    // Keep explicitly configured RC. Otherwise derive signal RC from the
    // lowest routing layer with physical RC in the LEF, without platform names.
    for(auto* scene : timing()->scenes()) {
        const double resistance = estimator->wireSignalResistance(scene);
        const double capacitance = estimator->wireSignalCapacitance(scene);
        if(std::isfinite(resistance) && std::isfinite(capacitance) &&
           resistance > 0 && capacitance > 0)
            continue;
        if(resistance != 0 || capacitance != 0)
            throw std::runtime_error(
                "incomplete or invalid signal RC; configure set_wire_rc in "
                "--rc or the input script");
        odb::dbTechLayer* selected = nullptr;
        std::optional< LefLayerRC > selected_rc;
        for(auto* layer : tech->getLayers()) {
            if(layer->getType() != odb::dbTechLayerType::ROUTING)
                continue;
            const auto rc = lefLayerRC(
                double(layer->getWidth()) / tech->getDbUnitsPerMicron(),
                layer->getResistance(), layer->getCapacitance(),
                layer->getEdgeCapacitance());
            if(rc && rc->resistance_per_meter > 0 &&
               rc->capacitance_per_meter > 0 &&
               (!selected ||
                layer->getRoutingLevel() < selected->getRoutingLevel())) {
                selected = layer;
                selected_rc = rc;
            }
        }
        if(!selected_rc)
            throw std::runtime_error(
                "LEF has no usable wire RC; provide --rc with set_wire_rc");
        estimator->setHWireSignalRC(scene, selected_rc->resistance_per_meter,
                                    selected_rc->capacitance_per_meter);
        estimator->setVWireSignalRC(scene, selected_rc->resistance_per_meter,
                                    selected_rc->capacitance_per_meter);
        spdlog::info("Placement RC from LEF layer {}", selected->getName());
    }
}

void DesignSession::initializeParasitics() {
    initializePlacementRC();
    auto* estimate_parasitics = design_->getOpenRoad()->getEstimateParasitics();
    if(inputs_.global_routing || !inputs_.spef.empty() ||
       hasExtractedParasitics()) {
        auto* tech = tech_->getTech();
        const double dbu_per_micron = tech->getDbUnitsPerMicron();
        auto* router = design_->getGlobalRouter();
        int low = router->getMinRoutingLayer(),
            high = router->getMaxRoutingLayer();
        if(low <= 0)
            low = 1;
        if(high < low)
            high = tech->getRoutingLayerCount();
        auto* min_layer = tech->findRoutingLayer(low);
        auto* max_layer = tech->findRoutingLayer(high);
        if(!min_layer || !max_layer)
            throw std::runtime_error("no routing layers");
        router->setMinRoutingLayer(low);
        router->setMaxRoutingLayer(high);
        int initialized_routing_layers = 0;
        int initialized_cut_layers = 0;
        for(auto* layer : tech->getLayers()) {
            const bool routing =
                layer->getType() == odb::dbTechLayerType::ROUTING;
            const bool cut = layer->getType() == odb::dbTechLayerType::CUT;
            if(!routing && !cut) {
                continue;
            }

            std::optional< ecc::LefLayerRC > lef_rc;
            if(routing && dbu_per_micron > 0.0) {
                lef_rc = ecc::lefLayerRC(
                    layer->getWidth() / dbu_per_micron, layer->getResistance(),
                    layer->getCapacitance(), layer->getEdgeCapacitance());
            }
            std::optional< double > lef_via_resistance;
            bool conflicting_vias = false;
            if(cut && layer->getResistance() <= 0.0) {
                for(auto* via : tech->getVias()) {
                    auto* bottom = via->getBottomLayer();
                    if(!via->isDefault() || !bottom ||
                       bottom->getUpperLayer() != layer ||
                       via->getTopLayer() != layer->getUpperLayer() ||
                       via->getResistance() <= 0.0) {
                        continue;
                    }
                    const double resistance = via->getResistance();
                    if(lef_via_resistance &&
                       std::abs(*lef_via_resistance - resistance) >
                           1e-9 * std::max(1.0, *lef_via_resistance)) {
                        conflicting_vias = true;
                        break;
                    }
                    lef_via_resistance = resistance;
                }
            }
            for(auto* scene : timing()->scenes()) {
                double resistance = 0.0;
                double capacitance = 0.0;
                estimate_parasitics->layerRC(layer, scene, resistance,
                                             capacitance);
                if(routing && lef_rc) {
                    const auto filled = ecc::fillMissingLayerRC(
                        {resistance, capacitance}, *lef_rc);
                    if(filled.resistance_per_meter != resistance ||
                       filled.capacitance_per_meter != capacitance) {
                        estimate_parasitics->setLayerRC(
                            layer, scene, filled.resistance_per_meter,
                            filled.capacitance_per_meter);
                        ++initialized_routing_layers;
                    }
                }
                else if(cut && resistance == 0.0) {
                    const double lef_cut_resistance =
                        layer->getResistance() > 0.0
                            ? layer->getResistance()
                            : (!conflicting_vias && lef_via_resistance
                                   ? *lef_via_resistance
                                   : 0.0);
                    if(lef_cut_resistance > 0.0) {
                        estimate_parasitics->setLayerRC(
                            layer, scene, lef_cut_resistance, capacitance);
                        ++initialized_cut_layers;
                    }
                }

                if(cut) {
                    estimate_parasitics->layerRC(layer, scene, resistance,
                                                 capacitance);
                    if(resistance <= 0.0) {
                        spdlog::debug("No unambiguous LEF via RC for {}",
                                      layer->getName());
                    }
                }
                if(routing &&
                   layer->getRoutingLevel() >= min_layer->getRoutingLevel() &&
                   layer->getRoutingLevel() <= max_layer->getRoutingLevel()) {
                    estimate_parasitics->layerRC(layer, scene, resistance,
                                                 capacitance);
                    if(!std::isfinite(resistance) ||
                       !std::isfinite(capacitance) || resistance <= 0.0 ||
                       capacitance <= 0.0) {
                        throw std::runtime_error(
                            "invalid routing-layer RC; configure --rc");
                    }
                }
            }
        }

        spdlog::debug(
            "Initialized RC for {} routing and {} cut layer/scene entries",
            initialized_routing_layers, initialized_cut_layers);
    }
    if(!inputs_.spef.empty()) {
        command({"read_spef", inputs_.spef});
    }
    if(!inputs_.spef.empty() || script_spef_) {
        estimate_parasitics->initSpefParasitics();
        if(!estimate_parasitics->hasSpefNets())
            throw std::runtime_error(
                "SPEF contains no usable design parasitics");
    }
}

bool DesignSession::hasExtractedParasitics() const {
    return script_spef_ ||
           design_->getOpenRoad()->getEstimateParasitics()->hasSpefNets();
}

void DesignSession::reconnectSupplies() {
    block()->globalConnect(false, false);
    connectMissingWildcardSpecialNetPins(block());
    if(missingSupplies(block()) > baseline_missing_supplies_)
        throw std::runtime_error(
            "repair introduced disconnected or mismatched supply pins");
}

void DesignSession::refresh() {
    reconnectSupplies();
    auto* estimator = design_->getOpenRoad()->getEstimateParasitics();
    if(hasExtractedParasitics()) {
        design_->getResizer()->updateParasiticsAndTiming();
        estimator->refreshNonSpefParasitics(inputs_.global_routing);
    }
    else if(inputs_.global_routing) {
        command({"estimate_parasitics", "-placement"});
        design_->getGlobalRouter()->clear();
        command({"global_route", "-allow_congestion"});
        command({"estimate_parasitics", "-global_routing"});
    }
    else {
        command({"estimate_parasitics", "-placement"});
    }
    timing()->findRequireds();
}

void DesignSession::write(const std::string& prefix) const {
    const auto parent = std::filesystem::path(prefix).parent_path();
    if(!parent.empty())
        std::filesystem::create_directories(parent);
    design_->writeDef(prefix + ".def");
    command({"write_verilog", prefix + ".v"});
    design_->writeDb(prefix + ".odb");
    spdlog::info("Wrote {}.def, {}.v and {}.odb", prefix, prefix, prefix);
}
}  // namespace ecc
