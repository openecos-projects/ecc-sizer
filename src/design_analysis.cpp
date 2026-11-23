// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "design_analysis.h"
#include "sta/Fuzzy.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "sta/GraphDelayCalc.hh"
#include "sta/Graph.hh"
#include "sta/TimingRole.hh"
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Sdc.hh"
#include "sta/Scene.hh"
#include "sta/TableModel.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingModel.hh"

namespace ecc::detail {

double area(odb::dbMaster* master) {
    return double(master->getWidth()) * master->getHeight();
}

float DesignAnalysis::DelayPrediction::operator()(float in_slew) const {
    if (arcs_.empty()) {
        return kInf;
    }
    float worst = 0;
    for (const auto& arc : arcs_) {
        float delay = 0;
        if (arc.table) {
            auto values = arc.values;
            for (size_t i = 0; i < values.size(); ++i) {
                if (arc.slew_axis[i]) {
                    values[i] = in_slew;
                }
            }
            delay =
                arc.table->findValue(arc.cell, pvt_, values[0], values[1], values[2]);
        } else if (arc.fallback) {
            float slew;
            arc.fallback->gateDelay(pvt_, in_slew, cap_, delay, slew);
        }
        if (!std::isfinite(delay)) {
            return kInf;
        }
        worst = std::max(worst, delay);
    }
    return worst;
}

bool DesignAnalysis::resizeExcluded(odb::dbInst* inst) const {
    return excluded_.contains(inst) ||
           std::find(constraints_.excluded_resize_instances.begin(),
                     constraints_.excluded_resize_instances.end(),
                     inst) != constraints_.excluded_resize_instances.end();
}

DesignAnalysis::DesignAnalysis(odb::dbBlock* block, sta::dbSta* sta,
                               const RepairCommonOptions& options,
                               const DesignConstraints& constraints)
    : block_(block), sta_(sta), network_(sta->getDbNetwork()), opts_(options),
      constraints_(constraints), excluded_(constraints.excluded_instances.begin(),
                                           constraints.excluded_instances.end()) {
    std::unordered_set<odb::dbMaster*> excluded_masters(
        constraints_.excluded_masters.begin(), constraints_.excluded_masters.end());
    for (auto* lib : block->getDataBase()->getLibs()) {
        for (auto* master : lib->getMasters()) {
            if (!master->isFrozen() || !master->isCore() ||
                excluded_masters.contains(master)) {
                continue;
            }
            auto* cell = network_->libertyCell(network_->dbToSta(master));
            if (cell == nullptr || cell->dontUse() || cell->hasSequentials() ||
                cell->isClockGate() || cell->isIsolationCell() ||
                cell->isLevelShifter()) {
                continue;
            }
            sta::LibertyPort *input = nullptr, *output = nullptr;
            unsigned inputs = 0, outputs = 0;
            bool invalid = false;
            for (auto* term : master->getMTerms()) {
                if (term->getSigType() == odb::dbSigType::POWER ||
                    term->getSigType() == odb::dbSigType::GROUND) {
                    continue;
                }
                auto* port = cell->findLibertyPort(term->getConstName());
                if (!port) {
                    invalid = true;
                    break;
                }
                if (term->getIoType() == odb::dbIoType::INPUT) {
                    input = port;
                    ++inputs;
                } else if (term->getIoType() == odb::dbIoType::OUTPUT) {
                    output = port;
                    ++outputs;
                } else {
                    invalid = true;
                }
            }
            if (invalid || inputs == 0 || outputs != 1 || !output->function() ||
                output->tristateEnable()) {
                continue;
            }
            float cap = 0;
            bool exists = false;
            output->capacitanceLimit(sta::MinMax::max(), cap, exists);
            if (!exists) {
                cell->libertyLibrary()->defaultMaxCapacitance(cap, exists);
            }
            if (!exists) {
                cap = kInf;
            }
            cells_.push_back({master, cell, nullptr, output, area(master), cap});
            if (cell->isBuffer() && inputs == 1) {
                buffers_.push_back({master, cell, input, output, area(master), cap});
            }
        }
    }
    std::sort(buffers_.begin(), buffers_.end(),
              [](const CellModel& a, const CellModel& b) {
                  if (a.area != b.area) {
                      return a.area < b.area;
                  }
                  return a.master->getName() < b.master->getName();
              });
}

std::vector<NetState> DesignAnalysis::analyze(DesignMetrics& metrics, bool all_nets) {
    analysis_initialized_ = false;
    analysis_nets_.clear();
    analysis_instances_.clear();
    sta_->findRequireds();
    sta_->checkSlewsPreamble();
    sta_->checkCapacitancesPreamble(sta_->scenes());
    sta_->checkFanoutPreamble();
    metrics = {};
    metrics.setup_wns = std::min(0.0f, float(sta_->worstSlack(sta::MinMax::max())));
    metrics.hold_wns = std::min(0.0f, float(sta_->worstSlack(sta::MinMax::min())));
    metrics.setup_tns = float(sta_->totalNegativeSlack(sta::MinMax::max()));
    metrics.hold_tns = float(sta_->totalNegativeSlack(sta::MinMax::min()));
    metrics.total_slew_violations = sta_->maxSlewViolationCount();
    metrics.total_cap_violations = sta_->maxCapacitanceViolationCount();
    metrics.total_fanout_violations =
        sta_->fanoutViolationCount(sta::MinMax::max(), sta_->modes());
    for (auto* inst : block_->getInsts()) {
        metrics.area += area(inst->getMaster());
    }
    std::vector<NetState> states;
    for (auto* net : block_->getNets()) {
        auto state = analyzeNet(net, metrics);
        if (state && (all_nets || state->violating())) {
            states.push_back(std::move(*state));
        }
    }
    return states;
}

std::vector<NetState>
DesignAnalysis::analyzeIncremental(DesignMetrics& metrics,
                                   RepairAnalysisChanges changes) {
    sta_->findRequireds();
    sta_->checkSlewsPreamble();
    sta_->checkCapacitancesPreamble(sta_->scenes());
    sta_->checkFanoutPreamble();
    ++analysis_statistics_.inspections;
    if (!analysis_initialized_) {
        ++analysis_statistics_.full_scans;
        for (auto* net : block_->getNets()) {
            changes.nets.insert(net);
        }
        for (auto* inst : block_->getInsts()) {
            changes.instances.insert(inst);
        }
        changes.ports = true;
        analysis_initialized_ = true;
    }
    for (auto id : changes.removed_nets) {
        analysis_nets_.erase(id);
    }
    for (auto id : changes.removed_instances) {
        analysis_instances_.erase(id);
    }
    for (auto* net : changes.nets) {
        DesignMetrics contribution;
        auto state = analyzeNet(net, contribution);
        analysis_nets_[net->getId()] = {std::move(state), contribution};
        ++analysis_statistics_.net_checks;
    }
    for (auto* inst : changes.instances) {
        analysis_instances_[inst->getId()] = pinCounts(network_->dbToSta(inst));
        analysis_instances_[inst->getId()].area = area(inst->getMaster());
        ++analysis_statistics_.instance_checks;
    }
    if (changes.ports) {
        analysis_ports_ = pinCounts(network_->topInstance());
    }
    metrics = {};
    metrics.setup_wns = std::min(0.0f, float(sta_->worstSlack(sta::MinMax::max())));
    metrics.hold_wns = std::min(0.0f, float(sta_->worstSlack(sta::MinMax::min())));
    metrics.setup_tns = float(sta_->totalNegativeSlack(sta::MinMax::max()));
    metrics.hold_tns = float(sta_->totalNegativeSlack(sta::MinMax::min()));
    const auto add_pins = [&](const DesignMetrics& value) {
        metrics.total_slew_violations += value.total_slew_violations;
        metrics.total_cap_violations += value.total_cap_violations;
        metrics.total_fanout_violations += value.total_fanout_violations;
        metrics.area += value.area;
    };
    add_pins(analysis_ports_);
    for (const auto& [id, value] : analysis_instances_) {
        add_pins(value);
    }
    std::vector<NetState> states;
    states.reserve(analysis_nets_.size());
    // Reduce in DB-ID order, like the full scan, to preserve floating point
    // severity accumulation at strict electrical acceptance boundaries.
    for (const auto& [id, value] : analysis_nets_) {
        const auto& m = value.second;
        metrics.skipped_nets += m.skipped_nets;
        metrics.fanout_nets += m.fanout_nets;
        metrics.capacitance_nets += m.capacitance_nets;
        metrics.slew_nets += m.slew_nets;
        metrics.severity += m.severity;
        if (value.first) {
            states.push_back(*value.first);
        }
    }
    return states;
}

DesignAnalysis::DelayPrediction
DesignAnalysis::prepareDelayPrediction(const CellModel& buffer, float cap) const {
    DelayPrediction prediction;
    prediction.cap_ = cap;
    prediction.pvt_ = sta_->cmdMode()->sdc()->operatingConditions(sta::MinMax::max());
    if (!buffer.cell || !buffer.output) {
        return prediction;
    }
    auto* scene = sta_->cmdScene();
    for (auto* set : buffer.cell->timingArcSets()) {
        if ((buffer.input && set->from() != buffer.input) || !set->from() ||
            set->to() != buffer.output) {
            continue;
        }
        for (auto* arc : set->arcs()) {
            auto* model = arc->gateModel(scene, sta::MinMax::max());
            if (!model) {
                continue;
            }
            DelayPrediction::Arc entry;
            entry.fallback = model;
            const auto* scene_arc =
                arc->sceneArc(scene->libertyIndex(sta::MinMax::max()));
            const auto* table = dynamic_cast<const sta::GateTableModel*>(model);
            // A separate operating-condition model may belong to another
            // cell. Leave that case to OpenSTA instead of guessing its PVT.
            if (table && model == scene_arc->model()) {
                entry.cell = scene_arc->set()->libertyCell();
                entry.table = table->delayModel();
                if (!entry.table || sta::GateTableModel::checkAxes(entry.table)) {
                    entry.fallback = nullptr;
                    if (entry.table) {
                        const std::array axes{entry.table->axis1(),
                                              entry.table->axis2(),
                                              entry.table->axis3()};
                        for (size_t i = 0; i < axes.size(); ++i) {
                            if (!axes[i]) {
                                continue;
                            }
                            const auto variable = axes[i]->variable();
                            entry.slew_axis[i] =
                                variable ==
                                    sta::TableAxisVariable::input_transition_time ||
                                variable ==
                                    sta::TableAxisVariable::input_net_transition;
                            if (variable ==
                                sta::TableAxisVariable::total_output_net_capacitance) {
                                entry.values[i] = cap;
                            }
                        }
                    }
                } else {
                    entry.table = nullptr;
                }
            }
            prediction.arcs_.push_back(entry);
        }
    }
    return prediction;
}

CellModel DesignAnalysis::driverModel(const NetState& state) const {
    auto* inst = state.driver->getInst();
    auto* cell = network_->libertyCell(inst);
    auto* port = network_->libertyPort(network_->dbToSta(state.driver));
    return {inst->getMaster(), cell, nullptr, port, area(inst->getMaster()), kInf};
}

std::pair<float, float> DesignAnalysis::predictedTiming(const CellModel& buffer,
                                                        float cap,
                                                        float in_slew) const {
    if (!buffer.cell || !buffer.output) {
        return {kInf, kInf};
    }
    float worst_delay = 0, worst_slew = 0;
    bool found = false, invalid_delay = false, invalid_slew = false;
    const auto* pvt = sta_->cmdMode()->sdc()->operatingConditions(sta::MinMax::max());
    const PredictionKey key{buffer.cell,
                            buffer.input,
                            buffer.output,
                            sta_->cmdScene(),
                            pvt,
                            std::bit_cast<uint32_t>(cap),
                            std::bit_cast<uint32_t>(in_slew)};
    if (prediction_cache_active_) {
        const auto found = predictions_.find(key);
        if (found != predictions_.end()) {
            found->second.epoch = prediction_epoch_;
            return found->second.value;
        }
    }
    for (auto* set : buffer.cell->timingArcSets()) {
        if ((buffer.input && set->from() != buffer.input) || !set->from() ||
            set->to() != buffer.output) {
            continue;
        }
        for (auto* arc : set->arcs()) {
            auto* model = arc->gateModel(sta_->cmdScene(), sta::MinMax::max());
            if (!model) {
                continue;
            }
            float delay, slew;
            model->gateDelay(pvt, in_slew, cap, delay, slew);
            invalid_delay |= !std::isfinite(delay);
            invalid_slew |= !std::isfinite(slew);
            worst_delay = std::max(worst_delay, delay);
            worst_slew = std::max(worst_slew, slew);
            found = true;
        }
    }
    const std::pair<float, float> value{found && !invalid_delay ? worst_delay : kInf,
                                        found && !invalid_slew ? worst_slew : kInf};
    if (prediction_cache_active_) {
        predictions_.emplace(key, Prediction{value, prediction_epoch_});
    }
    return value;
}

const CellModel* DesignAnalysis::select(float cap, float slew_limit,
                                        float in_slew) const {
    for (const auto& b : buffers_) {
        if (cap > b.max_cap * (1 - opts_.margin)) {
            continue;
        }
        float candidate_limit = slew_limit;
        float buffer_limit;
        bool exists = false;
        b.output->slewLimit(sta::MinMax::max(), buffer_limit, exists);
        if (!exists) {
            b.cell->libertyLibrary()->defaultMaxSlew(buffer_limit, exists);
        }
        if (exists) {
            candidate_limit = std::min(candidate_limit, buffer_limit);
        }
        const float predicted = predictedSlew(b, cap, in_slew);
        if (std::isfinite(predicted) &&
            predicted <= candidate_limit * (1 - opts_.margin)) {
            return &b;
        }
    }
    return nullptr;
}

bool DesignAnalysis::isVtSwap(odb::dbMaster* from, odb::dbMaster* to) const {
    if (opts_.vt_suffixes.empty() || from == to || from->getWidth() != to->getWidth() ||
        from->getHeight() != to->getHeight()) {
        return false;
    }
    const auto stem = [&](const std::string& name) {
        for (const auto& suffix : opts_.vt_suffixes) {
            if (name.size() > suffix.size() &&
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
                return name.substr(0, name.size() - suffix.size());
            }
        }
        return std::string{};
    };
    const auto a = stem(from->getName()), b = stem(to->getName());
    if (a.empty() || a != b) {
        return false;
    }
    const auto* ca = network_->libertyCell(network_->dbToSta(from));
    const auto* cb = network_->libertyCell(network_->dbToSta(to));
    if (!ca || !cb) {
        return false;
    }
    const auto* la = ca->libertyLibrary();
    const auto* lb = cb->libertyLibrary();
    const auto* oa = la->defaultOperatingConditions();
    const auto* ob = lb->defaultOperatingConditions();
    return la->nominalVoltage() > 0 && la->nominalVoltage() == lb->nominalVoltage() &&
           la->nominalProcess() == lb->nominalProcess() &&
           la->nominalTemperature() == lb->nominalTemperature() &&
           ((!oa && !ob) ||
            (oa && ob && oa->process() == ob->process() &&
             oa->voltage() == ob->voltage() && oa->temperature() == ob->temperature()));
}

bool DesignAnalysis::protectedInputs(odb::dbInst* inst) const {
    for (auto* pin : inst->getITerms()) {
        if (pin->getSigType() == odb::dbSigType::POWER ||
            pin->getSigType() == odb::dbSigType::GROUND) {
            continue;
        }
        auto* net = pin->getNet();
        if (!net || pin->getIoType() == odb::dbIoType::INOUT || net->isDoNotTouch() ||
            net->isSpecial() ||
            sta_->isClock(network_->dbToSta(net), sta_->cmdMode())) {
            return true;
        }
    }
    return false;
}

std::optional<NetState> DesignAnalysis::analyzeNet(odb::dbNet* net,
                                                   DesignMetrics& metrics) {
    if (net->isSpecial() || net->isDoNotTouch() ||
        net->getSigType() != odb::dbSigType::SIGNAL ||
        sta_->isClock(network_->dbToSta(net), sta_->cmdMode())) {
        ++metrics.skipped_nets;
        return std::nullopt;
    }
    NetState state;
    state.net = net;
    unsigned drivers = 0;
    bool protected_net = false;
    for (auto* pin : net->getITerms()) {
        auto* inst = pin->getInst();
        if (inst->isDoNotTouch() || excluded_.contains(inst)) {
            protected_net = true;
        }
        if (pin->getIoType() == odb::dbIoType::INOUT) {
            protected_net = true;
        }
        if (pin->getIoType() == odb::dbIoType::OUTPUT) {
            ++drivers;
            state.driver = pin;
        } else if (pin->getIoType() == odb::dbIoType::INPUT) {
            int x, y;
            inst->getLocation(x, y);
            auto* port = network_->libertyPort(network_->dbToSta(pin));
            state.loads.push_back(
                {pin, port ? port->capacitance(sta::MinMax::max()) : 0.0f, x, y});
        }
    }
    for (auto* pin : net->getBTerms()) {
        if (pin->getIoType() == odb::dbIoType::INPUT) {
            ++drivers;
        } else if (pin->getIoType() == odb::dbIoType::INOUT) {
            protected_net = true;
        } else if (pin->getIoType() == odb::dbIoType::OUTPUT) {
            auto box = pin->getBBox();
            state.loads.push_back({pin, 0, box.xCenter(), box.yCenter()});
        }
    }
    // Input-port drive modelling is left to a later stage. Never split
    // multi-driver/inout nets or move a protected load across nets.
    if (protected_net || drivers != 1 || !state.driver || state.loads.empty()) {
        ++metrics.skipped_nets;
        return std::nullopt;
    }
    auto* driver_pin = network_->dbToSta(state.driver);
    const sta::RiseFall* rf = nullptr;
    const sta::Scene* scene = nullptr;
    float cap, limit, slack;
    sta_->checkCapacitance(driver_pin, sta_->scenes(), sta::MinMax::max(), cap, limit,
                           slack, rf, scene);
    state.capacitance = sta_->graphDelayCalc()->loadCap(driver_pin, sta_->cmdScene(),
                                                        sta::MinMax::max());
    if (scene && limit > 0) {
        state.cap_ratio = cap / limit;
    }
    auto check_slew = [&](sta::Pin* pin) {
        sta::Slew slew;
        const sta::Scene* slew_scene = nullptr;
        float slew_limit, slew_slack;
        sta_->checkSlew(pin, sta_->scenes(), sta::MinMax::max(), false, slew,
                        slew_limit, slew_slack, rf, slew_scene);
        if (slew_scene && slew_limit > 0 && std::isfinite(slew_limit)) {
            state.slew_ratio = std::max(state.slew_ratio, slew / slew_limit);
            state.slew_limit = std::min(state.slew_limit, slew_limit);
            state.slew = std::max(state.slew, float(slew));
        }
    };
    check_slew(driver_pin);
    for (auto& load : state.loads) {
        check_slew(network_->dbToSta(load.term));
    }
    float fanout, fanout_limit, fanout_slack;
    sta_->checkFanout(driver_pin, sta_->cmdMode(), sta::MinMax::max(), fanout,
                      fanout_limit, fanout_slack);
    state.fanout_ratio = float(state.loads.size()) / opts_.max_fanout;
    if (fanout_limit > 0) {
        state.fanout_ratio = std::max(state.fanout_ratio, fanout / fanout_limit);
    }
    metrics.fanout_nets += state.fanout_ratio > 1;
    metrics.capacitance_nets += state.cap_ratio > 1;
    metrics.slew_nets += state.slew_ratio > 1;
    metrics.severity += std::max(0.0f, state.fanout_ratio - 1) +
                        std::max(0.0f, state.cap_ratio - 1) +
                        std::max(0.0f, state.slew_ratio - 1);
    {
        for (auto* input : state.driver->getInst()->getITerms()) {
            if (input->getIoType() != odb::dbIoType::INPUT) {
                continue;
            }
            auto* vertex = sta_->graph()->pinLoadVertex(network_->dbToSta(input));
            if (vertex) {
                state.input_slew =
                    std::max(state.input_slew,
                             float(sta_->slew(vertex, sta::RiseFallBoth::riseFall(),
                                              sta_->scenes(), sta::MinMax::max())));
            }
            sta::Slew input_slew;
            const sta::Scene* input_scene = nullptr;
            float input_limit, input_slack;
            sta_->checkSlew(network_->dbToSta(input), sta_->scenes(),
                            sta::MinMax::max(), false, input_slew, input_limit,
                            input_slack, rf, input_scene);
            if (input_scene && input_limit > 0 && input_slew > input_limit) {
                state.propagated_slew = true;
            }
        }
    }
    return state;
}

DesignMetrics DesignAnalysis::pinCounts(const sta::Instance* inst) {
    DesignMetrics result;
    std::unique_ptr<sta::InstancePinIterator> pins(network_->pinIterator(inst));
    while (pins->hasNext()) {
        const auto* pin = pins->next();
        const sta::RiseFall* rf = nullptr;
        const sta::Scene* scene = nullptr;
        sta::Slew slew;
        float value, limit, slack;
        // Global slew counts include clocks, unlike candidate NetState.
        sta_->checkSlew(pin, sta_->scenes(), sta::MinMax::max(), true, slew, limit,
                        slack, rf, scene);
        result.total_slew_violations += scene && slack < 0;
        if (!network_->isDriver(pin)) {
            continue;
        }
        // Check each scene before selecting a violation, matching STA's
        // fuzzy comparison even if another scene is just above zero.
        bool cap_violation = false;
        for (const auto* check_scene : sta_->scenes()) {
            sta_->checkCapacitance(pin, {const_cast<sta::Scene*>(check_scene)},
                                   sta::MinMax::max(), value, limit, slack, rf, scene);
            cap_violation |= scene && sta::fuzzyLess(slack, 0.0f);
        }
        result.total_cap_violations += cap_violation;
        for (const auto* mode : sta_->modes()) {
            sta_->checkFanout(pin, mode, sta::MinMax::max(), value, limit, slack);
            result.total_fanout_violations += slack < 0;
        }
    }
    return result;
}

} // namespace ecc::detail

namespace ecc {
std::vector<sta::Vertex*>
repairTimingCone(sta::dbSta* sta, const std::unordered_set<odb::dbNet*>& nets) {
    auto* graph = sta->ensureGraph();
    auto* network = sta->getDbNetwork();
    std::unordered_set<sta::Vertex*> visited;
    std::vector<sta::Vertex*> queue;
    const auto add = [&](sta::Vertex* vertex) {
        if (vertex && visited.insert(vertex).second) {
            queue.push_back(vertex);
        }
    };
    const auto pin_vertices = [&](const sta::Pin* pin) {
        sta::Vertex *load = nullptr, *driver = nullptr;
        graph->pinVertices(pin, load, driver);
        add(load);
        add(driver);
    };
    for (auto* net : nets) {
        for (auto* pin : net->getITerms()) {
            pin_vertices(network->dbToSta(pin));
        }
        for (auto* pin : net->getBTerms()) {
            pin_vertices(network->dbToSta(pin));
        }
    }
    // add() appends to this work queue. Re-read size() on each iteration and
    // avoid iterators/references that vector growth would invalidate.
    // NOLINTNEXTLINE(modernize-loop-convert)
    for (size_t i = 0; i < queue.size(); ++i) {
        sta::VertexOutEdgeIterator edges(queue[i], graph);
        while (edges.hasNext()) {
            auto* edge = edges.next();
            if (!edge->role()->isTimingCheck()) {
                add(edge->to(graph));
            }
        }
    }
    return queue;
}
} // namespace ecc
