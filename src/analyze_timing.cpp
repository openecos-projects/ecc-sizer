///////////////////////////////////////////////////////////////////////////////
// Authors: Hyein Lee and Jiajia Li
//          (Ph.D. advisor: Andrew B. Kahng)
//
//          Many subsequent improvements were made by Minsoo Kim
//          leading up to the initial release.
//
// BSD 3-Clause License
//
// Copyright (c) 2018, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

// Direct OpenSTA queries and sizing-unit adaptation. Input parsing belongs to
// the OpenROAD session; this module never interprets timing report text.
#include "analyze_timing.h"
#include "cell_sizing.h"
#include "design_edit.h"
#include "utils.h"
#include "db_sta/dbNetwork.hh"
#include "sta/Sta.hh"
#include "sta/TimingRole.hh"
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace ecc {

const sta::Pin* findTimingPin(sta::dbSta* sta, const std::string& name) {
    auto* network = sta->getDbNetwork();
    auto* block = network->block();
    if(auto* term = block->findITerm2(name.c_str()))
        return network->dbToSta(term);
    if(auto* term = block->findBTerm(name.c_str()))
        return network->dbToSta(term);
    throw std::invalid_argument("timing pin not found: " + name);
}

std::array<double, 2> pinTiming(sta::dbSta* sta, const sta::Pin* pin,
                                PinTimingMetric metric, const sta::MinMax* min_max) {
    auto* graph = sta->ensureGraph();
    auto* vertex = graph->pinLoadVertex(pin);
    auto* driver = graph->pinDrvrVertex(pin);
    std::array<double, 2> result;
    for(auto* rf : sta::RiseFall::range()) {
        auto* edge = rf->asRiseFallBoth();
        double value;
        switch(metric) {
        case PinTimingMetric::Slack:
            value = sta->slack(pin, edge, sta->scenes(), min_max);
            break;
        case PinTimingMetric::Arrival:
            value = sta->arrival(pin, edge, min_max);
            break;
        case PinTimingMetric::Slew:
            value = vertex ? double(sta->slew(vertex, edge, sta->scenes(), min_max)) : 0.0;
            if(driver && driver != vertex)
                value = min_max == sta::MinMax::max()
                            ? std::max(value, double(sta->slew(driver, edge, sta->scenes(), min_max)))
                            : std::min(value, double(sta->slew(driver, edge, sta->scenes(), min_max)));
            break;
        }
        if(std::abs(value) >= sta::INF)
            value = std::copysign(std::numeric_limits<double>::infinity(), value);
        result[rf->index()] = value;
    }
    return result;
}

std::array<double, 2> timingArcDelays(sta::dbSta* sta, const sta::Pin* from,
                                      const sta::Pin* to, bool wire) {
    sta->findDelays();
    auto* graph = sta->graph();
    auto* source = wire ? graph->pinDrvrVertex(from) : graph->pinLoadVertex(from);
    auto* target = wire ? graph->pinLoadVertex(to) : graph->pinDrvrVertex(to);
    std::array<double, 2> result = {-std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity()};
    if(source && target) {
        sta::VertexOutEdgeIterator edges(source, graph);
        while(edges.hasNext()) {
            auto* edge = edges.next();
            if(edge->to(graph) != target || edge->isWire() != wire ||
               edge->role()->isTimingCheck())
                continue;
            for(auto* scene : sta->scenes()) {
                const auto ap = scene->dcalcAnalysisPtIndex(sta::MinMax::max());
                for(auto* arc : edge->timingArcSet()->arcs()) {
                    const auto* transition = arc->toEdge()->asRiseFall();
                    if(!transition) continue;
                    const auto index = transition->index();
                    result[index] = std::max(result[index], double(sta->arcDelay(edge, arc, ap)));
                }
            }
        }
    }
    // No graph arc means this pin pair has no modeled propagation delay.
    for(auto& delay : result)
        if(!std::isfinite(delay)) delay = 0.0;
    return result;
}
}  // namespace ecc

designTiming::designTiming(Sizer* sizer) : _sizer(sizer) {}

void designTiming::pinValues(double& rise, double& fall, const std::string& name,
                              ecc::PinTimingMetric metric, const sta::MinMax* min_max) {
    const auto begin = cpuTime();
    auto* sta = _sizer->_ckt->_ord_timing->getSta();
    const auto values = ecc::pinTiming(sta, ecc::findTimingPin(sta, name), metric, min_max);
    rise = values[0] / _sizer->time_unit;
    fall = values[1] / _sizer->time_unit;
    pt_time += cpuTime() - begin;
}

void designTiming::getCellDelay(double& rise, double& fall, string from, string to) {
    const auto begin = cpuTime();
    auto* sta = _sizer->_ckt->_ord_timing->getSta();
    const auto values = ecc::timingArcDelays(sta, ecc::findTimingPin(sta, from),
                                            ecc::findTimingPin(sta, to), false);
    rise = values[0] / _sizer->time_unit;
    fall = values[1] / _sizer->time_unit;
    pt_time += cpuTime() - begin;
}

void designTiming::getNetDelay(double& delay, string from, string to) {
    const auto begin = cpuTime();
    auto* sta = _sizer->_ckt->_ord_timing->getSta();
    const auto values = ecc::timingArcDelays(sta, ecc::findTimingPin(sta, from),
                                            ecc::findTimingPin(sta, to), true);
    delay = std::max(values[0], values[1]) / _sizer->time_unit;
    pt_time += cpuTime() - begin;
}

void designTiming::getTranVio(double &tot, double &max, int &num) {
    double begin = cpuTime();
    auto design = _sizer->_ckt->_ord_design;
    sta::dbNetwork* network =
        _sizer->_ckt->_ord_timing->getSta()->getDbNetwork();
    for(auto inst : design->getBlock()->getInsts()) {
        for(auto pin_ : inst->getITerms()) {
            if(pin_->getNet() && pin_->getNet()->getSigType() != "POWER" &&
               pin_->getNet()->getSigType() != "GROUND" &&
               pin_->getNet()->getSigType() != "CLOCK") {
                auto m_term = pin_->getMTerm();
                sta::Port* port = network->dbToSta(m_term);
                sta::LibertyPort* lib_port =
                    port ? network->libertyPort(port) : nullptr;
                if(lib_port == nullptr) {
                    continue;
                }
                double r_slew = _sizer->_ckt->_ord_timing->getPinSlew(
                    pin_, ord::Timing::Rise);
                double f_slew = _sizer->_ckt->_ord_timing->getPinSlew(
                    pin_, ord::Timing::Fall);
                double now_slew = std::max(r_slew, f_slew);
                double slew_limit =
                    _sizer->_ckt->_ord_timing->getMaxSlewLimit(m_term);
                double slew_diff = std::max(
                    (now_slew - slew_limit) / _sizer->time_unit, 0.0);
                tot += slew_diff;
                if(slew_diff > 0) {
                    ecc::logDebug() << pin_->getName()
                        << " max tran vio: " << now_slew / _sizer->time_unit
                        << " " << slew_limit / _sizer->time_unit << endl;
                    num++;
                }
                max = std::max(max, slew_diff);
            }
        }
    }
    pt_time += cpuTime() - begin;
    return;
}

void designTiming::getCapVio(double &tot, double &max, int &num) {
    double begin = cpuTime();
    auto design = _sizer->_ckt->_ord_design;
    auto corner = _sizer->_ckt->_ord_timing->getCorners()[0];
    sta::dbSta *sta = _sizer->_ckt->_ord_timing->getSta();
    sta::dbNetwork* network = sta->getDbNetwork();
    for(auto inst : design->getBlock()->getInsts()) {
        for(auto pin_ : inst->getITerms()) {
            if(pin_->getNet() && pin_->getNet()->getSigType() != "POWER" &&
               pin_->getNet()->getSigType() != "GROUND" &&
               pin_->getNet()->getSigType() != "CLOCK" &&
               pin_->isOutputSignal()) {
                auto m_term = pin_->getMTerm();
                sta::Port* port = network->dbToSta(m_term);
                sta::LibertyPort* lib_port =
                    port ? network->libertyPort(port) : nullptr;
                if(lib_port == nullptr) {
                    continue;
                }
                float pin_cap;
                float wire_cap;
                sta::Net *sta_net =
                    network->dbToSta(pin_->getNet());
                _sizer->_sta->connectedCap(
                    sta_net, corner, sta::MinMax::max(), pin_cap, wire_cap);
                wire_cap /= _sizer->cap_unit;
                pin_cap /= _sizer->cap_unit;
                double now_cap = wire_cap + pin_cap;
                double cap_limit =
                    _sizer->_ckt->_ord_timing->getMaxCapLimit(m_term) /
                    _sizer->cap_unit;
                double cap_diff = std::max((now_cap - cap_limit), 0.0);
                tot += cap_diff;
                if(cap_diff > 0) {
                    ecc::logDebug() << pin_->getName() << " max cap vio: " << now_cap
                        << " " << wire_cap << " " << cap_limit << endl;
                    num++;
                }
                max = std::max(max, cap_diff);
            }
        }
    }
    pt_time += cpuTime() - begin;
    return;
}

double designTiming::getWorstSlack(string _clkName) {
    const auto begin = cpuTime();
    auto* timing = _sizer->_ckt->_ord_timing->getSta();
    const double value = timing->worstSlack(sta::MinMax::max());
    pt_time += cpuTime() - begin;
    return value >= sta::INF ? 0.0 : value / _sizer->time_unit;
}

double designTiming::getWorstSlackHold(string _clkName) {
    const auto begin = cpuTime();
    auto* timing = _sizer->_ckt->_ord_timing->getSta();
    const double value = timing->worstSlack(sta::MinMax::min());
    pt_time += cpuTime() - begin;
    return value >= sta::INF ? 0.0 : value / _sizer->time_unit;
}

double designTiming::getTNS(string _clkName) {
    const auto begin = cpuTime();
    auto* timing = _sizer->_ckt->_ord_timing->getSta();
    const double value = timing->totalNegativeSlack(sta::MinMax::max());
    pt_time += cpuTime() - begin;
    return value >= sta::INF ? 0.0 : value / _sizer->time_unit;
}

double designTiming::getLeakPower() {
    double begin = cpuTime();
    double totalLeakagePower = 0.0;
    auto corner = _sizer->_ckt->_ord_timing->getCorners()[0];

    if(_sizer->cells == nullptr) {
        for(auto inst : _sizer->_ckt->_ord_design->getBlock()->getInsts()) {
            totalLeakagePower +=
                _sizer->_ckt->_ord_timing->staticPower(inst, corner);
        }
    }
    else {
        for(int i = 0; i < _sizer->numcells; i++) {
            if(_sizer->cells[i].isStaticChanged) {
                auto inst = _sizer->_ckt->_ord_design->getBlock()->findInst(
                    _sizer->cells[i].name.c_str());
                _sizer->cells[i].static_power =
                    _sizer->_ckt->_ord_timing->staticPower(inst, corner);
                _sizer->cells[i].isStaticChanged = false;
            }
            totalLeakagePower += _sizer->cells[i].static_power;
        }
    }
    totalLeakagePower /= _sizer->sw_adj;
    pt_time += cpuTime() - begin;
    return totalLeakagePower;
}

double designTiming::getCellPower(string cell_name) {
    double begin = cpuTime();
    double totalLeakagePower = 0.0;
    auto inst =
        _sizer->_ckt->_ord_design->getBlock()->findInst(cell_name.c_str());
    if(!inst)
        throw std::invalid_argument("timing instance not found: " + cell_name);
    auto corner = _sizer->_ckt->_ord_timing->getCorners()[0];
    totalLeakagePower +=
        _sizer->_ckt->_ord_timing->staticPower(inst, corner);
    totalLeakagePower /= _sizer->sw_adj;
    pt_time += cpuTime() - begin;
    return totalLeakagePower;
}

double designTiming::getTotPower() {
    // The sizing search uses static leakage as its power objective.
    return getLeakPower();
}

double designTiming::getTNSHold(string) {
    const auto begin = cpuTime();
    const auto value = _sizer->_ckt->_ord_timing->getSta()->totalNegativeSlack(sta::MinMax::min());
    pt_time += cpuTime() - begin;
    return value >= sta::INF ? 0.0 : value / _sizer->time_unit;
}

bool designTiming::sizeCell(string instance, string master) {
    auto* block = _sizer->_ckt->_ord_design->getBlock();
    auto* inst = block->findInst(instance.c_str());
    auto* replacement = block->getDb()->findMaster(master.c_str());
    if(!inst || !replacement)
        return false;
    if(inst->getMaster() == replacement)
        return true;
    return _sizer->replaceCell(inst, replacement);
}

double designTiming::getCeff(string name) {
    const auto begin = cpuTime();
    auto* sta = _sizer->_ckt->_ord_timing->getSta();
    auto* net = sta->network()->net(ecc::findTimingPin(sta, name));
    float pin_cap = 0, wire_cap = 0;
    if(net)
        sta->connectedCap(net, sta->scenes().front(), sta::MinMax::max(), pin_cap, wire_cap);
    pt_time += cpuTime() - begin;
    // Preserve the sizing model's total connected-load convention.
    return (pin_cap + wire_cap) / _sizer->cap_unit;
}

string designTiming::getLibCell(string name) {
    auto* inst = _sizer->_ckt->_ord_design->getBlock()->findInst(name.c_str());
    if(!inst)
        throw std::invalid_argument("timing instance not found: " + name);
    return inst->getMaster()->getName();
}

void designTiming::getPinSlack(double& rise, double& fall, string name) {
    pinValues(rise, fall, name, ecc::PinTimingMetric::Slack, sta::MinMax::max());
}

void designTiming::getPinMinSlack(double& rise, double& fall, string name) {
    pinValues(rise, fall, name, ecc::PinTimingMetric::Slack, sta::MinMax::min());
}

void designTiming::getPinTran(double& rise, double& fall, string name) {
    pinValues(rise, fall, name, ecc::PinTimingMetric::Slew, sta::MinMax::max());
}

void designTiming::getPinArrival(double& rise, double& fall, string name) {
    pinValues(rise, fall, name, ecc::PinTimingMetric::Arrival, sta::MinMax::max());
}

double designTiming::getRiseSlack(string name) {
    double rise, fall;
    getPinSlack(rise, fall, name);
    return rise;
}

double designTiming::getFallSlack(string name) {
    double rise, fall;
    getPinSlack(rise, fall, name);
    return fall;
}

double designTiming::getRiseTran(string name) {
    double rise, fall;
    getPinTran(rise, fall, name);
    return rise;
}

double designTiming::getFallTran(string name) {
    double rise, fall;
    getPinTran(rise, fall, name);
    return fall;
}

double designTiming::getRiseArrival(string name) {
    double rise, fall;
    getPinArrival(rise, fall, name);
    return rise;
}

double designTiming::getFallArrival(string name) {
    double rise, fall;
    getPinArrival(rise, fall, name);
    return fall;
}

bool designTiming::writePins(const string& input, const string& output, PinReport report) {
    std::ifstream in(input);
    if(!in) throw std::runtime_error("cannot read pin list: " + input);
    std::ofstream out(output);
    if(!out) throw std::runtime_error("cannot write pin report: " + output);
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    auto write = [&](double value) {
        if(std::isinf(value)) out << (value < 0 ? " -INFINITY" : " INFINITY");
        else out << ' ' << value;
    };
    for(string name; std::getline(in, name);) {
        if(!name.empty() && name.back() == '\r') name.pop_back();
        if(name.empty()) continue;
        out << name;
        double rise, fall;
        if(report != PinReport::Slew) {
            getPinSlack(rise, fall, name);
            write(rise); write(fall);
        }
        if(report != PinReport::Slack) {
            getPinTran(rise, fall, name);
            write(rise); write(fall);
        }
        if(report == PinReport::All) {
            getPinArrival(rise, fall, name);
            write(rise); write(fall);
        }
        out << '\n';
    }
    return bool(out);
}

bool designTiming::writePinSlack(string input, string output) {
    return writePins(input, output, PinReport::Slack);
}
bool designTiming::writePinTran(string input, string output) {
    return writePins(input, output, PinReport::Slew);
}
bool designTiming::writePinAll(string input, string output) {
    return writePins(input, output, PinReport::All);
}
