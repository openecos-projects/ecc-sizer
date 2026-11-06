// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair_design.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>

#include "buffer_inserter.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "sta/GraphDelayCalc.hh"
#include "sta/Graph.hh"
#include "sta/EquivCells.hh"
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Sdc.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingModel.hh"

namespace ecc::detail {
constexpr float kInf = std::numeric_limits< float >::infinity();

// Design repair follows OpenROAD's unbounded insertion behavior by default.
// A positive limit remains available as an explicit experiment/runtime guard.
inline unsigned remainingBufferBudget(unsigned max_inserted_buffers,
                                      unsigned inserted_buffers) {
    if(max_inserted_buffers == 0)
        return std::numeric_limits< unsigned >::max();
    if(inserted_buffers >= max_inserted_buffers)
        return 0;
    return max_inserted_buffers - inserted_buffers;
}

struct Load {
    odb::dbObject* term;
    float capacitance;
    int x;
    int y;
};
struct NetState {
    odb::dbNet* net;
    odb::dbITerm* driver = nullptr;
    std::vector< Load > loads;
    float capacitance = 0;
    float cap_ratio = 0;
    float slew_ratio = 0;
    float fanout_ratio = 0;
    float slew_limit = kInf;
    float slew = 0;
    float input_slew = 0;
    bool propagated_slew = false;
    bool violating() const {
        return cap_ratio > 1 || slew_ratio > 1 || fanout_ratio > 1;
    }
};
struct Buffer {
    odb::dbMaster* master;
    sta::LibertyCell* cell;
    sta::LibertyPort* input;
    sta::LibertyPort* output;
    double area;
    float max_cap;
};

inline double area(odb::dbMaster* master) {
    return double(master->getWidth()) * master->getHeight();
}

class Repair {
   public:
    Repair(odb::dbBlock* block, sta::dbSta* sta,
           const RepairCommonOptions& options)
        : block_(block),
          sta_(sta),
          network_(sta->getDbNetwork()),
          opts_(options),
          excluded_(options.excluded_instances.begin(),
                    options.excluded_instances.end()) {
        std::unordered_set< odb::dbMaster* > excluded_masters(
            opts_.excluded_masters.begin(), opts_.excluded_masters.end());
        for(auto* lib : block->getDataBase()->getLibs()) {
            for(auto* master : lib->getMasters()) {
                if(!master->isFrozen() || !master->isCore() ||
                   excluded_masters.count(master))
                    continue;
                auto* cell = network_->libertyCell(network_->dbToSta(master));
                if(cell == nullptr || cell->dontUse() ||
                   cell->hasSequentials() || cell->isClockGate() ||
                   cell->isIsolationCell() || cell->isLevelShifter())
                    continue;
                sta::LibertyPort *input = nullptr, *output = nullptr;
                unsigned inputs = 0, outputs = 0;
                bool invalid = false;
                for(auto* term : master->getMTerms()) {
                    if(term->getSigType() == odb::dbSigType::POWER ||
                       term->getSigType() == odb::dbSigType::GROUND)
                        continue;
                    auto* port = cell->findLibertyPort(term->getConstName());
                    if(!port) {
                        invalid = true;
                        break;
                    }
                    if(term->getIoType() == odb::dbIoType::INPUT) {
                        input = port;
                        ++inputs;
                    }
                    else if(term->getIoType() == odb::dbIoType::OUTPUT) {
                        output = port;
                        ++outputs;
                    }
                    else
                        invalid = true;
                }
                if(invalid || inputs == 0 || outputs != 1 ||
                   !output->function() || output->tristateEnable())
                    continue;
                float cap = 0;
                bool exists = false;
                output->capacitanceLimit(sta::MinMax::max(), cap, exists);
                if(!exists)
                    cell->libertyLibrary()->defaultMaxCapacitance(cap, exists);
                if(!exists)
                    cap = kInf;
                cells_.push_back(
                    {master, cell, nullptr, output, area(master), cap});
                if(cell->isBuffer() && inputs == 1)
                    buffers_.push_back(
                        {master, cell, input, output, area(master), cap});
            }
        }
        std::sort(buffers_.begin(), buffers_.end(),
                  [](const Buffer& a, const Buffer& b) {
                      if(a.area != b.area)
                          return a.area < b.area;
                      return a.master->getName() < b.master->getName();
                  });
    }

    bool hasCells() const {
        return !cells_.empty();
    }

    std::vector< NetState > analyze(DesignRepairMetrics& metrics,
                                    bool all_nets = false) {
        sta_->findRequireds();
        sta_->checkSlewsPreamble();
        sta_->checkCapacitancesPreamble(sta_->scenes());
        sta_->checkFanoutPreamble();
        metrics = {};
        metrics.setup_wns =
            std::min(0.0f, float(sta_->worstSlack(sta::MinMax::max())));
        metrics.hold_wns =
            std::min(0.0f, float(sta_->worstSlack(sta::MinMax::min())));
        metrics.setup_tns = float(sta_->totalNegativeSlack(sta::MinMax::max()));
        metrics.hold_tns = float(sta_->totalNegativeSlack(sta::MinMax::min()));
        metrics.total_slew_violations = sta_->maxSlewViolationCount();
        metrics.total_cap_violations = sta_->maxCapacitanceViolationCount();
        metrics.total_fanout_violations =
            sta_->fanoutViolationCount(sta::MinMax::max(), sta_->modes());
        for(auto* inst : block_->getInsts())
            metrics.area += area(inst->getMaster());
        std::vector< NetState > states;
        for(auto* net : block_->getNets()) {
            if(net->isSpecial() || net->isDoNotTouch() ||
               net->getSigType() != odb::dbSigType::SIGNAL ||
               sta_->isClock(network_->dbToSta(net), sta_->cmdMode())) {
                ++metrics.skipped_nets;
                continue;
            }
            NetState state;
            state.net = net;
            unsigned drivers = 0;
            bool protected_net = false;
            for(auto* pin : net->getITerms()) {
                auto* inst = pin->getInst();
                if(inst->isDoNotTouch() || excluded_.count(inst))
                    protected_net = true;
                if(pin->getIoType() == odb::dbIoType::INOUT)
                    protected_net = true;
                if(pin->getIoType() == odb::dbIoType::OUTPUT) {
                    ++drivers;
                    state.driver = pin;
                }
                else if(pin->getIoType() == odb::dbIoType::INPUT) {
                    int x, y;
                    inst->getLocation(x, y);
                    auto* port = network_->libertyPort(network_->dbToSta(pin));
                    state.loads.push_back(
                        {pin,
                         port ? port->capacitance(sta::MinMax::max()) : 0.0f, x,
                         y});
                }
            }
            for(auto* pin : net->getBTerms()) {
                if(pin->getIoType() == odb::dbIoType::INPUT)
                    ++drivers;
                else if(pin->getIoType() == odb::dbIoType::INOUT)
                    protected_net = true;
                else if(pin->getIoType() == odb::dbIoType::OUTPUT) {
                    auto box = pin->getBBox();
                    state.loads.push_back(
                        {pin, 0, box.xCenter(), box.yCenter()});
                }
            }
            // Input-port drive modelling is left to a later stage. Never split
            // multi-driver/inout nets or move a protected load across nets.
            if(protected_net || drivers != 1 || !state.driver ||
               state.loads.empty()) {
                ++metrics.skipped_nets;
                continue;
            }
            auto* driver_pin = network_->dbToSta(state.driver);
            const sta::RiseFall* rf = nullptr;
            const sta::Scene* scene = nullptr;
            float cap, limit, slack;
            sta_->checkCapacitance(driver_pin, sta_->scenes(),
                                   sta::MinMax::max(), cap, limit, slack, rf,
                                   scene);
            state.capacitance = sta_->graphDelayCalc()->loadCap(
                driver_pin, sta_->cmdScene(), sta::MinMax::max());
            if(scene && limit > 0)
                state.cap_ratio = cap / limit;
            auto check_slew = [&](sta::Pin* pin) {
                sta::Slew slew;
                const sta::Scene* slew_scene = nullptr;
                float slew_limit, slew_slack;
                sta_->checkSlew(pin, sta_->scenes(), sta::MinMax::max(), false,
                                slew, slew_limit, slew_slack, rf, slew_scene);
                if(slew_scene && slew_limit > 0 && std::isfinite(slew_limit)) {
                    state.slew_ratio =
                        std::max(state.slew_ratio, slew / slew_limit);
                    state.slew_limit = std::min(state.slew_limit, slew_limit);
                    state.slew = std::max(state.slew, float(slew));
                }
            };
            check_slew(driver_pin);
            for(auto& load : state.loads)
                check_slew(network_->dbToSta(load.term));
            float fanout, fanout_limit, fanout_slack;
            sta_->checkFanout(driver_pin, sta_->cmdMode(), sta::MinMax::max(),
                              fanout, fanout_limit, fanout_slack);
            state.fanout_ratio = float(state.loads.size()) / opts_.max_fanout;
            if(fanout_limit > 0)
                state.fanout_ratio =
                    std::max(state.fanout_ratio, fanout / fanout_limit);
            metrics.fanout_nets += state.fanout_ratio > 1;
            metrics.capacitance_nets += state.cap_ratio > 1;
            metrics.slew_nets += state.slew_ratio > 1;
            metrics.severity += std::max(0.0f, state.fanout_ratio - 1) +
                                std::max(0.0f, state.cap_ratio - 1) +
                                std::max(0.0f, state.slew_ratio - 1);
            {
                for(auto* input : state.driver->getInst()->getITerms()) {
                    if(input->getIoType() != odb::dbIoType::INPUT)
                        continue;
                    auto* vertex =
                        sta_->graph()->pinLoadVertex(network_->dbToSta(input));
                    if(vertex)
                        state.input_slew =
                            std::max(state.input_slew,
                                     float(sta_->slew(
                                         vertex, sta::RiseFallBoth::riseFall(),
                                         sta_->scenes(), sta::MinMax::max())));
                    sta::Slew input_slew;
                    const sta::Scene* input_scene = nullptr;
                    float input_limit, input_slack;
                    sta_->checkSlew(network_->dbToSta(input), sta_->scenes(),
                                    sta::MinMax::max(), false, input_slew,
                                    input_limit, input_slack, rf, input_scene);
                    if(input_scene && input_limit > 0 &&
                       input_slew > input_limit)
                        state.propagated_slew = true;
                }
            }
            if(all_nets || state.violating())
                states.push_back(std::move(state));
        }
        return states;
    }

    float predictedSlew(const Buffer& buffer, float cap, float in_slew) const {
        return predict(buffer, cap, in_slew, true);
    }
    float predictedDelay(const Buffer& buffer, float cap, float in_slew) const {
        return predict(buffer, cap, in_slew, false);
    }
    Buffer driverModel(const NetState& state) const {
        auto* inst = state.driver->getInst();
        auto* cell = network_->libertyCell(inst);
        auto* port = network_->libertyPort(network_->dbToSta(state.driver));
        return {inst->getMaster(),       cell, nullptr, port,
                area(inst->getMaster()), kInf};
    }
    float predict(const Buffer& buffer, float cap, float in_slew,
                  bool want_slew) const {
        if(!buffer.cell || !buffer.output)
            return kInf;
        float worst = 0;
        bool found = false;
        const auto* pvt =
            sta_->cmdMode()->sdc()->operatingConditions(sta::MinMax::max());
        for(auto* set : buffer.cell->timingArcSets()) {
            if((buffer.input && set->from() != buffer.input) || !set->from() ||
               set->to() != buffer.output)
                continue;
            for(auto* arc : set->arcs()) {
                auto* model =
                    arc->gateModel(sta_->cmdScene(), sta::MinMax::max());
                if(!model)
                    continue;
                float delay, slew;
                model->gateDelay(pvt, in_slew, cap, delay, slew);
                const float value = want_slew ? slew : delay;
                if(!std::isfinite(value))
                    return kInf;
                worst = std::max(worst, value);
                found = true;
            }
        }
        return found ? worst : kInf;
    }

    const Buffer* select(float cap, float slew_limit, float in_slew) const {
        for(const auto& b : buffers_) {
            if(cap > b.max_cap * (1 - opts_.margin))
                continue;
            float candidate_limit = slew_limit;
            float buffer_limit;
            bool exists = false;
            b.output->slewLimit(sta::MinMax::max(), buffer_limit, exists);
            if(!exists)
                b.cell->libertyLibrary()->defaultMaxSlew(buffer_limit, exists);
            if(exists)
                candidate_limit = std::min(candidate_limit, buffer_limit);
            const float predicted = predictedSlew(b, cap, in_slew);
            if(std::isfinite(predicted) &&
               predicted <= candidate_limit * (1 - opts_.margin))
                return &b;
        }
        return nullptr;
    }

    // VT identity is explicit and independent of library names or VT ordering.
    // Require the same drive stem, footprint and characterized PVT.
    bool isVtSwap(odb::dbMaster* from, odb::dbMaster* to) const {
        if(opts_.vt_suffixes.empty() || from == to ||
           from->getWidth() != to->getWidth() ||
           from->getHeight() != to->getHeight())
            return false;
        const auto stem = [&](const std::string& name) {
            for(const auto& suffix : opts_.vt_suffixes)
                if(name.size() > suffix.size() &&
                   name.compare(name.size() - suffix.size(), suffix.size(),
                                suffix) == 0)
                    return name.substr(0, name.size() - suffix.size());
            return std::string{};
        };
        const auto a = stem(from->getName()), b = stem(to->getName());
        if(a.empty() || a != b)
            return false;
        const auto* ca = network_->libertyCell(network_->dbToSta(from));
        const auto* cb = network_->libertyCell(network_->dbToSta(to));
        if(!ca || !cb)
            return false;
        const auto* la = ca->libertyLibrary();
        const auto* lb = cb->libertyLibrary();
        const auto* oa = la->defaultOperatingConditions();
        const auto* ob = lb->defaultOperatingConditions();
        return la->nominalVoltage() > 0 &&
               la->nominalVoltage() == lb->nominalVoltage() &&
               la->nominalProcess() == lb->nominalProcess() &&
               la->nominalTemperature() == lb->nominalTemperature() &&
               ((!oa && !ob) || (oa && ob && oa->process() == ob->process() &&
                                 oa->voltage() == ob->voltage() &&
                                 oa->temperature() == ob->temperature()));
    }

    bool protectedInputs(odb::dbInst* inst) const {
        for(auto* pin : inst->getITerms()) {
            if(pin->getSigType() == odb::dbSigType::POWER ||
               pin->getSigType() == odb::dbSigType::GROUND)
                continue;
            auto* net = pin->getNet();
            if(!net || pin->getIoType() == odb::dbIoType::INOUT ||
               net->isDoNotTouch() || net->isSpecial() ||
               sta_->isClock(network_->dbToSta(net), sta_->cmdMode()))
                return true;
        }
        return false;
    }

    std::vector< const Buffer* > resizeCandidates(const NetState& state,
                                                  bool downsize = false,
                                                  bool vt_only = false) {
        std::vector< const Buffer* > result;
        auto* driver = state.driver->getInst();
        auto* original = network_->libertyCell(driver);
        if(!original || driver->isFixed() || driver->isDoNotTouch() ||
           excluded_.count(driver) || state.fanout_ratio > 1 ||
           (vt_only && protectedInputs(driver)) ||
           (!original->isBuffer() && !opts_.allow_gate_resize))
            return result;
        auto [it, fresh] = resize_candidates_.try_emplace(original);
        if(fresh) {
            for(const auto& candidate : cells_) {
                if((candidate.cell->libertyLibrary() !=
                        original->libertyLibrary() &&
                    !isVtSwap(driver->getMaster(), candidate.master)) ||
                   !sta::equivCellsArcs(original, candidate.cell))
                    continue;
                it->second.push_back(candidate);
            }
            std::sort(it->second.begin(), it->second.end(),
                      [](const Buffer& a, const Buffer& b) {
                          return a.area != b.area ? a.area < b.area
                                                  : a.master->getName() <
                                                        b.master->getName();
                      });
        }
        for(const auto& candidate : it->second) {
            auto* master = driver->getMaster();
            if((vt_only ? !isVtSwap(master, candidate.master)
                        : candidate.cell->libertyLibrary() !=
                                  original->libertyLibrary() ||
                              (downsize ? candidate.area >= area(master)
                                        : candidate.area <= area(master))) ||
               candidate.master->getHeight() != master->getHeight() ||
               candidate.master->getSite() != master->getSite() ||
               candidate.master->getMTerms().size() !=
                   master->getMTerms().size() ||
               state.capacitance > candidate.max_cap * (1 - opts_.margin))
                continue;
            bool compatible = true;
            for(auto* term : master->getMTerms()) {
                auto* other = candidate.master->findMTerm(term->getConstName());
                if(!other || other->getIoType() != term->getIoType() ||
                   other->getSigType() != term->getSigType())
                    compatible = false;
            }
            if(!compatible)
                continue;
            float limit = state.slew_limit, local_limit;
            bool exists = false;
            candidate.output->slewLimit(sta::MinMax::max(), local_limit,
                                        exists);
            if(!exists)
                candidate.cell->libertyLibrary()->defaultMaxSlew(local_limit,
                                                                 exists);
            if(exists)
                limit = std::min(limit, local_limit);
            const float slew =
                predictedSlew(candidate, state.capacitance, state.input_slew);
            if(std::isfinite(slew) && slew <= limit * (1 - opts_.margin))
                result.push_back(&candidate);
        }
        return result;
    }

    const Buffer* selectResize(const NetState& state) {
        // Keep electrically feasible drive-strength repair first. A same-size
        // VT change is a fallback for DRV; setup compares its delay directly,
        // and power recovery can reclaim drive/VT after timing is established.
        const auto sized = resizeCandidates(state);
        if(!sized.empty())
            return sized.front();
        const auto vt = resizeCandidates(state, false, true);
        if(vt.empty())
            return nullptr;
        return *std::min_element(
            vt.begin(), vt.end(), [&](const auto* a, const auto* b) {
                const float sa =
                    predictedSlew(*a, state.capacitance, state.input_slew);
                const float sb =
                    predictedSlew(*b, state.capacitance, state.input_slew);
                return sa != sb ? sa < sb
                                : a->master->getName() < b->master->getName();
            });
    }

    bool edit(NetState& state, unsigned& inserted, unsigned& resized_buffers,
              unsigned& resized_gates, unsigned remaining, bool allow_resize,
              unsigned& vt_swaps) {
        // Buffer trees use recovered input slew for sizing. Existing gate
        // replacements use the measured input slew on every incoming arc.
        float in_slew = std::isfinite(state.slew_limit)
                            ? state.slew_limit * opts_.recovered_slew_fraction
                            : state.slew;
        const Buffer* replacement =
            allow_resize ? selectResize(state) : nullptr;
        // Spatial ordering groups nearby loads; estimated wire/external-port
        // capacitance is apportioned conservatively from the current net total.
        std::stable_sort(state.loads.begin(), state.loads.end(),
                         [](const Load& a, const Load& b) {
                             return a.x != b.x ? a.x < b.x : a.y < b.y;
                         });
        float pin_sum = 0;
        for(const auto& load : state.loads)
            pin_sum += load.capacitance;
        float extra =
            std::max(0.0f, state.capacitance - pin_sum) / state.loads.size();
        size_t max_group = opts_.max_fanout;
        if(state.fanout_ratio > 1) {
            max_group = std::min(
                max_group, std::max(size_t(1), size_t(state.loads.size() /
                                                      state.fanout_ratio)));
        }
        // Limiting an overloaded driver's group size provides relief even if a
        // large buffer could carry the entire net. Subsequent passes fix the
        // root.
        if(state.cap_ratio > 1 || state.slew_ratio > 1)
            max_group = std::min(
                max_group,
                std::max(size_t(1),
                         size_t(std::ceil(state.loads.size() *
                                          opts_.overload_group_fraction))));
        struct Group {
            size_t begin, end;
            const Buffer* buffer;
        };
        std::vector< Group > groups;
        double buffer_area = 0;
        for(size_t begin = 0; begin < state.loads.size();) {
            size_t end = begin;
            float group_cap = 0;
            const Buffer* selected = nullptr;
            while(end < state.loads.size() && end - begin < max_group) {
                float next_cap =
                    group_cap + state.loads[end].capacitance + extra;
                const Buffer* next =
                    select(next_cap, state.slew_limit, in_slew);
                if(!next)
                    break;
                group_cap = next_cap;
                selected = next;
                ++end;
            }
            if(end == begin) {
                ++begin;
                continue;
            }

            // A singleton that is already a low-capacitance buffer input cannot
            // benefit from repeatedly adding another identical buffer.
            if(end - begin == 1 &&
               state.loads[begin].term->getObjectType() == odb::dbITermObj) {
                auto* load_pin =
                    static_cast< odb::dbITerm* >(state.loads[begin].term);
                if(network_->libertyCell(load_pin->getInst()) &&
                   network_->libertyCell(load_pin->getInst())->isBuffer() &&
                   selected->input->capacitance() >=
                       state.loads[begin].capacitance) {
                    begin = end;
                    continue;
                }
            }
            groups.push_back({begin, end, selected});
            buffer_area += selected->area;
            begin = end;
        }
        // Compare feasible edit area before the shared STA acceptance step.
        // Count budgeting never forbids a gate replacement that adds no cell.
        const unsigned available = remaining - inserted;
        if(replacement &&
           (groups.empty() || groups.size() > available ||
            replacement->area - area(state.driver->getInst()->getMaster()) <=
                buffer_area)) {
            auto* driver = state.driver->getInst();
            const bool buffer = network_->libertyCell(driver)->isBuffer();
            const bool vt = isVtSwap(driver->getMaster(), replacement->master);
            if(driver->swapMaster(replacement->master)) {
                vt_swaps += vt;
                if(buffer)
                    ++resized_buffers;
                else
                    ++resized_gates;
                return true;
            }
        }
        bool changed = false;
        for(const auto& group : groups) {
            if(inserted >= remaining)
                break;
            const auto begin = group.begin, end = group.end;
            const auto* selected = group.buffer;
            std::vector< odb::dbObject* > terms;
            long long x = 0, y = 0;
            for(size_t i = begin; i < end; ++i) {
                terms.push_back(state.loads[i].term);
                x += state.loads[i].x;
                y += state.loads[i].y;
            }
            BufferInsertionOptions options;
            options.location =
                BufferLocation{int(x / terms.size()), int(y / terms.size())};
            options.buffer_name_prefix = "sizer_drv_buf";
            options.net_name_prefix = "sizer_drv_net";
            auto result = OpenDbBufferInserter::insertBufferBeforeLoads(
                state.net, terms, selected->master, options);
            if(!result)
                throw std::runtime_error(result.message);
            ++inserted;
            changed = true;
        }
        return changed;
    }

   private:
    odb::dbBlock* block_;
    sta::dbSta* sta_;
    sta::dbNetwork* network_;
    const RepairCommonOptions& opts_;
    std::unordered_set< odb::dbInst* > excluded_;
    std::vector< Buffer > buffers_;
    std::vector< Buffer > cells_;
    std::unordered_map< sta::LibertyCell*, std::vector< Buffer > >
        resize_candidates_;
};
}  // namespace ecc::detail
