// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_load.h"

namespace ecc::detail {
LoadDownsizePlanner::LoadDownsizePlanner(Repair& backend, sta::dbSta* sta,
                                         const std::vector< NetState >& states,
                                         const RepairCommonOptions& options)
    : backend_(backend), sta_(sta), options_(options) {
    if(options.allow_load_downsize)
        for(const auto& state : states)
            outputs_.emplace(state.driver->getInst(), &state);
}

std::vector< LoadDownsizeCandidate > LoadDownsizePlanner::candidates(
    const NetState& source,
    const std::unordered_set< odb::dbObject* >& protected_loads) {
    std::vector< LoadDownsizeCandidate > result;
    if(!options_.allow_load_downsize)
        return result;
    auto* network = sta_->getDbNetwork();
    std::unordered_set< odb::dbInst* > protected_instances;
    for(auto* pin : protected_loads)
        if(pin->getObjectType() == odb::dbITermObj)
            protected_instances.insert(
                static_cast< odb::dbITerm* >(pin)->getInst());
    std::unordered_set< odb::dbInst* > seen;
    const auto driver_model = backend_.driverModel(source);
    const float source_delay = backend_.predictedDelay(
        driver_model, source.capacitance, source.input_slew);
    for(const auto& load : source.loads) {
        if(load.term->getObjectType() != odb::dbITermObj)
            continue;
        auto* inst = static_cast< odb::dbITerm* >(load.term)->getInst();
        if(inst == source.driver->getInst() ||
           protected_instances.count(inst) || !seen.insert(inst).second)
            continue;
        const auto found = outputs_.find(inst);
        if(found == outputs_.end())
            continue;  // Includes protected/fixed-function/undriven output
                       // nets.
        const auto& output = *found->second;
        bool protected_net = false;
        for(auto* pin : inst->getITerms()) {
            if(pin->getSigType() == odb::dbSigType::POWER ||
               pin->getSigType() == odb::dbSigType::GROUND)
                continue;
            auto* net = pin->getNet();
            if(!net || pin->getIoType() == odb::dbIoType::INOUT ||
               net->isDoNotTouch() || net->isSpecial() ||
               sta_->isClock(network->dbToSta(net), sta_->cmdMode()))
                protected_net = true;
        }
        if(protected_net)
            continue;
        const auto replacements = backend_.resizeCandidates(output, true);
        if(replacements.empty())
            continue;
        const float slack = float(sta_->slack(
            network->dbToSta(output.driver), sta::RiseFallBoth::riseFall(),
            sta_->scenes(), sta::MinMax::max()));
        if(std::isnan(slack) || slack < options_.load_downsize_slack_margin)
            continue;
        const auto original = backend_.driverModel(output);
        const float old_delay = backend_.predictedDelay(
            original, output.capacitance, output.input_slew);
        if(!std::isfinite(old_delay))
            continue;
        for(const auto* replacement : replacements) {
            float reduction = 0;
            bool increases_input_cap = false;
            // Sum all pins of the same cell on this source, avoiding double
            // counting tied inputs. Do not increase another input net's load.
            for(auto* pin : inst->getITerms()) {
                if(pin->getIoType() != odb::dbIoType::INPUT)
                    continue;
                const auto* old_port =
                    network->libertyPort(network->dbToSta(pin));
                const auto* new_port = replacement->cell->findLibertyPort(
                    pin->getMTerm()->getConstName());
                if(!old_port || !new_port) {
                    increases_input_cap = true;
                    break;
                }
                const float delta = old_port->capacitance(sta::MinMax::max()) -
                                    new_port->capacitance(sta::MinMax::max());
                increases_input_cap |= delta < 0;
                if(pin->getNet() == source.net)
                    reduction += delta;
            }
            if(increases_input_cap || reduction <= 0)
                continue;
            const float new_delay = backend_.predictedDelay(
                *replacement, output.capacitance, output.input_slew);
            const float penalty = std::max(0.0f, new_delay - old_delay);
            if(!std::isfinite(new_delay) ||
               slack < penalty + options_.load_downsize_slack_margin)
                continue;
            const float after = backend_.predictedDelay(
                driver_model, std::max(0.0f, source.capacitance - reduction),
                source.input_slew);
            const float gain = source_delay - after;
            if(!std::isfinite(gain) || gain <= 0)
                continue;
            result.push_back({inst, replacement->master, reduction, gain,
                              slack - penalty,
                              replacement->area - original.area});
        }
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        if(a.cap_reduction != b.cap_reduction)
            return a.cap_reduction > b.cap_reduction;
        if(a.slack != b.slack)
            return a.slack > b.slack;
        if(a.added_area != b.added_area)
            return a.added_area < b.added_area;
        if(a.inst->getName() != b.inst->getName())
            return a.inst->getName() < b.inst->getName();
        return a.master->getName() < b.master->getName();
    });
    return result;
}
}  // namespace ecc::detail
