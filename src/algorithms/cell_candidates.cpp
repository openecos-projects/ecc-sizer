// SPDX-License-Identifier: Apache-2.0
#include "algorithms/cell_candidates.h"
#include "actions/netlist.h"
#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "sta/Liberty.hh"
#include "sta/EquivCells.hh"
namespace ecc::detail {
CellCandidates::CellCandidates(DesignAnalysis &analysis, sta::dbSta *sta,
                               const RepairCommonOptions &options)
    : analysis_(analysis), network_(sta->getDbNetwork()), opts_(options) {}
std::vector<const CellModel *> CellCandidates::resize(const NetState &state,
                                                      bool downsize, bool vt_only,
                                                      bool check_load) {
    std::vector<const CellModel *> result;
    auto *driver = state.driver->getInst();
    auto *original = network_->libertyCell(driver);
    if (!original || driver->isFixed() || driver->isDoNotTouch() ||
        std::ranges::find(opts_.excluded_instances, driver) !=
            opts_.excluded_instances.end() ||
        std::ranges::find(opts_.excluded_resize_instances, driver) !=
            opts_.excluded_resize_instances.end() ||
        state.fanout_ratio > 1 || (vt_only && analysis_.protectedInputs(driver)))
        return result;
    auto [it, fresh] = equivalents_.try_emplace(original);
    if (fresh) {
        for (const auto &candidate : analysis_.cellChoices()) {
            if ((candidate.cell->libertyLibrary() != original->libertyLibrary() &&
                 !analysis_.isVtSwap(driver->getMaster(), candidate.master)) ||
                !sta::equivCellsArcs(original, candidate.cell))
                continue;
            it->second.push_back(candidate);
        }
        std::sort(it->second.begin(), it->second.end(),
                  [](const CellModel &a, const CellModel &b) {
                      return a.area != b.area
                                 ? a.area < b.area
                                 : a.master->getName() < b.master->getName();
                  });
    }
    for (const auto &candidate : it->second) {
        auto *master = driver->getMaster();
        if ((vt_only ? !analysis_.isVtSwap(master, candidate.master)
                     : candidate.cell->libertyLibrary() != original->libertyLibrary() ||
                           (downsize ? candidate.area >= area(master)
                                     : candidate.area <= area(master))) ||
            !compatibleCellMasters(master, candidate.master) ||
            (check_load && state.capacitance > candidate.max_cap * (1 - opts_.margin)))
            continue;
        float limit = state.slew_limit, local_limit;
        bool exists = false;
        candidate.output->slewLimit(sta::MinMax::max(), local_limit, exists);
        if (!exists)
            candidate.cell->libertyLibrary()->defaultMaxSlew(local_limit, exists);
        if (exists)
            limit = std::min(limit, local_limit);
        const float slew =
            analysis_.predictedSlew(candidate, state.capacitance, state.input_slew);
        if (!check_load || (std::isfinite(slew) && slew <= limit * (1 - opts_.margin)))
            result.push_back(&candidate);
    }
    return result;
}

LoadDownsizePlanner::LoadDownsizePlanner(DesignAnalysis &analysis, sta::dbSta *sta,
                                         const std::vector<NetState> &states,
                                         const RepairCommonOptions &options)
    : analysis_(analysis), cells_(analysis, sta, options), sta_(sta),
      options_(options) {
    if (options.actions.load_downsize)
        for (const auto &state : states)
            outputs_.emplace(state.driver->getInst(), &state);
}

std::vector<LoadDownsizeCandidate> LoadDownsizePlanner::candidates(
    const NetState &source,
    const std::unordered_set<odb::dbObject *> &protected_loads) {
    std::vector<LoadDownsizeCandidate> result;
    if (!options_.actions.load_downsize)
        return result;
    auto *network = sta_->getDbNetwork();
    std::unordered_set<odb::dbInst *> protected_instances;
    for (auto *pin : protected_loads)
        if (pin->getObjectType() == odb::dbITermObj)
            protected_instances.insert(static_cast<odb::dbITerm *>(pin)->getInst());
    std::unordered_set<odb::dbInst *> seen;
    const auto driver_model = analysis_.driverModel(source);
    const float source_delay =
        analysis_.predictedDelay(driver_model, source.capacitance, source.input_slew);
    for (const auto &load : source.loads) {
        if (load.term->getObjectType() != odb::dbITermObj)
            continue;
        auto *inst = static_cast<odb::dbITerm *>(load.term)->getInst();
        if (inst == source.driver->getInst() || protected_instances.count(inst) ||
            !seen.insert(inst).second)
            continue;
        const auto found = outputs_.find(inst);
        if (found == outputs_.end())
            continue; // Includes protected/fixed-function/undriven output
                      // nets.
        const auto &output = *found->second;
        bool protected_net = false;
        for (auto *pin : inst->getITerms()) {
            if (pin->getSigType() == odb::dbSigType::POWER ||
                pin->getSigType() == odb::dbSigType::GROUND)
                continue;
            auto *net = pin->getNet();
            if (!net || pin->getIoType() == odb::dbIoType::INOUT ||
                net->isDoNotTouch() || net->isSpecial() ||
                sta_->isClock(network->dbToSta(net), sta_->cmdMode()))
                protected_net = true;
        }
        if (protected_net)
            continue;
        const auto replacements = cells_.resize(output, true);
        if (replacements.empty())
            continue;
        const float slack = float(sta_->slack(network->dbToSta(output.driver),
                                              sta::RiseFallBoth::riseFall(),
                                              sta_->scenes(), sta::MinMax::max()));
        if (std::isnan(slack) || slack < options_.load_downsize_slack_margin)
            continue;
        const auto original = analysis_.driverModel(output);
        const float old_delay =
            analysis_.predictedDelay(original, output.capacitance, output.input_slew);
        if (!std::isfinite(old_delay))
            continue;
        for (const auto *replacement : replacements) {
            float reduction = 0;
            bool increases_input_cap = false;
            // Sum all pins of the same cell on this source, avoiding double
            // counting tied inputs. Do not increase another input net's load.
            for (auto *pin : inst->getITerms()) {
                if (pin->getIoType() != odb::dbIoType::INPUT)
                    continue;
                const auto *old_port = network->libertyPort(network->dbToSta(pin));
                const auto *new_port =
                    replacement->cell->findLibertyPort(pin->getMTerm()->getConstName());
                if (!old_port || !new_port) {
                    increases_input_cap = true;
                    break;
                }
                const float delta = old_port->capacitance(sta::MinMax::max()) -
                                    new_port->capacitance(sta::MinMax::max());
                increases_input_cap |= delta < 0;
                if (pin->getNet() == source.net)
                    reduction += delta;
            }
            if (increases_input_cap || reduction <= 0)
                continue;
            const float new_delay = analysis_.predictedDelay(
                *replacement, output.capacitance, output.input_slew);
            const float penalty = std::max(0.0f, new_delay - old_delay);
            if (!std::isfinite(new_delay) ||
                slack < penalty + options_.load_downsize_slack_margin)
                continue;
            const float after = analysis_.predictedDelay(
                driver_model, std::max(0.0f, source.capacitance - reduction),
                source.input_slew);
            const float gain = source_delay - after;
            if (!std::isfinite(gain) || gain <= 0)
                continue;
            result.push_back({inst, replacement->master, reduction, gain,
                              slack - penalty, replacement->area - original.area});
        }
    }
    std::sort(result.begin(), result.end(), [](const auto &a, const auto &b) {
        if (a.cap_reduction != b.cap_reduction)
            return a.cap_reduction > b.cap_reduction;
        if (a.slack != b.slack)
            return a.slack > b.slack;
        if (a.added_area != b.added_area)
            return a.added_area < b.added_area;
        if (a.inst->getName() != b.inst->getName())
            return a.inst->getName() < b.inst->getName();
        return a.master->getName() < b.master->getName();
    });
    return result;
}
} // namespace ecc::detail
