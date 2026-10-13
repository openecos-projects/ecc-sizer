// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_placement.h"

#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "dpl/Opendp.h"
#include "odb/db.h"

namespace ecc {
void legalizeRepairPlacement(
    odb::dbBlock* block, dpl::Opendp* placer,
    const std::vector< odb::dbInst* >& protected_instances) {
    if(!block || !placer || block->getRows().empty())
        throw std::runtime_error(
            "native repair legalization requires a placer and core rows");
    const std::unordered_set< odb::dbInst* > protected_set(
        protected_instances.begin(), protected_instances.end());
    std::vector< std::pair< odb::dbInst*, odb::dbPlacementStatus > > statuses;
    for(auto* inst : block->getInsts()) {
        if(!inst->isFixed() &&
           (inst->isDoNotTouch() || protected_set.count(inst))) {
            statuses.emplace_back(inst, inst->getPlacementStatus());
            inst->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
        }
    }
    const auto restore = [&]() {
        for(const auto& [inst, status] : statuses)
            inst->setPlacementStatus(status);
    };
    try {
        // Default DPL minimizes movement better than the negotiation/abacus
        // mode on the repair workloads. Never treat a placed flag as legality.
        placer->detailedPlacement(0, 0);
        placer->checkPlacement(false);
    }
    catch(...) {
        restore();
        throw;
    }
    restore();
}
}  // namespace ecc
