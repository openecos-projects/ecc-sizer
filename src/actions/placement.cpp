// SPDX-License-Identifier: Apache-2.0
#include "placement.h"
#include "legalize_core.h"
#include "dpl/Opendp.h"
#include "odb/db.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <unordered_set>

namespace ecc {
static bool legalizeAbacus(odb::dbBlock *block, dpl::Opendp *placer) {
    // Regions, blockages and asymmetric padding remain OpenDP responsibilities.
    if (!block->getRegions().empty() || !block->getBlockages().empty())
        return false;
    std::vector<odb::dbRow *> rows;
    for (auto *row : block->getRows()) {
        rows.push_back(row);
    }
    if (rows.empty()) {
        spdlog::debug("dp-legalize: no rows, fall back to detailed_placement");
        return false;
    }
    const int site_w = rows[0]->getSite()->getWidth();
    const int site_h = rows[0]->getSite()->getHeight();
    if (site_w <= 0 || site_h <= 0)
        return false;
    std::vector<LegalizeRowSeg> segs;
    std::map<int, odb::dbOrientType> orient_of_y;
    for (auto *row : rows) {
        if (row->getDirection() != odb::dbRowDir::HORIZONTAL ||
            row->getSpacing() != site_w || row->getSite()->getWidth() != site_w ||
            row->getSite()->getHeight() != site_h) {
            spdlog::debug("dp-legalize: non-uniform rows, fall back");
            return false;
        }
        segs.push_back(
            {row->getOrigin().x(), row->getOrigin().y(), row->getSiteCount()});
        int ry = row->getOrigin().y();
        if (orient_of_y.count(ry) && orient_of_y[ry] != row->getOrient()) {
            spdlog::debug("dp-legalize: inconsistent orient at y={}, fall back", ry);
            return false;
        }
        orient_of_y[ry] = row->getOrient();
    }

    std::vector<LegalizeNode> nodes;
    std::vector<odb::dbInst *> insts;
    for (auto *inst : block->getInsts()) {
        auto *master = inst->getMaster();
        if (placer->padLeft(inst) || placer->padRight(inst))
            return false;
        if (inst->isFixed())
            continue;
        if (!master->isCore())
            return false;
        if (static_cast<int>(master->getHeight()) != site_h ||
            master->getSite() != rows.front()->getSite()) {
            spdlog::debug("dp-legalize: multi-row movable cell {}, fall back",
                          master->getName().c_str());
            return false;
        }
        int ix = 0, iy = 0;
        inst->getLocation(ix, iy);
        insts.push_back(inst);
        nodes.push_back({double(ix), double(iy), double(master->getWidth()),
                         double(master->getHeight()), false});
    }
    const int num_movable = static_cast<int>(insts.size());
    for (auto *inst : block->getInsts()) {
        // Any fixed instance (macros included) is an obstacle for the ops.
        if (!inst->isFixed()) {
            continue;
        }
        auto *bbox = inst->getBBox();
        int x_min = bbox->xMin();
        int y_min = bbox->yMin();
        int x_max = bbox->xMax();
        int y_max = bbox->yMax();
        if (inst->getHalo() != nullptr) {
            odb::Rect halo = inst->getTransformedHalo();
            x_min -= halo.xMin();
            y_min -= halo.yMin();
            x_max += halo.xMax();
            y_max += halo.yMax();
        }
        nodes.push_back({double(x_min), double(y_min), double(x_max - x_min),
                         double(y_max - y_min), true});
    }
    if (num_movable == 0) {
        return true;
    }

    std::vector<double> out_x;
    std::vector<int> out_row;
    int base_x = 0, base_y = 0, num_rows = 0;
    if (dpLegalizeCore(segs, site_w, site_h, nodes, num_movable, 0.0, out_x, out_row,
                       base_x, base_y, num_rows) != 0) {
        return false;
    }

    double max_disp = 0.0;
    for (int i = 0; i < num_movable; ++i) {
        const int ry = base_y + out_row[i] * site_h;
        insts[i]->setOrient(orient_of_y[ry]);
        insts[i]->setLocation(static_cast<int>(std::lround(out_x[i])), ry);
        insts[i]->setPlacementStatus(odb::dbPlacementStatus::PLACED);
        max_disp = std::max(max_disp, std::abs(out_x[i] - nodes[i].x) +
                                          std::abs(ry - nodes[i].y));
    }
    spdlog::debug("dp-legalize: placed {} cells, max displacement {:.0f} dbu",
                  num_movable, max_disp);
    return true;
}

void legalizeRepairPlacement(odb::dbBlock *block, dpl::Opendp *placer,
                             const std::vector<odb::dbInst *> &protected_instances,
                             bool incremental, PlacementLegalizer legalizer) {
    if (!block || !placer || block->getRows().empty())
        throw std::runtime_error(
            "native repair legalization requires a placer and core rows");
    const std::unordered_set<odb::dbInst *> protected_set(protected_instances.begin(),
                                                          protected_instances.end());
    std::vector<std::pair<odb::dbInst *, odb::dbPlacementStatus>> statuses;
    for (auto *inst : block->getInsts()) {
        if (!inst->isFixed() && (inst->isDoNotTouch() || protected_set.count(inst))) {
            statuses.emplace_back(inst, inst->getPlacementStatus());
            inst->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
        }
    }
    const auto restore = [&]() {
        for (const auto &[inst, status] : statuses)
            inst->setPlacementStatus(status);
    };
    try {
        // Default DPL minimizes movement better than the negotiation/abacus
        // mode on the repair workloads. Never treat a placed flag as legality.
        if (legalizer != PlacementLegalizer::Abacus || !legalizeAbacus(block, placer))
            placer->detailedPlacement(0, 0, "", incremental);
        placer->checkPlacement(false);
    } catch (...) {
        restore();
        throw;
    }
    restore();
}
} // namespace ecc
