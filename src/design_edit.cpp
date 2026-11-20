// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "design_edit.h"
#include "dpl/Opendp.h"
#include "legalize_core.h"
#include "spdlog/spdlog.h"
#include <map>
#include <cmath>
#include <stdexcept>

#include <algorithm>
#include <unordered_set>
#include <utility>

#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"

namespace ecc {
EcoTransaction::EcoTransaction(odb::dbBlock* block) : block_(block) {
    if(!block_) throw std::invalid_argument("missing ECO block");
    odb::dbDatabase::beginEco(block_);
}
EcoTransaction::~EcoTransaction() { rollback(); }
void EcoTransaction::suspend() {
    if(!active_ || suspended_) throw std::logic_error("ECO is not recording");
    odb::dbDatabase::endEco(block_);
    suspended_ = true;
}
void EcoTransaction::commit() {
    if(!active_) throw std::logic_error("ECO is already closed");
    odb::dbDatabase::commitEco(block_);
    active_ = false;
}
void EcoTransaction::rollback() {
    if(!active_) return;
    // Clear ownership first: callbacks invoked by OpenDB must never trigger
    // a second undo of the same journal during exception unwinding.
    active_ = false;
    odb::dbDatabase::undoEco(block_);
}

namespace {

BufferInsertionResult failure(BufferInsertionStatus status,
                              std::string message) {
    BufferInsertionResult result;
    result.status = status;
    result.message = std::move(message);
    return result;
}

odb::dbNet* getNet(odb::dbObject* term) {
    if(term == nullptr) {
        return nullptr;
    }
    if(term->getObjectType() == odb::dbITermObj) {
        return static_cast< odb::dbITerm* >(term)->getNet();
    }
    if(term->getObjectType() == odb::dbBTermObj) {
        return static_cast< odb::dbBTerm* >(term)->getNet();
    }
    return nullptr;
}

bool isLoad(odb::dbObject* term) {
    if(term == nullptr) {
        return false;
    }
    if(term->getObjectType() == odb::dbITermObj) {
        const odb::dbIoType io_type =
            static_cast< odb::dbITerm* >(term)->getIoType();
        return io_type == odb::dbIoType::INPUT ||
               io_type == odb::dbIoType::INOUT;
    }
    if(term->getObjectType() == odb::dbBTermObj) {
        const odb::dbIoType io_type =
            static_cast< odb::dbBTerm* >(term)->getIoType();
        return io_type == odb::dbIoType::OUTPUT ||
               io_type == odb::dbIoType::INOUT;
    }
    return false;
}

bool isSimpleBuffer(odb::dbMaster* master) {
    if(master == nullptr) {
        return false;
    }
    int input_count = 0;
    int output_count = 0;
    for(odb::dbMTerm* term : master->getMTerms()) {
        if(term->getIoType() == odb::dbIoType::INPUT) {
            ++input_count;
        }
        else if(term->getIoType() == odb::dbIoType::OUTPUT) {
            ++output_count;
        }
    }
    return input_count == 1 && output_count == 1;
}

const odb::Point* location(const BufferInsertionOptions& options,
                           std::optional< odb::Point >& storage) {
    if(!options.location.has_value()) {
        return nullptr;
    }
    storage.emplace(options.location->x, options.location->y);
    return &*storage;
}

void clampToCore(odb::dbInst* buffer) {
    if(buffer == nullptr || !buffer->isPlaced()) {
        return;
    }
    odb::dbBlock* block = buffer->getBlock();
    const odb::Rect core = block->getCoreArea();
    if(core.dx() <= 0 || core.dy() <= 0) {
        return;
    }

    int x = 0;
    int y = 0;
    buffer->getLocation(x, y);
    const int cell_width = static_cast< int >(buffer->getMaster()->getWidth());
    const int cell_height =
        static_cast< int >(buffer->getMaster()->getHeight());
    const int max_x = std::max(core.xMin(), core.xMax() - cell_width);
    const int max_y = std::max(core.yMin(), core.yMax() - cell_height);
    x = std::clamp(x, core.xMin(), max_x);
    y = std::clamp(y, core.yMin(), max_y);
    buffer->setLocation(x, y);
}

void addAffectedNet(BufferInsertionResult& result,
                    std::unordered_set< odb::dbNet* >& seen, odb::dbNet* net) {
    if(net != nullptr && seen.insert(net).second) {
        result.affected_nets.push_back(net);
    }
}

BufferInsertionResult makeResult(
    odb::dbInst* buffer,
    const std::vector< odb::dbNet* >& previously_affected_nets = {}) {
    if(buffer == nullptr) {
        return failure(BufferInsertionStatus::kInsertionFailed,
                       "OpenDB did not insert a buffer");
    }

    BufferInsertionResult result;
    result.status = BufferInsertionStatus::kSuccess;
    result.buffer = buffer;
    std::unordered_set< odb::dbNet* > seen;
    for(odb::dbNet* net : previously_affected_nets) {
        addAffectedNet(result, seen, net);
    }
    for(odb::dbITerm* term : buffer->getITerms()) {
        if(term->getIoType() == odb::dbIoType::INPUT) {
            result.input_pin = term;
        }
        else if(term->getIoType() == odb::dbIoType::OUTPUT) {
            result.output_pin = term;
        }
    }
    if(result.input_pin == nullptr || result.output_pin == nullptr) {
        return failure(BufferInsertionStatus::kInsertionFailed,
                       "inserted buffer pins could not be identified");
    }

    result.input_net = result.input_pin->getNet();
    result.output_net = result.output_pin->getNet();
    addAffectedNet(result, seen, result.input_net);
    addAffectedNet(result, seen, result.output_net);
    clampToCore(buffer);
    return result;
}

BufferInsertionResult validateCommon(odb::dbNet* net,
                                     odb::dbMaster* buffer_master) {
    if(net == nullptr) {
        return failure(BufferInsertionStatus::kInvalidNet,
                       "buffer insertion requires a net");
    }
    if(!isSimpleBuffer(buffer_master) || !buffer_master->isFrozen()) {
        return failure(BufferInsertionStatus::kInvalidBufferMaster,
                       "buffer master must be frozen with one input and one "
                       "output");
    }
    BufferInsertionResult result;
    result.status = BufferInsertionStatus::kSuccess;
    return result;
}

}  // namespace

BufferInsertionResult OpenDbBufferInserter::insertBufferAfterDriver(
    odb::dbNet* net, odb::dbMaster* buffer_master,
    const BufferInsertionOptions& options) {
    BufferInsertionResult validation = validateCommon(net, buffer_master);
    if(!validation) {
        return validation;
    }
    odb::dbObject* driver = net->getFirstDriverTerm();
    if(driver == nullptr) {
        return failure(BufferInsertionStatus::kMissingDriver,
                       "net has no driver");
    }

    std::optional< odb::Point > point;
    odb::dbInst* buffer = net->insertBufferAfterDriver(
        driver, buffer_master, location(options, point),
        options.buffer_name_prefix.c_str(), options.net_name_prefix.c_str(),
        odb::dbNameUniquifyType::ALWAYS);
    return makeResult(buffer);
}

BufferInsertionResult OpenDbBufferInserter::insertBufferBeforeLoad(
    odb::dbObject* load, odb::dbMaster* buffer_master,
    const BufferInsertionOptions& options) {
    if(!isLoad(load)) {
        return failure(BufferInsertionStatus::kInvalidLoad,
                       "load must be an input pin");
    }
    odb::dbNet* net = getNet(load);
    BufferInsertionResult validation = validateCommon(net, buffer_master);
    if(!validation) {
        return validation;
    }

    std::optional< odb::Point > point;
    odb::dbInst* buffer = net->insertBufferBeforeLoad(
        load, buffer_master, location(options, point),
        options.buffer_name_prefix.c_str(), options.net_name_prefix.c_str(),
        odb::dbNameUniquifyType::ALWAYS);
    return makeResult(buffer);
}

BufferInsertionResult OpenDbBufferInserter::insertBufferBeforeLoads(
    odb::dbNet* net, const std::vector< odb::dbObject* >& loads,
    odb::dbMaster* buffer_master, const BufferInsertionOptions& options) {
    BufferInsertionResult validation = validateCommon(net, buffer_master);
    if(!validation) {
        return validation;
    }
    if(loads.empty()) {
        return failure(BufferInsertionStatus::kInvalidLoad,
                       "at least one load is required");
    }
    std::vector< odb::dbNet* > original_nets;
    std::unordered_set< odb::dbNet* > seen_original_nets;
    for(odb::dbObject* load : loads) {
        odb::dbNet* load_net = getNet(load);
        if(!isLoad(load) || load_net == nullptr) {
            return failure(BufferInsertionStatus::kInvalidLoad,
                           "all loads must be connected input pins");
        }
        if(load_net->getBlock() != net->getBlock()) {
            return failure(BufferInsertionStatus::kInvalidLoad,
                           "all loads must belong to the target block");
        }
        if(!options.loads_on_different_nets && load_net != net) {
            return failure(BufferInsertionStatus::kLoadsOnDifferentNets,
                           "all loads must belong to the target net");
        }
        if(seen_original_nets.insert(load_net).second) {
            original_nets.push_back(load_net);
        }
    }

    std::optional< odb::Point > point;
    odb::dbInst* buffer = net->insertBufferBeforeLoads(
        loads, buffer_master, location(options, point),
        options.buffer_name_prefix.c_str(), options.net_name_prefix.c_str(),
        odb::dbNameUniquifyType::ALWAYS, options.loads_on_different_nets);
    return makeResult(buffer, original_nets);
}

bool compatibleCellMasters(odb::dbMaster* from, odb::dbMaster* to) {
    if(!from || !to || from->getHeight() != to->getHeight() ||
       from->getSite() != to->getSite() ||
       from->getMTerms().size() != to->getMTerms().size())
        return false;
    for(auto* term : from->getMTerms()) {
        auto* other = to->findMTerm(term->getConstName());
        if(!other || term->getIoType() != other->getIoType() ||
           term->getSigType() != other->getSigType())
            return false;
    }
    return true;
}

bool replaceCell(const CellReplacement& replacement) {
    auto* inst = replacement.instance;
    auto* master = replacement.master;
    if(!inst || !master)
        return false;
    if(inst->getMaster() == master)
        return true;
    return !inst->isFixed() && !inst->isDoNotTouch() &&
           compatibleCellMasters(inst->getMaster(), master) &&
           inst->swapMaster(master);
}

NetRepairEdits applyRepairPlan(const NetRepairPlan& plan) {
    NetRepairEdits result;
    if(plan.replacement) {
        const auto& replacement = *plan.replacement;
        if(!replacement.instance || !replacement.master)
            throw std::invalid_argument(
                "cell replacement requires an instance and master");
        if(replaceCell(replacement)) {
            result.replaced = true;
            return result;
        }
    }
    for(const auto& insertion : plan.buffers) {
        const auto inserted = OpenDbBufferInserter::insertBufferBeforeLoads(
            insertion.net, insertion.loads, insertion.master,
            insertion.options);
        if(!inserted)
            throw std::runtime_error(inserted.message);
        ++result.inserted_buffers;
    }
    return result;
}

static bool legalizeAbacus(odb::dbBlock* block, dpl::Opendp* placer) {
    // Regions, blockages and asymmetric padding remain OpenDP responsibilities.
    if(!block->getRegions().empty() || !block->getBlockages().empty())
        return false;
    std::vector< odb::dbRow* > rows;
    for(auto* row : block->getRows()) {
        rows.push_back(row);
    }
    if(rows.empty()) {
        spdlog::debug("dp-legalize: no rows, fall back to detailed_placement");
        return false;
    }
    const int site_w = rows[0]->getSite()->getWidth();
    const int site_h = rows[0]->getSite()->getHeight();
    if(site_w <= 0 || site_h <= 0)
        return false;
    std::vector< LegalizeRowSeg > segs;
    std::map< int, odb::dbOrientType > orient_of_y;
    for(auto* row : rows) {
        if(row->getDirection() != odb::dbRowDir::HORIZONTAL ||
           row->getSpacing() != site_w ||
           row->getSite()->getWidth() != site_w ||
           row->getSite()->getHeight() != site_h) {
            spdlog::debug("dp-legalize: non-uniform rows, fall back");
            return false;
        }
        segs.push_back(
            {row->getOrigin().x(), row->getOrigin().y(), row->getSiteCount()});
        int ry = row->getOrigin().y();
        if(orient_of_y.count(ry) && orient_of_y[ry] != row->getOrient()) {
            spdlog::debug("dp-legalize: inconsistent orient at y={}, fall back",
                          ry);
            return false;
        }
        orient_of_y[ry] = row->getOrient();
    }

    std::vector< LegalizeNode > nodes;
    std::vector< odb::dbInst* > insts;
    for(auto* inst : block->getInsts()) {
        auto* master = inst->getMaster();
        if(placer->padLeft(inst) || placer->padRight(inst))
            return false;
        if(inst->isFixed())
            continue;
        if(!master->isCore())
            return false;
        if(static_cast< int >(master->getHeight()) != site_h ||
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
    const int num_movable = static_cast< int >(insts.size());
    for(auto* inst : block->getInsts()) {
        // Any fixed instance (macros included) is an obstacle for the ops.
        if(!inst->isFixed()) {
            continue;
        }
        auto* bbox = inst->getBBox();
        int x_min = bbox->xMin();
        int y_min = bbox->yMin();
        int x_max = bbox->xMax();
        int y_max = bbox->yMax();
        if(inst->getHalo() != nullptr) {
            odb::Rect halo = inst->getTransformedHalo();
            x_min -= halo.xMin();
            y_min -= halo.yMin();
            x_max += halo.xMax();
            y_max += halo.yMax();
        }
        nodes.push_back({double(x_min), double(y_min), double(x_max - x_min),
                         double(y_max - y_min), true});
    }
    if(num_movable == 0) {
        return true;
    }

    std::vector< double > out_x;
    std::vector< int > out_row;
    int base_x = 0, base_y = 0, num_rows = 0;
    if(dpLegalizeCore(segs, site_w, site_h, nodes, num_movable, 0.0, out_x,
                      out_row, base_x, base_y, num_rows) != 0) {
        return false;
    }

    double max_disp = 0.0;
    for(int i = 0; i < num_movable; ++i) {
        const int ry = base_y + out_row[i] * site_h;
        insts[i]->setOrient(orient_of_y[ry]);
        insts[i]->setLocation(static_cast< int >(std::lround(out_x[i])), ry);
        insts[i]->setPlacementStatus(odb::dbPlacementStatus::PLACED);
        max_disp = std::max(max_disp, std::abs(out_x[i] - nodes[i].x) +
                                          std::abs(ry - nodes[i].y));
    }
    spdlog::debug("dp-legalize: placed {} cells, max displacement {:.0f} dbu",
                  num_movable, max_disp);
    return true;
}

void legalizeRepairPlacement(
    odb::dbBlock* block, dpl::Opendp* placer,
    const std::vector< odb::dbInst* >& protected_instances, bool incremental,
    PlacementLegalizer legalizer) {
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
        if(legalizer != PlacementLegalizer::Abacus ||
           !legalizeAbacus(block, placer))
            placer->detailedPlacement(0, 0, "", incremental);
        placer->checkPlacement(false);
    }
    catch(...) {
        restore();
        throw;
    }
    restore();
}
}  // namespace ecc
