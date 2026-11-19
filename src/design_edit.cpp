// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "design_edit.h"
#include "dpl/Opendp.h"
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

NetRepairEdits applyRepairPlan(const NetRepairPlan& plan) {
    NetRepairEdits result;
    if(plan.replacement) {
        const auto& replacement = *plan.replacement;
        if(!replacement.instance || !replacement.master)
            throw std::invalid_argument(
                "cell replacement requires an instance and master");
        if(replacement.instance->swapMaster(replacement.master)) {
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

void legalizeRepairPlacement(
    odb::dbBlock* block, dpl::Opendp* placer,
    const std::vector< odb::dbInst* >& protected_instances, bool incremental) {
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
