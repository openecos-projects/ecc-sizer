// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "buffer_inserter.h"

#include <algorithm>
#include <unordered_set>
#include <utility>

#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"

namespace ecc {
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

}  // namespace ecc
