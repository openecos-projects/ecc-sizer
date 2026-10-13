// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "repair_fanout.h"

#include <algorithm>
#include <deque>
#include <unordered_set>
#include <utility>
#include <vector>

#include "odb/db.h"
#include "odb/dbTypes.h"

namespace ecc {
namespace {

FanoutRepairResult failure(FanoutRepairStatus status, std::string message) {
    FanoutRepairResult result;
    result.status = status;
    result.message = std::move(message);
    return result;
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

std::vector< odb::dbObject* > loads(odb::dbNet* net) {
    std::vector< odb::dbObject* > result;
    if(net == nullptr) {
        return result;
    }
    result.reserve(net->getITerms().size() + net->getBTerms().size());
    for(odb::dbITerm* term : net->getITerms()) {
        if(isLoad(term)) {
            result.push_back(term);
        }
    }
    for(odb::dbBTerm* term : net->getBTerms()) {
        if(isLoad(term)) {
            result.push_back(term);
        }
    }
    return result;
}

bool shouldRepair(
    odb::dbNet* net, bool repair_clock_nets,
    const std::unordered_set< odb::dbNet* >* excluded_nets = nullptr) {
    if(net == nullptr || net->isDoNotTouch() || net->isSpecial() ||
       (excluded_nets != nullptr && excluded_nets->count(net) > 0)) {
        return false;
    }
    const odb::dbSigType signal_type = net->getSigType();
    if(signal_type == odb::dbSigType::POWER ||
       signal_type == odb::dbSigType::GROUND) {
        return false;
    }
    return repair_clock_nets || signal_type != odb::dbSigType::CLOCK;
}

unsigned maxFanoutWithPolicy(
    odb::dbBlock* block, bool include_clock_nets,
    const std::unordered_set< odb::dbNet* >* excluded_nets) {
    unsigned max_fanout = 0;
    if(block == nullptr) {
        return max_fanout;
    }
    for(odb::dbNet* net : block->getNets()) {
        if(shouldRepair(net, include_clock_nets, excluded_nets)) {
            max_fanout =
                std::max(max_fanout, OpenDbFanoutRepairer::loadCount(net));
        }
    }
    return max_fanout;
}

bool isSimpleBuffer(odb::dbMaster* master) {
    if(master == nullptr || !master->isFrozen()) {
        return false;
    }
    unsigned input_count = 0;
    unsigned output_count = 0;
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

void addAffectedNet(FanoutRepairResult& result,
                    std::unordered_set< odb::dbNet* >& seen, odb::dbNet* net) {
    if(net != nullptr && seen.insert(net).second) {
        result.affected_nets.push_back(net);
    }
}

}  // namespace

unsigned OpenDbFanoutRepairer::loadCount(odb::dbNet* net) {
    return loads(net).size();
}

unsigned OpenDbFanoutRepairer::maxFanout(odb::dbBlock* block,
                                         bool include_clock_nets) {
    return maxFanoutWithPolicy(block, include_clock_nets, nullptr);
}

FanoutRepairResult OpenDbFanoutRepairer::repair(
    odb::dbBlock* block, odb::dbMaster* buffer_master,
    const FanoutRepairOptions& options) {
    if(block == nullptr) {
        return failure(FanoutRepairStatus::kInvalidBlock,
                       "fanout repair requires a block");
    }
    if(!isSimpleBuffer(buffer_master)) {
        return failure(FanoutRepairStatus::kInvalidBufferMaster,
                       "fanout repair requires a frozen buffer master with "
                       "one input and one output");
    }
    if(options.max_fanout < 2) {
        return failure(FanoutRepairStatus::kInvalidMaxFanout,
                       "max fanout must be at least two");
    }

    FanoutRepairResult result;
    result.status = FanoutRepairStatus::kSuccess;
    const std::unordered_set< odb::dbNet* > excluded_nets(
        options.excluded_nets.begin(), options.excluded_nets.end());
    result.before_max_fanout =
        maxFanoutWithPolicy(block, options.repair_clock_nets, &excluded_nets);
    std::unordered_set< odb::dbNet* > seen_affected_nets;

    // Keep the tree construction atomic. OpenDB supports nested ECO journals,
    // so this also composes with a caller-owned transaction.
    odb::dbDatabase::beginEco(block);
    const auto rollback = [&](FanoutRepairStatus status,
                              const std::string& message) {
        odb::dbDatabase::undoEco(block);
        result.status = status;
        result.message = message;
        result.inserted_buffers.clear();
        result.affected_nets.clear();
        result.after_max_fanout = maxFanoutWithPolicy(
            block, options.repair_clock_nets, &excluded_nets);
        return result;
    };

    std::vector< odb::dbNet* > candidate_nets;
    candidate_nets.reserve(block->getNets().size());
    for(odb::dbNet* net : block->getNets()) {
        if(shouldRepair(net, options.repair_clock_nets, &excluded_nets)) {
            candidate_nets.push_back(net);
        }
    }

    for(odb::dbNet* net : candidate_nets) {
        ++result.nets_examined;
        std::vector< odb::dbObject* > net_loads = loads(net);
        if(net_loads.size() <= options.max_fanout) {
            continue;
        }
        ++result.violating_nets;

        struct TreeNode {
            odb::dbObject* term;
            unsigned depth;
        };
        std::deque< TreeNode > root_loads;
        for(odb::dbObject* load : net_loads) {
            root_loads.push_back({load, 0});
        }

        // Each insertion reduces the root fanout by at most K - 1. Always
        // take that full reduction except for the last insertion, which
        // reaches K exactly. Taking the shallowest nodes first also keeps the
        // resulting minimum-size tree balanced.
        while(root_loads.size() > options.max_fanout) {
            const size_t excess = root_loads.size() - options.max_fanout;
            const size_t group_size =
                std::min(static_cast< size_t >(options.max_fanout), excess + 1);
            std::vector< odb::dbObject* > group;
            group.reserve(group_size);
            unsigned group_depth = 0;
            for(size_t index = 0; index < group_size; ++index) {
                group.push_back(root_loads.front().term);
                group_depth = std::max(group_depth, root_loads.front().depth);
                root_loads.pop_front();
            }
            const unsigned buffer_depth = group_depth + 1;
            if(buffer_depth > options.max_levels) {
                return rollback(
                    FanoutRepairStatus::kMaxLevelsExceeded,
                    "fanout repair exceeded the maximum tree depth");
            }

            BufferInsertionResult insertion =
                OpenDbBufferInserter::insertBufferBeforeLoads(
                    net, group, buffer_master, options.insertion);
            if(!insertion) {
                return rollback(FanoutRepairStatus::kInsertionFailed,
                                insertion.message);
            }
            result.inserted_buffers.push_back(insertion.buffer);
            for(odb::dbNet* affected : insertion.affected_nets) {
                addAffectedNet(result, seen_affected_nets, affected);
            }
            root_loads.push_back({insertion.input_pin, buffer_depth});
        }
    }

    result.after_max_fanout =
        maxFanoutWithPolicy(block, options.repair_clock_nets, &excluded_nets);
    if(result.after_max_fanout > options.max_fanout) {
        return rollback(FanoutRepairStatus::kInsertionFailed,
                        "fanout repair finished with an unresolved violation");
    }
    odb::dbDatabase::commitEco(block);
    return result;
}

}  // namespace ecc
