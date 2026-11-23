// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#include "netlist.h"
#include <stdexcept>

#include <algorithm>
#include <unordered_set>
#include <utility>

#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"

namespace ecc {
EcoTransaction::EcoTransaction(odb::dbBlock *block) : block_(block) {
    if (!block_)
        throw std::invalid_argument("missing ECO block");
    odb::dbDatabase::beginEco(block_);
}
EcoTransaction::~EcoTransaction() { rollback(); }
void EcoTransaction::suspend() {
    if (!active_ || suspended_)
        throw std::logic_error("ECO is not recording");
    odb::dbDatabase::endEco(block_);
    suspended_ = true;
}
void EcoTransaction::commit() {
    if (!active_)
        throw std::logic_error("ECO is already closed");
    odb::dbDatabase::commitEco(block_);
    active_ = false;
}
void EcoTransaction::rollback() {
    if (!active_)
        return;
    // Clear ownership first: callbacks invoked by OpenDB must never trigger
    // a second undo of the same journal during exception unwinding.
    active_ = false;
    odb::dbDatabase::undoEco(block_);
}

static BufferInsertionResult failure(BufferInsertionStatus status,
                                     std::string message) {
    BufferInsertionResult result;
    result.status = status;
    result.message = std::move(message);
    return result;
}

static odb::dbNet *getNet(odb::dbObject *term) {
    if (term == nullptr) {
        return nullptr;
    }
    if (term->getObjectType() == odb::dbITermObj) {
        return static_cast<odb::dbITerm *>(term)->getNet();
    }
    if (term->getObjectType() == odb::dbBTermObj) {
        return static_cast<odb::dbBTerm *>(term)->getNet();
    }
    return nullptr;
}

static bool isLoad(odb::dbObject *term) {
    if (term == nullptr) {
        return false;
    }
    if (term->getObjectType() == odb::dbITermObj) {
        const odb::dbIoType io_type = static_cast<odb::dbITerm *>(term)->getIoType();
        return io_type == odb::dbIoType::INPUT || io_type == odb::dbIoType::INOUT;
    }
    if (term->getObjectType() == odb::dbBTermObj) {
        const odb::dbIoType io_type = static_cast<odb::dbBTerm *>(term)->getIoType();
        return io_type == odb::dbIoType::OUTPUT || io_type == odb::dbIoType::INOUT;
    }
    return false;
}

static bool isSimpleBuffer(odb::dbMaster *master) {
    if (master == nullptr) {
        return false;
    }
    int input_count = 0;
    int output_count = 0;
    for (odb::dbMTerm *term : master->getMTerms()) {
        if (term->getIoType() == odb::dbIoType::INPUT) {
            ++input_count;
        } else if (term->getIoType() == odb::dbIoType::OUTPUT) {
            ++output_count;
        }
    }
    return input_count == 1 && output_count == 1;
}

static const odb::Point *location(const BufferInsertionOptions &options,
                                  std::optional<odb::Point> &storage) {
    if (!options.location.has_value()) {
        return nullptr;
    }
    storage.emplace(options.location->x, options.location->y);
    return &*storage;
}

static void clampToCore(odb::dbInst *buffer) {
    if (buffer == nullptr || !buffer->isPlaced()) {
        return;
    }
    odb::dbBlock *block = buffer->getBlock();
    const odb::Rect core = block->getCoreArea();
    if (core.dx() <= 0 || core.dy() <= 0) {
        return;
    }

    int x = 0;
    int y = 0;
    buffer->getLocation(x, y);
    const int cell_width = static_cast<int>(buffer->getMaster()->getWidth());
    const int cell_height = static_cast<int>(buffer->getMaster()->getHeight());
    const int max_x = std::max(core.xMin(), core.xMax() - cell_width);
    const int max_y = std::max(core.yMin(), core.yMax() - cell_height);
    x = std::clamp(x, core.xMin(), max_x);
    y = std::clamp(y, core.yMin(), max_y);
    buffer->setLocation(x, y);
}

static void addAffectedNet(BufferInsertionResult &result,
                           std::unordered_set<odb::dbNet *> &seen, odb::dbNet *net) {
    if (net != nullptr && seen.insert(net).second) {
        result.affected_nets.push_back(net);
    }
}

static BufferInsertionResult
makeResult(odb::dbInst *buffer,
           const std::vector<odb::dbNet *> &previously_affected_nets = {}) {
    if (buffer == nullptr) {
        return failure(BufferInsertionStatus::kInsertionFailed,
                       "OpenDB did not insert a buffer");
    }

    BufferInsertionResult result;
    result.status = BufferInsertionStatus::kSuccess;
    result.buffer = buffer;
    std::unordered_set<odb::dbNet *> seen;
    for (odb::dbNet *net : previously_affected_nets) {
        addAffectedNet(result, seen, net);
    }
    for (odb::dbITerm *term : buffer->getITerms()) {
        if (term->getIoType() == odb::dbIoType::INPUT) {
            result.input_pin = term;
        } else if (term->getIoType() == odb::dbIoType::OUTPUT) {
            result.output_pin = term;
        }
    }
    if (result.input_pin == nullptr || result.output_pin == nullptr) {
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

static BufferInsertionResult validateCommon(odb::dbNet *net,
                                            odb::dbMaster *buffer_master) {
    if (net == nullptr) {
        return failure(BufferInsertionStatus::kInvalidNet,
                       "buffer insertion requires a net");
    }
    if (!isSimpleBuffer(buffer_master) || !buffer_master->isFrozen()) {
        return failure(BufferInsertionStatus::kInvalidBufferMaster,
                       "buffer master must be frozen with one input and one "
                       "output");
    }
    BufferInsertionResult result;
    result.status = BufferInsertionStatus::kSuccess;
    return result;
}

BufferInsertionResult
OpenDbBufferInserter::insertBufferAfterDriver(odb::dbNet *net,
                                              odb::dbMaster *buffer_master,
                                              const BufferInsertionOptions &options) {
    BufferInsertionResult validation = validateCommon(net, buffer_master);
    if (!validation) {
        return validation;
    }
    odb::dbObject *driver = net->getFirstDriverTerm();
    if (driver == nullptr) {
        return failure(BufferInsertionStatus::kMissingDriver, "net has no driver");
    }

    std::optional<odb::Point> point;
    odb::dbInst *buffer = net->insertBufferAfterDriver(
        driver, buffer_master, location(options, point),
        options.buffer_name_prefix.c_str(), options.net_name_prefix.c_str(),
        odb::dbNameUniquifyType::ALWAYS);
    return makeResult(buffer);
}

BufferInsertionResult
OpenDbBufferInserter::insertBufferBeforeLoad(odb::dbObject *load,
                                             odb::dbMaster *buffer_master,
                                             const BufferInsertionOptions &options) {
    if (!isLoad(load)) {
        return failure(BufferInsertionStatus::kInvalidLoad,
                       "load must be an input pin");
    }
    odb::dbNet *net = getNet(load);
    BufferInsertionResult validation = validateCommon(net, buffer_master);
    if (!validation) {
        return validation;
    }

    std::optional<odb::Point> point;
    odb::dbInst *buffer = net->insertBufferBeforeLoad(
        load, buffer_master, location(options, point),
        options.buffer_name_prefix.c_str(), options.net_name_prefix.c_str(),
        odb::dbNameUniquifyType::ALWAYS);
    return makeResult(buffer);
}

BufferInsertionResult OpenDbBufferInserter::insertBufferBeforeLoads(
    odb::dbNet *net, const std::vector<odb::dbObject *> &loads,
    odb::dbMaster *buffer_master, const BufferInsertionOptions &options) {
    BufferInsertionResult validation = validateCommon(net, buffer_master);
    if (!validation) {
        return validation;
    }
    if (loads.empty()) {
        return failure(BufferInsertionStatus::kInvalidLoad,
                       "at least one load is required");
    }
    std::vector<odb::dbNet *> original_nets;
    std::unordered_set<odb::dbNet *> seen_original_nets;
    for (odb::dbObject *load : loads) {
        odb::dbNet *load_net = getNet(load);
        if (!isLoad(load) || load_net == nullptr) {
            return failure(BufferInsertionStatus::kInvalidLoad,
                           "all loads must be connected input pins");
        }
        if (load_net->getBlock() != net->getBlock()) {
            return failure(BufferInsertionStatus::kInvalidLoad,
                           "all loads must belong to the target block");
        }
        if (!options.loads_on_different_nets && load_net != net) {
            return failure(BufferInsertionStatus::kLoadsOnDifferentNets,
                           "all loads must belong to the target net");
        }
        if (seen_original_nets.insert(load_net).second) {
            original_nets.push_back(load_net);
        }
    }

    std::optional<odb::Point> point;
    odb::dbInst *buffer = net->insertBufferBeforeLoads(
        loads, buffer_master, location(options, point),
        options.buffer_name_prefix.c_str(), options.net_name_prefix.c_str(),
        odb::dbNameUniquifyType::ALWAYS, options.loads_on_different_nets);
    return makeResult(buffer, original_nets);
}

bool compatibleCellMasters(odb::dbMaster *from, odb::dbMaster *to) {
    if (!from || !to || from->getHeight() != to->getHeight() ||
        from->getSite() != to->getSite() ||
        from->getMTerms().size() != to->getMTerms().size())
        return false;
    for (auto *term : from->getMTerms()) {
        auto *other = to->findMTerm(term->getConstName());
        if (!other || term->getIoType() != other->getIoType() ||
            term->getSigType() != other->getSigType())
            return false;
    }
    return true;
}

bool replaceCell(const CellReplacement &replacement) {
    auto *inst = replacement.instance;
    auto *master = replacement.master;
    if (!inst || !master)
        return false;
    if (inst->getMaster() == master)
        return true;
    return !inst->isFixed() && !inst->isDoNotTouch() &&
           compatibleCellMasters(inst->getMaster(), master) && inst->swapMaster(master);
}

BufferTreeEdits applyBufferTree(const BufferTreePlan &plan,
                                const BufferInsertionOptions &naming) {
    BufferTreeEdits result;
    if (plan.nodes.empty())
        return result;
    if (!plan.net || plan.net->isDoNotTouch() || plan.net->isSpecial() ||
        plan.net->getSigType() != odb::dbSigType::SIGNAL ||
        !plan.net->getFirstDriverTerm())
        throw std::invalid_argument(
            "buffer tree requires an editable driven signal net");
    std::vector<bool> has_parent(plan.nodes.size(), false);
    std::unordered_set<odb::dbObject *> terminals;
    for (std::size_t i = 0; i < plan.nodes.size(); ++i) {
        const auto &node = plan.nodes[i];
        const auto valid = validateCommon(plan.net, node.master);
        if (!valid)
            throw std::invalid_argument(valid.message);
        if (node.loads.empty() && node.children.empty())
            throw std::invalid_argument("buffer tree node has no loads");
        for (auto *load : node.loads) {
            if (!isLoad(load) || getNet(load) != plan.net ||
                !terminals.insert(load).second)
                throw std::invalid_argument("buffer tree has stale or duplicate loads");
        }
        for (auto child : node.children) {
            if (child >= i || has_parent[child])
                throw std::invalid_argument(
                    "buffer tree requires unique postorder children");
            has_parent[child] = true;
        }
    }
    result.insertions.reserve(plan.nodes.size());
    auto options = naming;
    options.loads_on_different_nets = false;
    for (std::size_t i = 0; i < plan.nodes.size(); ++i) {
        const auto &node = plan.nodes[i];
        auto loads = node.loads;
        for (auto child : node.children)
            loads.push_back(result.insertions[child].input_pin);
        options.location = node.location;
        auto edit = OpenDbBufferInserter::insertBufferBeforeLoads(plan.net, loads,
                                                                  node.master, options);
        if (!edit)
            throw std::runtime_error(edit.message);
        result.insertions.push_back(std::move(edit));
        if (!has_parent[i])
            result.roots.push_back(i);
    }
    return result;
}

unsigned insertBuffers(const std::vector<BufferInsertion> &insertions) {
    unsigned inserted = 0;
    for (const auto &edit : insertions) {
        const auto result = OpenDbBufferInserter::insertBufferBeforeLoads(
            edit.net, edit.loads, edit.master, edit.options);
        if (!result)
            throw std::runtime_error(result.message);
        ++inserted;
    }
    return inserted;
}
} // namespace ecc
