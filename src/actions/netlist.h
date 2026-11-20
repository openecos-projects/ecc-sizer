// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace odb {
class dbBlock;
class dbInst;
class dbITerm;
class dbMaster;
class dbNet;
class dbObject;
} // namespace odb

namespace dpl {
class Opendp;
}
namespace ecc {

// Owns one OpenDB journal. Nested transactions must finish in LIFO order;
// committing a child merges its edits into the suspended parent. Timing and
// placement refresh remain explicit algorithm policy, including after rollback.
class EcoTransaction {
  public:
    explicit EcoTransaction(odb::dbBlock *block);
    ~EcoTransaction();
    EcoTransaction(const EcoTransaction &) = delete;
    EcoTransaction &operator=(const EcoTransaction &) = delete;
    EcoTransaction(EcoTransaction &&) = delete;
    EcoTransaction &operator=(EcoTransaction &&) = delete;

    bool active() const { return active_; }
    void suspend();
    void commit();
    void rollback();

  private:
    odb::dbBlock *block_;
    bool active_ = true;
    bool suspended_ = false;
};

struct BufferLocation {
    int x = 0;
    int y = 0;
};

struct BufferInsertionOptions {
    std::optional<BufferLocation> location;
    std::string buffer_name_prefix = "buf";
    std::string net_name_prefix = "net";
    bool loads_on_different_nets = false;
};

enum class BufferInsertionStatus {
    kSuccess,
    kInvalidNet,
    kInvalidBufferMaster,
    kMissingDriver,
    kInvalidLoad,
    kLoadsOnDifferentNets,
    kInsertionFailed,
};

struct BufferInsertionResult {
    BufferInsertionStatus status = BufferInsertionStatus::kInsertionFailed;
    std::string message;
    odb::dbInst *buffer = nullptr;
    odb::dbITerm *input_pin = nullptr;
    odb::dbITerm *output_pin = nullptr;
    odb::dbNet *input_net = nullptr;
    odb::dbNet *output_net = nullptr;
    std::vector<odb::dbNet *> affected_nets;

    bool ok() const { return status == BufferInsertionStatus::kSuccess; }
    explicit operator bool() const { return ok(); }
};

struct CellReplacement {
    odb::dbInst *instance = nullptr;
    odb::dbMaster *master = nullptr;
};

// Physical/pin compatibility only; algorithms establish logical equivalence.
bool compatibleCellMasters(odb::dbMaster *from, odb::dbMaster *to);
// Reject protected instances and incompatible masters. Timing refresh belongs
// to the caller; an unchanged master is a successful no-op.
bool replaceCell(const CellReplacement &replacement);

struct BufferInsertion {
    odb::dbNet *net = nullptr;
    std::vector<odb::dbObject *> loads;
    odb::dbMaster *master = nullptr;
    BufferInsertionOptions options;
};

// Apply every supplied insertion; the caller owns rollback on any failure.
unsigned insertBuffers(const std::vector<BufferInsertion> &insertions);

// Validates and applies buffer edits to OpenDB. Candidate selection and
// acceptance belong to algorithms; this layer has no Resizer dependency.
class OpenDbBufferInserter {
  public:
    static BufferInsertionResult
    insertBufferAfterDriver(odb::dbNet *net, odb::dbMaster *buffer_master,
                            const BufferInsertionOptions &options = {});

    static BufferInsertionResult
    insertBufferBeforeLoad(odb::dbObject *load, odb::dbMaster *buffer_master,
                           const BufferInsertionOptions &options = {});

    static BufferInsertionResult
    insertBufferBeforeLoads(odb::dbNet *net, const std::vector<odb::dbObject *> &loads,
                            odb::dbMaster *buffer_master,
                            const BufferInsertionOptions &options = {});
};

} // namespace ecc
