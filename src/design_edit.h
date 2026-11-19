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
}  // namespace odb

namespace dpl {
class Opendp;
}
namespace ecc {

struct BufferLocation {
    int x = 0;
    int y = 0;
};

struct BufferInsertionOptions {
    std::optional< BufferLocation > location;
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
    odb::dbInst* buffer = nullptr;
    odb::dbITerm* input_pin = nullptr;
    odb::dbITerm* output_pin = nullptr;
    odb::dbNet* input_net = nullptr;
    odb::dbNet* output_net = nullptr;
    std::vector< odb::dbNet* > affected_nets;

    bool ok() const {
        return status == BufferInsertionStatus::kSuccess;
    }
    explicit operator bool() const {
        return ok();
    }
};

struct CellReplacement {
    odb::dbInst* instance = nullptr;
    odb::dbMaster* master = nullptr;
};

struct BufferInsertion {
    odb::dbNet* net = nullptr;
    std::vector< odb::dbObject* > loads;
    odb::dbMaster* master = nullptr;
    BufferInsertionOptions options;
};

// A net repair may prefer a cheaper cell replacement. If OpenDB refuses
// that swap, use the planned buffers. The plan is tied to the current
// design state and must be discarded after an intervening edit or rollback.
struct NetRepairPlan {
    std::optional< CellReplacement > replacement;
    std::vector< BufferInsertion > buffers;
};

struct NetRepairEdits {
    bool replaced = false;
    unsigned inserted_buffers = 0;
    explicit operator bool() const {
        return replaced || inserted_buffers != 0;
    }
};

// Mutation only: no candidate ranking, placement or timing policy. The caller
// owns the OpenDB ECO transaction and must undo it on failure/rejection.
NetRepairEdits applyRepairPlan(const NetRepairPlan& plan);

// Validates and applies buffer edits to OpenDB. Candidate selection and
// acceptance belong to algorithms; this layer has no Resizer dependency.
class OpenDbBufferInserter {
   public:
    static BufferInsertionResult insertBufferAfterDriver(
        odb::dbNet* net, odb::dbMaster* buffer_master,
        const BufferInsertionOptions& options = {});

    static BufferInsertionResult insertBufferBeforeLoad(
        odb::dbObject* load, odb::dbMaster* buffer_master,
        const BufferInsertionOptions& options = {});

    static BufferInsertionResult insertBufferBeforeLoads(
        odb::dbNet* net, const std::vector< odb::dbObject* >& loads,
        odb::dbMaster* buffer_master,
        const BufferInsertionOptions& options = {});
};

// Legalize and validate; protect DNT/explicit exclusions without changing
// their persistent placement status. Throws on failure. May run inside an ECO.
void legalizeRepairPlacement(
    odb::dbBlock* block, dpl::Opendp* placer,
    const std::vector< odb::dbInst* >& protected_instances = {},
    bool incremental = false);
}  // namespace ecc
