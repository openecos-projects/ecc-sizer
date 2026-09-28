// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#ifndef ECC_SIZER_BUFFER_INSERTER_H_
#define ECC_SIZER_BUFFER_INSERTER_H_

#include <optional>
#include <string>
#include <vector>

namespace odb {
class dbInst;
class dbITerm;
class dbMaster;
class dbNet;
class dbObject;
}  // namespace odb

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

// Owns Sizer's buffer insertion policy while using OpenDB as the current
// netlist storage backend. It deliberately has no dependency on rsz::Resizer.
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

}  // namespace ecc

#endif  // ECC_SIZER_BUFFER_INSERTER_H_
