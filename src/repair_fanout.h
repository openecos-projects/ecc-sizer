// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0

#ifndef ECC_SIZER_REPAIR_FANOUT_H_
#define ECC_SIZER_REPAIR_FANOUT_H_

#include <string>
#include <vector>

#include "buffer_inserter.h"

namespace odb {
class dbBlock;
class dbInst;
class dbMaster;
class dbNet;
}  // namespace odb

namespace ecc {

struct FanoutRepairOptions {
    unsigned max_fanout = 64;
    unsigned max_levels = 32;
    bool repair_clock_nets = false;
    std::vector< odb::dbNet* > excluded_nets;
    BufferInsertionOptions insertion;
};

enum class FanoutRepairStatus {
    kSuccess,
    kInvalidBlock,
    kInvalidBufferMaster,
    kInvalidMaxFanout,
    kInsertionFailed,
    kMaxLevelsExceeded,
};

struct FanoutRepairResult {
    FanoutRepairStatus status = FanoutRepairStatus::kInsertionFailed;
    std::string message;
    unsigned nets_examined = 0;
    unsigned violating_nets = 0;
    unsigned before_max_fanout = 0;
    unsigned after_max_fanout = 0;
    std::vector< odb::dbInst* > inserted_buffers;
    std::vector< odb::dbNet* > affected_nets;

    bool ok() const {
        return status == FanoutRepairStatus::kSuccess;
    }
    explicit operator bool() const {
        return ok();
    }
};

class OpenDbFanoutRepairer {
   public:
    static FanoutRepairResult repair(odb::dbBlock* block,
                                     odb::dbMaster* buffer_master,
                                     const FanoutRepairOptions& options = {});

    static unsigned loadCount(odb::dbNet* net);
    static unsigned maxFanout(odb::dbBlock* block,
                              bool include_clock_nets = false);
};

}  // namespace ecc

#endif  // ECC_SIZER_REPAIR_FANOUT_H_
