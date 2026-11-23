// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <vector>

namespace odb {
class dbInst;
class dbMaster;
} // namespace odb
namespace ecc {
struct PathBufferTrial;
struct PathFanoutCheck;
struct PathProgress;
struct RepairAnalysisChanges;

// Object references belong to the current design, never to serialized policy.
struct DesignConstraints {
    std::vector<odb::dbMaster*> excluded_masters;
    std::vector<odb::dbInst*> excluded_instances;
    // Protect masters during compound recovery without hiding timing cones.
    std::vector<odb::dbInst*> excluded_resize_instances;
};

// refresh remains an explicit required argument of repair/run. prepare is
// optional and owned by the caller; solvers receive only DesignConstraints.
struct RepairRuntime {
    DesignConstraints constraints;
    std::function<void()> prepare;
};
struct PathRuntime : RepairRuntime {
    // A rejected trial may still have deferred timing refresh.
    std::function<void(unsigned, const PathBufferTrial&)> trial_observer;
    std::function<void(const PathFanoutCheck&)> fanout_observer;
    // Checkpoint notification, not a timer thread. Observers must not mutate
    // the design or query live STA; the snapshot uses last accepted metrics.
    std::function<void(const PathProgress&)> progress_observer;
    // Full RC/STA verification, including after final legalization.
    std::function<void()> verification_refresh;
    std::function<void()> incremental_verification;
    std::function<RepairAnalysisChanges()> analysis_changes;
};
} // namespace ecc
