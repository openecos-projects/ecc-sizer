// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "repair/config.h"
#include "repair/runtime.h"
#include "odb/db.h"
#include <map>
#include <optional>
#include <memory>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>

namespace sta {
class dbSta;
class Vertex;
class dbNetwork;
class LibertyCell;
class LibertyPort;
class TableModel;
class GateTimingModel;
class Pvt;
class Instance;
} // namespace sta
namespace ecc {
struct DesignMetrics {
    unsigned fanout_nets = 0;
    unsigned capacitance_nets = 0;
    unsigned slew_nets = 0;
    unsigned skipped_nets = 0;
    double severity = 0.0;
    double area = 0.0;      // OpenDB area units (DBU squared).
    float setup_wns = 0.0f; // STA internal seconds.
    float hold_wns = 0.0f;
    float setup_tns = 0.0f;
    float hold_tns = 0.0f;
    unsigned total_slew_violations = 0;
    unsigned total_cap_violations = 0;
    unsigned total_fanout_violations = 0;
};

// Changes since the last electrical inspection, including rejected ECOs and
// their rollback. IDs of destroyed objects are never dereferenced.
struct AnalysisChanges {
    std::unordered_set<odb::dbNet*> nets;
    std::unordered_set<odb::dbInst*> instances;
    std::unordered_set<unsigned> removed_nets, removed_instances;
    bool ports = false;
};

std::vector<sta::Vertex*> timingCone(sta::dbSta* sta,
                                     const std::unordered_set<odb::dbNet*>& nets);
} // namespace ecc

namespace ecc::detail {
constexpr float kInf = std::numeric_limits<float>::infinity();

struct Load {
    odb::dbObject* term;
    float capacitance;
    int x;
    int y;
};
struct NetState {
    odb::dbNet* net;
    odb::dbITerm* driver = nullptr;
    std::vector<Load> loads;
    float capacitance = 0;
    float cap_ratio = 0;
    float slew_ratio = 0;
    float fanout_ratio = 0;
    float slew_limit = kInf;
    float slew = 0;
    float input_slew = 0;
    bool propagated_slew = false;
    bool violating() const {
        return cap_ratio > 1 || slew_ratio > 1 || fanout_ratio > 1;
    }
};
struct CellModel {
    odb::dbMaster* master;
    sta::LibertyCell* cell;
    sta::LibertyPort* input;
    sta::LibertyPort* output;
    double area;
    float max_cap;
};

double area(odb::dbMaster* master);

class DesignAnalysis {
  public:
    DesignAnalysis(odb::dbBlock* block, sta::dbSta* sta,
                   const RepairCommonOptions& options,
                   const DesignConstraints& constraints = {});
    bool resizeExcluded(odb::dbInst* inst) const;

    const std::vector<CellModel>& cellChoices() const { return cells_; }
    bool hasCells() const { return !cells_.empty(); }
    const std::vector<CellModel>& bufferChoices() const { return buffers_; }

    std::vector<NetState> analyze(DesignMetrics& metrics, bool all_nets = false);

    // Only for a phase with immutable SDC/libraries and complete edit-cone
    // invalidation. Global timing is always queried from the live STA engine.
    std::vector<NetState> analyzeIncremental(DesignMetrics& metrics,
                                             AnalysisChanges changes);
    struct AnalysisStatistics {
        size_t inspections = 0, full_scans = 0, net_checks = 0, instance_checks = 0;
    };
    const AnalysisStatistics& analysisStatistics() const {
        return analysis_statistics_;
    }

    float predictedSlew(const CellModel& buffer, float cap, float in_slew) const {
        return predict(buffer, cap, in_slew, true);
    }
    float predictedDelay(const CellModel& buffer, float cap, float in_slew) const {
        return predict(buffer, cap, in_slew, false);
    }
    // A snapshot-local evaluator for a downstream gate whose load is fixed.
    // Preserve OpenSTA's interpolation and PVT scaling, but avoid arc lookup,
    // hashing and the unused output-slew table on every candidate/load pair.
    class DelayPrediction {
      public:
        float operator()(float in_slew) const;

      private:
        friend class DesignAnalysis;
        struct Arc {
            const sta::LibertyCell* cell = nullptr;
            const sta::TableModel* table = nullptr;
            const sta::GateTimingModel* fallback = nullptr;
            std::array<float, 3> values{};
            std::array<bool, 3> slew_axis{};
        };
        std::vector<Arc> arcs_;
        const sta::Pvt* pvt_ = nullptr;
        float cap_ = 0;
    };
    DelayPrediction prepareDelayPrediction(const CellModel& buffer, float cap) const;
    CellModel driverModel(const NetState& state) const;
    float predict(const CellModel& buffer, float cap, float in_slew,
                  bool want_slew) const {
        const auto [delay, slew] = predictedTiming(buffer, cap, in_slew);
        return want_slew ? slew : delay;
    }
    void beginTimingPredictionEpoch() {
        ++prediction_epoch_;
        prediction_cache_active_ = true;
    }
    void endTimingPredictionEpoch() {
        prediction_cache_active_ = false;
        // Keep only this candidate snapshot's working set. Reuse exact table
        // queries across successive ECOs without retaining the entire search.
        std::erase_if(predictions_, [&](const auto& entry) {
            return entry.second.epoch != prediction_epoch_;
        });
    }
    // Liberty interpolation returns both values. Path forecasts need both;
    // sharing this evaluation avoids traversing/interpolating each arc twice.
    std::pair<float, float> predictedTiming(const CellModel& buffer, float cap,
                                            float in_slew) const;

    const CellModel* select(float cap, float slew_limit, float in_slew) const;

    // VT identity is explicit and independent of library names or VT ordering.
    // Require the same drive stem, footprint and characterized PVT.
    bool isVtSwap(odb::dbMaster* from, odb::dbMaster* to) const;

    bool protectedInputs(odb::dbInst* inst) const;

  private:
    std::optional<NetState> analyzeNet(odb::dbNet* net, DesignMetrics& metrics);
    DesignMetrics pinCounts(const sta::Instance* inst);
    bool analysis_initialized_ = false;
    AnalysisStatistics analysis_statistics_;
    std::map<unsigned, std::pair<std::optional<NetState>, DesignMetrics>>
        analysis_nets_;
    std::map<unsigned, DesignMetrics> analysis_instances_;
    DesignMetrics analysis_ports_;
    // Library models are immutable during a repair phase. Exact input bits,
    // model ports, scene and operating condition identify a pure table query;
    // no quantization or estimated timing state is reused.
    struct PredictionKey {
        const void *cell, *input, *output, *scene, *pvt;
        uint32_t cap, slew;
        bool operator==(const PredictionKey&) const = default;
    };
    struct PredictionHash {
        size_t operator()(const PredictionKey& key) const {
            size_t value = 0;
            for (const auto* pointer :
                 {key.cell, key.input, key.output, key.scene, key.pvt}) {
                value = std::rotl(value, 1) ^ std::hash<const void*>{}(pointer);
            }
            return std::rotl(value, 1) ^
                   std::hash<uint64_t>{}((uint64_t(key.cap) << 32) | key.slew);
        }
    };
    struct Prediction {
        std::pair<float, float> value;
        size_t epoch;
    };
    mutable std::unordered_map<PredictionKey, Prediction, PredictionHash> predictions_;
    size_t prediction_epoch_ = 0;
    bool prediction_cache_active_ = false;
    odb::dbBlock* block_;
    sta::dbSta* sta_;
    sta::dbNetwork* network_;
    const RepairCommonOptions& opts_;
    DesignConstraints constraints_;
    std::unordered_set<odb::dbInst*> excluded_;
    std::vector<CellModel> buffers_;
    std::vector<CellModel> cells_;
};
inline float tnsTolerance(float baseline, const RepairTolerances& tolerances) {
    return std::max(tolerances.tns_absolute,
                    std::abs(baseline) * tolerances.tns_relative);
}
inline bool timingNonRegressing(const DesignMetrics& before, const DesignMetrics& after,
                                const RepairTolerances& tolerances) {
    return after.setup_wns + tolerances.wns_absolute >= before.setup_wns &&
           after.hold_wns + tolerances.wns_absolute >= before.hold_wns &&
           after.setup_tns + tnsTolerance(before.setup_tns, tolerances) >=
               before.setup_tns &&
           after.hold_tns + tnsTolerance(before.hold_tns, tolerances) >=
               before.hold_tns;
}
inline bool noNewDrv(const DesignMetrics& before, const DesignMetrics& after) {
    return after.total_slew_violations <= before.total_slew_violations &&
           after.total_cap_violations <= before.total_cap_violations &&
           after.total_fanout_violations <= before.total_fanout_violations;
}
} // namespace ecc::detail
