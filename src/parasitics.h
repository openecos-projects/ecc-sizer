// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <stdexcept>
#include <unordered_set>
#include <vector>
#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "est/EstimateParasitics.h"
#include "odb/db.h"
#include "odb/dbBlockCallBackObj.h"
#include "sta/Parasitics.hh"
#include "sta/Scene.hh"
#include "sta/Graph.hh"
#include "sta/TimingRole.hh"
#include "design_analysis.h"

namespace ecc {

// Estimator callbacks track topology, but not all placement changes. In
// particular undoEco restores orientation flags without a move callback.
// Forward moves do notify us. Keep watching every moved/created instance
// for this updater's lifetime, including after a commit, so silent orientation
// restores during undoEco are detected without scanning untouched instances.
// Compare against the LAST PARASITIC UPDATE, not the last committed design.
class PlacementParasiticUpdater {
  public:
    struct Statistics {
        unsigned updates = 0;
        size_t watched_instances = 0, updated_nets = 0, refreshed_vertices = 0;
        size_t analysis_vertices = 0, analysis_slew_changes = 0;
        unsigned verifications = 0;
        double geometry_ms = 0, rc_ms = 0, sta_ms = 0;
    };
    const Statistics& statistics() const { return statistics_; }
    PlacementParasiticUpdater(odb::dbBlock* block, sta::dbSta* sta,
                              est::EstimateParasitics* estimator)
        : block_(block), sta_(sta), estimator_(requirePlacement(estimator)),
          guard_(estimator_) {
        if (!block_ || !sta_) {
            throw std::invalid_argument("missing placement RC backend");
        }
        changes_.addOwner(block_);
    }
    void update() {
        auto start = std::chrono::steady_clock::now();
        auto next = capture();
        statistics_.geometry_ms += elapsed(start);
        statistics_.watched_instances += next.size();
        statistics_.updated_nets += changes_.nets.size();
        ++statistics_.updates;
        start = std::chrono::steady_clock::now();
        auto* network = sta_->getDbNetwork();
        auto* library = network->defaultLibertyLibrary();
        network->sta::Network::clear();
        network->setDefaultLibertyLibrary(library);
        for (auto id : changes_.nets) {
            auto* net = odb::dbNet::getNet(block_, id);
            auto* sta_net = network->dbToSta(net);
            // makeSteinerTree can return no tree (e.g. an unplaced terminal).
            // A full estimate starts empty; an incremental estimate must also
            // erase the old network/reduced model instead of retaining it.
            for (auto* scene : sta_->scenes()) {
                auto* max = scene->parasitics(sta::MinMax::max());
                auto* min = scene->parasitics(sta::MinMax::min());
                if (max) {
                    max->deleteParasitics(sta_net);
                }
                if (min && min != max) {
                    min->deleteParasitics(sta_net);
                }
            }
            estimator_->parasiticsInvalid(net);
        }
        guard_.update();
        statistics_.rc_ms += elapsed(start);
        start = std::chrono::steady_clock::now();
        sta_->findRequireds();
        statistics_.sta_ms += elapsed(start);
        for (const auto& [id, placement] : next) {
            if (id >= placements_.size()) {
                placements_.resize(id + 1);
            }
            placements_[id] = placement;
        }
        changes_.nets.clear();
    }
    void verifyTiming() {
        const auto start = std::chrono::steady_clock::now();
        invalidateTimingCone();
        sta_->findRequireds();
        statistics_.sta_ms += elapsed(start);
        ++statistics_.verifications;
        changes_.timing_nets.clear();
    }
    RepairAnalysisChanges takeAnalysisChanges() {
        sta_->findRequireds();
        auto changed = std::move(changes_.analysis);
        changes_.analysis = {};
        auto* network = sta_->getDbNetwork();
        auto* graph = sta_->ensureGraph();
        if (!analysis_slews_initialized_ ||
            analysis_slew_count_ != graph->slewCount()) {
            analysis_slew_count_ = graph->slewCount();
            analysis_slew_pins_.clear();
            analysis_slews_.clear();
            sta::VertexIterator vertices(graph);
            while (vertices.hasNext()) {
                analysisSlewChanged(vertices.next());
            }
            analysis_slews_initialized_ = true;
            // Initialize both caches from the same STA snapshot. A changed
            // analysis-point layout also requires a full electrical refresh.
            for (auto* net : block_->getNets()) {
                changed.nets.insert(net);
            }
            for (auto* inst : block_->getInsts()) {
                changed.instances.insert(inst);
            }
            changed.ports = true;
            return changed;
        }
        // Physical edits can change capacitance/fanout even when the computed
        // slew is identical. Always inspect all pins on those nets, including
        // the original driver, and every explicitly changed instance.
        for (auto* net : changed.nets) {
            for (auto* pin : net->getITerms()) {
                changed.instances.insert(pin->getInst());
            }
            if (!net->getBTerms().empty()) {
                changed.ports = true;
            }
        }
        for (auto* vertex : repairTimingCone(sta_, changed.nets)) {
            // Arrival/required changes alone do not alter electrical checks.
            // Compare every rise/fall and analysis-point slew exactly; this is
            // not a tolerance-based propagation cutoff. SDC/libs are immutable
            // during the incremental inspection phase.
            if (!analysisSlewChanged(vertex)) {
                continue;
            }
            auto* object = network->staToDb(vertex->pin());
            if (!object) {
                continue;
            }
            if (object->getObjectType() == odb::dbITermObj) {
                auto* pin = static_cast<odb::dbITerm*>(object);
                changed.instances.insert(pin->getInst());
                if (pin->getNet()) {
                    changed.nets.insert(pin->getNet());
                }
            } else if (object->getObjectType() == odb::dbBTermObj) {
                auto* pin = static_cast<odb::dbBTerm*>(object);
                changed.ports = true;
                if (pin->getNet()) {
                    changed.nets.insert(pin->getNet());
                }
            }
        }
        // NetState also reads the input slew of the driver instance. Include
        // its other outputs even if a disabled timing arc stopped propagation.
        for (auto* inst : changed.instances) {
            for (auto* pin : inst->getITerms()) {
                if (pin->getNet() && pin->getIoType() != odb::dbIoType::INPUT) {
                    changed.nets.insert(pin->getNet());
                }
            }
        }
        return changed;
    }
    PlacementParasiticUpdater(const PlacementParasiticUpdater&) = delete;
    PlacementParasiticUpdater& operator=(const PlacementParasiticUpdater&) = delete;

  private:
    bool analysisSlewChanged(sta::Vertex* vertex) {
        auto* graph = sta_->graph();
        const size_t id = graph->id(vertex);
        if (id >= analysis_slew_pins_.size()) {
            analysis_slew_pins_.resize(id + 1, nullptr);
            analysis_slews_.resize((id + 1) * analysis_slew_count_);
        }
        bool changed = analysis_slew_pins_[id] != vertex->pin();
        analysis_slew_pins_[id] = vertex->pin();
        for (size_t i = 0; i < analysis_slew_count_; ++i) {
            auto& previous = analysis_slews_[id * analysis_slew_count_ + i];
            const float current = float(graph->slew(vertex, i));
            changed |= current != previous;
            previous = current;
        }
        ++statistics_.analysis_vertices;
        statistics_.analysis_slew_changes += changed;
        return changed;
    }
    void invalidateTimingCone() {
        // OpenSTA may stop incremental slew propagation at fuzzy equality.
        // Tiny retained errors can change strict aggregate DRV acceptance.
        // Explicitly refresh the affected delay cone, including rollback
        // nets, without discarding parasitics or timing outside that cone.
        auto* graph = sta_->ensureGraph();
        auto* network = sta_->getDbNetwork();
        std::unordered_set<sta::Vertex*> visited;
        std::vector<sta::Vertex*> queue;
        const auto enqueue = [&](sta::Vertex* vertex) {
            if (vertex && visited.insert(vertex).second) {
                queue.push_back(vertex);
            }
        };
        const auto pin_vertices = [&](const sta::Pin* pin) {
            sta::Vertex *load = nullptr, *driver = nullptr;
            graph->pinVertices(pin, load, driver);
            enqueue(load);
            enqueue(driver);
        };
        for (auto id : changes_.timing_nets) {
            auto* net = odb::dbNet::getNet(block_, id);
            for (auto* pin : net->getITerms()) {
                pin_vertices(network->dbToSta(pin));
            }
            for (auto* pin : net->getBTerms()) {
                pin_vertices(network->dbToSta(pin));
            }
        }
        while (!queue.empty()) {
            auto* vertex = queue.back();
            queue.pop_back();
            sta_->delaysInvalidFrom(vertex);
            sta::VertexOutEdgeIterator edges(vertex, graph);
            while (edges.hasNext()) {
                auto* edge = edges.next();
                if (!edge->role()->isTimingCheck()) {
                    enqueue(edge->to(graph));
                }
            }
        }
        statistics_.refreshed_vertices += visited.size();
    }
    static double elapsed(std::chrono::steady_clock::time_point start) {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - start)
            .count();
    }
    struct NetChanges : odb::dbBlockCallBackObj {
        std::unordered_set<unsigned> nets;
        RepairAnalysisChanges analysis;
        // Include prior rejected edits until a cone verification succeeds;
        // otherwise their small residual slew errors could survive rollback.
        std::unordered_set<unsigned> timing_nets;
        std::unordered_set<odb::dbInst*> geometry;
        void touch(odb::dbNet* net) {
            if (net && !net->isSpecial() &&
                net->getSigType() != odb::dbSigType::POWER &&
                net->getSigType() != odb::dbSigType::GROUND) {
                analysis.nets.insert(net);
                nets.insert(net->getId());
                timing_nets.insert(net->getId());
            }
        }
        void inDbNetCreate(odb::dbNet* net) override { touch(net); }
        void inDbNetDestroy(odb::dbNet* net) override {
            analysis.nets.erase(net);
            analysis.removed_nets.insert(net->getId());
            nets.erase(net->getId());
            timing_nets.erase(net->getId());
        }
        void inDbITermPostConnect(odb::dbITerm* pin) override {
            touch(pin->getNet());
            analysis.instances.insert(pin->getInst());
        }
        void inDbITermPostDisconnect(odb::dbITerm* pin, odb::dbNet* net) override {
            touch(net);
            analysis.instances.insert(pin->getInst());
        }
        void inDbBTermPostConnect(odb::dbBTerm* pin) override {
            touch(pin->getNet());
            analysis.ports = true;
        }
        void inDbBTermPostDisConnect(odb::dbBTerm*, odb::dbNet* net) override {
            touch(net);
            analysis.ports = true;
        }
        void inDbInstCreate(odb::dbInst* inst) override {
            geometry.insert(inst);
            analysis.instances.insert(inst);
        }
        void inDbInstDestroy(odb::dbInst* inst) override {
            geometry.erase(inst);
            analysis.instances.erase(inst);
            analysis.removed_instances.insert(inst->getId());
        }
        void inDbPostMoveInst(odb::dbInst* inst) override {
            geometry.insert(inst);
            analysis.instances.insert(inst);
        }
        void inDbInstSwapMasterAfter(odb::dbInst* inst) override {
            geometry.insert(inst);
            analysis.instances.insert(inst);
            for (auto* pin : inst->getITerms()) {
                touch(pin->getNet());
            }
        }
    };
    struct Placement {
        int x = 0, y = 0;
        odb::dbOrientType orientation;
        odb::dbMaster* master = nullptr;
        bool operator==(const Placement& other) const {
            return x == other.x && y == other.y && orientation == other.orientation &&
                   master == other.master;
        }
    };
    static est::EstimateParasitics* requirePlacement(est::EstimateParasitics* ep) {
        if (!ep || ep->getParasiticsSrc() != est::ParasiticsSrc::kPlacement) {
            throw std::invalid_argument(
                "incremental repair requires a placement RC seed");
        }
        return ep;
    }
    std::vector<std::pair<unsigned, Placement>> capture() {
        std::vector<std::pair<unsigned, Placement>> next;
        next.reserve(changes_.geometry.size());
        for (auto* inst : changes_.geometry) {
            int x, y;
            inst->getLocation(x, y);
            Placement placement{x, y, inst->getOrient(), inst->getMaster()};
            const auto id = inst->getId();
            if (id >= placements_.size() || !(placements_[id] == placement)) {
                for (auto* pin : inst->getITerms()) {
                    changes_.touch(pin->getNet());
                }
            }
            next.emplace_back(id, placement);
        }
        return next;
    }
    odb::dbBlock* block_;
    sta::dbSta* sta_;
    est::EstimateParasitics* estimator_;
    est::IncrementalParasiticsGuard guard_;
    NetChanges changes_;
    std::vector<Placement> placements_;
    bool analysis_slews_initialized_ = false;
    size_t analysis_slew_count_ = 0;
    std::vector<const sta::Pin*> analysis_slew_pins_;
    std::vector<float> analysis_slews_;
    Statistics statistics_;
};

} // namespace ecc
