// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <unordered_set>
#include <vector>
#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "odb/db.h"
#include "sta/Graph.hh"
#include "sta/TimingRole.hh"

namespace ecc {

// Changes since the last electrical inspection, including rejected ECOs and
// their rollback. IDs of destroyed objects are never dereferenced.
struct RepairAnalysisChanges {
    std::unordered_set<odb::dbNet*> nets;
    std::unordered_set<odb::dbInst*> instances;
    std::unordered_set<unsigned> removed_nets, removed_instances;
    bool ports = false;
};

// Conservatively include all downstream delay arcs (also clock-to-Q), but
// not timing-check arcs. No path sampling or slack threshold limits this cone.
inline std::vector<sta::Vertex*> repairTimingCone(
    sta::dbSta* sta, const std::unordered_set<odb::dbNet*>& nets) {
    auto* graph = sta->ensureGraph();
    auto* network = sta->getDbNetwork();
    std::unordered_set<sta::Vertex*> visited;
    std::vector<sta::Vertex*> queue;
    const auto add = [&](sta::Vertex* vertex) {
        if(vertex && visited.insert(vertex).second) queue.push_back(vertex);
    };
    const auto pin_vertices = [&](const sta::Pin* pin) {
        sta::Vertex *load = nullptr, *driver = nullptr;
        graph->pinVertices(pin, load, driver);
        add(load); add(driver);
    };
    for(auto* net : nets) {
        for(auto* pin : net->getITerms()) pin_vertices(network->dbToSta(pin));
        for(auto* pin : net->getBTerms()) pin_vertices(network->dbToSta(pin));
    }
    for(size_t i = 0; i < queue.size(); ++i) {
        sta::VertexOutEdgeIterator edges(queue[i], graph);
        while(edges.hasNext()) {
            auto* edge = edges.next();
            if(!edge->role()->isTimingCheck()) add(edge->to(graph));
        }
    }
    return queue;
}
} // namespace ecc
