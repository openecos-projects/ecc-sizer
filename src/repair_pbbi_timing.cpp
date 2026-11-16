// SPDX-License-Identifier: Apache-2.0
#include "repair_pbbi_timing.h"
#include <algorithm>
#include <queue>
#include <stdexcept>
namespace ecc::pbbi {
TimingProjection propagate(const std::vector< TimingVertex >& ref,
                           const std::vector< TimingEdge >& edges,
                           const std::vector< TimingVertex >& launch) {
    TimingProjection p;
    if(!launch.empty() && launch.size() != ref.size())
        throw std::invalid_argument("PBBI launch size mismatch");
    p.vertices = launch.empty() ? ref : launch;
    p.incoming.resize(ref.size());
    p.outgoing.resize(ref.size());
    p.predecessor.resize(ref.size(), {-1, -1});
    p.predecessor_rf.resize(ref.size());
    std::vector< unsigned > indegree(ref.size());
    for(unsigned i = 0; i < edges.size(); ++i) {
        const auto& e = edges[i];
        if(e.from >= ref.size() || e.to >= ref.size())
            throw std::invalid_argument("PBBI forecast edge outside graph");
        p.incoming[e.to].push_back(i);
        p.outgoing[e.from].push_back(i);
        ++indegree[e.to];
    }
    std::queue< unsigned > queue;
    for(unsigned v = 0; v < ref.size(); ++v)
        if(!indegree[v])
            queue.push(v);
    while(!queue.empty()) {
        unsigned v = queue.front();
        queue.pop();
        p.order.push_back(v);
        for(unsigned ei : p.outgoing[v])
            if(--indegree[edges[ei].to] == 0)
                queue.push(edges[ei].to);
    }
    if(p.order.size() != ref.size())
        throw std::invalid_argument(
            "PBBI forecast requires acyclic combinational graph");
    std::vector< std::array< ArcResponse, 2 > > projected(edges.size());
    for(unsigned v : p.order) {
        std::array< double, 2 > old_at{{-INFINITY, -INFINITY}}, at = old_at,
                                                                slew{{0, 0}};
        for(unsigned ei : p.incoming[v]) {
            const auto& e = edges[ei];
            for(unsigned in = 0; in < 2; ++in) {
                auto& r = projected[ei][in];
                r = e.predict ? e.predict(p.vertices[e.from].slew[in], in)
                              : e.reference[in];
                for(unsigned out = 0; out < 2; ++out) {
                    if(!std::isfinite(e.reference[in].delay[out]) ||
                       !std::isfinite(r.delay[out]))
                        continue;
                    if(!std::isfinite(r.slew[out]) || r.slew[out] < 0)
                        throw std::invalid_argument(
                            "PBBI forecast invalid slew");
                    old_at[out] =
                        std::max(old_at[out], ref[e.from].arrival[in] +
                                                  e.reference[in].delay[out]);
                    double a = p.vertices[e.from].arrival[in] + r.delay[out];
                    if(a > at[out]) {
                        at[out] = a;
                        p.predecessor[v][out] = int(ei);
                        p.predecessor_rf[v][out] = in;
                    }
                    slew[out] = std::max(slew[out], r.slew[out]);
                }
            }
        }
        for(unsigned rf = 0; rf < 2; ++rf)
            if(std::isfinite(old_at[rf]) && std::isfinite(ref[v].arrival[rf])) {
                // Preserve constraints/tags lost by graph projection at the
                // reference operating point. A no-change forecast is identity.
                p.vertices[v].arrival[rf] =
                    ref[v].arrival[rf] + (at[rf] - old_at[rf]);
                p.vertices[v].slew[rf] = slew[rf];
            }
    }
    for(auto it = p.order.rbegin(); it != p.order.rend(); ++it) {
        unsigned v = *it;
        for(unsigned in = 0; in < 2; ++in) {
            double old_q = INFINITY, q = INFINITY;
            for(unsigned ei : p.outgoing[v]) {
                const auto& e = edges[ei];
                for(unsigned out = 0; out < 2; ++out) {
                    if(!std::isfinite(e.reference[in].delay[out]) ||
                       !std::isfinite(projected[ei][in].delay[out]))
                        continue;
                    old_q = std::min(old_q, ref[e.to].required[out] -
                                                e.reference[in].delay[out]);
                    q = std::min(q, p.vertices[e.to].required[out] -
                                        projected[ei][in].delay[out]);
                }
            }
            if(std::isfinite(old_q) && std::isfinite(ref[v].required[in]))
                p.vertices[v].required[in] = ref[v].required[in] + (q - old_q);
        }
    }
    return p;
}
TimingGraph::TimingGraph(std::vector< TimingVertex > reference,
                         std::vector< TimingEdge > edges)
    : reference_(std::move(reference)), edges_(std::move(edges)) {
    for(auto& edge : edges_)
        edge.predict = {};
    // Validate bounds and acyclicity using the same graph construction as the
    // full AT/RAT projection. Reference edges have no candidate callbacks.
    auto topology = propagate(reference_, edges_);
    order_ = std::move(topology.order);
    incoming_ = std::move(topology.incoming);
}

std::vector< TimingVertex > TimingGraph::forward(
    const std::vector< TimingVertex >& launch, const Evaluator& evaluate) const {
    if(!launch.empty() && launch.size() != reference_.size())
        throw std::invalid_argument("PBBI launch size mismatch");
    auto vertices = launch.empty() ? reference_ : launch;
    for(unsigned v : order_) {
        std::array< double, 2 > old_at{{-INFINITY, -INFINITY}}, at = old_at,
                                                                slew{{0, 0}};
        for(unsigned ei : incoming_[v]) {
            const auto& e = edges_[ei];
            for(unsigned in = 0; in < 2; ++in) {
                auto r = evaluate ? evaluate(ei, vertices[e.from].slew[in], in)
                                  : e.reference[in];
                for(unsigned out = 0; out < 2; ++out) {
                    if(!std::isfinite(e.reference[in].delay[out]) ||
                       !std::isfinite(r.delay[out]))
                        continue;
                    if(!std::isfinite(r.slew[out]) || r.slew[out] < 0)
                        throw std::invalid_argument("PBBI forecast invalid slew");
                    old_at[out] = std::max(old_at[out],
                        reference_[e.from].arrival[in] + e.reference[in].delay[out]);
                    at[out] = std::max(at[out], vertices[e.from].arrival[in] + r.delay[out]);
                    slew[out] = std::max(slew[out], r.slew[out]);
                }
            }
        }
        for(unsigned rf = 0; rf < 2; ++rf)
            if(std::isfinite(old_at[rf]) &&
               std::isfinite(reference_[v].arrival[rf])) {
                vertices[v].arrival[rf] = reference_[v].arrival[rf] + (at[rf] - old_at[rf]);
                vertices[v].slew[rf] = slew[rf];
            }
    }
    return vertices;
}
}  // namespace ecc::pbbi
