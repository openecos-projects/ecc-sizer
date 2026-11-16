// SPDX-License-Identifier: Apache-2.0
#include "repair_pbbi_electrical.h"
#include "repair_pbbi_timing.h"
#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "odb/db.h"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/SearchPred.hh"
#include "sta/Search.hh"
#include "sta/PathEnd.hh"
#include "sta/Path.hh"
#include "sta/VisitPathEnds.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingModel.hh"
#include "sta/TimingRole.hh"
#include <algorithm>
#include <tuple>
#include <stdexcept>
namespace ecc {
namespace {
double finiteTime(float t) {
    return t >= sta::INF ? INFINITY : t <= -sta::INF ? -INFINITY : double(t);
}
class EndpointSlackVisitor : public sta::PathEndVisitor {
   public:
    explicit EndpointSlackVisitor(sta::dbSta* sta) : sta_(sta) {}
    sta::PathEndVisitor* copy() const override { return new EndpointSlackVisitor(*this); }
    void visit(sta::PathEnd* end) override {
        auto* path = end->path();
        if(end->isUnconstrained() || path->scene(sta_) != sta_->cmdScene()
           || path->minMax(sta_) != sta::MinMax::max()) return;
        const auto rf = path->transition(sta_)->index();
        slack[rf] = std::min(slack[rf], finiteTime(float(end->slack(sta_))));
    }
    std::array<double, 2> slack{INFINITY, INFINITY};
   private:
    sta::dbSta* sta_;
};
}  // namespace
std::array<double, 2> pbbiEndpointSlacks(sta::dbSta* sta, sta::Vertex* vertex) {
    if(vertex->hasFanout()) {
        EndpointSlackVisitor visitor(sta);
        sta->search()->visitPathEnds()->visitPathEnds(vertex, &visitor);
        return visitor.slack;
    }
    std::array<double, 2> slack;
    for(unsigned rf = 0; rf < 2; ++rf)
        slack[rf] = finiteTime(float(sta->slack(
            vertex, sta::RiseFall::find(rf)->asRiseFallBoth(),
            sta::SceneSeq{sta->cmdScene()}, sta::MinMax::max())));
    return slack;
}
struct PbbiElectricalModel::Impl {
    struct Pin {
        std::string name;
        odb::dbObject* object = nullptr;
        odb::dbNet* net = nullptr;
        odb::dbInst* inst = nullptr;
        std::string port;
        bool driver = false;
        double slew_limit = INFINITY, cap_limit = INFINITY;
        std::array< double, 2 > clock_slew{{0, 0}};
    };
    struct Arc {
        pbbi::TimingEdge edge;
        std::string input, output;
        pbbi::GateResponse original;
        std::array< pbbi::ArcResponse, 2 > operating;
    };
    struct Input {
        odb::dbNet* net;
        std::string port;
        double cap;
    };
    sta::Scene* scene;
    const sta::Pvt* pvt;
    std::vector< Pin > pins;
    std::vector< pbbi::TimingVertex > reference;
    std::vector< Arc > arcs;
    std::unique_ptr< pbbi::TimingGraph > graph;
    std::unordered_map< odb::dbNet*, double > caps;
    std::unordered_map< odb::dbInst*, odb::dbMaster* > masters;
    std::unordered_map< odb::dbInst*, std::vector< Input > > inputs;
    std::unordered_map< odb::dbMaster*, sta::LibertyCell* > cells;
    std::map< std::tuple< odb::dbMaster*, std::string, std::string >,
              pbbi::GateResponse >
        responses;
    std::vector< bool > endpoint;
    std::vector< std::array< double, 2 > > endpoint_slack;
    PbbiElectricalRisk baseline;
    pbbi::GateResponse response(odb::dbMaster* master, const std::string& input,
                                const std::string& output) {
        const auto key = std::make_tuple(master, input, output);
        auto found = responses.find(key);
        if(found != responses.end())
            return found->second;
        auto* cell = cells.at(master);
        if(!cell)
            return {};
        struct Model {
            const sta::GateTimingModel* model;
            unsigned in, out;
        };
        std::vector< Model > models;
        for(auto* set : cell->timingArcSets()) {
            if(!set->from() || !set->to() || set->to()->name() != output ||
               (!input.empty() && set->from()->name() != input))
                continue;
            for(auto* a : set->arcs()) {
                auto* model = a->gateModel(scene, sta::MinMax::max());
                auto* in = a->fromEdge()->asRiseFall();
                auto* out = a->toEdge()->asRiseFall();
                if(model && in && out)
                    models.push_back(
                        {model, unsigned(in->index()), unsigned(out->index())});
            }
        }
        pbbi::GateResponse result;
        if(!models.empty())
            result = [models = std::move(models), pvt = pvt,
                      sequential = cell->hasSequentials()](
                         double cap, double slew, unsigned rf) {
                pbbi::ArcResponse r;
                for(const auto& m : models)
                    if(sequential || m.in == rf) {
                        float d = 0, s = 0;
                        m.model->gateDelay(pvt, float(slew), float(cap), d, s);
                        r.delay[m.out] = std::max(r.delay[m.out], double(d));
                        r.slew[m.out] = std::max(r.slew[m.out], double(s));
                    }
                return r;
            };
        responses.emplace(key, result);
        return result;
    }
};
PbbiElectricalModel::PbbiElectricalModel(odb::dbBlock* block, sta::dbSta* sta)
    : impl_(std::make_unique< Impl >()) {
    if(!block || !sta || sta->getDbNetwork()->block() != block)
        throw std::invalid_argument(
            "PBBI electrical snapshot requires matching DB/STA");
    sta->checkSlewsPreamble();
    sta->checkCapacitancesPreamble(sta->scenes());
    auto& m = *impl_;
    auto* network = sta->getDbNetwork();
    auto* graph = sta->graph();
    m.scene = sta->cmdScene();
    m.pvt = sta->cmdMode()->sdc()->operatingConditions(sta::MinMax::max());
    for(auto* lib : block->getDataBase()->getLibs())
        for(auto* master : lib->getMasters())
            m.cells[master] = network->libertyCell(network->dbToSta(master));
    for(auto* inst : block->getInsts()) {
        m.masters[inst] = inst->getMaster();
        auto* cell = m.cells[inst->getMaster()];
        if(!cell)
            continue;
        for(auto* pin : inst->getITerms())
            if(pin->getNet() && pin->getIoType() == odb::dbIoType::INPUT) {
                auto* port =
                    cell->findLibertyPort(pin->getMTerm()->getConstName());
                if(port)
                    m.inputs[inst].push_back(
                        {pin->getNet(), port->name(),
                         port->capacitance(sta::MinMax::max())});
            }
    }
    const sta::SceneSeq scenes{m.scene};
    std::unordered_map< sta::Vertex*, unsigned > ids;
    std::vector< sta::Vertex* > vertices;
    sta::VertexIterator vi(graph);
    while(vi.hasNext()) {
        auto* v = vi.next();
        if(v->isRegClk() || v->isCheckClk())
            continue;
        const auto* pin = v->pin();
        Impl::Pin info;
        info.name = network->pathName(pin);
        info.object = network->staToDb(pin);
        info.driver = v->isDriver(network);
        if(info.object && info.object->getObjectType() == odb::dbITermObj) {
            auto* p = static_cast< odb::dbITerm* >(info.object);
            info.net = p->getNet();
            info.inst = p->getInst();
            info.port = p->getMTerm()->getName();
        }
        else if(info.object && info.object->getObjectType() == odb::dbBTermObj)
            info.net = static_cast< odb::dbBTerm* >(info.object)->getNet();
        pbbi::TimingVertex ref;
        for(unsigned rf = 0; rf < 2; ++rf) {
            auto* t = sta::RiseFall::find(rf)->asRiseFallBoth();
            ref.arrival[rf] = finiteTime(
                float(sta->arrival(v, t, scenes, sta::MinMax::max())));
            ref.required[rf] = finiteTime(
                float(sta->required(v, t, scenes, sta::MinMax::max())));
            ref.slew[rf] = std::max(
                0.0f, float(sta->slew(v, t, scenes, sta::MinMax::max())));
        }
        float limit = 0, slack = 0, cap = 0;
        sta::Slew slew;
        const sta::RiseFall* rf = nullptr;
        const sta::Scene* sc = nullptr;
        sta->checkSlew(pin, scenes, sta::MinMax::max(), false, slew, limit,
                       slack, rf, sc);
        if(sc && limit > 0 && limit < sta::INF)
            info.slew_limit = limit;
        if(info.driver && info.net) {
            m.caps[info.net] = sta->graphDelayCalc()->loadCap(
                pin, m.scene, sta::MinMax::max());
            sta->checkCapacitance(pin, scenes, sta::MinMax::max(), cap, limit,
                                  slack, rf, sc);
            if(sc && limit > 0 && limit < sta::INF)
                info.cap_limit = limit;
            if(info.inst && m.cells[m.masters.at(info.inst)] &&
               m.cells[m.masters.at(info.inst)]->hasSequentials()) {
                for(auto* set :
                    m.cells[m.masters.at(info.inst)]->timingArcSets())
                    if(set->from() && set->from()->isClock() && set->to() &&
                       set->to()->name() == info.port) {
                        auto* cp =
                            info.inst->findITerm(set->from()->name().c_str());
                        if(cp)
                            for(unsigned t = 0; t < 2; ++t)
                                info.clock_slew[t] = std::max(
                                    0.0f, float(sta->slew(
                                              graph->pinLoadVertex(
                                                  network->dbToSta(cp)),
                                              sta::RiseFall::find(t)
                                                  ->asRiseFallBoth(),
                                              scenes, sta::MinMax::max())));
                    }
            }
        }
        ids[v] = vertices.size();
        vertices.push_back(v);
        m.pins.push_back(std::move(info));
        m.reference.push_back(ref);
    }
    m.endpoint.resize(vertices.size(), true);
    sta::SearchPred1 predicate(sta);
    const auto ap = m.scene->dcalcAnalysisPtIndex(sta::MinMax::max());
    for(unsigned i = 0; i < vertices.size(); ++i) {
        auto* v = vertices[i];
        sta::VertexOutEdgeIterator ei(v, graph);
        while(ei.hasNext()) {
            auto* e = ei.next();
            auto target = ids.find(e->to(graph));
            if(target == ids.end() ||
               (!e->isWire() &&
                e->role() != sta::TimingRole::combinational()) ||
               !predicate.searchFrom(v) || !predicate.searchTo(e->to(graph)) ||
               !predicate.searchThru(e))
                continue;
            Impl::Arc arc;
            arc.edge.from = i;
            arc.edge.to = target->second;
            m.endpoint[i] = false;
            for(auto* a : e->timingArcSet()->arcs()) {
                auto* in = a->fromEdge()->asRiseFall();
                auto* out = a->toEdge()->asRiseFall();
                if(!in || !out)
                    continue;
                auto& r = arc.edge.reference[in->index()];
                r.delay[out->index()] =
                    std::max(r.delay[out->index()],
                             double(float(graph->arcDelay(e, a, ap))));
                r.slew[out->index()] =
                    m.reference[target->second].slew[out->index()];
            }
            if(!e->isWire()) {
                const auto& p = m.pins[target->second];
                arc.input = m.pins[i].port;
                arc.output = p.port;
                if(p.inst)
                    arc.original =
                        m.response(m.masters.at(p.inst), arc.input, arc.output);
            }
            if(arc.original)
                for(unsigned rf = 0; rf < 2; ++rf)
                    arc.operating[rf] =
                        arc.original(m.caps[m.pins[arc.edge.to].net],
                                     m.reference[i].slew[rf], rf);
            m.arcs.push_back(std::move(arc));
        }
    }
    std::vector< pbbi::TimingEdge > edges;
    edges.reserve(m.arcs.size());
    for(const auto& arc : m.arcs)
        edges.push_back(arc.edge);
    m.graph = std::make_unique< pbbi::TimingGraph >(m.reference, std::move(edges));
    m.endpoint_slack.resize(vertices.size(), {INFINITY, INFINITY});
    for(unsigned i = 0; i < vertices.size(); ++i) {
        const auto& ref = m.reference[i];
        if(m.endpoint[i]) {
            const double slack = std::min(ref.required[0] - ref.arrival[0],
                                          ref.required[1] - ref.arrival[1]);
            if(std::isfinite(slack)) {
                ++m.baseline.graph_endpoints;
                m.baseline.graph_wns = std::min(m.baseline.graph_wns, slack);
                m.baseline.graph_tns += std::min(0.0, slack);
            }
        }
        m.endpoint[i] = sta->search()->isEndpoint(vertices[i], sta->cmdMode());
        if(!m.endpoint[i]) continue;
        m.endpoint_slack[i] = pbbiEndpointSlacks(sta, vertices[i]);
        const double slack = std::min(m.endpoint_slack[i][0], m.endpoint_slack[i][1]);
        if(std::isfinite(slack)) {
            ++m.baseline.endpoints;
            m.baseline.baseline_wns = std::min(m.baseline.baseline_wns, slack);
            m.baseline.baseline_tns += std::min(0.0, slack);
        }
    }
}
PbbiElectricalModel::~PbbiElectricalModel() = default;
PbbiElectricalRisk PbbiElectricalModel::predict(
    const std::vector< std::pair< odb::dbInst*, odb::dbMaster* > >& sizes,
    const std::unordered_map< odb::dbNet*, PbbiNetResponse >& nets,
    bool capture_pins) {
    auto& m = *impl_;
    PbbiElectricalRisk result = m.baseline;
    std::unordered_map< odb::dbInst*, odb::dbMaster* > changed(sizes.begin(),
                                                               sizes.end());
    std::unordered_map< odb::dbNet*, double > loads;
    for(const auto& [inst, master] : sizes) {
        auto* cell = m.cells.at(master);
        for(const auto& input : m.inputs[inst]) {
            auto* port =
                cell ? cell->findLibertyPort(input.port.c_str()) : nullptr;
            if(!port) {
                ++result.unsupported;
                continue;
            }
            auto [it, added] = loads.emplace(input.net, m.caps[input.net]);
            it->second += port->capacitance(sta::MinMax::max()) - input.cap;
        }
    }
    for(const auto& [net, response] : nets)
        loads[net] = response.cap;
    auto cap = [&](odb::dbNet* net) {
        auto it = loads.find(net);
        return it == loads.end() ? m.caps[net] : std::max(0.0, it->second);
    };
    const auto evaluate = [&](unsigned ei, double slew, unsigned rf) {
        const auto& arc = m.arcs[ei];
        const auto& e = arc.edge;
        const auto& from = m.pins[e.from];
        const auto& to = m.pins[e.to];
        const auto& reference = e.reference;
        const auto& ref_slew = m.reference[e.from].slew;
        auto r = reference[rf];
        if(arc.output.empty()) {
            auto it = nets.find(from.net);
            if(it != nets.end()) {
                auto d = it->second.delta(to.object, slew, rf);
                if(std::isfinite(d.delay[rf]))
                    r.delay[rf] += d.delay[rf];
                r.slew[rf] = std::max(0.0, r.slew[rf] + d.slew[rf]);
            }
            else {
                const double old_cap = m.caps[from.net];
                const double scale = old_cap > 0 ? cap(from.net) / old_cap : 1;
                const double variance = std::max(0.0,
                    r.slew[rf] * r.slew[rf] - ref_slew[rf] * ref_slew[rf]);
                r.delay[rf] *= scale;
                r.slew[rf] = std::max(0.0, r.slew[rf] +
                    std::hypot(slew, std::sqrt(variance) * scale) -
                    std::hypot(ref_slew[rf], std::sqrt(variance)));
            }
        }
        else if(to.inst && arc.original) {
            auto found = changed.find(to.inst);
            const double old_cap = m.caps[to.net], new_cap = cap(to.net);
            if(found == changed.end() && new_cap == old_cap && slew == ref_slew[rf])
                return r;
            // Most arcs retain their Liberty model. Only sized gates need a
            // replacement response; all other arcs use it by reference.
            pbbi::GateResponse resized;
            const auto* response = &arc.original;
            if(found != changed.end()) {
                resized = m.response(found->second, arc.input, arc.output);
                response = &resized;
            }
            if(*response) {
                const auto& before = arc.operating[rf];
                const auto after = (*response)(new_cap, slew, rf);
                for(unsigned out = 0; out < 2; ++out)
                    if(std::isfinite(r.delay[out]) &&
                       std::isfinite(before.delay[out]) &&
                       std::isfinite(after.delay[out])) {
                        r.delay[out] += after.delay[out] - before.delay[out];
                        r.slew[out] = std::max(0.0,
                            r.slew[out] + after.slew[out] - before.slew[out]);
                    }
            }
            else if(rf == 0)
                ++result.unsupported;
        }
        else if(rf == 0)
            ++result.unsupported;
        return r;
    };
    auto launch = m.reference;
    for(unsigned i = 0; i < m.pins.size(); ++i) {
        const auto& p = m.pins[i];
        if(!p.driver || !p.inst || !m.cells[m.masters.at(p.inst)] ||
           !m.cells[m.masters.at(p.inst)]->hasSequentials() ||
           !loads.count(p.net))
            continue;
        auto response = m.response(m.masters.at(p.inst), "", p.port);
        if(!response) {
            ++result.unsupported;
            continue;
        }
        for(unsigned rf = 0; rf < 2; ++rf) {
            auto a = response(m.caps[p.net], p.clock_slew[rf], rf),
                 b = response(cap(p.net), p.clock_slew[rf], rf);
            if(std::isfinite(a.delay[rf]) && std::isfinite(b.delay[rf])) {
                launch[i].arrival[rf] += b.delay[rf] - a.delay[rf];
                launch[i].slew[rf] =
                    std::max(0.0, launch[i].slew[rf] + b.slew[rf] - a.slew[rf]);
            }
        }
    }
    auto projected = m.graph->forward(launch, evaluate);
    double after_wns = 0;
    for(unsigned i = 0; i < m.pins.size(); ++i) {
        const auto& p = m.pins[i];
        const auto& now = projected[i];
        const auto& old = m.reference[i];
        if(capture_pins)
            result.slew_by_pin[p.name] = std::max(now.slew[0], now.slew[1]);
        double limit = p.slew_limit;
        if(p.inst && changed.count(p.inst)) {
            auto* cell = m.cells.at(changed.at(p.inst));
            auto* port = cell ? cell->findLibertyPort(p.port.c_str()) : nullptr;
            if(port) {
                float v = 0;
                bool exists = false;
                port->slewLimit(sta::MinMax::max(), v, exists);
                if(exists && v > 0)
                    limit = std::min(limit, double(v));
            }
        }
        const double excess =
            std::max(now.slew[0], now.slew[1]) / limit -
            std::max(1.0, std::max(old.slew[0], old.slew[1]) / limit);
        if(excess > 0) {
            ++result.slew;
            result.severity += excess;
        }
        if(p.driver && p.net) {
            double limit = p.cap_limit;
            if(p.inst && changed.count(p.inst)) {
                auto* cell = m.cells.at(changed.at(p.inst));
                auto* port =
                    cell ? cell->findLibertyPort(p.port.c_str()) : nullptr;
                if(port) {
                    float v = 0;
                    bool exists = false;
                    port->capacitanceLimit(sta::MinMax::max(), v, exists);
                    if(exists && v > 0)
                        limit = std::min(limit, double(v));
                }
            }
            double excess =
                cap(p.net) / limit - std::max(1.0, m.caps[p.net] / limit);
            if(excess > 0) {
                ++result.cap;
                result.severity += excess;
            }
        }
        if(m.endpoint[i]) {
            if(now.arrival != old.arrival || now.slew != old.slew)
                result.affected_endpoints.push_back(p.name);
            const auto& slack = m.endpoint_slack[i];
            const double a = std::min(slack[0], slack[1]);
            auto projected_slack = [&](unsigned rf) {
                // Anchor to a real tagged STA slack, not independently
                // reduced arrival/required values from incompatible tags.
                return std::isfinite(old.arrival[rf]) && std::isfinite(now.arrival[rf])
                           ? slack[rf] - (now.arrival[rf] - old.arrival[rf])
                           : slack[rf];
            };
            const double b = std::min(projected_slack(0), projected_slack(1));
            if(capture_pins && std::isfinite(b))
                result.endpoint_slack_by_pin[p.name] = b;
            if(std::isfinite(a) && std::isfinite(b)) {
                after_wns = std::min(after_wns, b);
                result.tns_delta += std::min(0.0, b) - std::min(0.0, a);
            }
        }
    }
    result.wns_delta = after_wns - result.baseline_wns;
    return result;
}
}  // namespace ecc
