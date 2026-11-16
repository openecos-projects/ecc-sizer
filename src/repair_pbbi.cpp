// SPDX-License-Identifier: Apache-2.0
#include "repair_pbbi.h"
#include "repair_pbbi_electrical.h"
#include "repair_pbbi_solver.h"
#include "repair_pbbi_schedule.h"
#include "repair_backend.h"
#include "repair_acceptance.h"
#include "BufferedNet.hh"
#include "rsz/Resizer.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "sta/Scene.hh"
#include "sta/SearchPred.hh"
#include "sta/Search.hh"
#include "sta/TimingRole.hh"
#include "repair_pbbi_timing.h"
#include <chrono>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace ecc {
namespace {
using Clock = std::chrono::steady_clock;
struct Timer {
    double& total;
    Clock::time_point start = Clock::now();
    ~Timer() {
        total +=
            std::chrono::duration< double, std::milli >(Clock::now() - start)
                .count();
    }
};
using detail::NetState;
double staTime(float value) {
    if(value >= sta::INF)
        return INFINITY;
    if(value <= -sta::INF)
        return -INFINITY;
    return value;
}
struct Stage {
    const NetState* state;
    odb::dbObject* load;
    odb::dbITerm* input;
};
std::string stageKey(const Stage& stage, sta::dbSta* sta) {
    std::string key = stage.state->net->getName();
    key.push_back('\0');
    if(stage.input)
        key += sta->getDbNetwork()->pathName(
            sta->getDbNetwork()->dbToSta(stage.input));
    key.push_back('\0');
    key += sta->getDbNetwork()->pathName(
        sta->getDbNetwork()->dbToSta(stage.load));
    key.push_back('\0');
    return key;
}
struct Binding {
    odb::dbNet* net = nullptr;
    odb::dbObject* load = nullptr;
    odb::dbInst* gate = nullptr;
    odb::dbITerm* input = nullptr;
    BufferLocation location;
    double sink_output_cap = 0;
    std::array< double, 2 > output_slew{{0, 0}};
    std::vector< odb::dbMaster* > masters;
};
struct Problem {
    std::vector< pbbi::Node > nodes;
    std::vector< Binding > bindings;
    unsigned nets = 0, bounded_offpath_loads = 0;
    double baseline_q = 0;
    std::shared_ptr< const pbbi::Trace > baseline_trace;
    std::array< double, 2 > root_slew{{0, 0}};
    std::vector< double > slew_grid;
};
struct InstOrder {
    bool operator()(const odb::dbInst* a, const odb::dbInst* b) const {
        return a->getId() < b->getId();
    }
};
using SizeMap = std::map< odb::dbInst*, odb::dbMaster*, InstOrder >;
struct Plan {
    std::shared_ptr< Problem > problem;
    pbbi::State state;
    double gain = 0;
    double needed_gain = 0;
    unsigned path_rank = 0;
    std::string key;
    PbbiElectricalRisk electrical;
    std::vector< std::string > actions;
    std::shared_ptr< std::set< unsigned > > origins;
    pbbi::Footprint footprint;
    // Same candidate may retry once after a measured off-path disturbance.
    std::set< std::string > placement_guards;
};

// The surrogate uses the actual pin-to-pin Liberty arcs at the committed input
// slew, linearized in output load. It is a planning model, never acceptance
// STA.
pbbi::Choice model(detail::Repair& backend, sta::dbSta* sta, detail::Buffer b,
                   const char* input, double cap, float slew, double cost,
                   bool edit, double margin) {
    if(input)
        b.input = b.cell->findLibertyPort(input);
    if(input && !b.input)
        throw std::runtime_error("PBBI missing equivalent input port");
    const double d0 = backend.predictedDelay(b, 0, slew);
    const double d1 = backend.predictedDelay(b, float(cap), slew);
    if(!std::isfinite(d0) || !std::isfinite(d1))
        throw std::runtime_error(
            "PBBI unsupported gate timing arc: " + b.master->getName() + "/" +
            (input ? std::string(input) : "*"));
    const double resistance = cap > 0
                                  ? std::max(0.0, (d1 - d0) / cap)
                                  : std::max(0.0f, b.output->driveResistance());
    pbbi::Choice choice{b.input ? b.input->capacitance(sta::MinMax::max()) : 0,
                        d0,
                        resistance,
                        cost,
                        edit ? b.max_cap * (1 - margin) : detail::kInf,
                        edit};
    for(auto item : {std::make_pair(b.input, &choice.input_slew_limit),
                     std::make_pair(b.output, &choice.output_slew_limit)}) {
        if(!item.first)
            continue;
        float limit = 0;
        bool exists = false;
        sta->findSlewLimit(item.first, sta->cmdScene(), sta::MinMax::max(),
                           limit, exists);
        if(exists && limit > 0 && limit < sta::INF)
            *item.second = limit;
    }
    struct Arc {
        const sta::GateTimingModel* model;
        unsigned in, out;
    };
    std::vector< Arc > arcs;
    for(auto* set : b.cell->timingArcSets()) {
        if(!set->from() || (b.input && set->from() != b.input) ||
           set->to() != b.output)
            continue;
        for(auto* a : set->arcs()) {
            auto* m = a->gateModel(sta->cmdScene(), sta::MinMax::max());
            const auto* from = a->fromEdge()->asRiseFall();
            const auto* to = a->toEdge()->asRiseFall();
            if(m && from && to)
                arcs.push_back(
                    {m, unsigned(from->index()), unsigned(to->index())});
        }
    }
    const auto* pvt =
        sta->cmdMode()->sdc()->operatingConditions(sta::MinMax::max());
    const bool sequential = b.cell->hasSequentials();
    choice.response = [arcs = std::move(arcs), pvt, sequential](
                          double cap, double slew, unsigned in) {
        pbbi::ArcResponse r;
        for(const auto& a : arcs)
            if(sequential || a.in == in) {
                float d = 0, s = 0;
                a.model->gateDelay(pvt, float(slew), float(cap), d, s);
                if(!std::isfinite(d) || !std::isfinite(s))
                    throw std::runtime_error("PBBI nonfinite Liberty response");
                r.delay[a.out] = std::max(r.delay[a.out], double(d));
                r.slew[a.out] = std::max(r.slew[a.out], double(s));
            }
        return r;
    };
    return choice;
}

using PinTiming = std::unordered_map< const sta::Pin*, pbbi::TimingVertex >;
struct Sites {
    odb::Rect core;
    std::vector< odb::Rect > blocked;
    explicit Sites(odb::dbBlock* block) : core(block->getCoreArea()) {
        for(auto* b : block->getBlockages())
            if(!b->isSoft())
                blocked.push_back(b->getBBox()->getBox());
        for(auto* i : block->getInsts())
            if(i->isFixed() || i->isBlock())
                blocked.push_back(i->getBBox()->getBox());
    }
    bool legal(const BufferLocation& p, odb::dbMaster* m) const {
        odb::Rect r(p.x, p.y, p.x + int(m->getWidth()),
                    p.y + int(m->getHeight()));
        if(!core.contains(r))
            return false;
        for(const auto& b : blocked)
            if(r.overlaps(b))
                return false;
        return true;
    }
};

class Builder {
   public:
    Builder(detail::Repair& backend, sta::dbSta* sta, rsz::Resizer* resizer,
            const std::vector< NetState >& states,
            const PbbiRepairOptions& options,
            const std::vector< Stage >& stages, const Sites* sites = nullptr,
            const PinTiming* forecast = nullptr, bool estimate = false)
        : backend_(backend),
          sta_(sta),
          resizer_(resizer),
          options_(options),
          stages_(stages),
          sites_(sites),
          forecast_(forecast),
          estimate_(estimate) {
        if(options.sink_sizing)
            for(const auto& state : states)
                if(state.driver)
                    driven_[state.driver->getInst()] = &state;
        for(const auto& stage : stages_) {
            path_gates_.insert(stage.state->driver->getInst());
            for(const auto& load : stage.state->loads)
                if(load.term->getObjectType() == odb::dbITermObj)
                    ++occurrences_[static_cast< odb::dbITerm* >(load.term)
                                       ->getInst()];
        }
    }
    std::shared_ptr< Problem > build() {
        problem_ = std::make_shared< Problem >();
        problem_->nets = stages_.size();
        stage(0);
        problem_->root_slew = {stages_[0].state->input_slew,
                               stages_[0].state->input_slew};
        auto* root_input = stages_[0].input;
        auto original = backend_.driverModel(*stages_[0].state);
        if(original.cell->hasSequentials()) {
            root_input = nullptr;
            for(auto* set : original.cell->timingArcSets())
                if(set->from() && set->from()->isClock() &&
                   set->to() == original.output)
                    root_input = stages_[0].state->driver->getInst()->findITerm(
                        set->from()->name().c_str());
        }
        if(root_input) {
            auto* pin = sta_->getDbNetwork()->dbToSta(root_input);
            auto* v = sta_->graph()->pinLoadVertex(pin);
            for(unsigned rf = 0; rf < 2; ++rf)
                problem_->root_slew[rf] = std::max(
                    0.0f,
                    float(sta_->slew(
                        v, sta::RiseFall::find(rf)->asRiseFallBoth(),
                        sta::SceneSeq{sta_->cmdScene()}, sta::MinMax::max())));
        }
        double bound = std::max(problem_->root_slew[0], problem_->root_slew[1]);
        for(const auto& n : problem_->nodes) {
            if(std::isfinite(n.slew_limit))
                bound = std::max(bound, n.slew_limit);
            for(const auto& c : n.choices)
                for(double x : {c.input_slew_limit, c.output_slew_limit})
                    if(std::isfinite(x))
                        bound = std::max(bound, x);
        }
        if(bound <= 0)
            throw std::runtime_error("PBBI no finite slew scale");
        double smallest = bound;
        for(const auto& n : problem_->nodes)
            for(const auto& c : n.choices)
                if(c.response)
                    for(unsigned rf = 0; rf < 2; ++rf) {
                        auto r = c.response(0, 0, rf);
                        for(double x : r.slew)
                            if(x > 0 && std::isfinite(x))
                                smallest = std::min(smallest, x);
                    }
        for(double x : problem_->root_slew)
            if(x > 0)
                smallest = std::min(smallest, x);
        // A uniform grid across a loose library maximum loses the entire
        // useful ps-scale slew range. Resolve the library's intrinsic slew
        // scale logarithmically and include exact root operating points.
        problem_->slew_grid.push_back(0);
        if(options_.slew_samples == 2)
            problem_->slew_grid.push_back(bound);
        else
            for(unsigned i = 0; i + 1 < options_.slew_samples; ++i)
                problem_->slew_grid.push_back(
                    smallest *
                    std::pow(bound / smallest,
                             double(i) / (options_.slew_samples - 2)));
        for(double v : problem_->root_slew)
            problem_->slew_grid.push_back(v);
        std::sort(problem_->slew_grid.begin(), problem_->slew_grid.end());
        problem_->slew_grid.erase(
            std::unique(problem_->slew_grid.begin(), problem_->slew_grid.end()),
            problem_->slew_grid.end());
        return problem_;
    }

   private:
    int add(pbbi::Node node, Binding binding) {
        problem_->nodes.push_back(std::move(node));
        problem_->bindings.push_back(std::move(binding));
        return int(problem_->nodes.size()) - 1;
    }
    std::vector< detail::Buffer > sizes(const NetState& state, bool enabled) {
        std::vector< detail::Buffer > out{backend_.driverModel(state)};
        auto* inst = state.driver->getInst();
        if(!enabled || !options_.common.allow_gate_resize ||
           backend_.protectedInputs(inst))
            return out;
        for(bool down : {false, true})
            for(const auto* b :
                backend_.resizeCandidates(state, down, false, false))
                out.push_back(*b);
        return out;
    }
    double required(const sta::Pin* pin) {
        auto* vertex = sta_->graph()->pinLoadVertex(pin);
        if(!vertex)
            return detail::kInf;
        const sta::SceneSeq scenes{sta_->cmdScene()};
        return staTime(
            float(sta_->required(vertex, sta::RiseFallBoth::riseFall(), scenes,
                                 sta::MinMax::max())));
    }
    int stage(size_t index) {
        const auto& s = stages_.at(index);
        auto tree = resizer_->makeBufferedNetSteiner(
            sta_->getDbNetwork()->dbToSta(s.state->driver), sta_->cmdScene());
        if(!tree)
            throw std::runtime_error("PBBI missing Steiner tree");
        const int child = treeNode(tree, index);
        auto original = backend_.driverModel(*s.state);
        // Sequential/clock cells terminate the path. Their existing driver
        // contributes load-dependent delay, but cannot be resized here.
        const bool combinational = original.cell &&
                                   !original.cell->hasSequentials() &&
                                   !original.cell->isClockGate();
        if(!combinational && index != 0)
            throw std::runtime_error("PBBI crossed sequential boundary");
        pbbi::Node node;
        node.kind = pbbi::Kind::gate;
        node.left = child;
        Binding binding;
        binding.gate = s.state->driver->getInst();
        binding.net = s.state->net;
        for(unsigned rf = 0; rf < 2; ++rf) {
            auto* transition = sta::RiseFall::find(rf)->asRiseFallBoth();
            binding.output_slew[rf] = std::max(
                0.0f, float(sta_->slew(
                          sta_->graph()->pinDrvrVertex(
                              sta_->getDbNetwork()->dbToSta(s.state->driver)),
                          transition, sta::SceneSeq{sta_->cmdScene()},
                          sta::MinMax::max())));
            node.reference_slew[rf] =
                s.input
                    ? std::max(0.0f,
                               float(sta_->slew(
                                   sta_->graph()->pinLoadVertex(
                                       sta_->getDbNetwork()->dbToSta(s.input)),
                                   transition, sta::SceneSeq{sta_->cmdScene()},
                                   sta::MinMax::max())))
                    : s.state->input_slew;
        }
        binding.input = s.input;
        const char* name = combinational && s.input
                               ? s.input->getMTerm()->getConstName()
                               : nullptr;
        for(auto b : sizes(*s.state, combinational && options_.gate_sizing)) {
            const bool edit = b.master != binding.gate->getMaster();
            node.choices.push_back(
                model(backend_, sta_, b, name, s.state->capacitance,
                      s.state->input_slew, b.area - original.area, edit,
                      options_.common.margin));
            binding.masters.push_back(b.master);
        }
        return add(std::move(node), std::move(binding));
    }
    int treeNode(const rsz::BufferedNetPtr& tree, size_t index) {
        const auto& s = stages_[index];
        pbbi::Node node;
        Binding binding;
        binding.net = s.state->net;
        binding.location = {tree->location().x(), tree->location().y()};
        switch(tree->type()) {
            case rsz::BufferedNetType::load: {
                auto* term = sta_->getDbNetwork()->staToDb(tree->loadPin());
                if(!term)
                    throw std::runtime_error("PBBI unmapped sink");
                if(term == s.load && index + 1 < stages_.size())
                    return stage(index + 1);
                binding.load = term;
                node.cap = tree->cap();
                node.required = required(tree->loadPin());
                auto* load_vertex =
                    sta_->graph()->pinLoadVertex(tree->loadPin());
                std::array< double, 2 > measured_input_slew{{0, 0}};
                for(unsigned rf = 0; rf < 2; ++rf) {
                    auto* transition =
                        sta::RiseFall::find(rf)->asRiseFallBoth();
                    node.required_rf[rf] = staTime(float(sta_->required(
                        load_vertex, transition,
                        sta::SceneSeq{sta_->cmdScene()}, sta::MinMax::max())));
                    measured_input_slew[rf] = std::max(
                        0.0f, float(sta_->slew(load_vertex, transition,
                                               sta::SceneSeq{sta_->cmdScene()},
                                               sta::MinMax::max())));
                }
                const auto committed_required = node.required_rf;
                node.reference_slew = measured_input_slew;
                sta::Slew measured;
                float limit = 0, slack_limit = 0;
                const sta::RiseFall* transition = nullptr;
                const sta::Scene* scene = nullptr;
                sta_->checkSlew(tree->loadPin(),
                                sta::SceneSeq{sta_->cmdScene()},
                                sta::MinMax::max(), false, measured, limit,
                                slack_limit, transition, scene);
                if(scene && limit > 0 && limit < sta::INF)
                    node.slew_limit = limit;
                if(forecast_) {
                    auto it = forecast_->find(tree->loadPin());
                    if(it != forecast_->end()) {
                        node.required_rf = it->second.required;
                        node.required =
                            std::min(node.required_rf[0], node.required_rf[1]);
                    }
                }
                if(estimate_) {
                    node.required = 0;
                    node.required_rf = {0, 0};
                }
                if(std::isnan(node.required))
                    throw std::runtime_error("PBBI invalid sink RAT");
                if(!estimate_ && term != s.load) {
                    bool bounded = false;
                    for(unsigned rf = 0; rf < 2; ++rf) {
                        const double slack = staTime(float(sta_->slack(
                            load_vertex, sta::RiseFall::find(rf)->asRiseFallBoth(),
                            sta::SceneSeq{sta_->cmdScene()}, sta::MinMax::max())));
                        bounded |= std::isfinite(slack) &&
                                   node.required_rf[rf] > committed_required[rf];
                        node.required_rf[rf] = pbbi::offPathRequired(
                            node.required_rf[rf], committed_required[rf], slack);
                    }
                    problem_->bounded_offpath_loads += bounded;
                    node.required = std::min(node.required_rf[0], node.required_rf[1]);
                }
                if(term->getObjectType() == odb::dbITermObj &&
                   options_.sink_sizing) {
                    auto* pin = static_cast< odb::dbITerm* >(term);
                    auto* inst = pin->getInst();
                    const auto next = driven_.find(inst);
                    // One owner per physical gate: shared sinks/path gates
                    // cannot independently select conflicting sizes or pay cost
                    // twice.
                    if(!path_gates_.count(inst) && occurrences_[inst] == 1 &&
                       next != driven_.end()) {
                        const auto& state = *next->second;
                        const auto original = backend_.driverModel(state);
                        if(!original.cell || original.cell->hasSequentials() ||
                           original.cell->isClockGate())
                            return add(std::move(node), std::move(binding));
                        auto base = model(backend_, sta_, original,
                                          pin->getMTerm()->getConstName(),
                                          state.capacitance, state.input_slew,
                                          0, false, options_.common.margin);
                        binding.gate = inst;
                        binding.input = pin;
                        for(auto b : sizes(state, options_.gate_sizing)) {
                            auto c = model(backend_, sta_, b,
                                           pin->getMTerm()->getConstName(),
                                           state.capacitance, state.input_slew,
                                           b.area - original.area,
                                           b.master != inst->getMaster(),
                                           options_.common.margin);
                            if(state.capacitance > c.max_cap)
                                continue;
                            // A faster sink gate makes its input RAT later; the
                            // selected input cap propagates through the
                            // upstream tree.
                            c.intrinsic =
                                (base.intrinsic +
                                 base.resistance * state.capacitance) -
                                (c.intrinsic +
                                 c.resistance * state.capacitance);
                            const auto base_response = base.response;
                            const auto sized_response = c.response;
                            const double output_cap = state.capacitance;
                            binding.sink_output_cap = output_cap;
                            const auto reference_slew = measured_input_slew;
                            const auto rat = node.required_rf;
                            const double output_limit = c.output_slew_limit;
                            c.sink_required = [base_response, sized_response,
                                               output_cap, reference_slew, rat,
                                               output_limit](double slew,
                                                             unsigned rf) {
                                auto original = base_response(
                                    output_cap, reference_slew[rf], rf);
                                auto changed =
                                    sized_response(output_cap, slew, rf);
                                double q = INFINITY;
                                bool seen = false;
                                for(unsigned out = 0; out < 2; ++out)
                                    if(std::isfinite(changed.delay[out]) &&
                                       std::isfinite(original.delay[out])) {
                                        seen = true;
                                        if(changed.slew[out] > output_limit)
                                            return -double(INFINITY);
                                        q = std::min(
                                            q, rat[rf] + original.delay[out] -
                                                   changed.delay[out]);
                                    }
                                return seen ? q : -double(INFINITY);
                            };
                            node.choices.push_back(c);
                            binding.masters.push_back(b.master);
                        }
                    }
                }
                return add(std::move(node), std::move(binding));
            }
            case rsz::BufferedNetType::junction:
                node.kind = pbbi::Kind::junction;
                node.left = treeNode(tree->ref(), index);
                node.right = treeNode(tree->ref2(), index);
                break;
            case rsz::BufferedNetType::wire: {
                node.kind = pbbi::Kind::wire;
                node.left = treeNode(tree->ref(), index);
                double r = 0, c = 0;
                tree->wireRC(sta_->cmdScene(), resizer_,
                             resizer_->getEstimateParasitics(), r, c);
                const double length = resizer_->dbuToMeters(tree->length());
                node.cap = length * c;
                node.resistance = length * r;
                auto* lib =
                    backend_.driverModel(*s.state).cell->libertyLibrary();
                for(unsigned rf = 0; rf < 2; ++rf) {
                    auto* transition = sta::RiseFall::find(rf);
                    double low = lib->slewLowerThreshold(transition),
                           high = lib->slewUpperThreshold(transition);
                    if(low >= 0 && high > low && high < 1)
                        node.wire_slew_factor[rf] =
                            rf == 0 ? std::log((1 - low) / (1 - high))
                                    : (low > 0 ? std::log(high / low)
                                               : std::log(9.0));
                }
                // Both zero-length branch points and nonzero wire starts are
                // legal candidate sites; final placement feasibility is
                // verified.
                for(const auto& b : backend_.bufferChoices()) {
                    if(sites_ && !sites_->legal(binding.location, b.master))
                        continue;
                    node.choices.push_back(
                        model(backend_, sta_, b, b.input->name().c_str(),
                              s.state->capacitance, s.state->input_slew, b.area,
                              true, options_.common.margin));
                    binding.masters.push_back(b.master);
                }
                break;
            }
            default:
                throw std::runtime_error("PBBI unsupported Steiner node");
        }
        if(node.kind == pbbi::Kind::wire || node.kind == pbbi::Kind::junction)
            for(unsigned rf = 0; rf < 2; ++rf)
                node.reference_slew[rf] = std::max(
                    0.0f,
                    float(sta_->slew(
                        sta_->graph()->pinDrvrVertex(
                            sta_->getDbNetwork()->dbToSta(s.state->driver)),
                        sta::RiseFall::find(rf)->asRiseFallBoth(),
                        sta::SceneSeq{sta_->cmdScene()}, sta::MinMax::max())));
        return add(std::move(node), std::move(binding));
    }
    detail::Repair& backend_;
    sta::dbSta* sta_;
    rsz::Resizer* resizer_;
    const PbbiRepairOptions& options_;
    const std::vector< Stage >& stages_;
    const Sites* sites_;
    const PinTiming* forecast_;
    bool estimate_;
    std::unordered_map< odb::dbInst*, const NetState* > driven_;
    std::unordered_map< odb::dbInst*, unsigned > occurrences_;
    std::unordered_set< odb::dbInst* > path_gates_;
    std::shared_ptr< Problem > problem_;
};

struct NetPrediction {
    std::shared_ptr< Problem > problem;
    std::shared_ptr< const pbbi::Trace > trace;
    double cap = 0;
    std::unordered_map< odb::dbObject*, pbbi::LeafTiming > baseline;
    std::array< double, 2 > cached_slew{{-1, -1}};
    std::array< std::unordered_map< odb::dbObject*, pbbi::LeafTiming >, 2 >
        cached;
    pbbi::LeafTiming evaluate(odb::dbObject* sink, double slew, unsigned rf) {
        if(cached_slew[rf] != slew) {
            auto timing =
                pbbi::simulate(problem->nodes, trace, {slew, slew}, false);
            cached[rf].clear();
            for(const auto& leaf : timing.leaves)
                cached[rf].emplace(problem->bindings[leaf.node].load, leaf);
            cached_slew[rf] = slew;
        }
        return cached[rf].at(sink);
    }
};
struct Forecast {
    PinTiming pins;
    std::vector< std::pair< double, std::vector< const sta::Pin* > > > paths;
};
// Exact, invocation-local memoization. Liberty/scene/options stay fixed for
// this repair call. Response identities are the bound master and input pin;
// sink sizing additionally captures output capacitance, reference slew, RAT
// and original master (all included below). Keep full descriptor bytes, not
// a hash, so collisions cannot reuse a trace.
std::string problemKey(const Problem& p) {
    std::string key;
    auto put = [&](const auto& x) {
        key.append(reinterpret_cast< const char* >(&x), sizeof(x));
    };
    for(double v : p.root_slew)
        put(v);
    put(p.slew_grid.size());
    for(double v : p.slew_grid)
        put(v);
    put(p.nodes.size());
    for(size_t i = 0; i < p.nodes.size(); ++i) {
        const auto& n = p.nodes[i];
        const auto& b = p.bindings[i];
        put(n.kind);
        put(n.left);
        put(n.right);
        put(n.cap);
        put(n.resistance);
        put(n.required);
        put(n.slew_limit);
        for(double v : n.required_rf)
            put(v);
        for(double v : n.reference_slew)
            put(v);
        for(double v : n.wire_slew_factor)
            put(v);
        put(b.sink_output_cap);
        put(b.net);
        put(b.load);
        put(b.gate);
        put(b.input);
        auto* original = b.gate ? b.gate->getMaster() : nullptr;
        put(original);
        put(b.location.x);
        put(b.location.y);
        for(double v : b.output_slew)
            put(v);
        put(b.masters.size());
        for(auto* m : b.masters)
            put(m);
        put(n.choices.size());
        for(const auto& c : n.choices) {
            put(c.cap);
            put(c.intrinsic);
            put(c.resistance);
            put(c.cost);
            put(c.max_cap);
            put(c.edit);
            put(c.input_slew_limit);
            put(c.output_slew_limit);
        }
    }
    return key;
}
struct PathMemo {
    std::string key;
    std::shared_ptr< const pbbi::Trace > baseline;
    double baseline_q = 0;
    pbbi::Result solved;
};
using PathCache = std::unordered_map< std::string, PathMemo >;
std::string solveKey(const Problem& p, const pbbi::Limits& limits) {
    auto key = problemKey(p);
    auto put = [&](const auto& x) {
        key.append(reinterpret_cast< const char* >(&x), sizeof(x));
    };
    put(limits.max_states);
    put(limits.max_expansions);
    put(limits.max_cost);
    put(limits.max_buffers);
    put(limits.max_slew_cache_entries);
    put(limits.operating_point_pruning);
    put(limits.slew_grid.size());
    for(double x : limits.slew_grid)
        put(x);
    for(double x : limits.root_slew)
        put(x);
    return key;
}

struct ForecastMemo {
    std::string key;
    std::shared_ptr< const pbbi::Trace > baseline, selected;
    bool exhausted = false;
};
using ForecastCache = std::unordered_map< odb::dbNet*, ForecastMemo >;

Forecast forecastTiming(detail::Repair& backend, sta::dbSta* sta,
                        rsz::Resizer* resizer,
                        const std::vector< NetState >& states,
                        const PbbiRepairOptions& options, const Sites& sites,
                        PbbiRepairResult& result, ForecastCache& cache,
                        const std::function< bool() >& stop) {
    Timer timer{result.timing_ms};
    std::unordered_map< odb::dbNet*, std::shared_ptr< NetPrediction > >
        predictions;
    std::unordered_map< odb::dbNet*, const NetState* > by_net;
    for(const auto& state : states)
        if(state.driver)
            by_net[state.net] = &state;
    auto forecast_options = options;
    forecast_options.gate_sizing = false;
    forecast_options.sink_sizing = false;
    for(const auto& state : states) {
        if(stop && stop())
            return {};
        if(!state.driver)
            continue;
        ++result.timing_nets;
        std::vector< Stage > stage{{&state, nullptr, nullptr}};
        std::shared_ptr< Problem > p;
        try {
            p = Builder(backend, sta, resizer, states, forecast_options, stage,
                        &sites, nullptr, true)
                    .build();
        }
        catch(const std::runtime_error&) {
            ++result.timing_fallback_nets;
            continue;
        }
        ForecastMemo memo;
        if(options.forecast_cache) {
            memo.key = problemKey(*p);
            auto found = cache.find(state.net);
            if(found != cache.end() && found->second.key == memo.key) {
                memo = found->second;
                ++result.forecast_cache_hits;
            }
            else {
                ++result.forecast_cache_misses;
                if(found != cache.end())
                    ++result.forecast_cache_invalidations;
            }
        }
        auto found = cache.find(state.net);
        const bool reuse = options.forecast_cache && found != cache.end() &&
                           found->second.key == memo.key;
        if(!reuse) {
            pbbi::Limits limits;
            limits.stop_requested = stop;
            limits.max_states = options.timing_max_frontier_states;
            limits.max_expansions = options.timing_max_expansions;
            limits.edits = false;
            auto baseline =
                pbbi::solve(p->nodes, int(p->nodes.size()) - 1, limits);
            if(baseline.cancelled)
                return {};
            if(!baseline.frontier.empty()) {
                memo.baseline = baseline.frontier.front().trace;
                auto old = pbbi::simulate(p->nodes, memo.baseline, p->root_slew,
                                          false);
                limits.edits = true;
                // Linear proposals, checked with nonlinear Liberty and slew.
                auto solved =
                    pbbi::solve(p->nodes, int(p->nodes.size()) - 1, limits);
                if(solved.cancelled)
                    return {};
                result.timing_expansions += solved.expansions;
                result.timing_truncated += solved.truncated;
                memo.exhausted = solved.exhausted;
                double best_q = old.required;
                if(!solved.exhausted)
                    for(const auto& candidate : solved.frontier) {
                        auto checked = pbbi::simulate(p->nodes, candidate.trace,
                                                      p->root_slew, true);
                        if(checked.feasible &&
                           checked.required >
                               best_q + options.min_improvement) {
                            memo.selected =
                                candidate.buffers ? candidate.trace : nullptr;
                            best_q = checked.required;
                        }
                    }
            }
            if(options.forecast_cache)
                cache[state.net] = memo;
        }
        if(!memo.baseline) {
            ++result.timing_fallback_nets;
            continue;
        }
        if(memo.exhausted) {
            ++result.timing_exhausted_nets;
            continue;
        }
        if(!memo.selected)
            continue;
        auto old_trace = memo.baseline;
        auto prediction = std::make_shared< NetPrediction >();
        prediction->problem = p;
        prediction->trace = memo.selected->left;
        prediction->cap = memo.selected->left->cap;
        // Forecast graph wire edges start after the real gate output. The
        // gate's new load is handled by its own Liberty arc, never counted
        // twice.
        // The DRV backend's scalar slew is the worst over driver AND loads.
        // Calibrate at the actual driver output, separately for rise/fall,
        // exactly as the timing graph reference below does.
        std::array< double, 2 > output_slew;
        auto* vertex = sta->graph()->pinDrvrVertex(
            sta->getDbNetwork()->dbToSta(state.driver));
        for(unsigned rf = 0; rf < 2; ++rf)
            output_slew[rf] = std::max(
                0.0f, float(sta->slew(
                          vertex, sta::RiseFall::find(rf)->asRiseFallBoth(),
                          sta::SceneSeq{sta->cmdScene()}, sta::MinMax::max())));
        auto before =
            pbbi::simulate(p->nodes, old_trace->left, output_slew, false);
        for(const auto& leaf : before.leaves)
            prediction->baseline.emplace(p->bindings[leaf.node].load, leaf);
        predictions.emplace(state.net, std::move(prediction));
        ++result.timing_buffered_nets;
    }
    for(auto it = cache.begin(); it != cache.end();) {
        if(!by_net.count(it->first)) {
            ++result.forecast_cache_invalidations;
            it = cache.erase(it);
        }
        else
            ++it;
    }
    auto* graph = sta->graph();
    auto* network = sta->getDbNetwork();
    std::vector< sta::Vertex* > vertices;
    std::unordered_map< sta::Vertex*, unsigned > ids;
    std::vector< pbbi::TimingVertex > reference;
    sta::VertexIterator vi(graph);
    const sta::SceneSeq scenes{sta->cmdScene()};
    while(vi.hasNext()) {
        auto* v = vi.next();
        if(v->isRegClk() || v->isCheckClk())
            continue;
        pbbi::TimingVertex x;
        for(unsigned rf = 0; rf < 2; ++rf) {
            const auto* transition = sta::RiseFall::find(rf)->asRiseFallBoth();
            x.arrival[rf] = staTime(
                float(sta->arrival(v, transition, scenes, sta::MinMax::max())));
            x.required[rf] = staTime(float(
                sta->required(v, transition, scenes, sta::MinMax::max())));
            x.slew[rf] = std::max(0.0f, float(sta->slew(v, transition, scenes,
                                                        sta::MinMax::max())));
        }
        ids[v] = vertices.size();
        vertices.push_back(v);
        reference.push_back(x);
    }
    std::vector< pbbi::TimingEdge > edges;
    sta::SearchPred1 predicate(sta);
    const auto ap = sta->cmdScene()->dcalcAnalysisPtIndex(sta::MinMax::max());
    for(unsigned i = 0; i < vertices.size(); ++i) {
        auto* v = vertices[i];
        sta::VertexOutEdgeIterator ei(v, graph);
        while(ei.hasNext()) {
            auto* edge = ei.next();
            auto target = ids.find(edge->to(graph));
            if(target == ids.end() ||
               (!edge->isWire() &&
                edge->role() != sta::TimingRole::combinational()) ||
               !predicate.searchFrom(v) ||
               !predicate.searchTo(edge->to(graph)) ||
               !predicate.searchThru(edge))
                continue;
            pbbi::TimingEdge e;
            e.from = i;
            e.to = target->second;
            for(auto* arc : edge->timingArcSet()->arcs()) {
                auto* in = arc->fromEdge()->asRiseFall();
                auto* out = arc->toEdge()->asRiseFall();
                if(!in || !out)
                    continue;
                double delay = float(graph->arcDelay(edge, arc, ap));
                e.reference[in->index()].delay[out->index()] = std::max(
                    e.reference[in->index()].delay[out->index()], delay);
                e.reference[in->index()].slew[out->index()] =
                    reference[e.to].slew[out->index()];
            }
            const auto original = e.reference;
            if(edge->isWire()) {
                auto* object = network->staToDb(v->pin());
                auto* sink = network->staToDb(edge->to(graph)->pin());
                if(object && object->getObjectType() == odb::dbITermObj) {
                    auto found = predictions.find(
                        static_cast< odb::dbITerm* >(object)->getNet());
                    if(found != predictions.end() &&
                       found->second->baseline.count(sink)) {
                        auto prediction = found->second;
                        e.predict = [prediction, sink, original](double slew,
                                                                 unsigned rf) {
                            auto now = prediction->evaluate(sink, slew, rf);
                            const auto& before = prediction->baseline.at(sink);
                            pbbi::ArcResponse r = original[rf];
                            r.delay[rf] += now.arrival[rf] - before.arrival[rf];
                            r.slew[rf] =
                                std::max(0.0, r.slew[rf] + now.slew[rf] -
                                                  before.slew[rf]);
                            return r;
                        };
                    }
                }
                if(!e.predict) {
                    const auto before = reference[i].slew;
                    e.predict = [original, before](double slew, unsigned rf) {
                        auto r = original[rf];
                        double variance =
                            std::max(0.0, r.slew[rf] * r.slew[rf] -
                                              before[rf] * before[rf]);
                        r.slew[rf] = std::max(
                            0.0,
                            r.slew[rf] + std::hypot(slew, std::sqrt(variance)) -
                                std::hypot(before[rf], std::sqrt(variance)));
                        return r;
                    };
                }
            }
            else {
                auto* output = network->staToDb(edge->to(graph)->pin());
                if(output && output->getObjectType() == odb::dbITermObj) {
                    auto* term = static_cast< odb::dbITerm* >(output);
                    auto state = by_net.find(term->getNet());
                    NetState readonly{};
                    readonly.driver = term;
                    readonly.capacitance = sta->graphDelayCalc()->loadCap(
                        edge->to(graph)->pin(), sta->cmdScene(),
                        sta::MinMax::max());
                    readonly.input_slew =
                        std::max(reference[i].slew[0], reference[i].slew[1]);
                    const NetState* electrical =
                        state == by_net.end() ? &readonly : state->second;
                    {
                        auto gate = backend.driverModel(*electrical);
                        auto* input = network->libertyPort(v->pin());
                        if(input) {
                            try {
                                auto c = model(backend, sta, gate,
                                               input->name().c_str(),
                                               electrical->capacitance,
                                               electrical->input_slew, 0, false,
                                               options.common.margin);
                                double old_cap = electrical->capacitance,
                                       new_cap = old_cap;
                                auto net = predictions.find(term->getNet());
                                if(net != predictions.end())
                                    new_cap = net->second->cap;
                                if(state == by_net.end())
                                    ++result.timing_readonly_arcs;
                                auto response = c.response;
                                auto before = reference[i].slew;
                                e.predict = [response, old_cap, new_cap, before,
                                             original](double slew,
                                                       unsigned rf) {
                                    auto a = response(old_cap, before[rf], rf),
                                         b = response(new_cap, slew, rf),
                                         r = original[rf];
                                    for(unsigned out = 0; out < 2; ++out)
                                        if(std::isfinite(r.delay[out]) &&
                                           std::isfinite(a.delay[out]) &&
                                           std::isfinite(b.delay[out])) {
                                            r.delay[out] +=
                                                b.delay[out] - a.delay[out];
                                            r.slew[out] = std::max(
                                                0.0, r.slew[out] + b.slew[out] -
                                                         a.slew[out]);
                                        }
                                    return r;
                                };
                            }
                            catch(const std::runtime_error&) {
                            }
                        }
                    }
                }
            }
            if(!edge->isWire() && !e.predict)
                ++result.timing_fallback_arcs;
            edges.push_back(std::move(e));
        }
    }
    auto launch = reference;
    for(unsigned i = 0; i < vertices.size(); ++i) {
        auto* object = network->staToDb(vertices[i]->pin());
        if(!object || object->getObjectType() != odb::dbITermObj)
            continue;
        auto* term = static_cast< odb::dbITerm* >(object);
        auto prediction = predictions.find(term->getNet());
        auto state = by_net.find(term->getNet());
        if(prediction == predictions.end() || state == by_net.end() ||
           state->second->driver != term)
            continue;
        auto original = backend.driverModel(*state->second);
        if(!original.cell->hasSequentials())
            continue;
        const auto& root =
            prediction->second->problem->nodes.back().choices.front();
        for(unsigned rf = 0; rf < 2; ++rf) {
            double input = prediction->second->problem->root_slew[rf];
            auto before = root.response(state->second->capacitance, input, rf);
            auto after = root.response(prediction->second->cap, input, rf);
            if(std::isfinite(before.delay[rf]) &&
               std::isfinite(after.delay[rf])) {
                launch[i].arrival[rf] += after.delay[rf] - before.delay[rf];
                launch[i].slew[rf] = std::max(
                    0.0, launch[i].slew[rf] + after.slew[rf] - before.slew[rf]);
            }
        }
    }
    auto projected = pbbi::propagate(reference, edges, launch);
    result.timing_vertices += vertices.size();
    result.timing_edges += edges.size();
    Forecast forecast;
    for(unsigned i = 0; i < vertices.size(); ++i)
        forecast.pins[vertices[i]->pin()] = projected.vertices[i];
    for(unsigned end = 0; end < vertices.size(); ++end) {
        if(!projected.outgoing[end].empty())
            continue;
        for(unsigned rf = 0; rf < 2; ++rf) {
            double slack = projected.vertices[end].required[rf] -
                           projected.vertices[end].arrival[rf];
            if(!std::isfinite(slack) || slack >= 0)
                continue;
            unsigned v = end, transition = rf;
            std::vector< const sta::Pin* > path{vertices[v]->pin()};
            while(projected.predecessor[v][transition] >= 0) {
                unsigned e = projected.predecessor[v][transition];
                unsigned next = projected.predecessor_rf[v][transition];
                v = edges[e].from;
                transition = next;
                path.push_back(vertices[v]->pin());
            }
            std::reverse(path.begin(), path.end());
            forecast.paths.emplace_back(slack, std::move(path));
        }
    }
    std::stable_sort(
        forecast.paths.begin(), forecast.paths.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    return forecast;
}

void edits(const Problem& p, const std::shared_ptr< const pbbi::Trace >& t,
           SizeMap& sizes, std::vector< std::string >& keys) {
    if(!t)
        return;
    edits(p, t->left, sizes, keys);
    edits(p, t->right, sizes, keys);
    const auto& node = p.nodes[t->node];
    const auto& b = p.bindings[t->node];
    if(t->choice < 0)
        return;
    auto* master = b.masters.at(t->choice);
    if(b.gate && node.choices[t->choice].edit) {
        const auto [it, added] = sizes.emplace(b.gate, master);
        if(!added && it->second != master)
            throw std::runtime_error("PBBI conflicting gate sizes");
        keys.push_back("S:" + b.gate->getName() + ":" + master->getName());
    }
    else if(node.kind == pbbi::Kind::wire) {
        // Include subtree identity: distinct branch loads at the same point
        // must not accidentally share an edit key.
        std::vector< std::string > loads;
        std::function< void(int) > visit = [&](int i) {
            const auto& n = p.nodes[i];
            const auto& x = p.bindings[i];
            if(n.kind == pbbi::Kind::gate) {
                if(x.input)
                    loads.push_back(x.gate->getName() + "/" +
                                    x.input->getMTerm()->getName());
                return;
            }
            if(x.load) {
                if(x.load->getObjectType() == odb::dbITermObj) {
                    auto* pin = static_cast< odb::dbITerm* >(x.load);
                    loads.push_back(pin->getInst()->getName() + "/" +
                                    pin->getMTerm()->getName());
                }
                else
                    loads.push_back(
                        static_cast< odb::dbBTerm* >(x.load)->getName());
            }
            if(n.left >= 0)
                visit(n.left);
            if(n.right >= 0)
                visit(n.right);
        };
        visit(t->node);
        std::sort(loads.begin(), loads.end());
        std::string key = "B:" + b.net->getName() + ":" + master->getName() +
                          ":" + std::to_string(b.location.x) + "," +
                          std::to_string(b.location.y);
        for(const auto& load : loads)
            key += ":" + load;
        keys.push_back(std::move(key));
    }
}

// Split a joint path trace at real gate inputs. Each resulting tree models
// only one physical net; sink sizing changes its load cap, not its wire delay.
std::shared_ptr< NetPrediction > netPrediction(
    const Problem& p, const std::shared_ptr< const pbbi::Trace >& root) {
    auto net = std::make_shared< NetPrediction >();
    net->problem = std::make_shared< Problem >();
    std::function< std::shared_ptr< const pbbi::Trace >(
        const std::shared_ptr< const pbbi::Trace >&) >
        copy;
    copy = [&](const auto& t) -> std::shared_ptr< const pbbi::Trace > {
        auto n = p.nodes[t->node];
        auto b = p.bindings[t->node];
        pbbi::Trace trace = *t;
        if(n.kind == pbbi::Kind::gate || n.kind == pbbi::Kind::load) {
            if(n.kind == pbbi::Kind::gate)
                b.load = b.input;
            n.kind = pbbi::Kind::load;
            n.left = n.right = -1;
            n.cap = t->cap;
            n.required = 0;
            n.required_rf = {0, 0};
            n.choices.clear();
            trace.choice = -1;
            trace.left = trace.right = nullptr;
        }
        else {
            trace.left = copy(t->left);
            n.left = trace.left->node;
            if(t->right) {
                trace.right = copy(t->right);
                n.right = trace.right->node;
            }
        }
        trace.node = net->problem->nodes.size();
        net->problem->nodes.push_back(std::move(n));
        net->problem->bindings.push_back(std::move(b));
        return std::make_shared< const pbbi::Trace >(std::move(trace));
    };
    net->trace = copy(root);
    net->cap = root->cap;
    return net;
}
std::unordered_map< odb::dbNet*, PbbiNetResponse > candidateNets(
    const Plan& plan) {
    const auto& p = *plan.problem;
    std::unordered_map< int, std::shared_ptr< const pbbi::Trace > > originals;
    std::function< void(const std::shared_ptr< const pbbi::Trace >&) >
        visit_old;
    visit_old = [&](const auto& t) {
        if(!t)
            return;
        if(p.nodes[t->node].kind == pbbi::Kind::gate)
            originals[t->node] = t;
        visit_old(t->left);
        visit_old(t->right);
    };
    visit_old(p.baseline_trace);
    std::unordered_map< odb::dbNet*, PbbiNetResponse > result;
    std::function< void(const std::shared_ptr< const pbbi::Trace >&) > visit;
    visit = [&](const auto& t) {
        if(!t)
            return;
        if(p.nodes[t->node].kind == pbbi::Kind::gate) {
            const auto& binding = p.bindings[t->node];
            auto prediction = netPrediction(p, t->left);
            auto baseline = netPrediction(p, originals.at(t->node)->left);
            auto before =
                pbbi::simulate(baseline->problem->nodes, baseline->trace,
                               binding.output_slew, false);
            for(const auto& leaf : before.leaves)
                prediction
                    ->baseline[baseline->problem->bindings[leaf.node].load] =
                    leaf;
            PbbiNetResponse r;
            r.cap = prediction->cap;
            r.delta = [prediction](odb::dbObject* sink, double slew,
                                   unsigned rf) {
                pbbi::ArcResponse delta;
                auto found = prediction->baseline.find(sink);
                if(found == prediction->baseline.end())
                    return delta;
                auto now = prediction->evaluate(sink, slew, rf);
                delta.delay[rf] = now.arrival[rf] - found->second.arrival[rf];
                delta.slew[rf] = now.slew[rf] - found->second.slew[rf];
                return delta;
            };
            result[binding.net] = std::move(r);
        }
        visit(t->left);
        visit(t->right);
    };
    visit(plan.state.trace);
    return result;
}

std::vector< odb::dbObject* > applyTree(
    const Problem& p, const std::shared_ptr< const pbbi::Trace >& t,
    std::map< std::string, std::string >* inserted = nullptr) {
    const auto& n = p.nodes[t->node];
    const auto& b = p.bindings[t->node];
    if(n.kind == pbbi::Kind::load)
        return {b.load};
    auto loads = applyTree(p, t->left, inserted);
    if(n.kind == pbbi::Kind::gate)
        return b.input ? std::vector< odb::dbObject* >{b.input}
                       : std::vector< odb::dbObject* >{};
    if(t->right) {
        auto other = applyTree(p, t->right, inserted);
        loads.insert(loads.end(), other.begin(), other.end());
    }
    if(n.kind == pbbi::Kind::wire && t->choice >= 0) {
        BufferInsertionOptions o;
        o.location = b.location;
        o.buffer_name_prefix = "sizer_pbbi_buf";
        o.net_name_prefix = "sizer_pbbi_net";
        auto r = OpenDbBufferInserter::insertBufferBeforeLoads(
            b.net, loads, b.masters.at(t->choice), o);
        if(!r)
            throw std::runtime_error("PBBI insert: " + r.message);
        if(inserted) {
            SizeMap sizes;
            std::vector< std::string > keys;
            edits(p, t, sizes, keys);
            inserted->emplace(r.buffer->getName(), keys.back());
        }
        return {r.input_pin};
    }
    return loads;
}

// Capture values while STA is valid. String identities survive undoEco(),
// including pins on buffers which are destroyed by rollback.
using PinChecks = std::map< std::string, PbbiPinViolation >;
PinChecks electricalPins(odb::dbBlock* block, sta::dbSta* sta) {
    PinChecks values;
    auto* network = sta->getDbNetwork();
    auto check = [&](odb::dbObject* object, odb::dbNet* net,
                     odb::dbMaster* master, bool driver) {
        if(!net || net->getSigType() == odb::dbSigType::POWER ||
           net->getSigType() == odb::dbSigType::GROUND)
            return;
        auto* pin = network->dbToSta(object);
        for(const bool cap : {false, true}) {
            if(cap && !driver)
                continue;
            float value = 0, limit = 0, slack = 0;
            const sta::RiseFall* rf = nullptr;
            const sta::Scene* scene = nullptr;
            if(cap)
                sta->checkCapacitance(pin, sta->scenes(), sta::MinMax::max(),
                                      value, limit, slack, rf, scene);
            else {
                sta::Slew slew;
                sta->checkSlew(pin, sta->scenes(), sta::MinMax::max(), false,
                               slew, limit, slack, rf, scene);
                value = float(slew);
            }
            if(!scene || !(limit > 0) || limit >= sta::INF)
                continue;
            PbbiPinViolation v;
            v.pin = network->pathName(pin);
            v.net = net->getName();
            v.master = master ? master->getName() : "port";
            v.kind = cap ? "cap" : "slew";
            v.transition = rf ? rf->index() : 0;
            v.after = value;
            v.limit = limit;
            const auto key = v.kind + ":" + v.pin;
            values.emplace(key, std::move(v));
        }
    };
    for(auto* inst : block->getInsts())
        for(auto* pin : inst->getITerms())
            check(pin, pin->getNet(), inst->getMaster(),
                  pin->getIoType() == odb::dbIoType::OUTPUT);
    for(auto* pin : block->getBTerms())
        check(pin, pin->getNet(), nullptr,
              pin->getIoType() == odb::dbIoType::INPUT);
    return values;
}
void diagnose(PbbiTrial& trial, const PinChecks& before, const PinChecks& after,
              const Problem& problem, const SizeMap& sizes) {
    std::set< std::string > path_nets, other_inputs, resized_outputs, resized;
    for(const auto& b : problem.bindings)
        if(b.net)
            path_nets.insert(b.net->getName());
    for(const auto& [inst, master] : sizes) {
        resized.insert(inst->getName());
        for(auto* pin : inst->getITerms())
            if(pin->getNet()) {
                if(pin->getIoType() == odb::dbIoType::INPUT)
                    other_inputs.insert(pin->getNet()->getName());
                if(pin->getIoType() == odb::dbIoType::OUTPUT)
                    resized_outputs.insert(pin->getNet()->getName());
            }
    }
    for(const auto& [key, value] : after) {
        if(!(value.after > value.limit))
            continue;
        auto v = value;
        const auto old = before.find(key);
        if(old != before.end()) {
            v.had_before = true;
            v.before = old->second.after;
        }
        const auto slash = v.pin.rfind('/');
        const auto inst = v.pin.substr(0, slash);
        v.relation = !v.had_before                  ? "inserted"
                     : resized.count(inst)          ? "resized"
                     : path_nets.count(v.net)       ? "path_net"
                     : resized_outputs.count(v.net) ? "resized_output_net"
                     : other_inputs.count(v.net)    ? "other_input_net"
                                                    : "outside_path";
        trial.violations.push_back(std::move(v));
    }
}

std::vector< std::string > reject(const DesignRepairMetrics& a,
                                  const DesignRepairMetrics& b, double area,
                                  const PbbiRepairOptions& o,
                                  const pbbi::TimingTarget& target) {
    const auto& t = o.common.tolerances;
    std::vector< std::string > r;
    if(o.objective == "area")
        r = pbbi::setupRejections(
            {a.setup_wns, a.setup_tns, a.area}, {b.setup_wns, b.setup_tns, b.area},
            target, true, o.min_improvement, detail::tnsTolerance(a.setup_tns, t));
    if(!(b.area <= area))
        r.push_back("area");
    // Preserve the established float arithmetic and rejection ordering in
    // timing mode; target-bounded area policy uses the helper above.
    if(o.objective == "timing") {
        if(!(b.setup_wns + t.wns_absolute >= a.setup_wns))
            r.push_back("setup_wns");
        if(!(b.setup_tns + detail::tnsTolerance(a.setup_tns, t) >= a.setup_tns))
            r.push_back("setup_tns");
    }
    if(!(b.hold_wns + t.wns_absolute >= a.hold_wns))
        r.push_back("hold_wns");
    if(!(b.hold_tns + detail::tnsTolerance(a.hold_tns, t) >= a.hold_tns))
        r.push_back("hold_tns");
    if(o.objective == "timing" &&
       !(b.setup_wns > a.setup_wns + o.min_improvement ||
         b.setup_tns > a.setup_tns + o.min_improvement))
        r.push_back("no_setup_gain");
    if(b.total_slew_violations > a.total_slew_violations)
        r.push_back("slew");
    if(b.total_cap_violations > a.total_cap_violations)
        r.push_back("cap");
    if(b.total_fanout_violations > a.total_fanout_violations)
        r.push_back("fanout");
    if(!(b.severity <= a.severity + t.drv_severity))
        r.push_back("drv_severity");
    return r;
}
}  // namespace

PbbiRepairResult OpenDbPbbiRepairer::repair(
    odb::dbBlock* block, sta::dbSta* sta, rsz::Resizer* resizer,
    const std::function< void() >& refresh, const PbbiRepairOptions& options) {
    PbbiRepairResult result;
    const auto start = Clock::now();
    bool dirty = false, active = false;
    auto update = [&](bool rollback) {
        Timer t{result.refresh_ms};
        ++result.refreshes;
        if(rollback)
            ++result.rollback_refreshes;
        refresh();
        dirty = false;
    };
    try {
        options.validate();
        if(!block || !sta || !resizer || !refresh ||
           sta->getDbNetwork()->block() != block)
            throw std::invalid_argument(
                "PBBI requires matching OpenDB/STA and a tree provider");
        // Tree RC conversion requires Resizer's DBU context even when no
        // OpenROAD repair has ever run (native design repair does not init it).
        resizer->initBlock();
        detail::Repair backend(block, sta, options.common);
        Sites sites(block);
        auto inspect = [&](DesignRepairMetrics& m) {
            Timer t{result.measure_ms};
            return backend.analyze(m, true);
        };
        auto states = inspect(result.before);
        result.after = result.before;
        const bool area_mode = options.objective == "area";
        const pbbi::TimingTarget target{
            std::max(double(result.before.setup_wns), options.target_wns),
            std::max(double(result.before.setup_tns),
                     options.target_tns.value_or(result.before.setup_tns)),
            options.common.tolerances.wns_absolute,
            detail::tnsTolerance(
                std::max(double(result.before.setup_tns),
                         options.target_tns.value_or(result.before.setup_tns)),
                options.common.tolerances)};
        result.target_wns = target.wns;
        result.target_tns = target.tns;
        PinChecks pin_baseline;
        if(options.pin_diagnostics || options.failure_feedback || options.placement_feedback) {
            Timer t{result.measure_ms};
            pin_baseline = electricalPins(block, sta);
        }
        const double area_limit =
            result.before.area * (1 + options.max_area_growth);
        std::function< bool() > stop;
        if(options.max_seconds)
            stop = [&]() {
                return std::chrono::duration< double >(Clock::now() - start)
                           .count() >= options.max_seconds;
            };
        auto budget = [&]() {
            return result.attempted < options.max_trials &&
                   (!options.max_seconds ||
                    std::chrono::duration< double >(Clock::now() - start)
                            .count() < options.max_seconds) &&
                   (!options.max_refreshes ||
                    result.refreshes + 2 <= options.max_refreshes);
        };
        ForecastCache forecast_cache;
        PathCache path_cache;
        double last_path_seconds = 0, last_trial_seconds = 0;
        while(budget() && (area_mode || result.after.setup_wns < 0)) {
            const bool area_recovery = area_mode &&
                target.met(result.after.setup_wns, result.after.setup_tns);
            const auto predicts_target = [&](const Plan& plan) {
                return target.met(result.after.setup_wns + plan.electrical.wns_delta,
                                  result.after.setup_tns + plan.electrical.tns_delta);
            };
            std::unordered_map< odb::dbNet*, const NetState* > by_net;
            for(const auto& s : states)
                if(s.driver)
                    by_net[s.net] = &s;
            PathCache next_path_cache;
            struct PreparedPath {
                std::shared_ptr< Problem > problem;
                double slack;
                unsigned rank;
                std::vector< Plan > plans;
                bool solved = false;
                unsigned attempts = 0;
                pbbi::Footprint footprint;
                std::string identity;
                std::set< unsigned > origins;
            };
            std::vector< PreparedPath > prepared;
            std::unordered_map< std::string,
                                std::shared_ptr< std::set< unsigned > > >
                unique;
            std::unique_ptr< PbbiElectricalModel > electrical_model;
            {
                Timer collect{result.collect_ms};
                if(options.candidate_electrical_check) {
                    Timer t{result.electrical_ms};
                    electrical_model =
                        std::make_unique< PbbiElectricalModel >(block, sta);
                }
                Forecast forecast;
                if(options.buffer_aware_timing)
                    forecast =
                        forecastTiming(backend, sta, resizer, states, options,
                                       sites, result, forecast_cache, stop);
                sta::StringSeq groups;
                auto ends = sta->findPathEnds(
                    nullptr, nullptr, nullptr, false,
                    sta::SceneSeq{sta->cmdScene()}, sta::MinMaxAll::max(),
                    options.max_paths, options.paths_per_endpoint, true, false,
                    -detail::kInf, area_recovery ? detail::kInf : 0,
                    true, groups, true, false, false, false,
                    false, false);
                struct PathSnapshot {
                    std::vector< Stage > stages;
                    double slack;
                    unsigned rank;
                    std::set< unsigned > origins;
                };
                std::vector< PathSnapshot > snapshots;
                unsigned count = 0;
                // Required-time/tree queries may invalidate Search-owned
                // PathEnd objects. Copy all topology and slack before querying.
                for(auto* end : ends) {
                    if(count++ >= options.max_paths || !budget())
                        break;
                    ++result.paths;
                    sta::PathExpanded path(end->path(), sta);
                    std::vector< Stage > stages;
                    std::unordered_set< odb::dbNet* > seen;
                    for(size_t i = path.startIndex(); i + 1 < path.size();
                        ++i) {
                        auto* obj = sta->getDbNetwork()->staToDb(
                            path.path(i)->pin(sta));
                        auto* load = sta->getDbNetwork()->staToDb(
                            path.path(i + 1)->pin(sta));
                        if(!obj || obj->getObjectType() != odb::dbITermObj ||
                           !load)
                            continue;
                        auto* pin = static_cast< odb::dbITerm* >(obj);
                        if(pin->getIoType() != odb::dbIoType::OUTPUT)
                            continue;
                        auto it = by_net.find(pin->getNet());
                        // Do not stitch across protected or otherwise omitted
                        // nets.
                        if(it == by_net.end() || it->second->driver != pin ||
                           !seen.insert(pin->getNet()).second) {
                            stages.clear();
                            continue;
                        }
                        if(!stages.empty() &&
                           (stages.back().load->getObjectType() !=
                                odb::dbITermObj ||
                            static_cast< odb::dbITerm* >(stages.back().load)
                                    ->getInst() != pin->getInst()))
                            stages.clear();
                        odb::dbITerm* input = nullptr;
                        if(i > 0) {
                            auto* prev = sta->getDbNetwork()->staToDb(
                                path.path(i - 1)->pin(sta));
                            if(prev &&
                               prev->getObjectType() == odb::dbITermObj) {
                                auto* p = static_cast< odb::dbITerm* >(prev);
                                if(p->getInst() == pin->getInst() &&
                                   p->getIoType() == odb::dbIoType::INPUT)
                                    input = p;
                            }
                        }
                        stages.push_back({it->second, load, input});
                    }
                    if(stages.empty()) {
                        ++result.skipped_paths;
                        continue;
                    }
                    snapshots.push_back(
                        {std::move(stages), double(end->slack(sta)), count});
                }
                if(options.buffer_aware_timing) {
                    // Projected critical paths are extracted from the whole
                    // combinational graph, not just a rerank of STA's top k.
                    unsigned added = 0;
                    std::unordered_map< const sta::Pin*, unsigned >
                        endpoint_count;
                    for(const auto& [slack, pins] : forecast.paths) {
                        if(added >= options.max_paths)
                            break;
                        if(endpoint_count[pins.back()] >=
                           options.paths_per_endpoint)
                            continue;
                        std::vector< Stage > stages;
                        for(size_t i = 0; i + 1 < pins.size(); ++i) {
                            auto* obj = sta->getDbNetwork()->staToDb(pins[i]);
                            auto* load =
                                sta->getDbNetwork()->staToDb(pins[i + 1]);
                            if(!obj || !load ||
                               obj->getObjectType() != odb::dbITermObj)
                                continue;
                            auto* pin = static_cast< odb::dbITerm* >(obj);
                            if(pin->getIoType() != odb::dbIoType::OUTPUT)
                                continue;
                            auto it = by_net.find(pin->getNet());
                            if(it == by_net.end() ||
                               it->second->driver != pin) {
                                stages.clear();
                                continue;
                            }
                            if(!stages.empty() &&
                               (stages.back().load->getObjectType() !=
                                    odb::dbITermObj ||
                                static_cast< odb::dbITerm* >(stages.back().load)
                                        ->getInst() != pin->getInst()))
                                stages.clear();
                            odb::dbITerm* input = nullptr;
                            if(i > 0) {
                                auto* previous =
                                    sta->getDbNetwork()->staToDb(pins[i - 1]);
                                if(previous &&
                                   previous->getObjectType() ==
                                       odb::dbITermObj &&
                                   static_cast< odb::dbITerm* >(previous)
                                           ->getInst() == pin->getInst())
                                    input =
                                        static_cast< odb::dbITerm* >(previous);
                            }
                            stages.push_back({it->second, load, input});
                        }
                        if(stages.empty())
                            continue;
                        snapshots.push_back({std::move(stages), slack, 0});
                        ++added;
                        ++endpoint_count[pins.back()];
                        ++result.forecast_paths;
                    }
                    for(auto& snapshot : snapshots) {
                        auto* pin = sta->getDbNetwork()->dbToSta(
                            snapshot.stages.back().load);
                        auto it = forecast.pins.find(pin);
                        if(it != forecast.pins.end())
                            snapshot.slack = std::min(
                                it->second.required[0] - it->second.arrival[0],
                                it->second.required[1] - it->second.arrival[1]);
                    }
                    std::stable_sort(snapshots.begin(), snapshots.end(),
                                     [](const auto& a, const auto& b) {
                                         return a.slack < b.slack;
                                     });
                    std::set< std::string > seen;
                    snapshots.erase(
                        std::remove_if(
                            snapshots.begin(), snapshots.end(),
                            [&](const auto& p) {
                                std::string key;
                                for(const auto& s : p.stages)
                                    key += s.state->net->getName() + ":" +
                                           sta->getDbNetwork()->pathName(
                                               sta->getDbNetwork()->dbToSta(
                                                   s.load)) +
                                           ";";
                                return !seen.insert(key).second;
                            }),
                        snapshots.end());
                    if(snapshots.size() > options.max_paths)
                        snapshots.resize(options.max_paths);
                    for(unsigned i = 0; i < snapshots.size(); ++i)
                        snapshots[i].rank = i + 1;
                }
                for(auto& snapshot : snapshots)
                    snapshot.origins.insert(snapshot.rank);
                if(options.scope == "net") {
                    // Preserve the common path selection and timing model;
                    // only remove cross-net DP composition. Keep distinct
                    // driver-input/critical-load contexts: they change both
                    // the root timing arc and off-path RAT protection.
                    std::vector< PathSnapshot > nets;
                    std::map< std::string, size_t > seen;
                    for(const auto& snapshot : snapshots)
                        for(const auto& stage : snapshot.stages) {
                            const auto [it, inserted] =
                                seen.emplace(stageKey(stage, sta), nets.size());
                            if(inserted)
                                nets.push_back({{stage}, snapshot.slack,
                                                snapshot.rank, snapshot.origins});
                            else {
                                auto& net = nets[it->second];
                                net.origins.insert(snapshot.origins.begin(),
                                                   snapshot.origins.end());
                                net.slack = std::min(net.slack, snapshot.slack);
                                ++result.duplicate_problems;
                            }
                        }
                    snapshots = std::move(nets);
                }
                for(const auto& snapshot : snapshots) {
                    if(!budget())
                        break;
                    const auto& stages = snapshot.stages;
                    std::shared_ptr< Problem > p;
                    try {
                        p = Builder(backend, sta, resizer, states, options,
                                    stages, &sites,
                                    options.buffer_aware_timing ? &forecast.pins
                                                                : nullptr)
                                .build();
                    }
                    catch(const std::runtime_error& error) {
                        ++result.skipped_paths;
                        ++result.skip_reasons[error.what()];
                        continue;
                    }
                    ++result.collected_problems;
                    result.max_problem_nets =
                        std::max(result.max_problem_nets, p->nets);
                    prepared.push_back(
                        {std::move(p), snapshot.slack, snapshot.rank});
                    prepared.back().origins = snapshot.origins;
                    for(const auto& stage : stages)
                        prepared.back().identity += stageKey(stage, sta);
                    auto& footprint = prepared.back().footprint;
                    for(const auto& binding :
                        prepared.back().problem->bindings) {
                        if(binding.net)
                            footprint.nets.insert(binding.net->getName());
                        if(binding.gate) {
                            footprint.gates.insert(binding.gate->getName());
                            for(auto* pin : binding.gate->getITerms())
                                if(pin->getNet())
                                    footprint.nets.insert(
                                        pin->getNet()->getName());
                        }
                    }
                    footprint.endpoints.insert(sta->getDbNetwork()->pathName(
                        sta->getDbNetwork()->dbToSta(
                            snapshot.stages.back().load)));
                }
            }
            bool committed = false;
            std::set< std::string > failed_actions;
            auto penalty = [&](const Plan& plan) {
                return options.failure_feedback
                           ? std::count_if(plan.actions.begin(),
                                           plan.actions.end(),
                                           [&](const auto& key) {
                                               return failed_actions.count(key);
                                           })
                           : 0;
            };
            // All STA-dependent snapshots are prepared before the first ECO.
            // Solver and prediction are safe even after a deferred rollback.
            auto solvePath = [&](PreparedPath& snapshot) {
                auto p = snapshot.problem;
                auto& plans = snapshot.plans;
                if(!snapshot.solved) {
                    snapshot.solved = true;
                    {
                        Timer collect{result.collect_ms};
                        ++result.solved_paths;
                        pbbi::Limits limits;
                        limits.stop_requested = stop;
                        limits.max_slew_cache_entries =
                            options.slew_cache_entries;
                        limits.operating_point_pruning =
                            options.operating_point_pruning;
                        limits.max_states = options.max_frontier_states;
                        limits.max_expansions = options.max_expansions;
                        limits.max_cost = area_recovery ? 0
                                                       : area_limit - result.after.area;
                        if(options.max_inserted_buffers) {
                            if(result.inserted_buffers >=
                               options.max_inserted_buffers) {
                                // Gate-only search remains valid when the
                                // buffer cap is reached.
                                for(auto& n : p->nodes)
                                    if(n.kind == pbbi::Kind::wire)
                                        n.choices.clear();
                            }
                            else
                                limits.max_buffers =
                                    options.max_inserted_buffers -
                                    result.inserted_buffers;
                        }
                        pbbi::Result solved;
                        if(options.slew_propagation) {
                            limits.slew_grid = p->slew_grid;
                            limits.root_slew = p->root_slew;
                        }
                        std::string memo_key;
                        auto found = path_cache.find(snapshot.identity);
                        if(options.path_cache)
                            memo_key = solveKey(*p, limits);
                        const bool reuse = options.path_cache &&
                                           found != path_cache.end() &&
                                           found->second.key == memo_key;
                        if(reuse) {
                            ++result.path_cache_hits;
                            p->baseline_trace = found->second.baseline;
                            p->baseline_q = found->second.baseline_q;
                            solved = found->second.solved;
                            // These counters describe work performed, not the
                            // logical cached search that produced the frontier.
                            solved.expansions = solved.truncated = 0;
                            solved.slew_cache_hits = solved.slew_cache_misses =
                                0;
                            solved.slew_rejected = solved.response_evaluations =
                                0;
                        }
                        else {
                            if(options.path_cache) {
                                ++result.path_cache_misses;
                                if(found != path_cache.end())
                                    ++result.path_cache_invalidations;
                            }
                            Timer solve_timer{result.solve_ms};
                            auto baseline_limits = limits;
                            baseline_limits.edits = false;
                            baseline_limits.slew_grid.clear();
                            baseline_limits.root_slew = {0, 0};
                            auto baseline =
                                pbbi::solve(p->nodes, int(p->nodes.size()) - 1,
                                            baseline_limits);
                            if(baseline.frontier.empty()) {
                                result.cancelled_paths += baseline.cancelled;
                                ++result.skipped_paths;
                                return;
                            }
                            p->baseline_trace = baseline.frontier.front().trace;
                            if(!pbbi::simulate(p->nodes,
                                               baseline.frontier.front().trace,
                                               p->root_slew, true)
                                    .feasible)
                                ++result.baseline_model_violations;
                            p->baseline_q =
                                options.slew_propagation
                                    ? pbbi::simulate(
                                          p->nodes,
                                          baseline.frontier.front().trace,
                                          p->root_slew, false)
                                          .required
                                    : baseline.frontier.front().required;
                            solved = pbbi::solve(
                                p->nodes, int(p->nodes.size()) - 1, limits);
                        }
                        if(options.path_cache && !solved.cancelled)
                            next_path_cache[snapshot.identity] = {
                                std::move(memo_key), p->baseline_trace,
                                p->baseline_q, solved};
                        result.slew_cache_hits += solved.slew_cache_hits;
                        result.slew_cache_misses += solved.slew_cache_misses;
                        result.slew_rejected += solved.slew_rejected;
                        result.response_evaluations +=
                            solved.response_evaluations;
                        result.expansions += solved.expansions;
                        result.peak_states =
                            std::max(result.peak_states, solved.peak_states);
                        result.truncated_states += solved.truncated;
                        if(solved.exhausted) {
                            result.cancelled_paths += solved.cancelled;
                            ++result.exhausted_paths;
                            return;
                        }
                        for(auto& state : solved.frontier) {
                            double gain = state.required - p->baseline_q;
                            if((!state.buffers && !state.resizes) ||
                               !std::isfinite(gain) ||
                               (area_recovery ? state.cost >= 0 : gain <= 0))
                                continue;
                            SizeMap sizes;
                            std::vector< std::string > keys;
                            edits(*p, state.trace, sizes, keys);
                            std::sort(keys.begin(), keys.end());
                            std::string key;
                            for(const auto& k : keys)
                                key += k + ";";
                            auto [origin, first] = unique.try_emplace(
                                key,
                                std::make_shared< std::set< unsigned > >());
                            origin->second->insert(snapshot.origins.begin(),
                                                   snapshot.origins.end());
                            if(!first) {
                                ++result.duplicate_plans;
                                continue;
                            }
                            plans.push_back({p, std::move(state), gain,
                                             std::max(0.0, (area_mode ? target.wns : 0) - snapshot.slack),
                                             snapshot.rank, std::move(key)});
                            plans.back().actions = std::move(keys);
                            plans.back().origins = origin->second;
                            auto& footprint = plans.back().footprint;
                            for(const auto& binding : p->bindings)
                                if(binding.net)
                                    footprint.nets.insert(
                                        binding.net->getName());
                            for(const auto& [inst, master] : sizes) {
                                footprint.gates.insert(inst->getName());
                                for(auto* pin : inst->getITerms())
                                    if(pin->getNet())
                                        footprint.nets.insert(
                                            pin->getNet()->getName());
                            }
                        }
                    }
                    if(electrical_model) {
                        Timer t{result.electrical_ms};
                        for(auto& plan : plans) {
                            if(!budget())
                                break;
                            SizeMap sizes;
                            std::vector< std::string > keys;
                            edits(*p, plan.state.trace, sizes, keys);
                            plan.electrical = electrical_model->predict(
                                {sizes.begin(), sizes.end()},
                                candidateNets(plan));
                            plan.footprint.endpoints.insert(
                                plan.electrical.affected_endpoints.begin(),
                                plan.electrical.affected_endpoints.end());
                            ++result.electrical_evaluations;
                            result.electrical_unsupported +=
                                plan.electrical.unsupported;
                        }
                    }
                    std::stable_sort(
                        plans.begin(), plans.end(),
                        [&](const auto& a, const auto& b) {
                            if(a.electrical.severity != b.electrical.severity)
                                return a.electrical.severity <
                                       b.electrical.severity;
                            if(a.path_rank != b.path_rank)
                                return a.path_rank < b.path_rank;
                            if(area_mode) {
                                const bool am = predicts_target(a), bm = predicts_target(b);
                                if(am != bm) return am;
                                if(am && a.state.cost != b.state.cost)
                                    return a.state.cost < b.state.cost;
                            }
                            // Align the default path-local ordering with STA
                            // acceptance (and the global scheduler). A larger
                            // WNS gain must not hide a predicted TNS loss.
                            // Target-reaching area plans may spend surplus.
                            if(!area_mode || !predicts_target(a)) {
                                const auto regresses = [&](const auto& plan) {
                                    return pbbi::predictedSetupRegresses(
                                        plan.electrical.wns_delta,
                                        plan.electrical.tns_delta,
                                        options.common.tolerances.wns_absolute,
                                        detail::tnsTolerance(result.after.setup_tns,
                                                             options.common.tolerances));
                                };
                                if(regresses(a) != regresses(b))
                                    return !regresses(a);
                            }
                            if(a.electrical.wns_delta != b.electrical.wns_delta)
                                return a.electrical.wns_delta >
                                       b.electrical.wns_delta;
                            if(a.electrical.tns_delta != b.electrical.tns_delta)
                                return a.electrical.tns_delta >
                                       b.electrical.tns_delta;
                            const bool feasible_a = a.gain >= a.needed_gain,
                                       feasible_b = b.gain >= b.needed_gain;
                            if(feasible_a != feasible_b)
                                return feasible_a;
                            if(feasible_a && a.state.cost != b.state.cost)
                                return a.state.cost < b.state.cost;
                            if(a.gain != b.gain)
                                return a.gain > b.gain;
                            if(a.state.cost != b.state.cost)
                                return a.state.cost < b.state.cost;
                            return a.key < b.key;
                        });
                }
            };
            auto tryPlan = [&](PreparedPath& snapshot, size_t selected) {
                const auto trial_start = Clock::now();
                ++snapshot.attempts;
                auto& plans = snapshot.plans;
                auto plan = std::move(plans[selected]);
                plans.erase(plans.begin() + selected);
                PbbiTrial trial;
                trial.before = result.after;
                trial.area_recovery = area_recovery;
                trial.target_before = target.met(trial.before.setup_wns,
                                                  trial.before.setup_tns);
                trial.path_rank = plan.path_rank;
                trial.source_paths = plan.origins->size();
                trial.affected_nets = plan.footprint.nets.size();
                trial.affected_gates = plan.footprint.gates.size();
                trial.affected_endpoints = plan.footprint.endpoints.size();
                const auto setup_safe = [&](const Plan& candidate) {
                    return !pbbi::predictedSetupRegresses(
                        candidate.electrical.wns_delta, candidate.electrical.tns_delta,
                        options.common.tolerances.wns_absolute,
                        detail::tnsTolerance(result.after.setup_tns, options.common.tolerances));
                };
                trial.pool_nonregressing_setup = setup_safe(plan);
                for(const auto& path : prepared) {
                    trial.pool_nonregressing_setup +=
                        std::count_if(path.plans.begin(), path.plans.end(), setup_safe);
                    trial.pool_candidates += path.plans.size();
                    trial.pool_paths += !path.plans.empty();
                }
                ++trial.pool_candidates;
                if(plans.empty())
                    ++trial.pool_paths;
                trial.predicted_slew_violations = plan.electrical.slew;
                trial.predicted_cap_violations = plan.electrical.cap;
                trial.predicted_severity = plan.electrical.severity;
                trial.predicted_wns_delta = plan.electrical.wns_delta;
                trial.predicted_tns_delta = plan.electrical.tns_delta;
                trial.prediction_baseline_wns = plan.electrical.baseline_wns;
                trial.prediction_baseline_tns = plan.electrical.baseline_tns;
                trial.graph_baseline_wns = plan.electrical.graph_wns;
                trial.graph_baseline_tns = plan.electrical.graph_tns;
                trial.prediction_endpoints = plan.electrical.endpoints;
                trial.graph_endpoints = plan.electrical.graph_endpoints;
                trial.buffers = plan.state.buffers;
                trial.resized_cells = plan.state.resizes;
                trial.path_nets = plan.problem->nets;
                trial.bounded_offpath_loads = plan.problem->bounded_offpath_loads;
                trial.predicted_gain = plan.gain;
                trial.predicted_cost = plan.state.cost;
                trial.fingerprint = plan.key;
                ++result.attempted;
                const double p0 = result.prepare_ms, r0 = result.refresh_ms,
                             m0 = result.measure_ms;
                SizeMap sizes;
                std::vector< std::string > keys;
                edits(*plan.problem, plan.state.trace, sizes, keys);
                PbbiElectricalRisk prediction;
                if(electrical_model && options.pin_diagnostics) {
                    Timer t{result.electrical_ms};
                    prediction =
                        electrical_model->predict({sizes.begin(), sizes.end()},
                                                  candidateNets(plan), true);
                    ++result.electrical_evaluations;
                }
                odb::dbDatabase::beginEco(block);
                active = true;
                std::unordered_map< odb::dbInst*, std::pair<int, int> > positions;
                if(options.pin_diagnostics || options.placement_feedback)
                    for(auto* inst : block->getInsts()) {
                        int x, y;
                        inst->getLocation(x, y);
                        positions.emplace(inst, std::make_pair(x, y));
                    }
                for(const auto& [inst, master] : sizes)
                    if(!inst->swapMaster(master))
                        throw std::runtime_error("PBBI gate swap failed");
                std::map< std::string, std::string > inserted;
                applyTree(*plan.problem, plan.state.trace, &inserted);
                if(options.common.prepare) {
                    Timer t{result.prepare_ms};
                    std::vector< std::pair<odb::dbInst*, odb::dbPlacementStatus> > locked;
                    for(const auto& name : plan.placement_guards)
                        if(auto* inst = block->findInst(name.c_str()))
                            if(!inst->isFixed() && !sizes.count(inst)) {
                                locked.emplace_back(inst, inst->getPlacementStatus());
                                inst->setPlacementStatus(odb::dbPlacementStatus::LOCKED);
                            }
                    trial.protected_instances = locked.size();
                    const auto restore = [&]() {
                        for(const auto& [inst, status] : locked)
                            inst->setPlacementStatus(status);
                    };
                    try {
                        options.common.prepare();
                    }
                    catch(const std::exception&) {
                        restore();
                        if(locked.empty())
                            throw;
                        trial.placement_failed = true;
                    }
                    catch(...) {
                        restore();
                        throw;
                    }
                    restore();
                }
                dirty = true;
                std::vector< NetState > next;
                if(!trial.placement_failed) {
                    update(false);
                    next = inspect(trial.after);
                }
                else
                    trial.after = trial.before;
                auto displacement = [&](odb::dbInst* inst) {
                    auto old = positions.find(inst);
                    if(old == positions.end())
                        return 0.0;
                    int x, y;
                    inst->getLocation(x, y);
                    return std::abs(double(x) - old->second.first) +
                           std::abs(double(y) - old->second.second);
                };
                for(const auto& [inst, pos] : positions) {
                    const double moved = displacement(inst);
                    if(moved > 0) {
                        ++trial.moved_instances;
                        trial.moved_unsized_instances += !sizes.count(inst);
                        trial.max_displacement_dbu =
                            std::max(trial.max_displacement_dbu, moved);
                    }
                }
                trial.rejection_reasons =
                    reject(result.after, trial.after, area_limit, options,
                           target);
                trial.target_after = target.met(trial.after.setup_wns,
                                                 trial.after.setup_tns);
                if(trial.placement_failed)
                    trial.rejection_reasons = {"placement"};
                trial.accepted = trial.rejection_reasons.empty();
                PinChecks pin_next;
                if(!trial.placement_failed &&
                   (options.pin_diagnostics || options.failure_feedback || options.placement_feedback)) {
                    Timer t{result.measure_ms};
                    pin_next = electricalPins(block, sta);
                    diagnose(trial, pin_baseline, pin_next, *plan.problem,
                             sizes);
                    for(auto& v : trial.violations) {
                        if(auto* net = block->findNet(v.net.c_str())) {
                            std::unordered_set< odb::dbInst* > seen;
                            for(auto* pin : net->getITerms())
                                if(seen.insert(pin->getInst()).second) {
                                    const double moved = displacement(pin->getInst());
                                    v.moved_net_instances += moved > 0;
                                    v.net_max_displacement_dbu = std::max(
                                        v.net_max_displacement_dbu, moved);
                                }
                        }
                        auto it = prediction.slew_by_pin.find(v.pin);
                        if(v.kind == "slew" &&
                           it != prediction.slew_by_pin.end()) {
                            v.has_prediction = true;
                            v.predicted = it->second;
                        }
                    }
                }
                bool retry_placement = false;
                if(options.placement_feedback && !trial.accepted &&
                   plan.placement_guards.empty() && !trial.placement_failed)
                    for(const auto& v : trial.violations)
                        if(v.relation == "outside_path" && v.moved_net_instances)
                            if(auto* net = block->findNet(v.net.c_str()))
                                for(auto* pin : net->getITerms()) {
                                    auto* inst = pin->getInst();
                                    if(!inst->isFixed() && !sizes.count(inst) &&
                                       displacement(inst) > 0) {
                                        plan.placement_guards.insert(inst->getName());
                                        retry_placement = true;
                                    }
                                }
                if(options.failure_feedback && !trial.accepted) {
                    // A failed timing plan is evidence against repeating its
                    // action combination, not proof that each action is bad.
                    // Use the existing soft ranking penalty only; retain all
                    // alternatives and clear feedback at the next commit.
                    const bool timing_failure = std::any_of(
                        trial.rejection_reasons.begin(), trial.rejection_reasons.end(),
                        [](const auto& reason) {
                            return reason == "setup_wns" || reason == "setup_tns" ||
                                   reason == "no_setup_gain";
                        });
                    if(timing_failure)
                        failed_actions.insert(plan.actions.begin(), plan.actions.end());
                    for(const auto& v : trial.violations) {
                        const auto slash = v.pin.rfind('/');
                        const auto inst_name = v.pin.substr(0, slash);
                        auto found = inserted.find(inst_name);
                        if(found != inserted.end())
                            failed_actions.insert(found->second);
                        for(const auto& [inst, master] : sizes) {
                            bool affects = inst->getName() == inst_name;
                            for(auto* pin : inst->getITerms())
                                if(pin->getNet() &&
                                   pin->getNet()->getName() == v.net)
                                    affects = true;
                            if(affects)
                                failed_actions.insert("S:" + inst->getName() +
                                                      ":" + master->getName());
                        }
                    }
                }
                if(trial.accepted) {
                    pin_baseline = std::move(pin_next);
                    odb::dbDatabase::commitEco(block);
                    active = false;
                    ++result.accepted;
                    result.area_reductions += area_recovery;
                    result.inserted_buffers += trial.buffers;
                    result.resized_cells += sizes.size();
                    result.after = trial.after;
                    states = std::move(next);
                    committed = true;
                }
                else {
                    odb::dbDatabase::undoEco(block);
                    active = false;
                    dirty = true;
                    ++result.rejected;
                    if(!options.defer_rollback_refresh) {
                        update(true);
                        inspect(result.after);
                    }
                }
                trial.prepare_ms = result.prepare_ms - p0;
                trial.refresh_ms = result.refresh_ms - r0;
                trial.measure_ms = result.measure_ms - m0;
                result.trials.push_back(std::move(trial));
                if(retry_placement)
                    plans.insert(plans.begin(), std::move(plan));
                last_trial_seconds = std::chrono::duration< double >(
                                         Clock::now() - trial_start).count();
                return retry_placement;
            };
            if(options.scheduler == "sequential") {
                bool pending = true;
                while(pending && budget() && !committed) {
                    pending = false;
                    for(auto& snapshot : prepared) {
                        if(!budget())
                            break;
                        solvePath(snapshot);
                        auto& plans = snapshot.plans;
                        while(!plans.empty() && budget()) {
                            auto selected = std::min_element(
                                plans.begin(), plans.end(),
                                [&](const auto& x, const auto& y) {
                                    if(x.placement_guards.empty() != y.placement_guards.empty())
                                        return !x.placement_guards.empty();
                                    return penalty(x) < penalty(y);
                                });
                            if(selected != plans.begin())
                                ++result.feedback_reorders;
                            if(tryPlan(snapshot, selected - plans.begin()))
                                continue;
                            if(committed || options.failure_feedback)
                                break;
                        }
                        pending = pending || !plans.empty();
                        if(committed)
                            break;
                    }
                }
            }
            else {
                const bool adaptive = options.scheduler == "adaptive";
                // Bound speculative solving. Seed a small cross-path pool,
                // then admit one additional path after each failed trial.
                // Prefer distinct footprints when possible; all alternatives
                // remain eligible, including paths sharing nets or endpoints.
                auto admit = [&]() {
                    bool ready = false;
                    for(const auto& path : prepared)
                        ready = ready || !path.plans.empty();
                    // Preserve time for an already available real STA trial.
                    // Estimates only constrain speculative admission, never
                    // the deterministic ranking of available candidates.
                    if(adaptive && ready && options.max_seconds) {
                        const double remaining = options.max_seconds -
                            std::chrono::duration< double >(Clock::now() - start).count();
                        if(remaining <= last_path_seconds + last_trial_seconds) {
                            ++result.admission_deferred;
                            return false;
                        }
                    }
                    PreparedPath* selected = nullptr;
                    unsigned best_overlap = 0;
                    for(auto& path : prepared) {
                        if(path.solved)
                            continue;
                        unsigned overlap = 0;
                        for(const auto& active : prepared)
                            if(active.solved &&
                               active.footprint.overlaps(path.footprint))
                                ++overlap;
                        if(!selected || (!adaptive && overlap < best_overlap)) {
                            selected = &path;
                            best_overlap = overlap;
                        }
                    }
                    if(!selected || !budget())
                        return false;
                    if(best_overlap)
                        ++result.scheduler_overlap_admissions;
                    const auto solve_start = Clock::now();
                    solvePath(*selected);
                    last_path_seconds = std::chrono::duration< double >(
                                            Clock::now() - solve_start).count();
                    return true;
                };
                const unsigned warmup = adaptive ? 1 : options.scheduler_window;
                for(unsigned i = 0; i < warmup && budget();
                    ++i)
                    if(!admit())
                        break;
                while(budget() && !committed) {
                    PreparedPath* selected_path = nullptr;
                    size_t selected_index = 0;
                    pbbi::Priority best;
                    unsigned count = 0,
                             first_rank =
                                 std::numeric_limits< unsigned >::max();
                    for(auto& path : prepared) {
                        for(size_t i = 0; i < path.plans.size(); ++i) {
                            const auto& plan = path.plans[i];
                            ++count;
                            first_rank = std::min(first_rank, plan.path_rank);
                            pbbi::Priority priority;
                            priority.severity = plan.electrical.severity;
                            // With the full electrical model disabled, only
                            // local path gain is available; do not call it a
                            // global WNS/TNS forecast.
                            priority.wns_gain = electrical_model
                                                    ? plan.electrical.wns_delta
                                                    : plan.gain;
                            priority.tns_gain = plan.electrical.tns_delta;
                            priority.cost = plan.state.cost;
                            priority.target_met = predicts_target(plan);
                            priority.failed_actions = penalty(plan);
                            priority.actions = plan.actions.size();
                            priority.path_rank = plan.path_rank;
                            priority.key = plan.key;
                            // Rotate away from failed paths, including
                            // unmapped off-path violations. Keep all plans:
                            // this is a cooldown, not an irreversible ban.
                            const bool fewer_attempts = selected_path && adaptive &&
                                path.attempts < selected_path->attempts;
                            const bool equal_attempts = !selected_path || !adaptive ||
                                path.attempts == selected_path->attempts;
                            const bool retry = !plan.placement_guards.empty();
                            const bool selected_retry = selected_path &&
                                !selected_path->plans[selected_index].placement_guards.empty();
                            if(!selected_path || (retry && !selected_retry) ||
                               (retry == selected_retry && (fewer_attempts ||
                               (equal_attempts && pbbi::preferred(
                                   priority, best,
                                   options.common.tolerances.wns_absolute,
                                   detail::tnsTolerance(
                                       result.after.setup_tns,
                                       options.common.tolerances), area_mode))))) {
                                selected_path = &path;
                                selected_index = i;
                                best = std::move(priority);
                            }
                        }
                    }
                    if(!selected_path) {
                        if(admit())
                            continue;
                        break;
                    }
                    ++result.scheduler_decisions;
                    result.scheduler_peak_candidates =
                        std::max(result.scheduler_peak_candidates, count);
                    if(best.path_rank != first_rank)
                        ++result.scheduler_cross_path;
                    const bool retry = tryPlan(*selected_path, selected_index);
                    // A measured placement retry reuses an already solved
                    // plan. Do not spend its remaining budget admitting a
                    // speculative path before validating that plan.
                    if(!retry && !committed && budget())
                        admit();
                }
            }
            path_cache = std::move(next_path_cache);
            if(!committed)
                break;
        }
        if(dirty) {
            update(true);
            inspect(result.after);
        }
        result.ok = true;
        result.target_met = target.met(result.after.setup_wns,
                                       result.after.setup_tns);
        result.converged = area_mode ? result.target_met
                                     : result.after.setup_wns >= 0;
        result.message =
            result.converged
                ? (area_mode ? "PBBI timing target met; area search stopped"
                             : "PBBI setup timing met")
                : "PBBI stopped at budget or no accepted improving plan";
    }
    catch(const std::exception& error) {
        result.message = error.what();
        if(active) {
            odb::dbDatabase::undoEco(block);
            dirty = true;
        }
        if(dirty)
            try {
                update(true);
            }
            catch(const std::exception& restore) {
                result.message +=
                    "; restore failed: " + std::string(restore.what());
            }
    }
    if(!result.attempted && !result.skip_reasons.empty())
        result.message += "; " + result.skip_reasons.begin()->first;
    return result;
}
}  // namespace ecc
