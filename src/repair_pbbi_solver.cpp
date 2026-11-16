// SPDX-License-Identifier: Apache-2.0
#include "repair_pbbi_solver.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <unordered_map>
#include <tuple>

namespace ecc::pbbi {
struct SlewCache {
    struct Entry {
        size_t serial = 0;
        double slew = 0, value = 0;
        unsigned rf = 0;
    };
    std::vector< Entry > entries;
    size_t hits = 0, misses = 0;
    explicit SlewCache(size_t count) : entries(count) {
    }
    size_t slot(size_t serial, double slew, unsigned rf) const {
        return (std::hash< size_t >{}(serial) ^
                (std::hash< double >{}(slew) << 1) ^ rf) %
               entries.size();
    }
};
namespace {
bool dominates(const State& a, const State& b) {
    // Reject on scalar resource dimensions before walking the slew curves.
    // This changes only evaluation cost, never the dominance relation.
    if(!(a.cap <= b.cap && a.required >= b.required && a.cost <= b.cost &&
         a.buffers <= b.buffers && a.resizes <= b.resizes))
        return false;
    if(a.required_curve && b.required_curve)
        for(size_t i = 0; i < a.required_curve->size(); ++i)
            for(unsigned rf = 0; rf < 2; ++rf)
                if((*a.required_curve)[i][rf] < (*b.required_curve)[i][rf])
                    return false;
    // Counts are resource dimensions as well as cost. Never discard no-op
    // merely because a resize produces a better electrical state.
    return true;
}
void prune(std::vector< State >& states, const Limits& limits, Result& result) {
    std::stable_sort(states.begin(), states.end(),
                     [](const auto& a, const auto& b) {
                         if(a.cost != b.cost)
                             return a.cost < b.cost;
                         if(a.cap != b.cap)
                             return a.cap < b.cap;
                         return a.required > b.required;
                     });
    std::vector< State > frontier;
    for(auto& state : states) {
        if(limits.stop_requested && limits.stop_requested()) {
            result.cancelled = result.exhausted = true;
            states.clear();
            return;
        }
        if(std::any_of(frontier.begin(), frontier.end(),
                       [&](const auto& a) { return dominates(a, state); }))
            continue;
        // Input and retained states are ordered by increasing finite cost.
        // A new state can only dominate previous states with the same cost.
        if(!frontier.empty() && frontier.back().cost == state.cost) {
            const auto equal_cost = std::lower_bound(
                frontier.begin(), frontier.end(), state.cost,
                [](const auto& a, double cost) { return a.cost < cost; });
            frontier.erase(
                std::remove_if(equal_cost, frontier.end(),
                               [&](const auto& a) { return dominates(state, a); }),
                frontier.end());
        }
        frontier.push_back(std::move(state));
    }
    result.peak_states = std::max(result.peak_states, frontier.size());
    if(limits.max_states && frontier.size() > limits.max_states) {
        // Explicit approximation: retain no-op, then alternating best timing
        // and cheapest states. No approximation occurs with max_states=0.
        result.truncated += frontier.size() - limits.max_states;
        std::vector< State > retained;
        auto take = [&](auto comparator) {
            auto it =
                std::min_element(frontier.begin(), frontier.end(), comparator);
            retained.push_back(std::move(*it));
            frontier.erase(it);
        };
        auto noop = std::find_if(
            frontier.begin(), frontier.end(),
            [](const auto& s) { return s.buffers == 0 && s.resizes == 0; });
        if(noop != frontier.end()) {
            retained.push_back(std::move(*noop));
            frontier.erase(noop);
        }
        while(retained.size() < limits.max_states) {
            if(retained.size() % 2)
                take([](const auto& a, const auto& b) {
                    return a.ranking_required > b.ranking_required;
                });
            else
                take([](const auto& a, const auto& b) {
                    return a.cost < b.cost;
                });
        }
        frontier = std::move(retained);
    }
    states = std::move(frontier);
}
}  // namespace
namespace {
double exactRequired(const std::vector< Node >& nodes,
                     const std::shared_ptr< const Trace >& trace, double slew,
                     unsigned rf, SlewCache* cache);
double computeRequired(const std::vector< Node >& nodes,
                       const std::shared_ptr< const Trace >& trace, double slew,
                       unsigned rf, SlewCache* cache) {
    const auto& n = nodes[trace->node];
    const Choice* c = trace->choice < 0 ? nullptr : &n.choices[trace->choice];
    if(n.kind == Kind::load) {
        if(slew > n.slew_limit || (c && slew > c->input_slew_limit))
            return -INFINITY;
        double q = n.requiredFor(rf);
        return c ? (c->sink_required ? c->sink_required(slew, rf)
                                     : q + c->intrinsic)
                 : q;
    }
    if(n.kind == Kind::junction) {
        double left = exactRequired(nodes, trace->left, slew, rf, cache);
        if(left == -INFINITY)
            return left;
        return std::min(left,
                        exactRequired(nodes, trace->right, slew, rf, cache));
    }
    double wire_delay = n.kind == Kind::wire
                            ? n.resistance * (trace->left->cap + n.cap / 2)
                            : 0;
    auto downstream = [&](double s, unsigned out) {
        return exactRequired(
                   nodes, trace->left,
                   std::hypot(s, n.wire_slew_factor[out] * wire_delay), out,
                   cache) -
               wire_delay;
    };
    if(!c)
        return downstream(slew, rf);
    double cap = trace->left->cap + (n.kind == Kind::wire ? n.cap : 0);
    if(slew > c->input_slew_limit || cap > c->max_cap)
        return -INFINITY;
    ArcResponse r;
    if(c->response)
        r = c->response(cap, slew, rf);
    else {
        r.delay[rf] = c->intrinsic + c->resistance * cap;
        r.slew[rf] = slew;
    }
    double q = INFINITY;
    bool seen = false;
    for(unsigned out = 0; out < 2; ++out)
        if(std::isfinite(r.delay[out])) {
            seen = true;
            if(r.slew[out] > c->output_slew_limit)
                return -INFINITY;
            q = std::min(q, downstream(r.slew[out], out) - r.delay[out]);
            if(q == -INFINITY)
                return q;
        }
    return seen ? q : -double(INFINITY);
}
double exactRequired(const std::vector< Node >& nodes,
                     const std::shared_ptr< const Trace >& trace, double slew,
                     unsigned rf, SlewCache* cache) {
    if(!cache || cache->entries.empty())
        return computeRequired(nodes, trace, slew, rf, cache);
    const size_t slot = cache->slot(trace->serial, slew, rf);
    const auto old = cache->entries[slot];
    if(old.serial == trace->serial && old.slew == slew && old.rf == rf) {
        ++cache->hits;
        return old.value;
    }
    ++cache->misses;
    double value = computeRequired(nodes, trace, slew, rf, cache);
    // Recursive calls can overwrite this direct-mapped slot. Publish only
    // after computing the result. Serial IDs never reuse pruned trace
    // addresses.
    cache->entries[slot] = {trace->serial, slew, value, rf};
    return value;
}
}  // namespace
double requiredAt(const State& state, const std::vector< double >& grid,
                  double slew, unsigned rf) {
    if(!state.required_curve)
        return state.required;
    if(!std::isfinite(slew) || slew < 0)
        return -INFINITY;
    if(slew > grid.back())
        return state.models ? exactRequired(*state.models, state.trace, slew,
                                            rf, state.exact_cache.get())
                            : -double(INFINITY);
    auto it = std::lower_bound(grid.begin(), grid.end(), slew);
    size_t hi = it - grid.begin();
    double b = (*state.required_curve)[hi][rf];
    if(hi == 0 || *it == slew)
        return b;
    double a = (*state.required_curve)[hi - 1][rf];
    if(a == b)
        return a;
    // An infeasible upper knot does NOT make the whole interval infeasible.
    // Resolve the actual trace at constraint boundaries instead of discarding
    // valid sub-grid slews (especially across differently constrained gates).
    if(!std::isfinite(a) || !std::isfinite(b))
        return state.models ? exactRequired(*state.models, state.trace, slew,
                                            rf, state.exact_cache.get())
                            : -double(INFINITY);
    return a + (b - a) * (slew - grid[hi - 1]) / (grid[hi] - grid[hi - 1]);
}
Simulation simulate(const std::vector< Node >& nodes,
                    const std::shared_ptr< const Trace >& trace,
                    std::array< double, 2 > input_slew, bool enforce) {
    Simulation result;
    std::function< void(const std::shared_ptr< const Trace >&,
                        std::array< double, 2 >, std::array< double, 2 >,
                        std::array< double, 2 >, std::array< double, 2 >) >
        visit;
    visit = [&](const auto& t, std::array< double, 2 > at,
                std::array< double, 2 > slew, std::array< double, 2 > launch,
                std::array< double, 2 > wire_slew) {
        const auto& n = nodes.at(t->node);
        const Choice* c = t->choice < 0 ? nullptr : &n.choices.at(t->choice);
        if(n.kind == Kind::load) {
            for(unsigned rf = 0; rf < 2; ++rf) {
                double q = n.requiredFor(rf);
                if(c)
                    q = c->sink_required ? c->sink_required(slew[rf], rf)
                                         : q + c->intrinsic;
                if(enforce && (slew[rf] > n.slew_limit ||
                               (c && slew[rf] > c->input_slew_limit)))
                    result.feasible = false;
                result.required = std::min(result.required, q - at[rf]);
            }
            result.leaves.push_back({t->node, at, slew});
            return;
        }
        if(n.kind == Kind::junction) {
            visit(t->left, at, slew, launch, wire_slew);
            visit(t->right, at, slew, launch, wire_slew);
            return;
        }
        if(c) {
            std::array< double, 2 > next_at{{-INFINITY, -INFINITY}},
                next_slew{{0, 0}};
            double cap = t->left->cap + (n.kind == Kind::wire ? n.cap : 0);
            if(enforce && cap > c->max_cap)
                result.feasible = false;
            for(unsigned rf = 0; rf < 2; ++rf) {
                if(enforce && slew[rf] > c->input_slew_limit)
                    result.feasible = false;
                ArcResponse r;
                if(c->response)
                    r = c->response(cap, slew[rf], rf);
                else {
                    r.delay[rf] = c->intrinsic + c->resistance * cap;
                    r.slew[rf] = slew[rf];
                }
                for(unsigned out = 0; out < 2; ++out)
                    if(std::isfinite(r.delay[out])) {
                        next_at[out] =
                            std::max(next_at[out], at[rf] + r.delay[out]);
                        next_slew[out] = std::max(next_slew[out], r.slew[out]);
                        if(!std::isfinite(r.slew[out]) || r.slew[out] < 0 ||
                           (enforce && r.slew[out] > c->output_slew_limit))
                            result.feasible = false;
                    }
            }
            at = next_at;
            slew = next_slew;
            launch = next_slew;
            wire_slew = {0, 0};
            if(!std::isfinite(at[0]) || !std::isfinite(at[1]))
                result.feasible = false;
        }
        if(n.kind == Kind::wire) {
            double delay = n.resistance * (t->left->cap + n.cap / 2);
            for(unsigned rf = 0; rf < 2; ++rf) {
                at[rf] += delay;
                // Sum Elmore delay over the whole unbuffered connection;
                // splitting a wire must not artificially improve its slew.
                wire_slew[rf] += n.wire_slew_factor[rf] * delay;
                slew[rf] = std::hypot(launch[rf], wire_slew[rf]);
            }
        }
        visit(t->left, at, slew, launch, wire_slew);
    };
    visit(trace, {0, 0}, input_slew, input_slew, {0, 0});
    return result;
}
Result solve(const std::vector< Node >& nodes, int root, const Limits& limits) {
    if(root < 0 || root >= int(nodes.size()))
        throw std::invalid_argument("PBBI invalid root");
    if(!limits.slew_grid.empty()) {
        if(limits.slew_grid.size() < 2 || limits.slew_grid.front() != 0)
            throw std::invalid_argument(
                "PBBI slew grid must start at zero and have at least two "
                "points");
        double prev = -1;
        for(double x : limits.slew_grid) {
            if(!std::isfinite(x) || x <= prev)
                throw std::invalid_argument("PBBI invalid slew grid");
            prev = x;
        }
    }
    Result result;
    auto stop = [&]() {
        if(limits.stop_requested && limits.stop_requested())
            result.cancelled = result.exhausted = true;
        return result.cancelled;
    };
    auto models = limits.slew_grid.empty()
                      ? std::shared_ptr< const std::vector< Node > >{}
                      : std::make_shared< const std::vector< Node > >(nodes);
    auto exact_cache =
        limits.slew_grid.empty()
            ? std::shared_ptr< SlewCache >{}
            : std::make_shared< SlewCache >(limits.max_slew_cache_entries);
    std::vector< std::vector< State > > fronts(nodes.size());
    for(int id = 0; id <= root; ++id) {
        if(stop())
            return result;
        const auto& n = nodes[id];
        auto& out = fronts[id];
        if(n.cap < 0 || n.resistance < 0 || !std::isfinite(n.cap) ||
           !std::isfinite(n.resistance) || std::isnan(n.required)) {
            std::ostringstream message;
            message << "PBBI invalid RC or required time at node " << id
                    << " kind=" << int(n.kind) << " cap=" << n.cap
                    << " resistance=" << n.resistance
                    << " required=" << n.required;
            throw std::invalid_argument(message.str());
        }
        if(n.kind != Kind::load && (n.left < 0 || n.left >= id))
            throw std::invalid_argument("PBBI nodes must be postordered");
        if(n.kind == Kind::junction && (n.right < 0 || n.right >= id))
            throw std::invalid_argument("PBBI invalid junction");
        for(const auto& c : n.choices) {
            if(!std::isfinite(c.cap) || c.cap < 0 ||
               !std::isfinite(c.intrinsic) || !std::isfinite(c.resistance) ||
               c.resistance < 0 || !std::isfinite(c.cost) ||
               std::isnan(c.max_cap) || c.max_cap < 0)
                throw std::invalid_argument("PBBI invalid library choice");
        }
        using Curve = std::vector< std::array< double, 2 > >;
        using ResponseGrid = std::vector< std::array< ArcResponse, 2 > >;
        std::vector< std::unordered_map< double, ResponseGrid > > cache(
            n.choices.size());
        auto response = [&](int choice, double cap, size_t index, unsigned rf) {
            auto [it, added] = cache[choice].try_emplace(cap);
            if(added) {
                auto& grid = it->second;
                grid.resize(limits.slew_grid.size());
                const auto& c = n.choices[choice];
                for(size_t i = 0; i < grid.size(); ++i)
                    for(unsigned in = 0; in < 2; ++in) {
                        double slew = limits.slew_grid[i];
                        auto& r = grid[i][in];
                        if(c.response)
                            r = c.response(cap, slew, in);
                        else {
                            r.delay[in] = c.intrinsic + c.resistance * cap;
                            r.slew[in] = slew;
                        }
                        ++result.response_evaluations;
                    }
            }
            return it->second[index][rf];
        };
        auto emit = [&](State s, int choice, const State* a, const State* b) {
            ++result.expansions;
            if(limits.max_expansions &&
               result.expansions > limits.max_expansions) {
                result.exhausted = true;
                return;
            }
            if(!std::isfinite(s.cap) || std::isnan(s.required) ||
               !std::isfinite(s.cost) ||
               (limits.max_buffers && s.buffers > limits.max_buffers))
                return;
            // Do not prune partial cost: downstream upsizing may be offset
            // by a legal upstream downsize. Apply the area cap at the root.
            s.trace = std::make_shared< Trace >(
                Trace{id, choice, a ? a->trace : nullptr,
                      b ? b->trace : nullptr, s.cap, result.expansions});
            s.models = models;
            s.exact_cache = exact_cache;
            if(!limits.slew_grid.empty()) {
                auto curve = std::make_shared< Curve >(limits.slew_grid.size());
                const Choice* c = choice < 0 ? nullptr : &n.choices[choice];
                for(size_t i = 0; i < limits.slew_grid.size(); ++i) {
                    double slew = limits.slew_grid[i];
                    for(unsigned rf = 0; rf < 2; ++rf) {
                        double q = -INFINITY;
                        if(n.kind == Kind::load) {
                            q = n.requiredFor(rf);
                            if(c)
                                q = c->sink_required
                                        ? c->sink_required(slew, rf)
                                        : q + c->intrinsic;
                            if(slew > n.slew_limit ||
                               (c && slew > c->input_slew_limit))
                                q = -INFINITY;
                        }
                        else if(n.kind == Kind::junction) {
                            q = std::min((*a->required_curve)[i][rf],
                                         (*b->required_curve)[i][rf]);
                        }
                        else {
                            const double wire_delay =
                                n.kind == Kind::wire
                                    ? n.resistance * (a->cap + n.cap / 2)
                                    : 0;
                            auto downstream = [&](double launch, unsigned out) {
                                double receive =
                                    std::hypot(launch, n.wire_slew_factor[out] *
                                                           wire_delay);
                                return requiredAt(*a, limits.slew_grid, receive,
                                                  out) -
                                       wire_delay;
                            };
                            if(!c)
                                q = downstream(slew, rf);
                            else if(slew <= c->input_slew_limit) {
                                auto r = response(
                                    choice,
                                    a->cap + (n.kind == Kind::wire ? n.cap : 0),
                                    i, rf);
                                q = INFINITY;
                                bool connected = false;
                                for(unsigned out = 0; out < 2; ++out)
                                    if(std::isfinite(r.delay[out])) {
                                        connected = true;
                                        double v =
                                            r.slew[out] <= c->output_slew_limit
                                                ? downstream(r.slew[out], out) -
                                                      r.delay[out]
                                                : -INFINITY;
                                        q = std::min(q, v);
                                    }
                                if(!connected)
                                    q = -INFINITY;
                            }
                        }
                        (*curve)[i][rf] = q;
                    }
                }
                // Monotone conservative envelope: never improve required time
                // by worsening the incoming transition.
                for(size_t i = 1; i < curve->size(); ++i)
                    for(unsigned rf = 0; rf < 2; ++rf)
                        (*curve)[i][rf] =
                            std::min((*curve)[i][rf], (*curve)[i - 1][rf]);
                if((*curve)[0][0] == -INFINITY || (*curve)[0][1] == -INFINITY) {
                    ++result.slew_rejected;
                    return;
                }
                s.required = std::min((*curve)[0][0], (*curve)[0][1]);
                s.required_curve = std::move(curve);
            }

            s.ranking_required = s.required;
            if(limits.operating_point_pruning && s.required_curve) {
                s.ranking_required = INFINITY;
                for(unsigned rf = 0; rf < 2; ++rf) {
                    double reference = id == root || n.reference_slew[rf] < 0
                                           ? limits.root_slew[rf]
                                           : n.reference_slew[rf];
                    s.ranking_required = std::min(
                        s.ranking_required,
                        requiredAt(s, limits.slew_grid, reference, rf));
                }
            }
            out.push_back(std::move(s));
        };
        if(n.kind == Kind::load) {
            if(n.choices.empty())
                emit({n.cap, n.required}, -1, nullptr, nullptr);
            for(size_t k = 0; k < n.choices.size(); ++k) {
                if(stop())
                    break;
                const auto& c = n.choices[k];
                if(!limits.edits && c.edit)
                    continue;
                emit({c.cap, n.required + c.intrinsic, c.cost, 0,
                      unsigned(c.edit)},
                     int(k), nullptr, nullptr);
            }
        }
        else if(n.kind == Kind::junction) {
            for(const auto& a : fronts[n.left]) {
                if(stop())
                    break;
                for(const auto& b : fronts[n.right]) {
                    emit({a.cap + b.cap, std::min(a.required, b.required),
                          a.cost + b.cost, a.buffers + b.buffers,
                          a.resizes + b.resizes},
                         -1, &a, &b);
                    if(result.exhausted)
                        break;
                }
                if(result.exhausted)
                    break;
            }
        }
        else {
            for(const auto& a : fronts[n.left]) {
                if(stop())
                    break;
                if(n.kind == Kind::wire) {
                    State wire = a;
                    wire.cap += n.cap;
                    wire.required -= n.resistance * (a.cap + n.cap / 2);
                    emit(wire, -1, &a, nullptr);
                    if(limits.edits)
                        for(size_t k = 0; k < n.choices.size(); ++k) {
                            const auto& c = n.choices[k];
                            if(wire.cap > c.max_cap)
                                continue;
                            emit({c.cap,
                                  wire.required - c.intrinsic -
                                      c.resistance * wire.cap,
                                  wire.cost + c.cost, wire.buffers + 1,
                                  wire.resizes},
                                 int(k), &a, nullptr);
                            if(result.exhausted)
                                break;
                        }
                }
                else
                    for(size_t k = 0; k < n.choices.size(); ++k) {
                        const auto& c = n.choices[k];
                        if((!limits.edits && c.edit) || a.cap > c.max_cap)
                            continue;
                        emit({c.cap,
                              a.required - c.intrinsic - c.resistance * a.cap,
                              a.cost + c.cost, a.buffers,
                              a.resizes + unsigned(c.edit)},
                             int(k), &a, nullptr);
                        if(result.exhausted)
                            break;
                    }
                if(result.exhausted)
                    break;
            }
        }
        if(result.exhausted) {
            if(exact_cache) {
                result.slew_cache_hits = exact_cache->hits;
                result.slew_cache_misses = exact_cache->misses;
            }
            return result;
        }
        prune(out, limits, result);
        if(result.cancelled)
            return result;
    }
    for(auto& s : fronts[root]) {
        if(stop()) {
            result.frontier.clear();
            return result;
        }
        if(s.cost > limits.max_cost)
            continue;
        if(!limits.slew_grid.empty()) {
            // Forward verification at the actual root slew eliminates any
            // feasibility error introduced by interpolation of sampled curves.
            auto checked = simulate(nodes, s.trace, limits.root_slew);
            if(!checked.feasible) {
                ++result.slew_rejected;
                continue;
            }
            s.required = checked.required;
        }
        s.exact_cache.reset();
        result.frontier.push_back(std::move(s));
    }
    if(exact_cache) {
        result.slew_cache_hits = exact_cache->hits;
        result.slew_cache_misses = exact_cache->misses;
    }
    return result;
}
}  // namespace ecc::pbbi
