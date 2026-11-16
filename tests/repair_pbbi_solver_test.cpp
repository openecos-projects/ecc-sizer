#include "repair_pbbi_solver.h"
#include "repair_pbbi_schedule.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <cmath>
#include <functional>

using namespace ecc::pbbi;
namespace {
std::vector<State> enumerate(const std::vector<Node>& nodes)
{
  std::vector<State> results;
  std::vector<int> choices(nodes.size(), -1);
  std::function<void(size_t)> pick = [&](size_t index) {
    if (index < nodes.size()) {
      const auto& n = nodes[index];
      if (n.kind == Kind::wire || n.choices.empty()) {
        choices[index] = -1;
        pick(index + 1);
      }
      for (size_t i = 0; i < n.choices.size(); ++i) {
        choices[index] = i;
        pick(index + 1);
      }
      return;
    }
    // Independent oracle: enumerate all discrete placements/sizes, compute
    // circuit caps, then propagate arrival times from the root to sinks.
    std::vector<double> caps(nodes.size());
    State state;
    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto& n = nodes[i];
      int k = choices[i];
      const Choice* c = k < 0 ? nullptr : &n.choices[k];
      if (n.kind == Kind::load)
        caps[i] = c ? c->cap : n.cap;
      if (n.kind == Kind::junction)
        caps[i] = caps[n.left] + caps[n.right];
      if (n.kind == Kind::wire)
        caps[i] = c ? c->cap : n.cap + caps[n.left];
      if (n.kind == Kind::gate)
        caps[i] = c->cap;
      if (c) {
        double load = n.kind == Kind::wire   ? n.cap + caps[n.left]
                      : n.kind == Kind::gate ? caps[n.left]
                                             : 0;
        if (load > c->max_cap)
          return;
        state.cost += c->cost;
        state.buffers += n.kind == Kind::wire;
        state.resizes += n.kind != Kind::wire && c->edit;
      }
    }
    state.cap = caps.back();
    state.required = INFINITY;
    std::function<void(int, double)> propagate = [&](int i, double at) {
      const auto& n = nodes[i];
      int k = choices[i];
      const Choice* c = k < 0 ? nullptr : &n.choices[k];
      if (n.kind == Kind::load)
        state.required = std::min(state.required,
                                  n.required + (c ? c->intrinsic : 0) - at);
      else if (n.kind == Kind::junction) {
        propagate(n.left, at);
        propagate(n.right, at);
      } else if (n.kind == Kind::gate)
        propagate(n.left, at + c->intrinsic + c->resistance * caps[n.left]);
      else {
        double delay = n.resistance * (caps[n.left] + n.cap / 2);
        if (c)
          delay += c->intrinsic + c->resistance * (caps[n.left] + n.cap);
        propagate(n.left, at + delay);
      }
    };
    propagate(int(nodes.size()) - 1, 0);
    results.push_back(state);
  };
  pick(0);
  return results;
}
bool covers(const State& a, const State& b)
{
  return a.cap <= b.cap + 1e-9 && a.required >= b.required - 1e-9
         && a.cost <= b.cost + 1e-9 && a.buffers <= b.buffers
         && a.resizes <= b.resizes;
}
}  // namespace
TEST(PbbiSolver, MatchesExhaustiveArrivalSimulationAcrossGatesAndBranches)
{
  std::vector<Node> n(8);
  n[0].cap = 4;
  n[0].required = 40;
  n[1].kind = Kind::wire;
  n[1].left = 0;
  n[1].cap = 2;
  n[1].resistance = 3;
  n[1].choices = {{1, 1, 1, 2, 100, true}, {2, 1, 0.5, 4, 100, true}};
  n[2].kind = Kind::gate;
  n[2].left = 1;
  n[2].choices = {{1, 2, 2, 0, 100, false}, {2, 1, 0.5, 3, 100, true}};
  n[3].required = 35;
  n[3].choices = {{2, 0, 0, 0, 100, false}, {4, 4, 0, 2, 100, true}};
  n[4].kind = Kind::junction;
  n[4].left = 2;
  n[4].right = 3;
  n[5] = n[1];
  n[5].left = 4;
  n[6].kind = Kind::gate;
  n[6].left = 5;
  n[6].choices = {{1, 2, 3, 0, 100, false}, {3, 1, 0.5, 4, 100, true}};
  n[7] = n[1];
  n[7].left = 6;
  Limits limits;
  limits.max_states = 0;
  limits.max_expansions = 0;
  const auto result = solve(n, 7, limits);
  const auto brute = enumerate(n);
  ASSERT_FALSE(result.frontier.empty());
  ASSERT_GT(brute.size(), 100u);
  for (const auto& b : brute)
    EXPECT_TRUE(std::any_of(result.frontier.begin(),
                            result.frontier.end(),
                            [&](const auto& a) { return covers(a, b); }));
  for (const auto& a : result.frontier)
    EXPECT_TRUE(std::any_of(brute.begin(), brute.end(), [&](const auto& b) {
      return covers(a, b) && covers(b, a);
    }));
  EXPECT_TRUE(std::any_of(brute.begin(), brute.end(), [](const auto& a) {
    return a.buffers >= 2 && a.resizes >= 2;
  }));
  EXPECT_EQ(result.truncated, 0u);
}
TEST(PbbiSolver, BudgetIsExplicitAndNoopSurvives)
{
  Node load;
  load.cap = 10;
  load.required = 10;
  Node wire;
  wire.kind = Kind::wire;
  wire.left = 0;
  wire.resistance = 1;
  wire.choices = {{1, 0, 0.1, 2, 100, true}, {2, 0, 0.01, 4, 100, true}};
  Limits limits;
  limits.max_states = 1;
  auto r = solve({load, wire}, 1, limits);
  EXPECT_GT(r.truncated, 0u);
  ASSERT_EQ(r.frontier.size(), 1u);
  EXPECT_EQ(r.frontier[0].buffers, 0u);
  limits.max_expansions = 1;
  EXPECT_TRUE(solve({load, wire}, 1, limits).exhausted);
  limits.max_expansions = 0;
  limits.max_states = 0;
  limits.edits = false;
  r = solve({load, wire}, 1, limits);
  ASSERT_EQ(r.frontier.size(), 1u);
  EXPECT_EQ(r.frontier[0].buffers, 0u);
  wire.left = 3;
  EXPECT_THROW(solve({load, wire}, 1), std::invalid_argument);
}
TEST(PbbiSolver, CostOrderedPruningPreservesTradeoffsAndRemovesSameCostEdits)
{
  Node load;
  load.cap = 1;
  load.required = 10;
  Node gate;
  gate.kind = Kind::gate;
  gate.left = 0;
  gate.choices = {{1, 1, 1, 0, 100, true},
                  {1, 1, 1, 0, 100, false},
                  {1, .5, .5, 1, 100, true},
                  {1, 2, 1, -1, 100, true}};
  for (bool slew : {false, true}) {
    Limits limits;
    limits.max_states = 0;
    if (slew) {
      limits.slew_grid = {0, 1, 2};
      limits.root_slew = {1, 1};
    }
    const auto result = solve({load, gate}, 1, limits);
    ASSERT_EQ(result.frontier.size(), 3u);
    EXPECT_EQ(result.frontier[0].trace->choice, 3);  // Cheap, slower edit.
    EXPECT_EQ(result.frontier[1].trace->choice, 1);  // Late no-op removes edit 0.
    EXPECT_EQ(result.frontier[2].trace->choice, 2);  // Faster, more expensive.
    EXPECT_EQ(result.frontier[1].resizes, 0u);
  }
}
TEST(PbbiSolver, DownsizeCanCompensateDownstreamArea)
{
  Node load;
  load.cap = 2;
  load.required = 10;
  load.choices = {{2, 0, 0, 4, 100, true}};
  Node gate;
  gate.kind = Kind::gate;
  gate.left = 0;
  gate.choices = {{1, 1, 1, -5, 100, true}};
  Limits limits;
  limits.max_cost = 0;
  auto r = solve({load, gate}, 1, limits);
  ASSERT_EQ(r.frontier.size(), 1u);
  EXPECT_EQ(r.frontier[0].cost, -1);
}

#include "repair_pbbi_timing.h"
TEST(PbbiSlew, RejectsFastButElectricallyInfeasibleGateBeforePruning)
{
  Node load;
  load.cap = 1;
  load.required = 100;
  load.slew_limit = 5;
  Node wire;
  wire.kind = Kind::wire;
  wire.left = 0;
  wire.resistance = 2;
  Node gate;
  gate.kind = Kind::gate;
  gate.left = 1;
  Choice fast{1, 1, 0, 0, 100, false}, slow{1, 3, 0, 1, 100, true};
  fast.response = [](double, double, unsigned rf) {
    ArcResponse r;
    r.delay[rf] = 1;
    r.slew[rf] = 4;
    return r;
  };
  slow.response = [](double, double, unsigned rf) {
    ArcResponse r;
    r.delay[rf] = 3;
    r.slew[rf] = 1;
    return r;
  };
  gate.choices = {fast, slow};
  Limits l;
  l.max_states = 0;
  l.slew_grid = {0, 1, 2, 3, 4, 5, 6};
  auto r = solve({load, wire, gate}, 2, l);
  ASSERT_EQ(r.frontier.size(), 1u);
  EXPECT_EQ(r.frontier[0].trace->choice, 1);
  EXPECT_NEAR(r.frontier[0].required, 95, 1e-12);
  EXPECT_GT(r.slew_rejected, 0u);
  auto simulated = simulate({load, wire, gate}, r.frontier[0].trace, {0, 0});
  EXPECT_NEAR(
      simulated.leaves[0].slew[0], std::hypot(1.0, 2 * std::log(9.0)), 1e-12);
}
TEST(PbbiSlew, PropagatesTransitionPolarityAndInputDependentDelayAcrossGates)
{
  Node load;
  load.cap = 1;
  load.required = 100;
  load.required_rf = {100, 80};
  load.slew_limit = 20;
  Node downstream;
  downstream.kind = Kind::gate;
  downstream.left = 0;
  Choice inv{1, 0, 0, 0, 100, false};
  inv.response = [](double, double slew, unsigned rf) {
    ArcResponse r;
    r.delay[1 - rf] = 2 + slew;
    r.slew[1 - rf] = slew / 2;
    return r;
  };
  downstream.choices = {inv};
  Node upstream;
  upstream.kind = Kind::gate;
  upstream.left = 1;
  Choice driver{1, 0, 0, 0, 100, false};
  driver.response = [](double, double slew, unsigned rf) {
    ArcResponse r;
    r.delay[rf] = 1 + slew;
    r.slew[rf] = 2 * slew;
    return r;
  };
  upstream.choices = {driver};
  Limits l;
  l.max_states = 0;
  l.slew_grid = {0, 1, 2, 3, 4, 6, 8, 12, 20};
  l.root_slew = {1, 2};
  auto r = solve({load, downstream, upstream}, 2, l);
  ASSERT_EQ(r.frontier.size(), 1u);
  // Root rise: 2 + (2+2) -> output fall; root fall: 3+(2+4) -> output rise.
  EXPECT_NEAR(r.frontier[0].required, std::min(80 - 6, 100 - 9), 1e-12);
  auto t = simulate({load, downstream, upstream}, r.frontier[0].trace, {1, 2});
  EXPECT_DOUBLE_EQ(t.leaves[0].arrival[0], 9);
  EXPECT_DOUBLE_EQ(t.leaves[0].arrival[1], 6);
  EXPECT_DOUBLE_EQ(t.leaves[0].slew[0], 2);
  EXPECT_DOUBLE_EQ(t.leaves[0].slew[1], 1);
}
TEST(PbbiSlew, KeepsCrossingSlewTradeoffsAndValidatesGrid)
{
  Node load;
  load.cap = 1;
  load.required = 100;
  Node gate;
  gate.kind = Kind::gate;
  gate.left = 0;
  Choice a{1, 0, 0, 0, 100, false}, b = a;
  a.response = [](double, double s, unsigned rf) {
    ArcResponse r;
    r.delay[rf] = 1 + 4 * s;
    r.slew[rf] = 1;
    return r;
  };
  b.response = [](double, double s, unsigned rf) {
    ArcResponse r;
    r.delay[rf] = 4 + s;
    r.slew[rf] = 1;
    return r;
  };
  gate.choices = {a, b};
  Limits l;
  l.max_states = 0;
  l.slew_grid = {0, 1, 2, 4};
  l.root_slew = {2, 2};
  auto r = solve({load, gate}, 1, l);
  ASSERT_EQ(r.frontier.size(), 2u);
  auto best = std::max_element(
      r.frontier.begin(), r.frontier.end(), [](auto& x, auto& y) {
        return x.required < y.required;
      });
  EXPECT_EQ(best->trace->choice, 1);
  EXPECT_DOUBLE_EQ(best->required, 94);
  l.slew_grid = {0, 2, 1};
  EXPECT_THROW(solve({load, gate}, 1, l), std::invalid_argument);
}
TEST(PbbiTiming, PropagatesForecastThroughReconvergenceAndBackwardToSidePaths)
{
  std::vector<TimingVertex> v(4);
  v[0].arrival = {0, 0};
  v[1].arrival = {10, 10};
  v[2].arrival = {8, 8};
  v[3].arrival = {15, 15};
  v[0].required = {5, 5};
  v[1].required = {15, 15};
  v[2].required = {18, 18};
  v[3].required = {20, 20};
  auto edge = [](unsigned from, unsigned to, double before, double after) {
    TimingEdge e;
    e.from = from;
    e.to = to;
    for (unsigned rf = 0; rf < 2; ++rf) {
      e.reference[rf].delay[rf] = before;
      e.reference[rf].slew[rf] = 1;
    }
    e.predict = [after](double s, unsigned rf) {
      ArcResponse r;
      r.delay[rf] = after;
      r.slew[rf] = s + 1;
      return r;
    };
    return e;
  };
  std::vector<TimingEdge> e{
      edge(0, 1, 10, 2), edge(0, 2, 8, 8), edge(1, 3, 5, 5), edge(2, 3, 2, 2)};
  auto p = propagate(v, e);
  EXPECT_DOUBLE_EQ(p.vertices[3].arrival[0], 10);  // critical branch switches
  EXPECT_DOUBLE_EQ(p.vertices[0].required[0], 10);
  EXPECT_EQ(p.predecessor[3][0], 3);
  EXPECT_DOUBLE_EQ(p.vertices[3].slew[0], 2);
  for (auto& x : e)
    x.predict = {};
  auto identity = propagate(v, e);
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_EQ(identity.vertices[i].arrival, v[i].arrival);
    EXPECT_EQ(identity.vertices[i].required, v[i].required);
  }
  e.push_back(edge(3, 0, 1, 1));
  EXPECT_THROW(propagate(v, e), std::invalid_argument);
}
TEST(PbbiTiming, LaunchSlewChangesDownstreamDelayAndRequired)
{
  std::vector<TimingVertex> v(2);
  v[0].arrival = {0, 0};
  v[0].required = {9, 9};
  v[0].slew = {1, 1};
  v[1].arrival = {1, 1};
  v[1].required = {10, 10};
  v[1].slew = {2, 2};
  TimingEdge e;
  e.from = 0;
  e.to = 1;
  for (unsigned rf = 0; rf < 2; ++rf) {
    e.reference[rf].delay[rf] = 1;
    e.reference[rf].slew[rf] = 2;
  }
  e.predict = [](double s, unsigned rf) {
    ArcResponse r;
    r.delay[rf] = s;
    r.slew[rf] = s + 1;
    return r;
  };
  auto launch = v;
  launch[0].arrival = {2, 3};
  launch[0].slew = {3, 4};
  auto p = propagate(v, {e}, launch);
  EXPECT_DOUBLE_EQ(p.vertices[1].arrival[0], 5);
  EXPECT_DOUBLE_EQ(p.vertices[1].arrival[1], 7);
  EXPECT_DOUBLE_EQ(p.vertices[0].required[0], 7);
  EXPECT_DOUBLE_EQ(p.vertices[0].required[1], 6);
  EXPECT_DOUBLE_EQ(p.vertices[1].slew[0], 4);
  EXPECT_DOUBLE_EQ(p.vertices[1].slew[1], 5);
}
TEST(PbbiSlew, ResolvesFeasibleSlewInsideIntervalEndingAtViolation)
{
  Node load;
  load.cap = 1;
  load.required = 20;
  load.slew_limit = 4;
  Node gate;
  gate.kind = Kind::gate;
  gate.left = 0;
  Choice c{1, 1, 0, 0, 100, false};
  c.response = [](double, double, unsigned rf) {
    ArcResponse r;
    r.delay[rf] = 1;
    r.slew[rf] = 3;
    return r;
  };
  gate.choices = {c};
  Limits l;
  l.max_states = 0;
  l.slew_grid = {0, 10};
  l.root_slew = {2, 2};
  auto r = solve({load, gate}, 1, l);
  ASSERT_EQ(r.frontier.size(), 1u);
  EXPECT_DOUBLE_EQ(r.frontier[0].required, 19);
  // solve's temporary node vector is already destroyed. Model ownership must
  // keep exact boundary evaluation safe for consumers of the returned state.
  EXPECT_DOUBLE_EQ(requiredAt(r.frontier[0], l.slew_grid, 5, 0), 19);
}
TEST(PbbiSlew, ForwardSlewIsInvariantToWireSubdivision)
{
  Node load;
  load.cap = 1;
  load.required = 100;
  load.required_rf = {100, INFINITY};
  Node whole;
  whole.kind = Kind::wire;
  whole.left = 0;
  whole.resistance = 2;
  Node first = whole;
  first.resistance = 1;
  Node second = first;
  second.left = 1;
  Limits l;
  l.edits = false;
  auto a = solve({load, whole}, 1, l), b = solve({load, first, second}, 2, l);
  auto x = simulate({load, whole}, a.frontier[0].trace, {1, 3});
  auto y = simulate({load, first, second}, b.frontier[0].trace, {1, 3});
  EXPECT_DOUBLE_EQ(x.leaves[0].arrival[0], y.leaves[0].arrival[0]);
  EXPECT_DOUBLE_EQ(x.leaves[0].slew[0], y.leaves[0].slew[0]);
  EXPECT_DOUBLE_EQ(x.leaves[0].slew[1], y.leaves[0].slew[1]);
  EXPECT_NEAR(x.leaves[0].slew[0], std::hypot(1.0, 2 * std::log(9.0)), 1e-12);
  EXPECT_NEAR(x.leaves[0].slew[1], std::hypot(3.0, 2 * std::log(9.0)), 1e-12);
  EXPECT_LT(x.leaves[0].slew[0], x.leaves[0].slew[1]);
  EXPECT_DOUBLE_EQ(load.requiredFor(1), INFINITY);
}
TEST(PbbiSlew, ExactCachePreservesFrontierEvenWithForcedCollisions)
{
  Node load;
  load.cap = 1;
  load.required = 20;
  load.slew_limit = 4;
  Node gate;
  gate.kind = Kind::gate;
  gate.left = 0;
  for (unsigned i = 0; i < 4; ++i) {
    Choice c{1.0 + i, 1.0 + i, 0, double(i), 100, i != 0};
    c.response = [i](double, double, unsigned rf) {
      ArcResponse r;
      r.delay[rf] = 1 + i;
      r.slew[rf] = 3;
      return r;
    };
    gate.choices.push_back(c);
  }
  Limits l;
  l.max_states = 0;
  l.slew_grid = {0, 10, 20};
  l.root_slew = {1, 2};
  l.max_slew_cache_entries = 0;
  auto ref = solve({load, gate}, 1, l);
  ASSERT_FALSE(ref.frontier.empty());
  for (size_t capacity : {1u, 64u}) {
    l.max_slew_cache_entries = capacity;
    auto cached = solve({load, gate}, 1, l);
    ASSERT_EQ(cached.frontier.size(), ref.frontier.size());
    for (size_t i = 0; i < ref.frontier.size(); ++i) {
      EXPECT_EQ(cached.frontier[i].required, ref.frontier[i].required);
      EXPECT_EQ(cached.frontier[i].cap, ref.frontier[i].cap);
      EXPECT_EQ(cached.frontier[i].cost, ref.frontier[i].cost);
      EXPECT_EQ(cached.frontier[i].trace->choice,
                ref.frontier[i].trace->choice);
      EXPECT_EQ(*cached.frontier[i].required_curve,
                *ref.frontier[i].required_curve);
    }
    if (capacity == 64)
      EXPECT_GT(cached.slew_cache_hits, 0u);
  }
}
TEST(PbbiSlew, TruncationRanksAtActualOperatingSlew)
{
  Node load;load.cap=1;load.required=100;
  Node gate;gate.kind=Kind::gate;gate.left=0;gate.reference_slew={0,0}; // Root uses actual launch slew, not this fallback.
  for(unsigned i=0;i<3;++i) {
    Choice c{1,i==0?20.0:i==1?1.0:3.0,0,double(i),100,i!=0};
    c.response=[i](double,double slew,unsigned rf){
      ArcResponse r;r.delay[rf]=i==0?20:i==1?1+8*slew:3+slew;r.slew[rf]=slew;return r;
    };
    gate.choices.push_back(c);
  }
  Limits l;l.max_states=2;l.slew_grid={0,1,2};l.root_slew={1,1};
  l.operating_point_pruning=false;auto zero=solve({load,gate},1,l);
  l.operating_point_pruning=true;auto operating=solve({load,gate},1,l);
  auto best=[](const Result& r){return std::max_element(r.frontier.begin(),r.frontier.end(),[](const auto& a,const auto& b){return a.required<b.required;});};
  ASSERT_FALSE(zero.frontier.empty());ASSERT_FALSE(operating.frontier.empty());
  EXPECT_EQ(best(zero)->trace->choice,1);EXPECT_EQ(best(operating)->trace->choice,2);
  EXPECT_GT(best(operating)->required,best(zero)->required);
  EXPECT_GT(operating.truncated,0u);
}

TEST(PbbiScheduler, GlobalBenefitPrecedesPathRankAndRejectsPredictedRegression) {
  Priority a,b;
  a.path_rank=1;a.wns_gain=1;a.tns_gain=1;
  b.path_rank=5;b.wns_gain=2;b.tns_gain=2;
  EXPECT_TRUE(preferred(b,a,0,0));
  b.tns_gain=-1;
  EXPECT_TRUE(preferred(a,b,0,0));
  b.tns_gain=2;b.severity=0.01;
  EXPECT_TRUE(preferred(a,b,0,0));
  b.severity=0;b.failed_actions=1;
  EXPECT_TRUE(preferred(a,b,0,0));
  b=a;b.cost=1;
  EXPECT_TRUE(preferred(a,b,0,0));
  EXPECT_FALSE(preferred(a,a,0,0));
}
TEST(PbbiScheduler, DifferentNetsCanShareEndpointsOrSizedGateInputs) {
  Footprint a,b;
  a.nets.insert("net_a");b.nets.insert("net_b");
  EXPECT_FALSE(a.overlaps(b));
  a.endpoints.insert("capture/D");b.endpoints.insert("capture/D");
  EXPECT_TRUE(a.overlaps(b));
  b.endpoints.clear();a.nets.insert("other_input");b.nets.insert("other_input");
  EXPECT_TRUE(a.overlaps(b));
}

TEST(PbbiSolver, CancellationNeverReturnsPartialFrontierAndCanRetry) {
  Node load;load.cap=1;load.required=100;
  Node gate;gate.kind=Kind::gate;gate.left=0;
  for(unsigned i=0;i<16;++i)
    gate.choices.push_back({1.0+i,16.0-i,0,double(i),100,i!=0});
  Limits limits;limits.max_states=0;
  const auto reference=solve({load,gate},1,limits);
  ASSERT_GT(reference.frontier.size(),1u);
  // Cover cancellation before expansion, within prune and during root
  // validation. An aborted DP must never escape as a usable partial plan.
  for(unsigned threshold=1;threshold<=20;++threshold) {
    unsigned calls=0;
    limits.stop_requested=[&](){return ++calls>=threshold;};
    auto aborted=solve({load,gate},1,limits);
    EXPECT_TRUE(aborted.cancelled)<<threshold;
    EXPECT_TRUE(aborted.exhausted);
    EXPECT_TRUE(aborted.frontier.empty());
  }
  limits.stop_requested=[](){return false;};
  auto retry=solve({load,gate},1,limits);
  EXPECT_FALSE(retry.cancelled);
  ASSERT_EQ(retry.frontier.size(),reference.frontier.size());
  for(size_t i=0;i<retry.frontier.size();++i) {
    EXPECT_EQ(retry.frontier[i].required,reference.frontier[i].required);
    EXPECT_EQ(retry.frontier[i].trace->choice,reference.frontier[i].trace->choice);
  }
}

TEST(PbbiTiming, ReusedForwardGraphMatchesFullProjectionAcrossCandidateChanges) {
  std::vector<TimingVertex> ref(5);
  for(unsigned i=0;i<ref.size();++i) {
    ref[i].arrival={double(i),double(i)+0.5};
    ref[i].slew={1,2};ref[i].required={20,21};
  }
  std::vector<TimingEdge> edges;
  for(auto [a,b]:std::vector<std::pair<unsigned,unsigned>>{{0,1},{0,2},{1,3},{2,3},{3,4}}) {
    TimingEdge e;e.from=a;e.to=b;
    for(unsigned in=0;in<2;++in)
      for(unsigned out=0;out<2;++out) {
        e.reference[in].delay[out]=1+in+out;
        e.reference[in].slew[out]=2+in+out;
      }
    edges.push_back(e);
  }
  TimingGraph cached(ref,edges);
  for(unsigned trial=0;trial<6;++trial) {
    auto launch=ref;launch[0].arrival[0]+=trial;launch[0].slew[1]+=trial;
    for(unsigned i=0;i<edges.size();++i)
      edges[i].predict=[i,trial](double slew,unsigned in) {
        ArcResponse r;
        for(unsigned out=0;out<2;++out) {
          r.delay[out]=1+in+out+(i%2?trial:-double(trial))*0.1+slew*0.2;
          r.slew[out]=slew+0.5+out;
        }
        return r;
      };
    auto full=propagate(ref,edges,launch);
    auto fast=cached.forward(launch,[&](unsigned i,double slew,unsigned rf){return edges[i].predict(slew,rf);});
    ASSERT_EQ(fast.size(),full.vertices.size());
    for(unsigned i=0;i<fast.size();++i) {
      EXPECT_EQ(fast[i].arrival,full.vertices[i].arrival);
      EXPECT_EQ(fast[i].slew,full.vertices[i].slew);
      EXPECT_EQ(fast[i].required,launch[i].required); // Fixed acceptance constraints.
    }
  }
  EXPECT_THROW(cached.forward(std::vector<TimingVertex>(1),{}),std::invalid_argument);
  edges.push_back(TimingEdge{4,0});
  EXPECT_THROW((TimingGraph(ref,edges)),std::invalid_argument);
}

TEST(PbbiSchedule, AreaPriorityRequiresTimingTargetBeforeChoosingCost) {
  TimingTarget target{-0.1,-10,1e-6,1e-5};
  EXPECT_TRUE(target.met(-0.1,-10));
  EXPECT_FALSE(target.met(-0.11,-9));EXPECT_FALSE(target.met(-0.09,-11));
  EXPECT_FALSE(target.met(INFINITY,-9));EXPECT_FALSE(target.met(NAN,-9));
  Priority cheap,fast;
  cheap.cost=-2;cheap.wns_gain=-0.01;cheap.tns_gain=-1;
  fast.cost=5;fast.wns_gain=0.01;fast.tns_gain=1;fast.target_met=true;
  EXPECT_TRUE(preferred(fast,cheap,0,0,true));
  cheap.target_met=true;
  EXPECT_TRUE(preferred(cheap,fast,0,0,true));
  EXPECT_TRUE(preferred(fast,cheap,0,0)); // Existing timing policy unchanged.
  cheap.severity=1;
  EXPECT_TRUE(preferred(fast,cheap,0,0,true)); // Electrical safety stays first.
  EXPECT_FALSE(preferred(fast,fast,0,0,true));
}

TEST(PbbiObjective, DirectTransitionCanSpendSurplusButMustMeetBothTargets) {
  const TimingTarget target{-0.1105949807481771,-34.23451841132854,1e-6,3.5e-5};
  const SetupPoint before{-0.115276,-27.5271,100};
  const SetupPoint candidate{-0.108245,-27.7829,99};
  EXPECT_FALSE(target.met(before.wns,before.tns));
  EXPECT_TRUE(target.met(candidate.wns,candidate.tns));
  EXPECT_TRUE(setupRejections(before,candidate,target,true,1e-4,3e-5).empty());
  EXPECT_FALSE(setupRejections(before,candidate,target,false,1e-4,3e-5).empty());
  auto misses_tns=candidate;misses_tns.tns=-35;
  EXPECT_FALSE(setupRejections(before,misses_tns,target,true,1e-4,3e-5).empty());
  auto misses_wns=candidate;misses_wns.wns=-0.12;
  EXPECT_FALSE(setupRejections(before,misses_wns,target,true,1e-4,3e-5).empty());
  auto cheaper=candidate;cheaper.wns=-0.11;cheaper.tns=-34;cheaper.area=98;
  EXPECT_TRUE(setupRejections(candidate,cheaper,target,true,1e-4,3e-5).empty());
  cheaper.area=candidate.area;
  EXPECT_FALSE(setupRejections(candidate,cheaper,target,true,1e-4,3e-5).empty());
}

TEST(PbbiScheduler, SetupFeasibilityPrecedesWnsGainWithoutDroppingZeroWnsPlans) {
  Priority tempting, feasible;
  // A measured AES failure: better predicted WNS but worse TNS. A modest
  // improvement preserving both constraints should get the scarce STA trial.
  tempting.wns_gain=0.000800229;tempting.tns_gain=-0.385859;
  feasible.wns_gain=0;feasible.tns_gain=0.01;
  EXPECT_TRUE(predictedSetupRegresses(tempting.wns_gain,tempting.tns_gain,1e-6,3e-5));
  EXPECT_FALSE(predictedSetupRegresses(feasible.wns_gain,feasible.tns_gain,1e-6,3e-5));
  EXPECT_TRUE(preferred(feasible,tempting,1e-6,3e-5));
  // Tolerance boundary is inclusive, and zero WNS gain remains eligible.
  EXPECT_FALSE(predictedSetupRegresses(-1e-6,-3e-5,1e-6,3e-5));
  EXPECT_TRUE(predictedSetupRegresses(0,NAN,1e-6,3e-5));
}

TEST(PbbiTiming, OffPathForecastCannotDoubleSpendExistingNegativeSlack) {
  // Actual RAT=10, AT=15 (slack=-5). A future branch predicts RAT=20.
  // Adding the old violation to that forecast gave 25, beyond the committed
  // boundary 15. Preserve the boundary without requiring all violations fixed.
  EXPECT_DOUBLE_EQ(offPathRequired(20,10,-5),15);
  EXPECT_DOUBLE_EQ(offPathRequired(10,10,-5),15);
  // Positive slack may be spent only up to actual RAT; an earlier forecast
  // remains conservative. Rise/fall constraints are applied independently.
  EXPECT_DOUBLE_EQ(offPathRequired(20,10,2),10);
  EXPECT_DOUBLE_EQ(offPathRequired(8,10,2),8);
  EXPECT_DOUBLE_EQ(offPathRequired(8,10,-5),13);
  EXPECT_DOUBLE_EQ(offPathRequired(20,INFINITY,INFINITY),20);
}
