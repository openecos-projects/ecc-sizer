// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair/path_feedback.h"
#include <gtest/gtest.h>

using namespace ecc::path_detail;

TEST(PathCoverage, EndpointProfileMatchesExplicitTnsIncludingNewViolations) {
    EndpointSlackProfile profile;
    const std::vector<double> slacks{-7, -2, -2, -.1, 0, .3, 8};
    for(double s : slacks) profile.add(s);
    profile.add(INFINITY);
    profile.add(NAN);
    profile.finish();
    EXPECT_EQ(profile.size(), slacks.size());
    for(double gain : {-10., -3., -.3, 0., .1, 2., 7., 10.}) {
        double expected = 0;
        for(double s : slacks) expected += std::min(0., s + gain) - std::min(0., s);
        EXPECT_NEAR(profile.gain(gain), expected, 1e-12);
    }
    EndpointSlackProfile empty;
    empty.finish();
    EXPECT_DOUBLE_EQ(empty.gain(3), 0);
}

TEST(PathCoverage, LoadGroupMemorySurvivesUnrelatedWorkAndReopensOnStructuralChange) {
    ContextCoverage coverage;
    coverage.observe("net-a/loads", "driver-and-loads-a");
    coverage.observe("net-b/loads", "driver-and-loads-b");
    EXPECT_EQ(coverage.attempts("net-a/loads", "driver-and-loads-a"), 1u);
    EXPECT_EQ(coverage.attempts("net-a/other-loads", "driver-and-loads-a"), 0u);
    EXPECT_EQ(coverage.attempts("net-a/loads", "new-driver"), 0u);
    coverage.observe("net-a/loads", "new-driver");
    EXPECT_EQ(coverage.attempts("net-a/loads", "new-driver"), 1u);
    EXPECT_EQ(coverage.attempts("net-a/loads", "driver-and-loads-a"), 0u);
}

TEST(PathFrontier, RequiresMeasuredDominanceAndAnIndependentAudit) {
    ExplorationFrontier frontier;
    using D = ExplorationFrontier::Decision;
    EXPECT_EQ(frontier.decide({.2, .3}, {1, 1}), D::Explore);
    frontier.observe({.4, .5}, {0, 0}, true, false);
    EXPECT_EQ(frontier.decide({.2, .3}, {1, 1}), D::Audit);
    frontier.observe({.3, .4}, {0, 0}, true, true);
    EXPECT_EQ(frontier.decide({.2, .3}, {1, 1}), D::Suspend);
    // A stronger WNS or TNS opportunity is outside observed coverage.
    EXPECT_EQ(frontier.decide({.6, .1}, {1, 1}), D::Explore);
    EXPECT_EQ(frontier.decide({.1, .6}, {1, 1}), D::Explore);
}

TEST(PathFrontier, ReopensAfterUnderpredictionAndPreservesTnsOnlyOpportunities) {
    ExplorationFrontier frontier;
    using D = ExplorationFrontier::Decision;
    frontier.observe({0, .5}, {0, .7}, true, false);
    // A new measurement enlarges the empirical error envelope: no suspension.
    frontier.observe({0, .4}, {0, .9}, true, true);
    EXPECT_EQ(frontier.decide({0, .3}, {1, 1}), D::Audit);
    frontier.observe({0, .3}, {0, .7}, true, true);
    EXPECT_EQ(frontier.decide({0, .2}, {1, 1}), D::Suspend);
    EXPECT_EQ(frontier.decide({0, .2}, {1, .6}), D::Explore);
    // Reset on commit invalidates all empirical coverage, including audits.
    frontier = {};
    EXPECT_EQ(frontier.decide({0, .2}, {1, 1}), D::Explore);
}

TEST(PathFrontier, DoesNotTreatAnAcceptanceOrInvalidMeasurementAsFailureEvidence) {
    ExplorationFrontier frontier;
    using D = ExplorationFrontier::Decision;
    frontier.observe({1, 2}, {2, 4}, false, true);
    EXPECT_EQ(frontier.decide({.1, .2}, {10, 10}), D::Explore);
    frontier.observe({INFINITY, 2}, {0, 0}, true, true);
    EXPECT_EQ(frontier.decide({.1, .2}, {10, 10}), D::Explore);
    EXPECT_EQ(frontier.decide({NAN, .2}, {10, 10}), D::Explore);
}

TEST(PathFrontier, LargerResidualReopensPreviouslySuspendedSiblings) {
    ExplorationFrontier frontier;
    using D = ExplorationFrontier::Decision;
    frontier.observe({.6, .6}, {0, 0}, true, false);
    frontier.observe({.5, .5}, {0, 0}, true, true);
    EXPECT_EQ(frontier.decide({.4, .4}, {1, 1}), D::Suspend);
    // A different tradeoff is not covered and must still be measured.
    EXPECT_EQ(frontier.decide({.7, .3}, {1, 1}), D::Explore);
    frontier.observe({.7, .3}, {0, .95}, true, false);
    EXPECT_EQ(frontier.decide({.4, .4}, {1, 1}), D::Explore);
}

TEST(PathFingerprint, CanonicalizesLoadsAndChildOrder) {
    std::vector<ActionNode> a{{"b1", 1, 2, {"p2", "p1"}, {}},
                              {"b2", 3, 4, {"p3"}, {}},
                              {"b3", 5, 6, {}, {0, 1}}};
    auto b = a;
    std::reverse(b[0].terminals.begin(), b[0].terminals.end());
    std::reverse(b[2].children.begin(), b[2].children.end());
    EXPECT_EQ(actionFingerprint("net", a), actionFingerprint("net", b));
    std::swap(b[0], b[1]); // The root's children still refer to both subtrees.
    EXPECT_EQ(actionFingerprint("net", a), actionFingerprint("net", b));
}

TEST(PathFingerprint, DistinguishesTopologyCellPositionAndTerminalMapping) {
    std::vector<ActionNode> a{{"b1", 1, 2, {"p1"}, {}},
                              {"b2", 3, 4, {"p2"}, {0}}};
    auto b = a;
    b[1].children.clear(); b[1].terminals.push_back("p1");
    EXPECT_NE(actionFingerprint("net", a), actionFingerprint("net", b));
    b = a; b[0].master = "b2";
    EXPECT_NE(actionFingerprint("net", a), actionFingerprint("net", b));
    EXPECT_EQ(actionFingerprint("net", a, false), actionFingerprint("net", b, false));
    b = a; ++b[0].x;
    EXPECT_NE(actionFingerprint("net", a), actionFingerprint("net", b));
    b = a; std::swap(b[0].terminals, b[1].terminals);
    EXPECT_NE(actionFingerprint("net", a), actionFingerprint("net", b));
    EXPECT_NE(actionFingerprint("net", a), actionFingerprint("other", a));
}

TEST(PathObjective, UnaffectedGlobalPathLimitsLocalImprovement) {
    const auto result = forecastObjective({{0, -2, -2, false}, {1, -.1, -.01, true}}, {}, -2);
    EXPECT_DOUBLE_EQ(result.wns_gain, 0);
    EXPECT_NEAR(result.tns_gain, .09, 1e-12);
    EXPECT_EQ(result.affected_endpoints, 1u);
}

TEST(PathObjective, KeepsAlternativePathAtSameEndpoint) {
    const auto result = forecastObjective({{0, -2, -.5, true}, {0, -1.9, -1.9, false}}, {}, -2);
    EXPECT_NEAR(result.wns_gain, .1, 1e-12);
    EXPECT_NEAR(result.tns_gain, .1, 1e-12);
    EXPECT_EQ(result.affected_endpoints, 1u);
}

TEST(PathObjective, MergesChecksWithoutDoubleCountingEndpointTns) {
    const auto result = forecastObjective({{0, -2, -1, true}, {0, -1.5, -.5, true}}, {}, -2);
    EXPECT_DOUBLE_EQ(result.wns_gain, 1);
    EXPECT_DOUBLE_EQ(result.tns_gain, 1);
    EXPECT_EQ(result.affected_endpoints, 1u);
}

TEST(PathObjective, PreservesUnsampledCriticalCheckAndGlobalBottleneck) {
    auto result = forecastObjective({{0, -1, -.5, true}}, {{0, -2}}, -2);
    EXPECT_DOUBLE_EQ(result.wns_gain, 0);
    EXPECT_DOUBLE_EQ(result.tns_gain, 0);
    result = forecastObjective({{0, -1, -.5, true}}, {}, -3);
    EXPECT_DOUBLE_EQ(result.wns_gain, 0);
    EXPECT_DOUBLE_EQ(result.tns_gain, .5);
}

TEST(PathObjective, IncludesDamageAndClampsRepairedSlackAtZero) {
    const auto result = forecastObjective({{0, -1, 1, true}, {1, -.1, -1.2, true}}, {}, -1);
    EXPECT_NEAR(result.wns_gain, -.2, 1e-12);
    EXPECT_NEAR(result.tns_gain, -.1, 1e-12);
}

TEST(PathObjective, KeepsStrengthInformationWhenGlobalGainSaturates) {
    // Both trees repair the sampled recovery endpoint; the unaffected setup
    // path becomes worst. A smaller cell must not win just because global
    // WNS and clipped TNS no longer distinguish their path margins.
    const auto weak = forecastObjective({{0, -188, 20, true}, {1, -8, -8, false}}, {}, -188);
    const auto strong = forecastObjective({{0, -188, 64, true}, {1, -8, -8, false}}, {}, -188);
    EXPECT_DOUBLE_EQ(weak.wns_gain, 180);
    EXPECT_DOUBLE_EQ(strong.wns_gain, weak.wns_gain);
    EXPECT_DOUBLE_EQ(strong.tns_gain, weak.tns_gain);
    EXPECT_GT(strong.path_gain, weak.path_gain);
    EXPECT_TRUE(betterElectricalAlternative({0, strong.path_gain}, {0, weak.path_gain}));
    // This is tie-breaking evidence, not permission to claim global 252 gain.
    EXPECT_DOUBLE_EQ(strong.path_gain, 252);
    EXPECT_LT(strong.wns_gain, strong.path_gain);
}

TEST(PathObjective, AffectedMarginIncludesDamagedPathsAndIgnoresUnchangedOnes) {
    const auto result = forecastObjective(
        {{0, -5, 1, true}, {1, -.1, -2, true}, {2, -10, -10, false}}, {}, -10);
    EXPECT_DOUBLE_EQ(result.wns_gain, 0);
    EXPECT_DOUBLE_EQ(result.path_gain, 3);
    EXPECT_DOUBLE_EQ(forecastObjective({{0, -1, -1, false}}, {}, -1).path_gain, 0);
}

TEST(PathFeedback, ElectricalAlternativesTradeMarginOnlyAfterElectricalRisk) {
    EXPECT_TRUE(betterElectricalAlternative({0, 2}, {.5, 5}));
    EXPECT_FALSE(betterElectricalAlternative({.5, 5}, {0, 2}));
    EXPECT_TRUE(betterElectricalAlternative({0, 5}, {0, 2}));
    EXPECT_FALSE(betterElectricalAlternative({0, 2}, {0, 2}));
}

TEST(PathFeedback, DistinguishesDuplicateDeferredAndInvalidatedFailures) {
    FailureHistory history;
    EXPECT_EQ(history.status("action", "electrical+critical-check", 0), Retry::Fresh);
    history.record("action", {"electrical+critical-check", {"no_setup_gain"}, 0, 0, .02});
    EXPECT_EQ(history.status("action", "electrical+critical-check", 0), Retry::Duplicate);
    EXPECT_EQ(history.status("action", "electrical+critical-check", 1), Retry::Deferred);
    EXPECT_EQ(history.status("action", "changed-driver-or-bottleneck", 1), Retry::Invalidated);
    EXPECT_EQ(history.status("different-tree", "electrical+critical-check", 1), Retry::Fresh);
    ASSERT_NE(history.find("action"), nullptr);
    EXPECT_DOUBLE_EQ(history.find("action")->tns_gain, .02);
}

TEST(PathFeedback, CalibratesOverestimatesAndZeroPredictionsFromMeasurements) {
    GainCalibration gain;
    EXPECT_DOUBLE_EQ(gain.predict(10), 10);
    gain.observe(10, 1); gain.observe(20, 2);
    EXPECT_DOUBLE_EQ(gain.predict(30), 3);
    GainCalibration missed;
    missed.observe(0, 2); missed.observe(0, 4);
    EXPECT_DOUBLE_EQ(missed.predict(0), 3);
    missed.observe(INFINITY, 2);
    EXPECT_EQ(missed.samples, 2u);
}

TEST(PathFeedback, CoversOtherFamiliesBeforeRepeatingVariantsWithoutACap) {
    FamilyCoverage coverage;
    EXPECT_EQ(coverage.attempts("net/group-a", 0), 0u);
    coverage.observe("net/group-a", 0);
    EXPECT_EQ(coverage.attempts("net/group-a", 0), 1u);
    EXPECT_EQ(coverage.attempts("net/group-b", 0), 0u);
    for(unsigned i = 0; i < 100; ++i) coverage.observe("net/group-a", 0);
    EXPECT_EQ(coverage.attempts("net/group-a", 0), 101u);
    // A newly committed design may make a previously poor family valuable.
    EXPECT_EQ(coverage.attempts("net/group-a", 1), 0u);
    coverage.observe("net/group-a", 1);
    EXPECT_EQ(coverage.attempts("net/group-a", 1), 1u);
}

TEST(PathFeedback, ReopensWhenForecastImprovesBeyondExistingAcceptanceScale) {
    FailureHistory history;
    history.record("action", {"topology", {"no_setup_gain"}, 0, 0, .02, .1, .2});
    EXPECT_FALSE(history.opportunityChanged("action", .1, .2, .01, .1));
    EXPECT_FALSE(history.opportunityChanged("action", .105, .25, .01, .1));
    EXPECT_TRUE(history.opportunityChanged("action", .12, .2, .01, .1));
    EXPECT_TRUE(history.opportunityChanged("action", .1, .4, .01, .1));
    EXPECT_EQ(history.status("action", "topology", 1), Retry::Deferred);
    EXPECT_EQ(history.status("action", "changed-topology", 1), Retry::Invalidated);
}

TEST(PathFeedback, FullyVerifiesEveryAcceptanceAndElectricalOnlyRejection) {
    EXPECT_TRUE(needsFullVerification({}));
    EXPECT_TRUE(needsFullVerification({"drv_severity"}));
    EXPECT_TRUE(needsFullVerification({"cap", "slew", "fanout"}));
    EXPECT_FALSE(needsFullVerification({"setup_wns", "drv_severity"}));
    EXPECT_FALSE(needsFullVerification({"no_setup_gain"}));
    EXPECT_FALSE(needsFullVerification({"hold_wns"}));
}

TEST(PathObjective, PreservesTnsOnlyGainWhenAnotherPathRemainsWorst) {
    const auto result = forecastObjective(
        {{0, -2, -2, false}, {1, -1, -.2, true}, {2, -.5, -.1, true}}, {}, -2);
    EXPECT_DOUBLE_EQ(result.wns_gain, 0);
    EXPECT_NEAR(result.tns_gain, 1.2, 1e-12);
    EXPECT_EQ(result.affected_endpoints, 2u);
}

TEST(PathFeedback, ReusesTnsRegressionOnlyUntilItsDependencyConeChanges) {
    DependencyFailures cache;
    cache.record("same full tree", {1, 2, 3}, -100e-9f, -101e-9f);
    EXPECT_TRUE(cache.rejects("same full tree", -80e-9f, 1e-12));
    EXPECT_FALSE(cache.rejects("different master or position", -80e-9f, 1e-12));
    cache.committed({8, 9});
    EXPECT_TRUE(cache.rejects("same full tree", -60e-9f, 1e-12));
    cache.committed({3, 7}); // A shared endpoint also invalidates evidence.
    EXPECT_FALSE(cache.rejects("same full tree", -60e-9f, 1e-12));
    cache.record("no gain", {1}, -100e-9f, -99e-9f);
    EXPECT_FALSE(cache.rejects("no gain", -100e-9f, 1e-12));
}

TEST(PathFeedback, DoesNotReplayTnsLossNearRoundingOrCurrentTolerance) {
    DependencyFailures cache;
    const float before = -100e-9f;
    const float after = std::nextafter(before, -INFINITY);
    cache.record("roundoff", {1}, before, after);
    EXPECT_FALSE(cache.rejects("roundoff", before, 0));
    cache.record("regression", {1}, -100e-9f, -101e-9f);
    EXPECT_FALSE(cache.rejects("regression", before, 2e-9));
    EXPECT_FALSE(cache.rejects("regression", -INFINITY, 0));
}

TEST(PathFeedback, EndpointWitnessSurvivesUnrelatedEditsAndReopensForFaninChanges) {
    EndpointFailures cache;
    cache.record("tree", std::make_shared<const DependencyFailures::Cone>(DependencyFailures::Cone{1, 2, 3}), -2e-9f);
    EXPECT_TRUE(cache.rejects("tree", -1e-9f, 1e-12));
    EXPECT_FALSE(cache.rejects("tree", -3e-9f, 1e-12));
    EXPECT_FALSE(cache.rejects("different tree", -1e-9f, 1e-12));
    cache.committed(8);
    EXPECT_TRUE(cache.rejects("tree", -1e-9f, 1e-12));
    cache.committed(3);
    EXPECT_FALSE(cache.rejects("tree", -1e-9f, 1e-12));
}
