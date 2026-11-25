// SPDX-License-Identifier: Apache-2.0
#include "algorithms/pbbi.h"
#include "gtest/gtest.h"

using namespace ecc::pbbi;

static Tree twoNets() {
    Tree tree;
    tree.nets = {nullptr, nullptr};
    tree.driver_resistance = 3;
    tree.nodes = {
        {.cap = 10, .required = 100, .net = 1},
        {.kind = NodeKind::Wire, .left = 0, .cap = 2, .resistance = 4, .net = 1},
        {.kind = NodeKind::Gate,
         .left = 1,
         .cap = 2,
         .resistance = 3,
         .intrinsic = 1,
         .net = 0},
        {.kind = NodeKind::Wire, .left = 2, .cap = 2, .resistance = 4, .net = 0}};
    return tree;
}

TEST(Pbbi, CrossGateOptimizationKeepsCostAndSeparatesNetTraceback) {
    const auto tree = twoNets();
    const std::vector<Buffer> buffers{{.cap = 1, .resistance = .5, .intrinsic = 1}};
    ecc::PbbiOptions options;
    options.max_frontier.reset();
    // Exhaustive scores: none=-5, upstream only=1, downstream only=21, both=27.
    for (const auto [target, expected_cost, expected_score] :
         {std::tuple{0.0, 1u, 21.0},
          {22.0, 2u, 27.0},
          {30.0, 2u, 27.0},
          {-6.0, 0u, -5.0}}) {
        const auto result = solve(tree, buffers, target, options);
        ASSERT_GE(result.selected, 0);
        EXPECT_EQ(result.frontiers.back()[result.selected].cost, expected_cost);
        EXPECT_DOUBLE_EQ(result.score, expected_score);
        const auto plans = decode(tree, buffers, result);
        unsigned count = 0;
        for (const auto& plan : plans) {
            count += plan.nodes.size();
        }
        EXPECT_EQ(count, expected_cost);
        if (expected_cost == 2) {
            ASSERT_EQ(plans.size(), 2);
            // The fixed gate is a terminal on the upstream net, never a child
            // buffer connecting two nets or a new inserted instance.
            EXPECT_EQ(plans[0].nodes.size(), 1);
            EXPECT_EQ(plans[1].nodes.size(), 1);
            EXPECT_TRUE(plans[0].nodes[0].children.empty());
        }
    }
}

TEST(Pbbi, SideBranchesAndBlockedSitesConstrainTheSolution) {
    auto tree = twoNets();
    tree.nodes[1].can_buffer = false;
    const std::vector<Buffer> buffers{{.cap = 1, .resistance = .5, .intrinsic = 1}};
    ecc::PbbiOptions options;
    options.max_frontier.reset();
    auto result = solve(tree, buffers, 22, options);
    EXPECT_DOUBLE_EQ(result.score, 1);
    options.max_buffers = 0;
    result = solve(tree, buffers, 22, options);
    EXPECT_DOUBLE_EQ(result.score, -5);
    options.max_buffers.reset();
    tree.nodes.push_back({.cap = 1, .required = -100});
    tree.nodes.push_back({.kind = NodeKind::Junction, .left = 3, .right = 4});
    result = solve(tree, buffers, 22, options);
    EXPECT_LT(result.score, -100);
}

TEST(Pbbi, EquationSevenSpendsPositiveSlackWithoutRelaxingInfeasiblePaths) {
    EXPECT_DOUBLE_EQ(adjustedRequired(60, 100, 80, 0), 50);
    EXPECT_DOUBLE_EQ(adjustedRequired(60, 100, 120, 0), 60);
    EXPECT_DOUBLE_EQ(adjustedRequired(infinity, 100, 80, 0), infinity);
    EXPECT_DOUBLE_EQ(adjustedRequired(60, 100, 80, 80), 60);
}
