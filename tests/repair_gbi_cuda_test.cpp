#include "repair_gbi_cuda.h"

#include <gtest/gtest.h>

#include <vector>

TEST(GbiCudaSelector, SelectsBestCandidatePerNetOrFallsBack) {
  const std::vector<int> offsets{0, 3, 5};
  const std::vector<double> cap{1, 2, 3, 1, 4};
  const std::vector<double> slack{1, 4, 2, 0, 8};
  const std::vector<unsigned> buffers{0, 2, 1, 0, 3};
  const std::vector<double> resistance{1, 2};
  const auto selected = ecc::gbiCudaSelect(offsets, cap, slack, buffers,
                                            resistance);
  ASSERT_EQ(selected.size(), 2u);
  if(ecc::gbiCudaAvailable()) {
    EXPECT_EQ(selected[0], 1);
    EXPECT_EQ(selected[1], 1);
  }
  else {
    EXPECT_EQ(selected[0], -1);
    EXPECT_EQ(selected[1], -1);
  }
}

#ifdef ECC_SIZER_ENABLE_GBI_CUDA
TEST(GbiCudaForest, PropagatesMergesPrunesAndTracesBuffers) {
  if(!ecc::gbiCudaAvailable()) GTEST_SKIP() << "CUDA device unavailable";
  using Type = ecc::GbiCudaNodeType;
  const std::vector<ecc::GbiCudaNode> nodes{
      {Type::load, -1, -1, 2, 0, 10},
      {Type::wire, 0, -1, 1, 2, 0},
      {Type::load, -1, -1, 1, 0, 8},
      {Type::wire, 2, -1, 0, 1, 0},
      {Type::junction, 1, 3, 0, 0, 0}};
  const auto result = ecc::gbiCudaSolve(nodes, {{0.5, 1, 1}}, {4}, {3});
  ASSERT_EQ(result.selected.size(), 1u);
  ASSERT_GE(result.selected[0], 0);
  const auto& root = result.frontiers.at(4).at(result.selected[0]);
  EXPECT_DOUBLE_EQ(root.cap, 1);
  EXPECT_DOUBLE_EQ(root.slack, 1);
  EXPECT_EQ(root.buffers, 2u);
  const auto& left = result.frontiers.at(1).at(root.left);
  const auto& right = result.frontiers.at(3).at(root.right);
  EXPECT_EQ(left.buffer, 0);
  EXPECT_EQ(right.buffer, 0);
  EXPECT_EQ(left.left, 0);
  EXPECT_EQ(right.left, 0);
  ASSERT_EQ(result.frontiers.at(4).size(), 2u);
  EXPECT_DOUBLE_EQ(result.frontiers.at(4)[1].cap, 3.5);
  EXPECT_DOUBLE_EQ(result.frontiers.at(4)[1].slack, 5);
}

TEST(GbiCudaForest, HandlesWideLevelsStableTiesAndNoBuffers) {
  if(!ecc::gbiCudaAvailable()) GTEST_SKIP() << "CUDA device unavailable";
  using Type = ecc::GbiCudaNodeType;
  std::vector<ecc::GbiCudaNode> nodes;
  std::vector<int> roots;
  std::vector<double> resistance;
  for(int i = 0; i < 257; ++i) {
    nodes.push_back({Type::load, -1, -1, 2, 0, 10});
    nodes.push_back({Type::wire, 2 * i, -1, 1, 2, 0});
    roots.push_back(2 * i + 1);
    resistance.push_back(3);
  }
  // Two electrically identical library cells must always select the first.
  const auto result = ecc::gbiCudaSolve(nodes, {{0.5, 1, 1}, {0.5, 1, 1}}, roots, resistance);
  ASSERT_EQ(result.selected.size(), roots.size());
  for(size_t i = 0; i < roots.size(); ++i) {
    ASSERT_GE(result.selected[i], 0);
    const auto& c = result.frontiers.at(roots[i]).at(result.selected[i]);
    EXPECT_EQ(c.buffer, 0);
    EXPECT_DOUBLE_EQ(c.cap, 0.5);
    EXPECT_DOUBLE_EQ(c.slack, 1);
  }
  const auto unbuffered = ecc::gbiCudaSolve(nodes, {}, roots, resistance);
  for(int selected : unbuffered.selected) EXPECT_EQ(selected, -1);
}

TEST(GbiCudaForest, RejectsMalformedTreesAndMemoryExhaustion) {
  if(!ecc::gbiCudaAvailable()) GTEST_SKIP() << "CUDA device unavailable";
  using Type = ecc::GbiCudaNodeType;
  EXPECT_THROW(ecc::gbiCudaSolve({{Type::wire, 0, -1, 1, 1, 0}}, {}, {0}, {1}),
               std::invalid_argument);
  EXPECT_THROW(ecc::gbiCudaSolve({{Type::load, -1, -1, 1, 0, 0}}, {}, {0}, {1}, 1),
               std::runtime_error);
  EXPECT_TRUE(ecc::gbiCudaSolve({}, {}, {}, {}).selected.empty());
}
#endif

TEST(GbiCudaSelector, HandlesEmptyGroupsAndStridedTieReduction) {
  std::vector<double> cap(300, 0), slack(300, 0);
  std::vector<unsigned> buffers(300, 1);
  slack[259] = slack[299] = 10;
  slack[0] = 20;
  buffers[0] = 0;  // An unbuffered candidate must not win the selector.
  const auto selected = ecc::gbiCudaSelect({0, 0, 300, 300}, cap, slack,
                                          buffers, {1, 1, 1});
  ASSERT_EQ(selected.size(), 3u);
  EXPECT_EQ(selected[0], -1);
  EXPECT_EQ(selected[1], ecc::gbiCudaAvailable() ? 259 : -1);
  EXPECT_EQ(selected[2], -1);
}
