// SPDX-License-Identifier: Apache-2.0
#include "repair_buffer_budget.h"
#include "gtest/gtest.h"

TEST(BufferPlanBudget, NeverTruncatesATreeAndSkipsOversizedPlans)
{
    EXPECT_EQ(ecc::selectWholeBufferPlans({65, 40, 30, 24, 1}, 64),
              (std::vector<std::size_t>{1, 3}));
}
TEST(BufferPlanBudget, ZeroIsUnlimitedAndEmptyPlansAreIgnored)
{
    EXPECT_EQ(ecc::selectWholeBufferPlans({0, 100, 5, 0, 10}, 0),
              (std::vector<std::size_t>{1, 2, 4}));
    EXPECT_TRUE(ecc::selectWholeBufferPlans({}, 0).empty());
    EXPECT_TRUE(ecc::selectWholeBufferPlans({0, 65}, 64).empty());
}
TEST(BufferPlanBudget, FitsExactlyAndPreservesNetOrder)
{
    EXPECT_EQ(ecc::selectWholeBufferPlans({32, 32, 1}, 64),
              (std::vector<std::size_t>{0, 1}));
}
