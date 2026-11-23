#include "algorithms/gobi_cuda.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

ecc::gobi::CudaContext oneWireContext()
{
    auto graph = std::make_shared<ecc::gobi::HostGraph>();
    graph->resize(3);
    graph->num_nodes = 3;
    graph->type = {-1, 0, 1};
    graph->left = {0, 0, 1};
    graph->right = {0, 0, 0};
    graph->cap = {0.0, 0.10, 0.20};
    graph->rat_or_res = {0.0, 10.0e15, 10.0};
    graph->root = {2};
    graph->root_level = {1};
    graph->context_id = {-1, 0, 0};
    graph->global_max_level = 1;
    graph->level_metas.resize(2);
    graph->level_metas[0] = {1, 2, 1, 0, 0};
    graph->level_metas[1] = {2, 3, 0, 1, 0};

    auto result = std::make_shared<ecc::gobi::Result>();
    ecc::gobi::CudaContext context;
    context.results = {result};
    context.buffer_library = std::make_shared<const std::vector<ecc::gobi::Buffer>>(
        std::vector<ecc::gobi::Buffer>{{0.01, 0.01, 1.0}});
    context.h_driver_res = {100.0};
    context.h_graph = std::move(graph);
    return context;
}

ecc::gobi::CudaContext twoBranchContext()
{
    auto graph = std::make_shared<ecc::gobi::HostGraph>();
    graph->resize(6);
    graph->num_nodes = 6;
    graph->type = {-1, 0, 0, 1, 1, 2};
    graph->left = {0, 0, 0, 1, 2, 3};
    graph->right = {0, 0, 0, 0, 0, 4};
    graph->cap = {0.0, 0.10, 0.10, 0.20, 0.20, 0.0};
    graph->rat_or_res = {0.0, 10.0e15, 10.0e15, 10.0, 10.0, 0.0};
    graph->root = {5};
    graph->root_level = {2};
    graph->context_id = {-1, 0, 0, 0, 0, 0};
    graph->global_max_level = 2;
    graph->level_metas.resize(3);
    graph->level_metas[0] = {1, 3, 2, 0, 0};
    graph->level_metas[1] = {3, 5, 0, 2, 0};
    graph->level_metas[2] = {5, 6, 0, 0, 1};

    auto result = std::make_shared<ecc::gobi::Result>();
    ecc::gobi::CudaContext context;
    context.results = {result};
    context.buffer_library = std::make_shared<const std::vector<ecc::gobi::Buffer>>(
        std::vector<ecc::gobi::Buffer>{{0.01, 0.01, 1.0}});
    context.h_driver_res = {100.0};
    context.h_graph = std::move(graph);
    return context;
}

}  // namespace

TEST(GobiCudaMigration, BackendAvailabilityIsExplicit)
{
#ifndef ECC_SIZER_ENABLE_GOBI_CUDA
    EXPECT_FALSE(ecc::gobi::gobiCudaAvailable());
#endif
}

TEST(GobiCudaMigration, OriginalKernelPipelineSolvesOneWire)
{
    if (!ecc::gobi::gobiCudaAvailable()) {
        GTEST_SKIP() << "GOBI CUDA backend or device unavailable";
    }

    auto context = oneWireContext();
    ASSERT_NO_THROW(ecc::gobi::gobiCudaSolve(context));
    ASSERT_EQ(context.results.size(), 1u);
    ASSERT_TRUE(std::isfinite(context.results[0]->slack));
    EXPECT_GT(context.results[0]->slack, 0.0);
    ASSERT_EQ(context.results[0]->inserted_buffers.size(), 1u);
    EXPECT_EQ(context.results[0]->inserted_buffers[0].node_id, 2);
    EXPECT_EQ(context.results[0]->inserted_buffers[0].buffer_type, 0);
}

TEST(GobiCudaMigration, OriginalJunctionMergeKernelRuns)
{
    if (!ecc::gobi::gobiCudaAvailable()) {
        GTEST_SKIP() << "GOBI CUDA backend or device unavailable";
    }

    auto context = twoBranchContext();
    ASSERT_NO_THROW(ecc::gobi::gobiCudaSolve(context));
    ASSERT_EQ(context.results.size(), 1u);
    EXPECT_TRUE(std::isfinite(context.results[0]->slack));
    EXPECT_LE(context.results[0]->inserted_buffers.size(), 6u);
}
