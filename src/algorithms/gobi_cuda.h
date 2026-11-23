#pragma once

// This interface is the standalone Sizer port of OpenROAD rsz's GobiCuda
// backend.  Host graph construction and OpenDB application stay outside the
// CUDA module so the kernel pipeline can be tested independently.

#include <stdexcept>
#include <string>
#include <cstddef>
#include <cstdlib>

#include <memory>
#include <vector>

namespace ecc {
namespace gobi {

// Whether to use predictive pruning
constexpr bool USING_PREDICTIVE_PRUNING = false;

constexpr double INF = 1e20;
constexpr double scale = 1e15; // s->fs

constexpr bool DEBUG = false;
// #define ENABLE_REQUIRE_ENSURE_CHECK
// #define ENABLE_KERNEL_WIRE_DEBUG
// #define ENABLE_KERNEL_JUNCTION_DEBUG
// #define ENABLE_KERNEL_PRUNE_DEBUG

#define CUDA_CHECK(call)                                                               \
    do {                                                                               \
        cudaError_t err = call;                                                        \
        if (err != cudaSuccess) {                                                      \
            throw std::runtime_error(std::string("CUDA: ") + cudaGetErrorString(err)); \
        }                                                                              \
    } while (0)

struct Buffer {
    double rb, cb, tb; // resistance, capacitance, delay
};

struct BufferResult {
    int node_id;
    int buffer_type;
};

// --- Level Meta Information ---
struct LevelMeta {
    int start_node = -1;
    int end_node = -1;
    int sink_count = 0;
    int wire_count = 0;
    int junc_count = 0;

    int size() const { return end_node - start_node; } // [start_node, end_node)
};

struct HostNode {
    double x, y;
    int old_id; // 1-based ID during build
    int new_id; // ID after sorting
    int type;   // -1:DUMMY, 0:SINK, 1:WIRE, 2:JUNCTION
    int depth;  // Depth from root
    int level;  // Calculated: (TreeMax - Depth)
    double cap = 0;
    double rat_or_res = 0;
    int left_old = 0;    // Child old_id
    int right_old = 0;   // Child old_id
    int context_id = -1; // Context for this node
};

struct HostGraph {
    std::vector<int> type;          // 0:SINK, 1:WIRE, 2:JUNCTION
    std::vector<int> left;          // Left Child Index in Graph
    std::vector<int> right;         // Right Child Index in Graph
    std::vector<double> cap;        // Capacitance of Wire / Sink
    std::vector<double> rat_or_res; // RAT for Sink, Res for Wire

    std::vector<int> root;       // Indices of root nodes
    std::vector<int> root_level; // Level of each root node
    // ====== Graph in Host Memory (Before GPU) =====
    int num_nodes = 0;

    std::vector<int> context_id; // Context id for this node, in host

    std::vector<LevelMeta> level_metas;
    int global_max_level = 0;

    std::vector<HostNode> all_nodes; // Global accumulator

    void resize(int n) {
        num_nodes = n;
        type.resize(n);
        left.resize(n);
        right.resize(n);
        cap.resize(n);
        rat_or_res.resize(n);
    }
};

struct Result {
    double slack;
    std::vector<BufferResult> inserted_buffers;
};

struct CudaContext {
    std::vector<std::shared_ptr<Result>> results; // One result per net

    std::shared_ptr<const std::vector<Buffer>> buffer_library;
    std::vector<double> h_driver_res;
    std::shared_ptr<HostGraph> h_graph;
};

bool gobiCudaAvailable();
void gobiCudaSolve(const CudaContext& context);

} // namespace gobi
} // namespace ecc
