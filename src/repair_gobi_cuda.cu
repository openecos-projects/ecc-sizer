// The kernels in this file are intentionally kept aligned with
// ISPD26-Contest/OpenROAD/src/rsz/src/GobiCuda.cu.  Only the public namespace
// and entry-point name are adapted for Sizer.
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_atomic_functions.h>
#include <driver_types.h>
#include <thrust/detail/raw_pointer_cast.h>
#include <thrust/device_vector.h>
#include <thrust/fill.h>
#include <thrust/functional.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/memory.h>
#include <thrust/scan.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cub/block/block_scan.cuh>
#include <cub/device/device_scan.cuh>
#include <cub/thread/thread_operators.cuh>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <thrust/detail/fill.inl>
#include <thrust/detail/scan.inl>
#include <thrust/detail/transform.inl>
#include <tuple>
#include <utility>
#include <vector>

#include "repair_gobi_cuda.h"
namespace ecc {
using namespace gobi;

// =========================================================
// Debug Configuration
// =========================================================

constexpr __device__ int DEBUG_NODES[] = {0};  // 0 => debug all
constexpr int DEBUG_NODE_COUNT = sizeof(DEBUG_NODES) / sizeof(DEBUG_NODES[0]);

// Helper to check if a node is in debug list
__device__ __forceinline__ bool is_debug_node(int node_id)
{
#pragma unroll
  for (int i = 0; i < DEBUG_NODE_COUNT; ++i) {
    if (DEBUG_NODES[i] == 0 || DEBUG_NODES[i] == node_id) {
      return true;
    }
  }
  return false;
}

// Dump List from Thread 0
__device__ __forceinline__ void dump_list(int node_id,
                                          const char* label,
                                          int count,
                                          const double* load,
                                          const double* slack,
                                          const int* src = nullptr)
{
  if (threadIdx.x != 0 || !is_debug_node(node_id)) {
    return;
  }
  printf("[GPU Node %d][HEAD] === %s (Count=%d) ===\n", node_id, label, count);
  for (int i = 0; i < count; ++i) {
    if (src != nullptr) {
      printf("  [Node %d][%d]: Load=%.6f, Slack=%.6f, Src=%d\n",
             node_id,
             i,
             load[i],
             slack[i],
             src[i]);
    } else {
      printf("  [Node %d][%d]: Load=%.6f, Slack=%.6f\n",
             node_id,
             i,
             load[i],
             slack[i]);
    }
  }
  printf("--------------------------------------\n");
}

// Prints from all threads (filtered by node_id)
#define GPU_PRINTF_ANY(tag, node_id, fmt, ...)        \
  do {                                                \
    if (is_debug_node(node_id)) {                     \
      printf("[%s][GPU Node %d][TID %03d] " fmt "\n", \
             tag,                                     \
             node_id,                                 \
             threadIdx.x,                             \
             ##__VA_ARGS__);                          \
    }                                                 \
  } while (0)

// Prints only from Thread 0 (filtered by node_id)
#define GPU_PRINTF_HEAD(tag, node_id, fmt, ...)                              \
  do {                                                                       \
    if (threadIdx.x == 0 && is_debug_node(node_id)) {                        \
      printf(                                                                \
          "[%s][GPU Node %d][HEAD] " fmt "\n", tag, node_id, ##__VA_ARGS__); \
    }                                                                        \
  } while (0)

// Only use in block-synchronized context
#define GPU_BLOCK_DUMP(node_id, label, count, load, slack, ...)   \
  do {                                                            \
    __syncthreads();                                              \
    dump_list(node_id, label, count, load, slack, ##__VA_ARGS__); \
    __syncthreads();                                              \
  } while (0)

// --- Wire Kernel Debug Macros ---
#ifdef ENABLE_KERNEL_WIRE_DEBUG
#define WIRE_LOG(node_id, fmt, ...) \
  GPU_PRINTF_ANY("WIRE", node_id, fmt, ##__VA_ARGS__)
#define WIRE_LOG_0(node_id, fmt, ...) \
  GPU_PRINTF_HEAD("WIRE", node_id, fmt, ##__VA_ARGS__)
#define WIRE_DUMP(node_id, label, count, load, slack, ...) \
  GPU_BLOCK_DUMP(node_id, label, count, load, slack, ##__VA_ARGS__)
#else
#define WIRE_LOG(...)
#define WIRE_LOG_0(...)
#define WIRE_DUMP(...)
#endif

// --- Junction Kernel Debug Macros ---
#ifdef ENABLE_KERNEL_JUNCTION_DEBUG
// FIX: Changed gpu_log_any/head to GPU_PRINTF_ANY/HEAD
#define JUNC_LOG(node_id, fmt, ...) \
  GPU_PRINTF_ANY("JUNC", node_id, fmt, ##__VA_ARGS__)
#define JUNC_LOG_0(node_id, fmt, ...) \
  GPU_PRINTF_HEAD("JUNC", node_id, fmt, ##__VA_ARGS__)
#define JUNC_DUMP(node_id, label, count, load, slack, ...) \
  GPU_BLOCK_DUMP(node_id, label, count, load, slack, ##__VA_ARGS__)
#else
#define JUNC_LOG(...)
#define JUNC_LOG_0(...)
#define JUNC_DUMP(...)
#endif

// --- Prune Kernel Debug Macros ---
#ifdef ENABLE_KERNEL_PRUNE_DEBUG
// FIX: Changed gpu_log_any/head to GPU_PRINTF_ANY/HEAD
#define PRUNE_LOG(node_id, fmt, ...) \
  GPU_PRINTF_ANY("PRUNE", node_id, fmt, ##__VA_ARGS__)
#define PRUNE_LOG_0(node_id, fmt, ...) \
  GPU_PRINTF_HEAD("PRUNE", node_id, fmt, ##__VA_ARGS__)
#define PRUNE_DUMP(node_id, label, count, load, slack, ...) \
  GPU_BLOCK_DUMP(node_id, label, count, load, slack, ##__VA_ARGS__)
#else
#define PRUNE_LOG(...)
#define PRUNE_LOG_0(...)
#define PRUNE_DUMP(...)
#endif

// =========================================================
// Configuration
// =========================================================
const int WARP_SIZE = 32;  // CUDA standard, do not change
const size_t MAX_BUFFER_TYPES = 32;

// =========================================================
// Data Structures (Host + Device)
// =========================================================

// version for non-const vectors
template <typename T>
inline T* raw_ptr(thrust::device_vector<T>& vec, size_t offset = 0)
{
  return thrust::raw_pointer_cast(vec.data()) + offset;
}

// version for const vectors
template <typename T>
inline const T* raw_ptr(const thrust::device_vector<T>& vec, size_t offset = 0)
{
  return thrust::raw_pointer_cast(vec.data()) + offset;
}

// ----------------------------------------------------------
// Graph Data Structures (Device)
// ----------------------------------------------------------

struct Graph
{
  Graph(const std::shared_ptr<HostGraph> h_graph)
  {
    int n = h_graph->type.size();
    type = h_graph->type;
    left = h_graph->left;
    right = h_graph->right;
    cap = h_graph->cap;
    rat_or_res = h_graph->rat_or_res;
    root = h_graph->root;
    root_level = h_graph->root_level;
    context_id = h_graph->context_id;
    num_nodes = n;
  }

  thrust::device_vector<int> type;           // 0:SINK, 1:WIRE, 2:JUNCTION
  thrust::device_vector<int> left;           // Left Child Index in Graph
  thrust::device_vector<int> right;          // Right Child Index in Graph
  thrust::device_vector<double> cap;         // Capacitance of Wire / Sink
  thrust::device_vector<double> rat_or_res;  // RAT for Sink, Res for Wire

  thrust::device_vector<int> root;        // Indices of root nodes
  thrust::device_vector<int> root_level;  // Level of each root node
  int num_nodes = 0;

  std::vector<int> context_id;  // Context id for this node, in host

  // Device View
  struct View
  {
    const int* __restrict__ type;
    const int* __restrict__ left;
    const int* __restrict__ right;
    const double* __restrict__ cap;
    const double* __restrict__ rat_or_res;

    __device__ __forceinline__ int get_type(int id) const { return type[id]; }

    __device__ __forceinline__ int get_left(int id) const { return left[id]; }

    __device__ __forceinline__ int get_right(int id) const { return right[id]; }

    __device__ __forceinline__ double get_cap(int id) const { return cap[id]; }

    __device__ __forceinline__ double get_res(int id) const
    {
      return rat_or_res[id];
    }

    __device__ __forceinline__ double get_rat(int id) const
    {
      return rat_or_res[id];
    }
  };

  View get_view() const
  {
    return View{raw_ptr(type),
                raw_ptr(left),
                raw_ptr(right),
                raw_ptr(cap),
                raw_ptr(rat_or_res)};
  }

  struct RootView
  {
    const int* __restrict__ roots;
    const int* __restrict__ root_levels;
    int size;

    __device__ __forceinline__ int get_root(int idx) const
    {
      return roots[idx];
    }

    __device__ __forceinline__ int get_root_level(int idx) const
    {
      return root_levels[idx];
    }
  };

  RootView get_root_view() const
  {
    return RootView{
        raw_ptr(root), raw_ptr(root_level), static_cast<int>(root.size())};
  }

  void resize(int n)
  {
    num_nodes = n;
    type.resize(n);
    left.resize(n);
    right.resize(n);
    cap.resize(n);
    rat_or_res.resize(n);
  }
};

// --- Buffer Library ---

__constant__ Buffer c_buffers[MAX_BUFFER_TYPES];  // Assuming max buffer types
__constant__ int c_buffer_size;  // Actual number of buffer types

void upload_buffer_library(const std::vector<Buffer>& buffers)
{
  if (buffers.size() > MAX_BUFFER_TYPES) {
    std::cout << "Error: Buffer library size exceeds MAX_BUFFER_TYPES ("
              << MAX_BUFFER_TYPES << ")\n";
  }
  size_t size = std::min(buffers.size(), MAX_BUFFER_TYPES);
  cudaMemcpyToSymbol(c_buffers, buffers.data(), size * sizeof(Buffer));
  cudaMemcpyToSymbol(c_buffer_size, &size, sizeof(int));
}

// --- Layout Structure ---
struct Layout
{
  thrust::device_vector<int> start;  // Start Index in Payload
  thrust::device_vector<int> count;  // Number of Solutions for Each Node

  struct View
  {
    int* __restrict__ start;
    int* __restrict__ count;

    __device__ __forceinline__ int get_count(int node_id) const
    {
      return count[node_id];
    }

    __device__ __forceinline__ int get_start(int node_id) const
    {
      return start[node_id];
    }

    __device__ __forceinline__ void set_count(int node_id, int c) const
    {
      count[node_id] = c;
    }

    __device__ __forceinline__ void set_start(int node_id, int s) const
    {
      start[node_id] = s;
    }
  };

  View get_view() { return View{raw_ptr(start), raw_ptr(count)}; }

  explicit Layout(int n)
  {
    start.resize(n);
    count.resize(n);
    start[0] = -1;  // Dummy Node 0 -> Payload Index -1 (No Solutions)
    count[0] = 0;   //  Dummy Node 0 -> Payload Index 0 (Size 0)
  }

  struct ReadView
  {
    const int* __restrict__ start;
    const int* __restrict__ count;

    __device__ __forceinline__ int get_count(int node_id) const
    {
      return count[node_id];
    }

    __device__ __forceinline__ int get_start(int node_id) const
    {
      return start[node_id];
    }
  };

  ReadView get_read_view() const
  {
    return ReadView{raw_ptr(start), raw_ptr(count)};
  }
};

struct Payload
{
  thrust::device_vector<double> load;
  thrust::device_vector<double> slack;

  struct ReadView
  {
    const double* __restrict__ load;
    const double* __restrict__ slack;
  };

  ReadView get_read_view() const
  {
    return ReadView{raw_ptr(load), raw_ptr(slack)};
  }

  struct View
  {
    double* __restrict__ load;
    double* __restrict__ slack;
  };

  View get_write_view() { return View{raw_ptr(load), raw_ptr(slack)}; }

  void ensure_capacity(size_t n)
  {  // Allocate enough capacity, but do not resize
    if (load.capacity() < n) {
      size_t new_cap = n * 1.5;
      load.reserve(new_cap);
      slack.reserve(new_cap);
    }
    // WARNNING: UB if we try to read uninitialized data!
    // There can be garbage data in the newly reserved space, but it's okay...
  }

  explicit Payload(int n) { ensure_capacity(n); }
};

// --- History Structure ---
struct History
{
  // Device
  thrust::device_vector<int> left_idx;   // Which solution came from left child
  thrust::device_vector<int> right_idx;  // Which solution came from right child
  thrust::device_vector<int> buffer_id;  // Buffer type (-1 = none)

  // Host
  std::vector<size_t> layer_offsets;  // Offsets for each layer
  size_t current_offset = 0;          // Current layer start offset

  size_t append_layer(size_t layer, size_t count)
  {
    size_t offset = current_offset;
    layer_offsets[layer] = offset;

    size_t new_total = offset + count;
    if (left_idx.size() < new_total) {  // Need to reserve more
      size_t new_size = new_total * 2;
      left_idx.resize(new_size, -1);
      right_idx.resize(new_size, -1);
      buffer_id.resize(new_size, -1);
    }
    current_offset = new_total;
    return offset;
  }

  struct View
  {
    int* __restrict__ l;
    int* __restrict__ r;
    int* __restrict__ b;

    __device__ __forceinline__ void record(int idx,
                                           int li,
                                           int ri,
                                           int bi) const
    {
      l[idx] = li;
      r[idx] = ri;
      b[idx] = bi;
    }
  };

  View get_view(size_t offset = 0)
  {
    return View{raw_ptr(left_idx) + offset,
                raw_ptr(right_idx) + offset,
                raw_ptr(buffer_id) + offset};
  }

  struct ReadView
  {
    const int* __restrict__ l;
    const int* __restrict__ r;
    const int* __restrict__ b;
  };

  ReadView get_read_view(size_t offset = 0) const
  {
    return ReadView{raw_ptr(left_idx) + offset,
                    raw_ptr(right_idx) + offset,
                    raw_ptr(buffer_id) + offset};
  }
};

// =========================================================
// Kernel Implementations
// =========================================================

// ----------------------------------------------------------
// Layout Kernel: Calculates Solution Count for Each Node
// ----------------------------------------------------------
__global__ void kernel_layout(Graph::View graph,
                              Layout::View layout,
                              int start_node,
                              int num_nodes)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_nodes) {
    return;
  }
  int node_id = start_node + idx;

  int type = graph.get_type(node_id);
  int count = 0;
  if (type == 0) {  // SINK
    count = 1;
  } else if (type == 1) {  // WIRE
    count = layout.get_count(graph.get_left(node_id)) + c_buffer_size;
  } else {  // JUNCTION
    count = layout.get_count(graph.get_left(node_id))
            + layout.get_count(graph.get_right(node_id));
  }
  layout.set_count(node_id, count);
}

// ----------------------------------------------------------
// Sink Kernel: Initializes Sink Node Solutions
// ----------------------------------------------------------
__global__ void kernel_sink(Graph::View graph,
                            Payload::View output,
                            Layout::View layout,
                            int start_node,
                            int num_nodes)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_nodes) {
    return;
  }
  int node_id = start_node + idx;

  int pos = layout.get_start(node_id);
  output.load[pos] = graph.get_cap(node_id);
  output.slack[pos] = graph.get_rat(node_id);
  // count is always 1 for sink, set in layout kernel
  // No need to record history for sink nodes
}

// ----------------------------------------------------------
// Device Helper Functions For Wire+Buffer Kernel
// ----------------------------------------------------------

// Upper Bound on Global Memory Array
// Searches in Input array, but treats values as (Input[i] + WireDelay)
// Returns the rank of (my_k, my_v) inside the "virtual" wire list
// Requires: Input list is sorted by Load ascending, Slack descending
__device__ int virtual_upper_bound(const double* in_load,
                                   const double* in_slack,
                                   int n,
                                   double w_cap,
                                   double w_res,
                                   double search_k,
                                   double search_v)
{
  int l = 0, r = n;
  while (l < r) {
    int mid = l + (r - l) / 2;
    // Compute "Virtual" Value (Wire Propagation on the fly)
    double my_k = in_load[mid] + w_cap;
    double my_v = in_slack[mid] - (0.5 * w_res * w_cap + w_res * in_load[mid]);

    bool go_right;
    // Check Pareto dominance to determine order
    if (my_k < search_k) {
      go_right = true;
    } else if (my_k > search_k) {
      go_right = false;
    } else {
      go_right
          = (my_v >= search_v);  // Loads are equal, sort by Slack descending
    }

    if (go_right) {
      l = mid + 1;
    } else {
      r = mid;
    }
  }
  return l;
}

// Lower Bound on Constant Memory Array
// Returns the rank of (search_k, search_v) inside the sorted (s_key, s_val)
// list
__device__ int cmem_lower_bound(const Buffer* s_key,
                                const double* s_val,
                                int n,
                                double search_k,
                                double search_v)
{
  int l = 0, r = n;
  while (l < r) {
    int mid = l + (r - l) / 2;
    bool go_right;
    if (s_key[mid].cb < search_k) {
      go_right = true;
    } else if (s_key[mid].cb > search_k) {
      go_right = false;
    } else {
      go_right = (s_val[mid]
                  > search_v);  // Loads are equal, sort by Slack descending
    }

    if (go_right) {
      l = mid + 1;
    } else {
      r = mid;
    }
  }
  return l;
}

// ----------------------------------------------------------
// Wire + Buffer Kernel: Propagates Wire Solutions and Inserts Buffers
// ----------------------------------------------------------

const int WIRE_TILE_SIZE = 256;
const int WIRE_WARPS = (WIRE_TILE_SIZE / WARP_SIZE);

// Requires:
// 1. Input list is sorted by Load ascending, Slack descending
// 2. Buffer library can fit into constant memory: c_buffer_size <=
// MAX_BUFFER_TYPES Ensures:
// 1. Output list is sorted by Load ascending, Slack descending
// 2. History is recorded properly
template <int NUM_BUFFERS = MAX_BUFFER_TYPES>
__global__ void kernel_wire_buffer(Graph::View graph,
                                   Payload::ReadView input,
                                   Payload::View output,
                                   History::View history,
                                   Layout::View layout,
                                   int start_node,
                                   int num_nodes)
{
  if (blockIdx.x >= num_nodes) {
    return;
  }
  int node_id = start_node + blockIdx.x;  // Each block processes one wire node

  // Shared Memory Setup
  __shared__ double s_load[WIRE_TILE_SIZE + 1];
  __shared__ double s_slack[WIRE_TILE_SIZE + 1];

  // Intermediate reduction storage (val + index)
  __shared__ double s_reduce_val[NUM_BUFFERS * WIRE_WARPS];
  __shared__ int s_reduce_idx[NUM_BUFFERS * WIRE_WARPS];

  // Final Buffer Results stored in Shared Memory
  __shared__ double s_final_b_slack[NUM_BUFFERS];
  __shared__ int s_final_b_src[NUM_BUFFERS];  // Records best_src_index

  // Runtime buffer count storage
  __shared__ int s_b_count_storage;

  int tid = threadIdx.x;

  // Graph Properties
  int child_id = graph.get_left(node_id);
  int child_cnt = layout.get_count(child_id);
  int in_start = layout.get_start(child_id);

  double r_wire = graph.get_res(node_id);
  double c_wire = graph.get_cap(node_id);

  WIRE_LOG_0(node_id,
             "Start Processing Wire+Buffer. Wire Res=%.6f, Cap=%.6f. Child "
             "ID=%d (Cnt=%d)",
             r_wire,
             c_wire,
             child_id,
             child_cnt);
  WIRE_DUMP(node_id,
            "Input From Child",
            child_cnt,
            input.load + layout.get_start(child_id),
            input.slack + layout.get_start(child_id));

  // ----------------------------------------------------------
  // PHASE 1: Compute Buffer Solutions (Fused Propagation + Insertion)
  // ----------------------------------------------------------

  // 1.1 Determine Buffer Count, only insert buffer if wire has parasitic
  if (tid == 0) {
    bool valid = (c_wire > 0 || r_wire > 0);
    s_b_count_storage = valid ? c_buffer_size : 0;
  }

  __syncthreads();
  int b_count = s_b_count_storage;

  // 1.2 Iterate over Child Solutions (Tiled)
  if (b_count > 0) {
    // Registers for finding max slack per buffer type
    double r_max_q[NUM_BUFFERS];
    int r_best_idx[NUM_BUFFERS];

#pragma unroll
    for (int b = 0; b < NUM_BUFFERS; ++b) {
      r_max_q[b] = -INF;
      r_best_idx[b] = -1;
    }

    for (int tile_base = 0; tile_base < child_cnt;
         tile_base += WIRE_TILE_SIZE) {
      int tile_limit = min(WIRE_TILE_SIZE, child_cnt - tile_base);

      // Load Child Solutions to SMEM
      if (tid < tile_limit) {
        int idx = in_start + tile_base + tid;
        s_load[tid] = input.load[idx];
        s_slack[tid] = input.slack[idx];
      }
      __syncthreads();

      // Compute Slack for each Buffer Type
      if (tid < tile_limit) {
        double w_load = s_load[tid] + c_wire;
        double w_slack
            = s_slack[tid]
              - ((0.5 * r_wire * c_wire + r_wire * s_load[tid]) * scale);

#pragma unroll
        for (int b = 0; b < NUM_BUFFERS; ++b) {
          if (b < c_buffer_size) {
            // Slack_in = Slack_req - (Rb * Load + Tb)
            double q
                = w_slack - c_buffers[b].tb - c_buffers[b].rb * w_load * scale;
            if (q > r_max_q[b]) {
              r_max_q[b] = q;
              r_best_idx[b] = tile_base + tid;  // Track best source index
            }
          }
        }
      }
      __syncthreads();
    }

// 1.3 Reduction (Warp Level + Block Level) to find best for each buffer type
#pragma unroll
    for (int b = 0; b < NUM_BUFFERS; ++b) {
      if (b < c_buffer_size) {
        double val = r_max_q[b];
        int idx = r_best_idx[b];

        // Warp Reduction
        for (int offset = WARP_SIZE / 2; offset > 0; offset /= 2) {
          double other_val = __shfl_down_sync(0xff'ff'ff'ff, val, offset);
          int other_idx = __shfl_down_sync(0xff'ff'ff'ff, idx, offset);
          if (other_val > val) {
            val = other_val;
            idx = other_idx;
          }
        }

        // First thread of each warp writes to SMEM
        if ((tid % WARP_SIZE) == 0) {
          int warp_id = tid / WARP_SIZE;
          s_reduce_val[b * WIRE_WARPS + warp_id] = val;
          s_reduce_idx[b * WIRE_WARPS + warp_id] = idx;
        }
      }
    }
    __syncthreads();

    // Final Reduction by first `c_buffer_size` threads
    if (tid < c_buffer_size) {
      double final_q = -INF;
      int final_idx = -1;

      for (int w = 0; w < WIRE_WARPS; ++w) {
        double v = s_reduce_val[tid * WIRE_WARPS + w];
        int i = s_reduce_idx[tid * WIRE_WARPS + w];
        if (v > final_q) {
          final_q = v;
          final_idx = i;
        }
      }

      // s_final_b_load[tid]  = c_buffers[tid].cb;
      s_final_b_slack[tid] = final_q;
      s_final_b_src[tid] = final_idx;  // Store the best source index
    }
  }
  __syncthreads();

  // Write Total Count
  if (tid == 0) {
    layout.set_count(node_id, child_cnt + b_count);
  }

  // ----------------------------------------------------------
  // PHASE 2: Merge Wire List and Buffer List
  // ----------------------------------------------------------

  int base = layout.get_start(node_id);

  // Job A: Stream Wire List and rank in Buffer List
  // These are the "Wire only" solutions
  for (int tile_base = 0; tile_base < child_cnt; tile_base += WIRE_TILE_SIZE) {
    int tile_limit = min(WIRE_TILE_SIZE, child_cnt - tile_base);
    int idx = in_start + tile_base + tid;

    if (tid < tile_limit) {
      // Wire Propagation
      double load = input.load[idx] + c_wire;
      double slack
          = input.slack[idx]
            - ((0.5 * r_wire * c_wire + r_wire * input.load[idx]) * scale);
      // Find Rank in Buffer List (stored in SMEM)
      int rank_in_b
          = cmem_lower_bound(c_buffers, s_final_b_slack, b_count, load, slack);
      // Calculate Final Position: Local_Index + Rank_in_Other
      int local_idx = tile_base + tid;
      int out_pos = base + local_idx + rank_in_b;

      output.load[out_pos] = load;
      output.slack[out_pos] = slack;
      // Record History: (child_idx, -1: no right, -1: no buffer)
      history.record(out_pos, local_idx, -1, -1);

      WIRE_LOG(node_id,
               ">> Job A (Wire): ChildIdx=%d, Pos=%d, RankInBuf=%d, Load=%.6f, "
               "Slack=%.6f, SrcIdx=%d",
               local_idx,
               out_pos,
               rank_in_b,
               load,
               slack,
               local_idx);
    }
  }

  // Job B: Stream Buffer List and rank in "Virtual" Wire List
  // These are the "Buffer inserted" solutions
  if (tid < b_count) {
    double load = c_buffers[tid].cb;
    double slack = s_final_b_slack[tid];
    int src = s_final_b_src[tid];
    // Find Rank in Wire List (stored in Global Memory)
    int rank_in_w = virtual_upper_bound(input.load + in_start,
                                        input.slack + in_start,
                                        child_cnt,
                                        c_wire,
                                        r_wire,
                                        load,
                                        slack);
    // Calculate Final Position: Local_Index + Rank_in_Other
    int out_pos = base + tid + rank_in_w;

    output.load[out_pos] = load;
    output.slack[out_pos] = slack;
    // Record History: (best_src_idx, -1: no right, buffer_id)
    history.record(out_pos, src, -1, tid);

    WIRE_LOG(node_id,
             ">> Job B (Buffer): BufType=%d, Pos=%d, RankInWire=%d, Load=%.6f, "
             "Slack=%.6f, "
             "BestSrcIdx=%d",
             tid,
             out_pos,
             rank_in_w,
             load,
             slack,
             src);
  }

  WIRE_DUMP(node_id,
            "Final Output",
            layout.get_count(node_id),
            output.load + base,
            output.slack + base);
}

// ----------------------------------------------------------
// Prune Kernel: Removes Dominated Solutions
// ----------------------------------------------------------

const int PRUNE_BLOCK_THREADS = 256;

// Requires:
// 1. Input list is sorted by Load ascending, Slack Descending for same Load
// Ensures:
// 1. Output list is sorted by Load ascending, Slack Ascending
__global__ void kernel_prune(Payload::View payload,
                             History::View history,
                             Layout::View layout,
                             int start_node,
                             int num_nodes,
                             double prune_rb = 0.0)
{
  if (blockIdx.x >= num_nodes) {
    return;
  }
  int node_id = start_node + blockIdx.x;

  // 1. Determine Ranges
  int base = layout.get_start(node_id);
  int group_len = layout.get_count(node_id);

  // 2. CUB Temp Storage
  using BlockScanMax = cub::BlockScan<double, PRUNE_BLOCK_THREADS>;
  using BlockScanSum = cub::BlockScan<int, PRUNE_BLOCK_THREADS>;

  __shared__ union
  {
    typename BlockScanMax::TempStorage max_storage;
    typename BlockScanSum::TempStorage sum_storage;
  } temp_storage;

  double running_max = -INF;  // Global running max slack
  int running_cnt = 0;        // Number of kept solutions
  int tid = threadIdx.x;
  // 3. Process in Tiles
  for (int block_base = 0; block_base < group_len;
       block_base += PRUNE_BLOCK_THREADS) {
    int current_idx_in_group = block_base + tid;
    int global_read_idx = base + current_idx_in_group;

    // --- Load Key (Slack) ---
    double val_slack = -INF, val_load;  // Be careful of RAW
    // Default to -INF to effectively remove out-of-bounds items
    double val_score = -INF;
    int l, r, b;
    if (current_idx_in_group < group_len) {
      val_slack = payload.slack[global_read_idx];
      val_load = payload.load[global_read_idx];
      val_score = val_slack - (prune_rb * val_load);

      l = history.l[global_read_idx];
      r = history.r[global_read_idx];
      b = history.b[global_read_idx];
    }
    __syncthreads();

    // --- Scan Max (Find Dominance) ---
    double exclusive_max;
    double tile_max_aggregate;
    BlockScanMax(temp_storage.max_storage)
        .ExclusiveScan(
            val_score, exclusive_max, -INF, cub::Max(), tile_max_aggregate);
    __syncthreads();

    // Update global running max
    double prev_max = max(running_max, exclusive_max);
    running_max = max(running_max, tile_max_aggregate);

    // --- Determine Keep Flag ---
    // Keep if current slack > max(all previous slacks)
    bool keep = (current_idx_in_group < group_len) && (val_score > prev_max);

    // --- Scan Sum (Calculate Write Offsets) ---
    int local_rank;
    int tile_aggregate;
    BlockScanSum(temp_storage.sum_storage)
        .ExclusiveSum(keep ? 1 : 0, local_rank, tile_aggregate);
    __syncthreads();

    // --- Compact / Scatter ---
    if (keep) {
      int out_idx = base + running_cnt + local_rank;

      payload.load[out_idx] = val_load;
      payload.slack[out_idx] = val_slack;
      history.record(out_idx, l, r, b);

      PRUNE_LOG(node_id,
                "Keeping Solution: InIdx=%d, OutIdx=%d, Load=%.6f, Slack=%.6f",
                global_read_idx,
                out_idx,
                val_load,
                val_slack);
    }

    running_cnt += tile_aggregate;
    __syncthreads();
  }

  // 4. Write Final Count
  if (threadIdx.x == 0) {
    layout.set_count(node_id, running_cnt);
  }
}

// ----------------------------------------------------------
// Device Helper Functions For Junction Merge Kernel
// ----------------------------------------------------------

// Strict Weak Ordering: Primary Key = Slack (Asc), Secondary Key = Load (Asc)
__device__ __forceinline__ bool is_less_equal(double key1,
                                              double val1,
                                              double key2,
                                              double val2)
{
  bool key_smaller = (key1 < key2);
  bool key_equal = (key1 == key2);

  return key_smaller || (key_equal && (val1 <= val2));
}

// Lower Bound on Global Memory Array
// Returns the rank of (search_k, search_v) inside the sorted (keys, vals) list
__device__ int device_lower_bound(const double* keys,
                                  const double* vals,
                                  int n,
                                  double search_k,
                                  double search_v)
{
  int l = 0, r = n;
  while (l < r) {
    int mid = l + (r - l) / 2;
    double mid_k = keys[mid];
    bool go_right;
    if (mid_k < search_k) {
      go_right = true;
    } else if (mid_k > search_k) {
      go_right = false;
    } else {
      go_right = (vals[mid] < search_v);
    }

    if (go_right) {
      l = mid + 1;
    } else {
      r = mid;
    }
  }
  return l;
}

// Upper Bound on Global Memory Array
// Returns the rank of (search_k, search_v) inside the sorted (keys, vals) list
__device__ int device_upper_bound(const double* keys,
                                  const double* vals,
                                  int n,
                                  double search_k,
                                  double search_v)
{
  int l = 0, r = n;
  while (l < r) {
    int mid = l + (r - l) / 2;
    double mid_k = keys[mid];
    bool go_right;
    if (mid_k < search_k) {
      go_right = true;
    } else if (mid_k > search_k) {
      go_right = false;
    } else {
      go_right = (vals[mid] <= search_v);
    }

    if (go_right) {
      l = mid + 1;
    } else {
      r = mid;
    }
  }
  return l;
}

// Get Path Intersection for Merge Path Algorithm
// Returns number of elements taken from A
__device__ int get_path_intersection(const double* A_key,
                                     const double* A_val,
                                     int A_len,
                                     const double* B_key,
                                     const double* B_val,
                                     int B_len,
                                     int diag)
{
  int begin = max(0, diag - B_len);
  int end = min(diag, A_len);
  while (begin < end) {
    int mid = (begin + end) >> 1;
    int b_idx = diag - 1 - mid;
    // Prefer A on Tie
    if (is_less_equal(A_key[mid], A_val[mid], B_key[b_idx], B_val[b_idx])) {
      begin = mid + 1;
    } else {
      end = mid;
    }
  }
  return begin;
}

// ----------------------------------------------------------
// Junction Merge Kernel: Merges Two Sorted Solution Lists
// ----------------------------------------------------------
const int MERGE_TILE_SIZE = 512;

// Requires:
// 1. Input lists are sorted by Slack ascending, Load ascending (need pruned
// before) Ensures:
// 1. Output list is sorted by Slack ascending, Load ascending
__global__ void kernel_junction_merge(Graph::View graph,
                                      Payload::ReadView input,
                                      Payload::View output,
                                      History::View history,
                                      Layout::View layout,
                                      int start_node,
                                      int num_nodes)
{
  if (blockIdx.x >= num_nodes) {
    return;
  }

  // Shared Memory for Tiles
  __shared__ alignas(
      double) char s_mem_A[(MERGE_TILE_SIZE + 1) * 2 * sizeof(double)];
  __shared__ alignas(
      double) char s_mem_B[(MERGE_TILE_SIZE + 1) * 2 * sizeof(double)];

  // MAPPING: Key = Slack, Val = Load
  auto* s_A_key = reinterpret_cast<double*>(s_mem_A);  // Slack
  auto* s_A_val
      = reinterpret_cast<double*>(s_A_key + (MERGE_TILE_SIZE + 1));  // Load
  auto* s_B_key = reinterpret_cast<double*>(s_mem_B);                // Slack
  auto* s_B_val
      = reinterpret_cast<double*>(s_B_key + (MERGE_TILE_SIZE + 1));  // Load
  __shared__ int s_total_res;

  int node_id = start_node + blockIdx.x;
  int tid = threadIdx.x;

  int left_id = graph.get_left(node_id);
  int right_id = graph.get_right(node_id);
  int A_count = layout.get_count(left_id);
  int B_count = layout.get_count(right_id);
  bool both_active = (A_count > 0 && B_count > 0);

  // GLOBAL PTRS: Key = Slack, Val = Load
  // Note: VG Dynamic stores (Load, Slack), so we map appropriately here.
  const double* g_A_key = input.slack + layout.start[left_id];   // Key = Slack
  const double* g_A_val = input.load + layout.start[left_id];    // Val = Load
  const double* g_B_key = input.slack + layout.start[right_id];  // Key = Slack
  const double* g_B_val = input.load + layout.start[right_id];   // Val = Load

  // 1. Calculate Result Size (Thread 0 Only)
  if (tid == 0) {
    int total_res = A_count + B_count;
    if (both_active) {  // Intersection Logic: Stops when the list with smaller
                        // max-key is exhausted
      double last_A_k = g_A_key[A_count - 1];
      double last_A_v = g_A_val[A_count - 1];
      double last_B_k = g_B_key[B_count - 1];
      double last_B_v = g_B_val[B_count - 1];

      if (is_less_equal(
              last_A_k, last_A_v, last_B_k, last_B_v)) {  // A ends first
        int b_cnt
            = device_lower_bound(g_B_key, g_B_val, B_count, last_A_k, last_A_v);
        total_res = A_count + b_cnt;
      } else {  // B ends first
        int a_cnt
            = device_upper_bound(g_A_key, g_A_val, A_count, last_B_k, last_B_v);
        total_res = B_count + a_cnt;
      }
    }
    s_total_res = total_res;
    layout.set_count(node_id, total_res);
  }
  __syncthreads();
  int total_count = s_total_res;

  if (total_count == 0) {
    return;
  }  // Should not happen...

  // 2. Tiled Merge Path Execution (Partitioning by Slack)
  auto get_global_partition = [&](int diag) {
    return get_path_intersection(
        g_A_key, g_A_val, A_count, g_B_key, g_B_val, B_count, diag);
  };

  int num_tiles = (total_count + MERGE_TILE_SIZE - 1) / MERGE_TILE_SIZE;
  int tile_start_A = 0;
  int tile_start_B = 0;
  int out_base = layout.get_start(node_id);

  for (int tile_idx = 0; tile_idx < num_tiles; ++tile_idx) {
    int diag_global_start = tile_idx * MERGE_TILE_SIZE;
    int diag_global_end = min(total_count, (tile_idx + 1) * MERGE_TILE_SIZE);
    int tile_num_elements = diag_global_end - diag_global_start;

    int next_tile_start_A = get_global_partition(diag_global_end);
    int next_tile_start_B = diag_global_end - next_tile_start_A;

    int tile_len_A = next_tile_start_A - tile_start_A;
    int tile_len_B = next_tile_start_B - tile_start_B;

    // ----------------------------------------------------------
    // PHASE 1: Load Tiles into Shared Memory
    // ----------------------------------------------------------
    for (int i = tid; i <= tile_len_A; i += blockDim.x) {
      bool is_valid_idx = (tile_start_A + i < A_count);
      if (i <= tile_len_A && is_valid_idx) {
        s_A_key[i] = g_A_key[tile_start_A + i];
        s_A_val[i] = g_A_val[tile_start_A + i];
      } else {
        s_A_key[i] = INF;  // Infinity Slack (Identity for min)
        s_A_val[i] = 0.0;  // Zero Load (Identity for sum)
      }
    }
    for (int i = tid; i <= tile_len_B; i += blockDim.x) {
      bool is_valid_idx = (tile_start_B + i < B_count);
      if (i <= tile_len_B && is_valid_idx) {
        s_B_key[i] = g_B_key[tile_start_B + i];
        s_B_val[i] = g_B_val[tile_start_B + i];
      } else {
        s_B_key[i] = INF;
        s_B_val[i] = 0.0;
      }
    }
    __syncthreads();

    // ----------------------------------------------------------
    // PHASE 2: Merge Tile
    // ----------------------------------------------------------
    auto get_local_partition = [&](int diag) {
      return get_path_intersection(
          s_A_key, s_A_val, tile_len_A, s_B_key, s_B_val, tile_len_B, diag);
    };

    int work_per_thread = (tile_num_elements + blockDim.x - 1) / blockDim.x;
    int diag_local_start = min(tile_num_elements, tid * work_per_thread);
    int diag_local_end = min(tile_num_elements, (tid + 1) * work_per_thread);

    int a_local = get_local_partition(diag_local_start);
    int b_local = diag_local_start - a_local;

    int k = diag_local_start;
    int global_i = tile_start_A + a_local;
    int global_j = tile_start_B + b_local;
    int i = a_local;
    int j = b_local;

    while (k < diag_local_end) {
      bool active_A = (i <= tile_len_A) && (global_i < A_count);
      bool active_B = (j <= tile_len_B) && (global_j < B_count);

      if (both_active) {  // Intersection Logic
        if (!active_A || !active_B) {
          break;
        }
      } else {  // Union Logic
        if (!active_A && !active_B) {
          break;
        }
      }

      bool use_A;
      if (active_A && active_B) {
        use_A = is_less_equal(s_A_key[i], s_A_val[i], s_B_key[j], s_B_val[j]);
      } else {
        use_A = active_A;
      }

      // If inactive (Sentinel), Key is INF, Val is 0.0
      double k_a = active_A ? s_A_key[i] : INF;
      double v_a = active_A ? s_A_val[i] : 0.0;

      double k_b = active_B ? s_B_key[j] : INF;
      double v_b = active_B ? s_B_val[j] : 0.0;

      int out_k = diag_global_start + k;
      int out_pos = out_base + out_k;

      // Key = Slack (Min), Val = Load (Sum)
      output.slack[out_pos] = min(k_a, k_b);
      output.load[out_pos] = v_a + v_b;

      history.record(out_pos, global_i, global_j, -1);
      JUNC_LOG(node_id,
               "Merged OutIdx=%d, Slack=%.6f, Load=%.6f, FromA=%d (Idx=%d), "
               "FromB=%d (Idx=%d)",
               out_pos,
               output.slack[out_pos],
               output.load[out_pos],
               use_A ? 1 : 0,
               use_A ? global_i : global_j,
               use_A ? -1 : global_i,
               use_A ? -1 : global_j);

      // Advanced small slack first
      if (use_A) {
        i++;
        global_i++;
      } else {
        j++;
        global_j++;
      }

      k++;
    }

    tile_start_A = next_tile_start_A;
    tile_start_B = next_tile_start_B;
    __syncthreads();
  }
}

// =========================================================
// Traceback: Extracts Inserted Buffers from History
// =========================================================

struct RootSolution
{
  double max_slack;
  int best_sol_idx;
};

// ----------------------------------------------------------
// Root Solve Kernel: Finds Best Solution at Each Root in roots_subset
// ----------------------------------------------------------
__global__ void kernel_solve_roots(const int* __restrict__ root_indices,
                                   int num_roots,
                                   Graph::RootView roots_global,
                                   Payload::ReadView payload,
                                   Layout::View layout,
                                   const double* __restrict__ driver_res_array,
                                   RootSolution* __restrict__ out_solutions)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_roots) {
    return;
  }

  // Index into the global arrays to get root node ID
  int root_idx_global = root_indices[i];
  int root_node_id = roots_global.get_root(root_idx_global);

  int count = layout.get_count(root_node_id);
  int start = layout.get_start(root_node_id);
  double driver_r = driver_res_array[root_idx_global];

  double max_q = -INF;
  int best_idx = -1;

  for (int k = 0; k < count; ++k) {
    double q
        = payload.slack[start + k] - driver_r * payload.load[start + k] * scale;
    if (q > max_q) {
      max_q = q;
      best_idx = k;
    }
  }

  out_solutions[root_idx_global] = {max_q, best_idx};
}

// ----------------------------------------------------------
// Static Traceback Kernels (Level-by-Level)
// ----------------------------------------------------------

// Seed the Traceback: Maps Root Solution (d_root_sols) -> Node Solution
// (node_sol_idx)
__global__ void kernel_traceback_seed(
    int num_roots,
    Graph::RootView roots_global,
    const RootSolution* __restrict__ root_sols,
    int* __restrict__ node_sol_idx)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_roots) {
    return;
  }

  int root_node_id = roots_global.get_root(i);
  int best_sol = root_sols[i].best_sol_idx;

  // Seed Node Solution Index
  node_sol_idx[root_node_id] = best_sol;
}

// Propagate Top-Down: Uses Warp-Aggregated Atomics for inserting buffers
__global__ void kernel_traceback_propagate(int start_node,
                                           int end_node,
                                           Graph::View graph,
                                           History::ReadView history,
                                           Layout::ReadView layout,
                                           int* __restrict__ node_sol_idx,
                                           BufferResult* results,
                                           int* result_count_global)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int node_id = start_node + idx;
  int lane_id = threadIdx.x % 32;

  bool active_node = (node_id < end_node);
  int h_buf = -1;
  int h_left = -1;

  if (active_node) {
    int my_sol = node_sol_idx[node_id];

    // If this node is on the critical path
    if (my_sol != -1) {
      // Calculate global history index
      int hist_idx = layout.get_start(node_id) + my_sol;

      // Load History
      h_left = history.l[hist_idx];
      h_buf = history.b[hist_idx];

      // Propagate to Children
      int type = graph.get_type(node_id);

      if (type == 1) {  // WIRE
        int child = graph.get_left(node_id);
        if (child > 0 && h_left != -1) {
          node_sol_idx[child] = h_left;
        }
      } else if (type == 2) {  // JUNCTION
        int l = graph.get_left(node_id);
        int r = graph.get_right(node_id);
        int h_right = history.r[hist_idx];
        if (l > 0 && h_left != -1) {
          node_sol_idx[l] = h_left;
        }
        if (r > 0 && h_right != -1) {
          node_sol_idx[r] = h_right;
        }
      }
    }
  }

  // Warp-Aggregated Atomic for Buffer Collection
  bool has_buffer = (active_node && h_buf != -1);
  unsigned mask = __ballot_sync(0xff'ff'ff'ff, static_cast<int>(has_buffer));

  if (has_buffer) {
    // My local index within the warp's active set
    int local_rank = __popc(mask & ((1U << lane_id) - 1));

    // The total number of buffers found by this warp
    int warp_total = __popc(mask);

    // Leader (lowest active lane) reserves space for the whole warp
    int warp_base_offset = 0;
    if (local_rank == 0) {
      warp_base_offset = atomicAdd(result_count_global, warp_total);
    }

    // Broadcast base offset to all active threads
    int leader_lane = __ffs(mask) - 1;
    warp_base_offset = __shfl_sync(mask, warp_base_offset, leader_lane);

    // Write result
    results[warp_base_offset + local_rank] = {node_id, h_buf};
  }
}

void run_traceback_gpu(const Graph& graph,
                       const History& history,
                       const Layout& layout,
                       const CudaContext& ctx,
                       const thrust::device_vector<RootSolution>& d_root_sols,
                       const std::vector<LevelMeta>& metas,
                       int max_level)
{
  if (graph.root.empty()) {
    return;
  }
  int num_roots = graph.root.size();
  int num_nodes = graph.num_nodes;

  // 1. Reset State
  thrust::device_vector<int> d_node_sol_idx(num_nodes, -1);
  thrust::device_vector<BufferResult> d_results(num_nodes);
  thrust::device_vector<int> d_result_count(1, 0);

  int threads = 256;

  // 2. Seed Roots
  int blocks_roots = (num_roots + threads - 1) / threads;
  kernel_traceback_seed<<<blocks_roots, threads>>>(num_roots,
                                                   graph.get_root_view(),
                                                   raw_ptr(d_root_sols),
                                                   raw_ptr(d_node_sol_idx));

  // 3. Traceback Loop (Top-Down)
  for (int level = max_level; level >= 1; --level) {
    const auto& meta = metas[level];
    int blocks = (meta.size() + threads - 1) / threads;
    kernel_traceback_propagate<<<blocks, threads>>>(
        meta.start_node,
        meta.end_node,
        graph.get_view(),
        history.get_read_view(history.layer_offsets[level]),
        layout.get_read_view(),
        raw_ptr(d_node_sol_idx),
        raw_ptr(d_results),
        raw_ptr(d_result_count));
  }
  CUDA_CHECK(cudaDeviceSynchronize());

  // 4. Download Results
  int h_count = 0;
  cudaMemcpy(
      &h_count, raw_ptr(d_result_count), sizeof(int), cudaMemcpyDeviceToHost);

  std::vector<BufferResult> final_buffers(h_count);
  if (h_count > 0) {
    cudaMemcpy(final_buffers.data(),
               raw_ptr(d_results),
               h_count * sizeof(BufferResult),
               cudaMemcpyDeviceToHost);
  }
  std::vector<RootSolution> h_root_sols(num_roots);
  cudaMemcpy(h_root_sols.data(),
             raw_ptr(d_root_sols),
             num_roots * sizeof(RootSolution),
             cudaMemcpyDeviceToHost);

  // 5. Update Contexts
  // Distribute Buffers
  for (const auto& buf : final_buffers) {
    int context_id = graph.context_id[buf.node_id];
    ctx.results[context_id]->inserted_buffers.push_back(
        {buf.node_id, buf.buffer_type});
  }

  // Distribute Slacks
  for (size_t i = 0; i < ctx.results.size(); ++i) {
    ctx.results[i]->slack = h_root_sols[i].max_slack / scale;  // fs to s
  }
}

// =========================================================
// Verification Contracts
// =========================================================
enum Stage
{
  SINK_OUTPUT,            // Output of Sink Kernel
  WIRE_BUFFER_INPUT,      // Input to Wire Kernel (Children)
  WIRE_BUFFER_OUTPUT,     // Output of Wire Kernel (Raw)
  WIRE_PRUNE_OUTPUT,      // Output of Prune (Final Wire)
  JUNCTION_MERGE_INPUT,   // Input to Junction Merge (Children)
  JUNCTION_MERGE_OUTPUT,  // Output of Merge (Raw)
  JUNCTION_PRUNE_OUTPUT   // Output of Prune (Final Junction)
};

// Internal Logic Modes
enum CheckMode
{
  CHECK_PARETO,               // strictly L ascending, S ascending
  CHECK_LOAD_ASC,             // L ascending, S dont care (Wire Input)
  CHECK_LOAD_ASC_SLACK_DESC,  // L ascending, S descending on tie (Wire
                              // Output/Prune Input)
  CHECK_SLACK_ASC,            // S ascending (Merge Path output)
  CHECK_FINITE                // Just check for NaNs
};

__device__ const char* get_stage_name(Stage s)
{
  switch (s) {
    case SINK_OUTPUT:
      return "Sink Output";
    case WIRE_BUFFER_INPUT:
      return "Wire Input (Child)";
    case WIRE_BUFFER_OUTPUT:
      return "Wire Buffer Output";
    case WIRE_PRUNE_OUTPUT:
      return "Wire Prune Output";
    case JUNCTION_MERGE_INPUT:
      return "Junction Input (Children)";
    case JUNCTION_MERGE_OUTPUT:
      return "Junction Merge Output";
    case JUNCTION_PRUNE_OUTPUT:
      return "Junction Prune Output";
    default:
      return "Unknown";
  }
}

// MAP: Stage -> What Validation Logic to use?
__device__ constexpr CheckMode get_check_mode(Stage s)
{
  switch (s) {
    // Inputs & Final Outputs must always be Pareto Optimal
    case SINK_OUTPUT:
    case WIRE_PRUNE_OUTPUT:
    case JUNCTION_MERGE_INPUT:
    case JUNCTION_PRUNE_OUTPUT:
      return CHECK_PARETO;

    // Wire Kernel input is sorted by Load
    case WIRE_BUFFER_INPUT:
      return CHECK_LOAD_ASC;
    // Raw Wire Kernel output is sorted by Load
    case WIRE_BUFFER_OUTPUT:
      return CHECK_LOAD_ASC_SLACK_DESC;

    // Raw Junction Merge output is usually sorted by Slack (Merge Path
    // standard) Change this if your merge kernel outputs Load-sorted data
    case JUNCTION_MERGE_OUTPUT:
      return CHECK_SLACK_ASC;
  }
  return CHECK_FINITE;
}

__device__ constexpr bool is_input_check(Stage s)
{
  return (s == WIRE_BUFFER_INPUT || s == JUNCTION_MERGE_INPUT);
}

__device__ void check_pair(double l1,
                           double s1,
                           double l2,
                           double s2,
                           int node_id,
                           int idx,
                           CheckMode mode,
                           const char* label,
                           bool* fail)
{
  const char* reason = nullptr;
  switch (mode) {
    case CHECK_PARETO:
      if (l1 >= l2) {
        reason = "Load Not Strictly Ascending";
      } else if (s1 >= s2) {
        reason = "Non-Pareto (Slack not increasing)";
      }
      break;
    case CHECK_LOAD_ASC:
      if (l1 > l2) {
        reason = "Load Not Ascending";
      }
      break;
    case CHECK_LOAD_ASC_SLACK_DESC:
      if (l1 > l2) {
        reason = "Load Not Ascending";
      } else if (l1 == l2 && s1 < s2) {
        reason = "Tie: Slack Not Descending";
      }
      break;
    case CHECK_SLACK_ASC:
      if (s1 > s2) {
        reason = "Slack Not Ascending";
      }
      break;
    case CHECK_FINITE:
      break;
  }

  if (reason != nullptr) {
    printf(
        "\n[VERIFY FAIL] %s | Node %d | Idx %d->%d\n"
        "  Reason: %s\n"
        "  A: {L=%.5f, S=%.5f}\n"
        "  B: {L=%.5f, S=%.5f}\n",
        label,
        node_id,
        idx,
        idx + 1,
        reason,
        l1,
        s1,
        l2,
        s2);
    *fail = true;
  }
}

__device__ void verify_list(const Payload::ReadView& p,
                            const Layout::ReadView& l,
                            int node_id,
                            CheckMode mode,
                            const char* label)
{
  int count = l.get_count(node_id);
  int base = l.get_start(node_id);
  if (count <= 1) {
    return;
  }

  bool fail = false;
  for (int i = 0; i < count - 1; ++i) {
    check_pair(p.load[base + i],
               p.slack[base + i],
               p.load[base + i + 1],
               p.slack[base + i + 1],
               node_id,
               i,
               mode,
               label,
               &fail);
    if (fail) {
      break;
    }
  }
  // if (fail) asm("trap;");
}

__global__ void kernel_verify_stage(Graph::View graph,
                                    Payload::ReadView payload,
                                    Layout::ReadView layout,
                                    int start_node,
                                    int num_nodes,
                                    Stage stage)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_nodes) {
    return;
  }
  int node_id = start_node + idx;

  CheckMode mode = get_check_mode(stage);
  const char* label = get_stage_name(stage);

  if (is_input_check(stage)) {  // Check Input Children
    int children[2] = {-1, -1};
    int type = graph.get_type(node_id);

    if (type == 1) {
      children[0] = graph.get_left(node_id);  // Wire
    } else if (type == 2) {                   // Junction
      children[0] = graph.get_left(node_id);
      children[1] = graph.get_right(node_id);
    }

    for (int k = 0; k < 2; ++k) {
      if (children[k] > 0) {  // Verify the child list using the *current* stage
                              // requirements
        verify_list(payload, layout, children[k], mode, label);
      }
    }
  } else {  // Check Node Itself
    verify_list(payload, layout, node_id, mode, label);
  }
}

void verify(const Graph& g,
            const Payload& p,
            Layout& l,
            int start,
            int count,
            Stage stage)
{
#ifdef ENABLE_REQUIRE_ENSURE_CHECK
  if (count == 0) {
    return;
  }
  int threads = 128;  // Smaller block size is fine for debug
  int blocks = (count + threads - 1) / threads;

  kernel_verify_stage<<<blocks, threads>>>(
      g.get_view(), p.get_read_view(), l.get_read_view(), start, count, stage);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
#endif
}

namespace gobi {

// ==========================================================
// Main VG Solve Function
// ==========================================================

void print_gpu_memory(const char* label)
{
  if (!DEBUG) {
    return;
  }
  size_t free_byte;
  size_t total_byte;
  cudaError_t cuda_status = cudaMemGetInfo(&free_byte, &total_byte);

  if (cuda_status != cudaSuccess) {
    std::cout << "[MEM] " << label << " | Error getting memory info : "
              << cudaGetErrorString(cuda_status) << '\n';
    return;
  }

  double free_mb = static_cast<double>(free_byte) / (1024.0 * 1024.0);
  double used_mb
      = static_cast<double>(total_byte - free_byte) / (1024.0 * 1024.0);

  std::cout << "[MEM] " << std::left << std::setw(20) << label
            << " | Used: " << std::right << std::setw(8) << std::fixed
            << std::setprecision(2) << used_mb << " MB | Free: " << std::right
            << std::setw(8) << std::fixed << std::setprecision(2) << free_mb
            << " MB" << '\n';
}

struct SortedRoots
{
  std::vector<int> sorted_indexs;  // The re-ordered root indices
  std::vector<int> counts;         // How many roots in each level
};

SortedRoots sort_roots_by_level(const std::vector<int>& root,
                                const std::vector<int>& root_level,
                                int max_level)
{
  SortedRoots result;
  result.sorted_indexs.resize(root.size());
  result.counts.resize(max_level + 1, 0);

  // Count how many roots are in each level
  for (int lvl : root_level) {
    if (lvl <= max_level) {
      result.counts[lvl]++;
    }
  }
  // Sort roots by level
  std::iota(result.sorted_indexs.begin(), result.sorted_indexs.end(), 0);
  sort(result.sorted_indexs.begin(),
       result.sorted_indexs.end(),
       [&](int i, int j) { return root_level[i] < root_level[j]; });

  return result;
}

bool gobiCudaAvailable()
{
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

void gobiCudaSolve(const CudaContext& ctx)
{
  print_gpu_memory("Start of Solve");
  if (ctx.results.empty()) {
    return;
  }

  const auto& h_graph = ctx.h_graph;
  Graph graph(h_graph);
  const auto& metas = ctx.h_graph->level_metas;
  int max_level = ctx.h_graph->global_max_level;

  upload_buffer_library(*ctx.buffer_library);
  double prune_rb = 0.0;
  if (USING_PREDICTIVE_PRUNING) {
    prune_rb = INF;
    for (const auto& buffer : *ctx.buffer_library) {
      prune_rb = std::min(prune_rb, buffer.rb);
    }
  }
  Graph::View graph_view = graph.get_view();

  Layout layout(graph.num_nodes + 1);  // extra for exclusive scan
  Layout::View layout_view = layout.get_view();

  Payload frontier_A(1024);
  Payload frontier_B(1024);

  const auto payload_bytes = [](const Payload& payload) {
    return (payload.load.capacity() + payload.slack.capacity())
           * sizeof(double);
  };
  const auto frontier_bytes = [&](const Payload& lhs, const Payload& rhs) {
    return payload_bytes(lhs) + payload_bytes(rhs);
  };

  size_t frontier_active_levels = 0;
  size_t frontier_total_solutions = 0;
  size_t frontier_peak_layer_total = 0;

  History history;
  history.layer_offsets.resize(max_level + 1);

  Payload* curr = &frontier_A;
  Payload* next = &frontier_B;

  // Prepare Persistent Root Solutions Storage
  int num_roots = graph.root.size();
  thrust::device_vector<RootSolution> d_root_sols(num_roots, {-INF, -1});

  // Prepare Driver Resistance Array
  thrust::device_vector<double> d_driver_res = ctx.h_driver_res;

  // Prepare Map: Level -> List of Root Indices
  SortedRoots sorted_roots = sort_roots_by_level(
      h_graph->root, h_graph->root_level, ctx.h_graph->global_max_level);
  thrust::device_vector<int> d_sorted_roots = sorted_roots.sorted_indexs;
  size_t offset = 0;

  // Dummy call to setup cub temp storage size
  size_t temp_storage_bytes = 0;
  cub::DeviceScan::ExclusiveSum(nullptr,
                                temp_storage_bytes,
                                raw_ptr(layout.count),
                                raw_ptr(layout.start),
                                graph.num_nodes);
  thrust::device_vector<uint8_t> d_temp_storage_vec(temp_storage_bytes);
  void* d_temp_storage = thrust::raw_pointer_cast(d_temp_storage_vec.data());

  int threads = 256;
  for (int level = 0; level <= max_level; ++level) {
    const auto& meta = metas[level];
    if (meta.size() == 0) {
      continue;
    }

    frontier_active_levels++;

    int blocks = (meta.size() + threads - 1) / threads;

    // 1. Calculate Layout count from children
    kernel_layout<<<blocks, threads>>>(
        graph_view, layout_view, meta.start_node, meta.size());

    // 2. Scan to get starts, starting from 0 since kernel explicitly handle
    // dummy@0
    int* d_in = raw_ptr(layout.count, meta.start_node);
    int* d_out = raw_ptr(layout.start, meta.start_node);

    cub::DeviceScan::ExclusiveSum(
        d_temp_storage, temp_storage_bytes, d_in, d_out, meta.size() + 1);
    int total = 0;
    cudaMemcpy(&total,
               raw_ptr(layout.start, meta.start_node + meta.size()),
               sizeof(int),
               cudaMemcpyDeviceToHost);
    frontier_total_solutions += static_cast<size_t>(total);
    frontier_peak_layer_total
      = std::max(frontier_peak_layer_total, static_cast<size_t>(total));
    // 3. Alloc
    next->ensure_capacity(total);
    size_t h_base = history.append_layer(level, total);
    // Read view after allocation to avoid stale pointer
    Payload::ReadView input = curr->get_read_view();
    Payload::View output = next->get_write_view();
    History::View history_view = history.get_view(h_base);

    // 4. Solve
    if (meta.sink_count > 0) {
      int blocks = (meta.sink_count + threads - 1) / threads;
      int start = meta.start_node;
      kernel_sink<<<blocks, threads>>>(
          graph_view, output, layout_view, start, meta.sink_count);
      CUDA_CHECK(cudaGetLastError());
      verify(
          graph, *next, layout, meta.start_node, meta.sink_count, SINK_OUTPUT);
    }
    if (meta.wire_count > 0) {
      int start = meta.start_node + meta.sink_count;
      verify(graph, *curr, layout, start, meta.wire_count, WIRE_BUFFER_INPUT);
      kernel_wire_buffer<<<meta.wire_count, WIRE_TILE_SIZE>>>(graph_view,
                                                              input,
                                                              output,
                                                              history_view,
                                                              layout_view,
                                                              start,
                                                              meta.wire_count);
      CUDA_CHECK(cudaGetLastError());
      verify(graph, *next, layout, start, meta.wire_count, WIRE_BUFFER_OUTPUT);

      kernel_prune<<<meta.wire_count, PRUNE_BLOCK_THREADS>>>(
          output, history_view, layout_view, start, meta.wire_count, prune_rb);
      CUDA_CHECK(cudaGetLastError());
      verify(graph, *next, layout, start, meta.wire_count, WIRE_PRUNE_OUTPUT);
    }
    if (meta.junc_count > 0) {
      int start = meta.start_node + meta.sink_count + meta.wire_count;
      verify(
          graph, *curr, layout, start, meta.junc_count, JUNCTION_MERGE_INPUT);
      kernel_junction_merge<<<meta.junc_count, MERGE_TILE_SIZE>>>(
          graph_view,
          input,
          output,
          history_view,
          layout_view,
          start,
          meta.junc_count);
      CUDA_CHECK(cudaGetLastError());
      verify(
          graph, *next, layout, start, meta.junc_count, JUNCTION_MERGE_OUTPUT);
      // This prune is not strictly necessary, but helps keep sizes down
      // kernel_prune<<<meta.junc_count, PRUNE_BLOCK_THREADS>>>(
      //     output, history_view, layout_view, start, meta.junc_count);
    }

    if (int root_count = sorted_roots.counts[level]; root_count > 0) {
      int blocks = (root_count + threads - 1) / threads;
      kernel_solve_roots<<<blocks, threads>>>(raw_ptr(d_sorted_roots, offset),
                                              root_count,
                                              graph.get_root_view(),
                                              next->get_read_view(),
                                              layout_view,
                                              raw_ptr(d_driver_res),
                                              raw_ptr(d_root_sols));
      offset += root_count;
      CUDA_CHECK(cudaGetLastError());
    }
    cudaDeviceSynchronize();
    std::swap(curr, next);
  }

  run_traceback_gpu(graph, history, layout, ctx, d_root_sols, metas, max_level);
  print_gpu_memory("End of Solve");

  const size_t frontier_peak_bytes = frontier_bytes(frontier_A, frontier_B);
  const size_t frontier_baseline_bytes
  = frontier_total_solutions * 2 * sizeof(double);
  const size_t frontier_saved_bytes = frontier_baseline_bytes > frontier_peak_bytes
                 ? frontier_baseline_bytes
                   - frontier_peak_bytes
                 : 0;
  const double frontier_saved_ratio = frontier_baseline_bytes == 0
              ? 0.0
              : static_cast<double>(frontier_saved_bytes)
                / static_cast<double>(frontier_baseline_bytes);

  std::cout << "[MEM] Frontier whole-solve"
            << " | Active levels: " << frontier_active_levels
    << " | Total solutions: " << frontier_total_solutions
    << " | Peak layer total: " << frontier_peak_layer_total
    << " | Peak reserved: " << std::fixed << std::setprecision(2)
    << static_cast<double>(frontier_peak_bytes) / (1024.0 * 1024.0)
    << " MB"
    << " | Baseline(all layers): " << std::fixed << std::setprecision(2)
    << static_cast<double>(frontier_baseline_bytes)
       / (1024.0 * 1024.0)
    << " MB"
    << " | Saved: " << std::fixed << std::setprecision(2)
    << static_cast<double>(frontier_saved_bytes) / (1024.0 * 1024.0)
    << " MB"
    << " | Saved ratio: " << std::fixed << std::setprecision(2)
    << frontier_saved_ratio * 100.0 << "%"
    << '\n';
  //  exit(0);
  // std::cout << "Total offset: " << history.current_offset << "\n";
  // int total_count = 0;
  // thrust::host_vector<int> h_count = layout.count;
  // for (int i = 0; i < graph.num_nodes; ++i) {
  //   total_count += h_count[i];
  // }
  // std::cout << "Total count: " << total_count << "\n";
}

}  // namespace gobi

}  // namespace ecc
