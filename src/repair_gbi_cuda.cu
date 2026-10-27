// SPDX-License-Identifier: Apache-2.0

#include "repair_gbi_cuda.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <stdexcept>

namespace ecc {
namespace {

__global__ void select_candidates_kernel(const int* offsets,
                                         const double* cap,
                                         const double* slack,
                                         const unsigned* buffer_count,
                                         const double* driver_resistance,
                                         int* selected) {
    const int net = blockIdx.x;
    const int begin = offsets[net];
    const int end = offsets[net + 1];
    const double resistance = driver_resistance[net];

    double best_score = -INFINITY;
    int best = -1;
    for(int i = begin + threadIdx.x; i < end; i += blockDim.x) {
        if(buffer_count[i] == 0)
            continue;
        const double score = slack[i] - resistance * cap[i];
        if(score > best_score || (score == best_score && (best < 0 || i < best))) {
            best_score = score;
            best = i;
        }
    }

    extern __shared__ unsigned char storage[];
    auto* scores = reinterpret_cast<double*>(storage);
    auto* indices = reinterpret_cast<int*>(scores + blockDim.x);
    scores[threadIdx.x] = best_score;
    indices[threadIdx.x] = best;
    __syncthreads();
    for(int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if(threadIdx.x < stride) {
            const int other = indices[threadIdx.x + stride];
            if(scores[threadIdx.x + stride] > scores[threadIdx.x] ||
               (scores[threadIdx.x + stride] == scores[threadIdx.x] &&
                other >= 0 && (indices[threadIdx.x] < 0 || other < indices[threadIdx.x]))) {
                scores[threadIdx.x] = scores[threadIdx.x + stride];
                indices[threadIdx.x] = other;
            }
        }
        __syncthreads();
    }
    if(threadIdx.x == 0)
        selected[net] = indices[0] < 0 ? -1 : indices[0] - begin;
}

bool validInput(const std::vector<int>& offsets,
                const std::vector<double>& cap,
                const std::vector<double>& slack,
                const std::vector<unsigned>& buffer_count,
                const std::vector<double>& driver_resistance) {
    if(offsets.size() != driver_resistance.size() + 1 || offsets.empty())
        return false;
    if(offsets.front() != 0 || offsets.back() < 0)
        return false;
    const auto total = static_cast<size_t>(offsets.back());
    if(cap.size() != total || slack.size() != total || buffer_count.size() != total)
        return false;
    for(size_t i = 1; i < offsets.size(); ++i)
        if(offsets[i] < offsets[i - 1])
            return false;
    return true;
}

}  // namespace

bool gbiCudaAvailable() {
    int count = 0;
    return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

std::vector<int> gbiCudaSelect(const std::vector<int>& offsets,
                               const std::vector<double>& cap,
                               const std::vector<double>& slack,
                               const std::vector<unsigned>& buffer_count,
                               const std::vector<double>& driver_resistance) {
    if(!validInput(offsets, cap, slack, buffer_count, driver_resistance))
        throw std::invalid_argument("invalid GBI CUDA candidate batch");
    const int nets = static_cast<int>(driver_resistance.size());
    std::vector<int> result(driver_resistance.size(), -1);
    if(nets == 0 || offsets.back() == 0)
        return result;

    int devices = 0;
    if(cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0)
        return result;
    int* d_offsets = nullptr;
    double* d_cap = nullptr;
    double* d_slack = nullptr;
    unsigned* d_buffer_count = nullptr;
    double* d_driver_resistance = nullptr;
    int* d_selected = nullptr;
    const auto free_all = [&]() {
        cudaFree(d_offsets);
        cudaFree(d_cap);
        cudaFree(d_slack);
        cudaFree(d_buffer_count);
        cudaFree(d_driver_resistance);
        cudaFree(d_selected);
    };
    const size_t total = static_cast<size_t>(offsets.back());
    auto check = [](cudaError_t error) {
        if(error != cudaSuccess)
            throw std::runtime_error(cudaGetErrorString(error));
    };
    try {
        check(cudaMalloc(&d_offsets, offsets.size() * sizeof(int)));
        check(cudaMalloc(&d_cap, total * sizeof(double)));
        check(cudaMalloc(&d_slack, total * sizeof(double)));
        check(cudaMalloc(&d_buffer_count, total * sizeof(unsigned)));
        check(cudaMalloc(&d_driver_resistance,
                         driver_resistance.size() * sizeof(double)));
        check(cudaMalloc(&d_selected, result.size() * sizeof(int)));
        check(cudaMemcpy(d_offsets, offsets.data(), offsets.size() * sizeof(int),
                         cudaMemcpyHostToDevice));
        check(cudaMemcpy(d_cap, cap.data(), total * sizeof(double),
                         cudaMemcpyHostToDevice));
        check(cudaMemcpy(d_slack, slack.data(), total * sizeof(double),
                         cudaMemcpyHostToDevice));
        check(cudaMemcpy(d_buffer_count, buffer_count.data(),
                         total * sizeof(unsigned), cudaMemcpyHostToDevice));
        check(cudaMemcpy(d_driver_resistance, driver_resistance.data(),
                         driver_resistance.size() * sizeof(double),
                         cudaMemcpyHostToDevice));
        const int threads = 128;
        select_candidates_kernel<<<nets, threads,
                                   threads * (sizeof(double) + sizeof(int))>>>(
            d_offsets, d_cap, d_slack, d_buffer_count, d_driver_resistance,
            d_selected);
        check(cudaGetLastError());
        check(cudaDeviceSynchronize());
        check(cudaMemcpy(result.data(), d_selected, result.size() * sizeof(int),
                         cudaMemcpyDeviceToHost));
    }
    catch(...) {
        free_all();
        throw;
    }
    free_all();
    return result;
}

}  // namespace ecc

// The level schedule and flat frontiers follow the original GBI CUDA design.
// Unlike its Context/History structures, these records refer only to local
// candidate indices and can be reconstructed into Sizer's BufferedNet trees.
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <climits>
#include <cmath>
#include <memory>
#include <limits>

namespace ecc {
namespace {
struct FrontierView {
    GbiCudaCandidate* data = nullptr;
    int count = 0;
};
struct LevelTask {
    int node;
    int begin;
    int count;
};
struct CandidateLess {
    __host__ __device__ bool operator()(const GbiCudaCandidate& a,
                                       const GbiCudaCandidate& b) const {
        if(a.node != b.node) return a.node < b.node;
        return a.cap != b.cap ? a.cap < b.cap : a.slack > b.slack;
    }
};

__global__ void propagate_level(const GbiCudaNode* nodes,
                                const GbiCudaBuffer* buffers,
                                int num_buffers,
                                const LevelTask* tasks,
                                const FrontierView* states,
                                GbiCudaCandidate* output) {
    const auto task = tasks[blockIdx.x];
    const auto node = nodes[task.node];
    for(int i = threadIdx.x; i < task.count; i += blockDim.x) {
        GbiCudaCandidate c;
        c.node = task.node;
        c.order = i;
        if(node.type == GbiCudaNodeType::load) {
            c.cap = node.cap;
            c.slack = node.required;
        } else if(node.type == GbiCudaNodeType::wire) {
            c.left = i / (num_buffers + 1);
            c.buffer = i % (num_buffers + 1) - 1;
            const auto child = states[node.left].data[c.left];
            c.cap = child.cap + node.cap;
            c.slack = child.slack - node.resistance * (child.cap + node.cap / 2.0);
            c.buffers = child.buffers;
            if(c.buffer >= 0) {
                const auto buffer = buffers[c.buffer];
                c.slack -= buffer.intrinsic + buffer.resistance * c.cap;
                c.cap = buffer.cap;
                ++c.buffers;
            }
        } else {
            c.left = i / states[node.right].count;
            c.right = i % states[node.right].count;
            const auto left = states[node.left].data[c.left];
            const auto right = states[node.right].data[c.right];
            c.cap = left.cap + right.cap;
            c.slack = left.slack < right.slack ? left.slack : right.slack;
            c.buffers = left.buffers + right.buffers;
        }
        output[task.begin + i] = c;
    }
}

// Each node is independent. Stable sorted order and the tolerance/preserved
// maximum-buffer candidate match the existing CPU pruneVg policy exactly.
__global__ void prune_level(const LevelTask* tasks,
                            GbiCudaCandidate* data,
                            FrontierView* states,
                            int* counts, int task_count) {
    const int t = blockIdx.x * blockDim.x + threadIdx.x;
    // grid's final block may contain spare threads.
    if(t >= task_count) return;
    const auto task = tasks[t];
    auto* candidates = data + task.begin;
    GbiCudaCandidate preserve = candidates[0];
    for(int i = 1; i < task.count; ++i) {
        const auto c = candidates[i];
        if(c.buffers > preserve.buffers ||
           (c.buffers == preserve.buffers && c.order < preserve.order))
            preserve = c;
    }
    int count = 0;
    bool preserved = false;
    double best = -INFINITY;
    for(int i = 0; i < task.count; ++i) {
        const auto c = candidates[i];
        if(c.slack > best + 1e-18) {
            best = c.slack;
            preserved |= c.order == preserve.order;
            candidates[count++] = c;
        }
    }
    if(!preserved) candidates[count++] = preserve;
    states[task.node] = {candidates, count};
    counts[t] = count;
}

__global__ void compact_level(const LevelTask* tasks, const int* offsets,
                              const GbiCudaCandidate* raw,
                              GbiCudaCandidate* compact,
                              FrontierView* states) {
    const int t = blockIdx.x;
    const auto task = tasks[t];
    const int count = offsets[t + 1] - offsets[t];
    for(int i = threadIdx.x; i < count; i += blockDim.x)
        compact[offsets[t] + i] = raw[task.begin + i];
    if(threadIdx.x == 0)
        states[task.node] = {compact + offsets[t], count};
}

void cudaCheck(cudaError_t status) {
    if(status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
template<class T> T* devicePointer(thrust::device_vector<T>& v) {
    return thrust::raw_pointer_cast(v.data());
}
}  // namespace

GbiCudaForestResult gbiCudaSolve(
    const std::vector<GbiCudaNode>& nodes,
    const std::vector<GbiCudaBuffer>& buffers,
    const std::vector<int>& roots,
    const std::vector<double>& driver_resistance,
    size_t max_memory_bytes) {
    if(roots.size() != driver_resistance.size() || nodes.size() > INT_MAX ||
       buffers.size() >= INT_MAX || max_memory_bytes == 0)
        throw std::invalid_argument("invalid GBI CUDA forest dimensions");
    std::vector<int> depth(nodes.size());
    std::vector<std::vector<int>> levels;
    for(size_t i = 0; i < nodes.size(); ++i) {
        const auto& n = nodes[i];
        if(!std::isfinite(n.cap) || n.cap < 0 ||
           !std::isfinite(n.resistance) || n.resistance < 0 ||
           !std::isfinite(n.required))
            throw std::invalid_argument("invalid GBI CUDA node RC/required time");
        if(n.type != GbiCudaNodeType::load) {
            if(n.left < 0 || n.left >= static_cast<int>(i))
                throw std::invalid_argument("GBI CUDA nodes must be postorder");
            depth[i] = depth[n.left] + 1;
        }
        if(n.type == GbiCudaNodeType::junction) {
            if(n.right < 0 || n.right >= static_cast<int>(i))
                throw std::invalid_argument("GBI CUDA junction child invalid");
            depth[i] = std::max(depth[i], depth[n.right] + 1);
        } else if(n.type != GbiCudaNodeType::load && n.type != GbiCudaNodeType::wire) {
            throw std::invalid_argument("GBI CUDA unknown node type");
        }
        if(levels.size() <= static_cast<size_t>(depth[i])) levels.resize(depth[i] + 1);
        levels[depth[i]].push_back(static_cast<int>(i));
    }
    for(const auto& b : buffers)
        if(!std::isfinite(b.cap) || b.cap < 0 || !std::isfinite(b.intrinsic) ||
           !std::isfinite(b.resistance) || b.resistance < 0)
            throw std::invalid_argument("invalid GBI CUDA buffer model");
    for(size_t i = 0; i < roots.size(); ++i)
        if(roots[i] < 0 || roots[i] >= static_cast<int>(nodes.size()) ||
           !std::isfinite(driver_resistance[i]))
            throw std::invalid_argument("invalid GBI CUDA root");
    if(!gbiCudaAvailable()) throw std::runtime_error("GBI CUDA device unavailable");
    GbiCudaForestResult result;
    result.frontiers.resize(nodes.size());
    if(nodes.empty()) return result;
    const size_t base_bytes = nodes.size() * (sizeof(GbiCudaNode) + sizeof(FrontierView)) +
                              buffers.size() * sizeof(GbiCudaBuffer);
    if(base_bytes > max_memory_bytes)
        throw std::runtime_error("GBI CUDA memory budget exceeded");
    thrust::device_vector<GbiCudaNode> d_nodes(nodes.begin(), nodes.end());
    thrust::device_vector<GbiCudaBuffer> d_buffers(buffers.begin(), buffers.end());
    thrust::device_vector<FrontierView> d_states(nodes.size());
    std::vector<int> counts(nodes.size());
    using DeviceCandidates = thrust::device_vector<GbiCudaCandidate>;
    std::vector<std::unique_ptr<DeviceCandidates>> retained;
    size_t retained_bytes = base_bytes;
    for(const auto& level : levels) {
        std::vector<LevelTask> tasks;
        size_t total = 0;
        for(int id : level) {
            const auto& n = nodes[id];
            size_t count = 1;
            if(n.type == GbiCudaNodeType::wire)
                count = size_t(counts[n.left]) * (buffers.size() + 1);
            else if(n.type == GbiCudaNodeType::junction)
                count = size_t(counts[n.left]) * counts[n.right];
            if(count == 0 || count > INT_MAX || total + count > INT_MAX)
                throw std::runtime_error("GBI CUDA candidate count overflow");
            tasks.push_back({id, static_cast<int>(total), static_cast<int>(count)});
            total += count;
        }
        // Include space for stable-sort temporary storage and level metadata.
        const size_t level_bytes = total * sizeof(GbiCudaCandidate);
        const size_t temporary_bytes = level.size() * (sizeof(LevelTask) + sizeof(int));
        if(retained_bytes > max_memory_bytes ||
           3 * level_bytes + temporary_bytes > max_memory_bytes - retained_bytes)
            throw std::runtime_error("GBI CUDA memory budget exceeded");
        auto data = std::make_unique<DeviceCandidates>(total);
        thrust::device_vector<LevelTask> d_tasks(tasks.begin(), tasks.end());
        thrust::device_vector<int> d_counts(level.size());
        propagate_level<<<level.size(), 128>>>(
            devicePointer(d_nodes), devicePointer(d_buffers), buffers.size(),
            devicePointer(d_tasks), devicePointer(d_states), devicePointer(*data));
        cudaCheck(cudaGetLastError());
        thrust::stable_sort(data->begin(), data->end(), CandidateLess{});
        // Pass task count explicitly rather than limiting number of nodes to
        // a fixed candidate pool or the original N_SEG constant.
        prune_level<<<(level.size() + 127) / 128, 128>>>(
            devicePointer(d_tasks), devicePointer(*data), devicePointer(d_states),
            devicePointer(d_counts), static_cast<int>(level.size()));
        cudaCheck(cudaGetLastError());
        std::vector<int> level_counts(level.size());
        cudaCheck(cudaMemcpy(level_counts.data(), devicePointer(d_counts),
                             level_counts.size() * sizeof(int), cudaMemcpyDeviceToHost));
        std::vector<int> compact_offsets{0};
        for(int count : level_counts)
            compact_offsets.push_back(compact_offsets.back() + count);
        thrust::device_vector<int> d_compact_offsets(compact_offsets.begin(), compact_offsets.end());
        auto compact = std::make_unique<DeviceCandidates>(compact_offsets.back());
        compact_level<<<level.size(), 128>>>(devicePointer(d_tasks),
            devicePointer(d_compact_offsets), devicePointer(*data),
            devicePointer(*compact), devicePointer(d_states));
        cudaCheck(cudaGetLastError());
        // Retain and download only pruned frontiers, not the Cartesian
        // products. The next level's device references point into compact.
        const size_t compact_bytes = compact->size() * sizeof(GbiCudaCandidate);
        std::vector<GbiCudaCandidate> host(compact->size());
        cudaCheck(cudaMemcpy(host.data(), devicePointer(*compact), compact_bytes,
                             cudaMemcpyDeviceToHost));
        for(size_t t = 0; t < tasks.size(); ++t) {
            counts[tasks[t].node] = level_counts[t];
            result.frontiers[tasks[t].node].assign(
                host.begin() + compact_offsets[t], host.begin() + compact_offsets[t + 1]);
        }
        retained_bytes += compact_bytes;
        retained.push_back(std::move(compact));
    }
    std::vector<int> offsets{0};
    std::vector<double> cap, slack;
    std::vector<unsigned> buffer_count;
    for(int root : roots) {
        for(const auto& c : result.frontiers[root]) {
            cap.push_back(c.cap);
            slack.push_back(c.slack);
            buffer_count.push_back(c.buffers);
        }
        if(cap.size() > INT_MAX) throw std::runtime_error("GBI CUDA root count overflow");
        offsets.push_back(static_cast<int>(cap.size()));
    }
    // Release search storage before final root selection.
    retained.clear();
    result.selected = gbiCudaSelect(offsets, cap, slack, buffer_count, driver_resistance);
    return result;
}
}  // namespace ecc
