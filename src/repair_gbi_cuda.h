// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <vector>

namespace ecc {

// Candidate arrays are grouped by net.  offsets has net_count + 1 entries and
// indexes the flat cap/slack/buffer_count arrays.  The returned index is local
// to each net, or -1 when no candidate is usable.  This interface deliberately
// contains no OpenDB or STA types so the CUDA implementation can be tested in
// isolation and the ECO path remains owned by Sizer.
bool gbiCudaAvailable();
std::vector<int> gbiCudaSelect(const std::vector<int>& offsets,
                               const std::vector<double>& cap,
                               const std::vector<double>& slack,
                               const std::vector<unsigned>& buffer_count,
                               const std::vector<double>& driver_resistance);

// Postorder forest: children precede parents. Wire RC is total segment RC;
// load nodes supply sink capacitance and required time. No N_SEG subdivision.
enum class GbiCudaNodeType { load, wire, junction };
struct GbiCudaNode {
    GbiCudaNodeType type = GbiCudaNodeType::load;
    int left = -1;
    int right = -1;
    double cap = 0;
    double resistance = 0;
    double required = 0;
};
struct GbiCudaBuffer {
    double cap = 0;
    double intrinsic = 0;
    double resistance = 0;
};
struct GbiCudaCandidate {
    double cap = 0;
    double slack = 0;
    unsigned buffers = 0;
    int left = -1;   // Candidate index in left child frontier.
    int right = -1;  // Candidate index in right child frontier.
    int buffer = -1; // Library index, or unbuffered wire.
    int node = -1;
    int order = -1;  // Stable enumeration order before Pareto pruning.
};
struct GbiCudaForestResult {
    std::vector<std::vector<GbiCudaCandidate>> frontiers;
    std::vector<int> selected;  // Local index in each root frontier.
};
// Propagate, merge, prune and select on GPU; host reconstructs only selected
// trees. Throws on invalid input, unavailable CUDA or memory budget exhaustion.
GbiCudaForestResult gbiCudaSolve(
    const std::vector<GbiCudaNode>& nodes,
    const std::vector<GbiCudaBuffer>& buffers,
    const std::vector<int>& roots,
    const std::vector<double>& driver_resistance,
    size_t max_memory_bytes = size_t{512} * 1024 * 1024);

}  // namespace ecc
