// SPDX-License-Identifier: Apache-2.0

#include "algorithms/vg_cuda.h"
#include <stdexcept>

namespace ecc {

bool vgCudaAvailable() { return false; }

std::vector<int> vgCudaSelect(const std::vector<int>& offsets,
                               const std::vector<double>&,
                               const std::vector<double>&,
                               const std::vector<unsigned>&,
                               const std::vector<double>& driver_resistance) {
    return std::vector<int>(driver_resistance.size(), -1);
}

VgCudaForestResult vgCudaSolve(
    const std::vector<VgCudaNode>&,
    const std::vector<VgCudaBuffer>&,
    const std::vector<int>&,
    const std::vector<double>&,
    size_t) {
    throw std::runtime_error("VG CUDA backend is not built");
}

}  // namespace ecc
