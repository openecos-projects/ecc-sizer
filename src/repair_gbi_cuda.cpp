// SPDX-License-Identifier: Apache-2.0

#include "repair_gbi_cuda.h"
#include <stdexcept>

namespace ecc {

bool gbiCudaAvailable() { return false; }

std::vector<int> gbiCudaSelect(const std::vector<int>& offsets,
                               const std::vector<double>&,
                               const std::vector<double>&,
                               const std::vector<unsigned>&,
                               const std::vector<double>& driver_resistance) {
    return std::vector<int>(driver_resistance.size(), -1);
}

GbiCudaForestResult gbiCudaSolve(
    const std::vector<GbiCudaNode>&,
    const std::vector<GbiCudaBuffer>&,
    const std::vector<int>&,
    const std::vector<double>&,
    size_t) {
    throw std::runtime_error("GBI CUDA backend is not built");
}

}  // namespace ecc
