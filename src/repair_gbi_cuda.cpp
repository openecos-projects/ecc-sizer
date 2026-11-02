// SPDX-License-Identifier: Apache-2.0
#include "repair_gbi_cuda.h"

namespace ecc::gbi {

bool gbiCudaAvailable()
{
  return false;
}

void gbiCudaSolve(const CudaContext&)
{
  throw std::runtime_error("GBI CUDA backend is not built");
}

}  // namespace ecc::gbi
