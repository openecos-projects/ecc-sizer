// SPDX-License-Identifier: Apache-2.0
#include "algorithms/gobi_cuda.h"

namespace ecc::gobi {

bool gobiCudaAvailable()
{
  return false;
}

void gobiCudaSolve(const CudaContext&)
{
  throw std::runtime_error("GOBI CUDA backend is not built");
}

}  // namespace ecc::gobi
