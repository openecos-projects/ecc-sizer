// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <vector>

namespace ecc {

// Apply complete net plans in collection order. An oversized plan is skipped
// without preventing later smaller plans from fitting. Zero means unlimited.
// This is an application budget, not a constraint on the solver's frontier.
inline std::vector<std::size_t> selectWholeBufferPlans(
    const std::vector<std::size_t>& counts, std::size_t max_buffers)
{
    std::vector<std::size_t> selected;
    auto remaining = max_buffers;
    for(std::size_t i = 0; i < counts.size(); ++i) {
        if(counts[i] && (!max_buffers || counts[i] <= remaining)) {
            selected.push_back(i);
            if(max_buffers) remaining -= counts[i];
        }
    }
    return selected;
}

}  // namespace ecc
