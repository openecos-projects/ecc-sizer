// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

namespace sta {
class Units;
class TableModel;
}

struct LibLUT {
    // Look up table is indexed by the output load and the input transition
    // values
    // Example:
    //   Let L = loadIndices[i]
    //       T = transitionIndices[j]
    //   Then, the table value corresponding to L and T will be:
    //       table[i][j]
    //
    // Always: index1 = load; index2 = tran
    std::string templ;
    std::vector< double > loadIndices;
    std::vector< double > transitionIndices;
    std::vector< std::vector< double > > tableVals;
};

// Convert OpenSTA scalar/1D/2D timing tables into Sizer's rectangular LUT.
void extract_lut(const sta::Units* units,
                 const sta::TableModel* model, LibLUT& lut);
