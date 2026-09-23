// Standalone (odb-free) legalization core built on the vendored DreamPlace
// greedy+abacus operators. Feeding plain arrays keeps it unit-testable.
#pragma once

#include <vector>

namespace ecc {

struct LegalizeRowSeg {
    int origin_x, origin_y, site_count;
};

struct LegalizeNode {
    double x, y, w, h;
    bool fixed;
};

// Legalize movable nodes (indices [0, num_movable) of `nodes`) against fixed
// obstacles on the shared row grid (all segments must share one site grid;
// every row y must be covered by a segment).
//
// On success returns 0 and fills out_x/out_row for the movable nodes;
// out_x is in DBU on the site grid, out_row is the row index (row y is
// base_y + out_row*site_h, reported via base_x/base_y/num_rows_out).
// Returns -1 when the input is inapplicable (caller should fall back).
int dpLegalizeCore(const std::vector<LegalizeRowSeg>& segs, int site_w,
                   int site_h, std::vector<LegalizeNode>& nodes,
                   int num_movable, double padding_sites,
                   std::vector<double>& out_x, std::vector<int>& out_row,
                   int& base_x, int& base_y, int& num_rows_out);

}  // namespace ecc
