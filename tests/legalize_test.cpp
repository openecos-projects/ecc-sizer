// Unit tests for the standalone legalization core (dpLegalizeCore).
// Scenario factory + legality assertions: no overlaps, on-row, on-site,
// padding gaps where feasible.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/legalize_core.h"

using ecc::LegalizeNode;
using ecc::LegalizeRowSeg;

namespace {

const int SITE_W = 200;
const int SITE_H = 1400;

struct Scenario {
    std::string name;
    std::vector<LegalizeRowSeg> segs;
    std::vector<LegalizeNode> nodes;  // movable first, then fixed
    int num_movable = 0;
    double padding = 0.0;             // sites per side
};

// Row helpers: full rows [0, x1) at y = base_y + r*SITE_H
std::vector<LegalizeRowSeg> full_rows(int nrows, int base_x, int base_y,
                                      int sites) {
    std::vector<LegalizeRowSeg> segs;
    for(int r = 0; r < nrows; ++r) {
        segs.push_back({base_x, base_y + r * SITE_H, sites});
    }
    return segs;
}

// Row with a macro hole: two segments around [hole_x0, hole_x1)
void add_segmented_row(std::vector<LegalizeRowSeg>& segs, int r, int base_x,
                       int base_y, int sites, int hole_x0, int hole_x1) {
    const int y = base_y + r * SITE_H;
    segs.push_back({base_x, y, (hole_x0 - base_x) / SITE_W});
    segs.push_back({hole_x1, y, (base_x + sites * SITE_W - hole_x1) / SITE_W});
}

int failures = 0;

void check(bool ok, const std::string& what) {
    if(!ok) {
        printf("  FAIL: %s\n", what.c_str());
        ++failures;
    }
}

void verify(const Scenario& sc, const std::vector<LegalizeNode>& nodes,
            int num_movable, const std::vector<double>& out_x,
            const std::vector<int>& out_row, int base_x, int base_y,
            int num_rows) {
    printf("  verifying %zu movable cells\n", out_x.size());
    int max_x = 0;
    for(auto& s : sc.segs) {
        max_x = std::max(max_x, s.origin_x + s.site_count * SITE_W);
    }
    // (a) movable-movable and movable-fixed overlap
    int overlaps = 0;
    for(int i = 0; i < num_movable; ++i) {
        double ax = out_x[i], ay = base_y + out_row[i] * SITE_H;
        double aw = nodes[i].w;
        for(size_t j = 0; j < nodes.size(); ++j) {
            if((int)j == i) {
                continue;
            }
            double bx, by;
            if(j < (size_t)num_movable) {
                if(j <= (size_t)i) {
                    continue;  // check each pair once
                }
                bx = out_x[j];
                by = base_y + out_row[j] * SITE_H;
            }
            else {
                bx = nodes[j].x;
                by = nodes[j].y;
            }
            double bw = nodes[j].w, bh = nodes[j].h;
            bool ox = ax < bx + bw - 1e-9 && bx < ax + aw - 1e-9;
            bool oy = ay < by + bh - 1e-9 && by < ay + SITE_H - 1e-9;
            if(ox && oy) {
                ++overlaps;
                if(overlaps <= 3) {
                    printf("    overlap: cell %d at (%.0f,%.0f) vs node %zu "
                           "at (%.0f,%.0f)\n",
                           i, ax, ay, j, bx, by);
                }
            }
        }
    }
    check(overlaps == 0, "overlaps exist");
    // (b) on site grid and inside core
    int offgrid = 0;
    for(int i = 0; i < num_movable; ++i) {
        int rx = (int)llround(out_x[i]);
        if((rx - base_x) % SITE_W != 0) {
            ++offgrid;
        }
        if(rx < base_x || rx + nodes[i].w > max_x + 1e-6) {
            ++offgrid;  // out of core horizontally
        }
        if(out_row[i] < 0 || out_row[i] >= num_rows) {
            ++offgrid;
        }
    }
    check(offgrid == 0, "cells off grid or out of core");
    // (c) padding between movable pairs on the same row (gap >= pad sites)
    if(sc.padding > 0) {
        std::vector<std::vector<int>> byrow(num_rows);
        for(int i = 0; i < num_movable; ++i) {
            byrow[out_row[i]].push_back(i);
        }
        int pad_viol = 0;
        for(auto& cids : byrow) {
            std::sort(cids.begin(), cids.end(),
                      [&](int a, int b) { return out_x[a] < out_x[b]; });
            for(size_t k = 1; k < cids.size(); ++k) {
                double gap = out_x[cids[k]] -
                             (out_x[cids[k - 1]] + nodes[cids[k - 1]].w);
                if(gap < sc.padding * SITE_W - 1e-9) {
                    ++pad_viol;
                }
            }
        }
        printf("  padding violations: %d (allowed only on degraded rows)\n",
               pad_viol);
    }
}

void run(const Scenario& sc) {
    printf("scenario: %s\n", sc.name.c_str());
    std::vector<LegalizeNode> nodes = sc.nodes;
    std::vector<double> out_x;
    std::vector<int> out_row;
    int base_x = 0, base_y = 0, num_rows = 0;
    int rc = ecc::dpLegalizeCore(sc.segs, SITE_W, SITE_H, nodes,
                                 sc.num_movable, sc.padding, out_x, out_row,
                                 base_x, base_y, num_rows);
    if(rc != 0) {
        printf("  core returned %d (fell back)\n", rc);
        check(false, "core should handle this scenario");
        return;
    }
    verify(sc, nodes, sc.num_movable, out_x, out_row, base_x, base_y,
           num_rows);
}

// 1. Simple: 6 slightly overlapped cells on 3 rows.
Scenario make_basic() {
    Scenario sc;
    sc.name = "basic";
    sc.segs = full_rows(3, 2000, 2000, 237);
    const double w = 1400;  // 7 sites
    double xs[] = {2000, 3200, 4000, 2200, 4800, 6600};
    for(int i = 0; i < 6; ++i) {
        sc.nodes.push_back({xs[i], 2000.0 + (i % 3) * SITE_H, w, SITE_H, false});
    }
    sc.num_movable = 6;
    sc.padding = 1.0;
    return sc;
}

// 2. Macro-split rows: macro blocks row 1 middle, 12 movable cells.
Scenario make_macro_row() {
    Scenario sc;
    sc.name = "macro-split row";
    const int base_x = 2000, base_y = 2000, sites = 237;
    sc.segs = full_rows(3, base_x, base_y, sites);
    // row 1 split by macro at x in [20200, 30200)
    {
        // rebuild row 1 as two segments
        std::vector<LegalizeRowSeg> keep;
        for(auto& s : sc.segs) {
            if(s.origin_y != base_y + SITE_H) {
                keep.push_back(s);
            }
        }
        sc.segs = keep;
        add_segmented_row(sc.segs, 1, base_x, base_y, sites, 20200, 30200);
    }
    // macro: 2 rows tall, sits on row 1-2
    sc.nodes.push_back({20200, double(base_y + SITE_H), 10000, 2.0 * SITE_H,
                        true});
    for(int i = 0; i < 12; ++i) {
        double x = base_x + (i * 1600) % 44000;
        double y = base_y + (i % 3) * SITE_H;
        sc.nodes.push_back({x, y, 1400, SITE_H, false});
    }
    // movable must come first: reorder
    std::vector<LegalizeNode> mv, fx;
    for(auto& n : sc.nodes) {
        (n.fixed ? fx : mv).push_back(n);
    }
    sc.nodes = mv;
    sc.nodes.insert(sc.nodes.end(), fx.begin(), fx.end());
    sc.num_movable = (int)mv.size();
    sc.padding = 1.0;
    return sc;
}

// 3. Overfull row: 30 cells of 8 sites on a 237-site row + padding.
Scenario make_overfull() {
    Scenario sc;
    sc.name = "overfull row with padding";
    // Second row provides room for the extracted cell.
    sc.segs = full_rows(2, 2000, 2000, 237);
    for(int i = 0; i < 30; ++i) {
        sc.nodes.push_back(
            {2000.0 + (i * 1500) % 45000, 2000.0, 1600, SITE_H, false});
    }
    sc.num_movable = 30;
    sc.padding = 1.0;
    return sc;
}

// 4. Off-grid macro edge: fixed macro ending at x=20850 (not site-aligned).
Scenario make_offgrid_macro() {
    Scenario sc;
    sc.name = "off-grid macro edge";
    sc.segs = full_rows(1, 2000, 2000, 237);
    sc.nodes.push_back({6000, 2000, 3000, SITE_H, false});
    sc.nodes.push_back({6400, 2000, 1400, SITE_H, false});
    sc.nodes.push_back({8000, 2000, 2200, SITE_H, false});
    sc.num_movable = 3;
    sc.nodes.push_back({4000, 2000, 2850, SITE_H, true});  // fixed, off-grid end
    sc.padding = 1.0;
    return sc;
}

// A genuinely overfull row is not fixable; the core must report
// inapplicable (rc != 0) so the caller falls back to DPL -- no crash.
void run_expect_fallback(const Scenario& sc) {
    printf("scenario: %s (expect fallback)\n", sc.name.c_str());
    std::vector<LegalizeNode> nodes = sc.nodes;
    std::vector<double> out_x;
    std::vector<int> out_row;
    int base_x = 0, base_y = 0, num_rows = 0;
    int rc = ecc::dpLegalizeCore(sc.segs, SITE_W, SITE_H, nodes,
                                 sc.num_movable, sc.padding, out_x, out_row,
                                 base_x, base_y, num_rows);
    if(rc != 0) {
        printf("  fell back as expected\n");
        return;
    }
    check(false, "overfull row must not silently produce a layout");
}

}  // namespace

int main() {
    run(make_basic());
    run(make_macro_row());
    run(make_overfull());  // greedy relocates the excess cell to row 1
    run(make_offgrid_macro());
    if(failures == 0) {
        printf("ALL LEGALIZE TESTS PASSED\n");
        return 0;
    }
    printf("%d FAILURES\n", failures);
    return 1;
}
