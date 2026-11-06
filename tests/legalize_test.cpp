// Unit tests for the standalone legalization core (dpLegalizeCore).
// Scenario factory + legality assertions: no overlaps, on-row, on-site,
// padding gaps where feasible.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
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
    double padding = 0.0;  // best-effort inter-cell gap in sites
    bool require_padding = false;
    bool expect_padding_relaxation = false;
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

void check_nodes_unchanged(const std::vector<LegalizeNode>& before,
                           const std::vector<LegalizeNode>& after) {
    check(before.size() == after.size(), "node count must not change");
    auto same = [](double a, double b) {
        return a == b || (std::isnan(a) && std::isnan(b));
    };
    for(size_t i = 0; i < std::min(before.size(), after.size()); ++i) {
        check(same(before[i].x, after[i].x) && same(before[i].y, after[i].y) &&
                  same(before[i].w, after[i].w) && same(before[i].h, after[i].h) &&
                  before[i].fixed == after[i].fixed,
              "input movable/fixed geometry must remain unchanged");
    }
}

void verify(const Scenario& sc, const std::vector<LegalizeNode>& nodes,
            int num_movable, const std::vector<double>& out_x,
            const std::vector<int>& out_row, int base_x, int base_y,
            int num_rows) {
    printf("  verifying %zu movable cells\n", out_x.size());
    check(out_x.size() == (size_t)num_movable &&
              out_row.size() == (size_t)num_movable,
          "result must include every movable");
    if(out_x.size() != (size_t)num_movable ||
       out_row.size() != (size_t)num_movable) {
        return;
    }
    int min_x = sc.segs[0].origin_x, min_y = sc.segs[0].origin_y;
    int max_y = min_y;
    int max_x = sc.segs[0].origin_x + sc.segs[0].site_count * SITE_W;
    for(auto& s : sc.segs) {
        max_x = std::max(max_x, s.origin_x + s.site_count * SITE_W);
        min_x = std::min(min_x, s.origin_x);
        min_y = std::min(min_y, s.origin_y);
        max_y = std::max(max_y, s.origin_y);
    }
    check(base_x == min_x && base_y == min_y &&
              num_rows == (max_y - min_y) / SITE_H + 1,
          "physical row grid must not be compacted or rebased");
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
        if(!std::isfinite(out_x[i]) || out_x[i] < min_x ||
           out_x[i] > max_x) {
            check(false, "invalid candidate x");
            return;
        }
        int rx = (int)llround(out_x[i]);
        if(std::fabs(out_x[i] - rx) > 1e-9 ||
           (rx - base_x) % SITE_W != 0) {
            ++offgrid;
        }
        if(rx < base_x || rx + nodes[i].w > max_x + 1e-6) {
            ++offgrid;  // out of core horizontally
        }
        if(out_row[i] < 0 || out_row[i] >= num_rows) {
            check(false, "invalid candidate row index");
            return;
        }
        const int y = base_y + out_row[i] * SITE_H;
        check(std::any_of(sc.segs.begin(), sc.segs.end(),
                          [&](const LegalizeRowSeg& seg) {
                              return y == seg.origin_y && rx >= seg.origin_x &&
                                     rx + nodes[i].w <=
                                         seg.origin_x + seg.site_count * SITE_W;
                          }),
              "cell must fit in one real ROW segment, never a missing row");
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
        if(sc.require_padding) {
            check(pad_viol == 0, "padding must be preserved when feasible");
        }
        if(sc.expect_padding_relaxation) {
            check(pad_viol > 0, "full row must relax extra padding");
        }
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
    check_nodes_unchanged(sc.nodes, nodes);
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
    sc.require_padding = true;
    return sc;
}

Scenario make_two_site_padding() {
    Scenario sc = make_basic();
    sc.name = "basic two-site padding";
    sc.padding = 2.0;
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

// 3. Fixed macro is completely covered by a segmented-row gap. The obstacle
// union fits, while summing the macro and gap independently would not.
Scenario make_macro_inside_gap() {
    Scenario sc;
    sc.name = "macro inside row gap";
    const int base_x = 2000, base_y = 2000;
    sc.segs = {{base_x, base_y, 15},
               {base_x + 35 * SITE_W, base_y, 15}};
    sc.nodes.push_back({base_x + 15 * SITE_W, double(base_y),
                        20.0 * SITE_W, double(SITE_H), true});
    for(int i = 0; i < 10; ++i) {
        sc.nodes.push_back({double(base_x + (i % 5) * 400),
                            double(base_y), 2.0 * SITE_W, double(SITE_H),
                            false});
    }
    std::vector<LegalizeNode> mv, fx;
    for(auto& n : sc.nodes) {
        (n.fixed ? fx : mv).push_back(n);
    }
    sc.nodes = mv;
    sc.nodes.insert(sc.nodes.end(), fx.begin(), fx.end());
    sc.num_movable = (int)mv.size();
    sc.padding = 0.0;
    return sc;
}

Scenario make_macro_inside_gap_with_padding() {
    Scenario sc = make_macro_inside_gap();
    sc.name = "macro inside row gap with padding";
    sc.padding = 1.0;
    sc.require_padding = true;
    return sc;
}

// Physical cells fit exactly; extra gaps must not create overlaps or overflow.
Scenario make_padding_relaxation() {
    Scenario sc;
    sc.name = "full row relaxes extra padding";
    sc.segs = full_rows(1, 2000, 2000, 10);
    for(int i = 0; i < 5; ++i) {
        sc.nodes.push_back({2000.0 + i * 400, 2000.0, 400, SITE_H,
                            false});
    }
    sc.num_movable = 5;
    sc.padding = 2.0;
    sc.expect_padding_relaxation = true;
    return sc;
}

// A hold repair can add many small movable buffers to an already dense
// placement. The physical cells still fit, while best-effort padding must
// relax without creating overlaps.
Scenario make_hold_buffer_burst() {
    Scenario sc;
    sc.name = "hold buffer burst with padding relaxation";
    sc.segs = full_rows(4, 2000, 2000, 40);
    for(int i = 0; i < 60; ++i) {
        sc.nodes.push_back({2000.0 + (i / 4) * 400,
                            2000.0 + (i % 4) * SITE_H, 400, SITE_H,
                            false});
    }
    // Model 20 newly inserted hold buffers after the original 60 cells.
    for(int i = 0; i < 20; ++i) {
        sc.nodes.push_back({2000.0 + (i / 4) * 400,
                            2000.0 + (i % 4) * SITE_H, 400, SITE_H,
                            false});
    }
    sc.num_movable = (int)sc.nodes.size();
    sc.padding = 2.0;
    sc.expect_padding_relaxation = true;
    return sc;
}

// 4. Overfull row: 30 cells of 8 sites on a 237-site row + padding.
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

// 5. A genuinely overfull single row must still fall back.
Scenario make_true_overfull() {
    Scenario sc;
    sc.name = "genuinely overfull row";
    sc.segs = full_rows(1, 2000, 2000, 20);
    for(int i = 0; i < 6; ++i) {
        sc.nodes.push_back({2000.0 + i * 400, 2000.0, 800, SITE_H,
                            false});
    }
    sc.num_movable = 6;
    sc.padding = 0.0;
    return sc;
}

// 6. Off-grid macro edge: fixed macro ending at x=20850 (not site-aligned).
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

Scenario make_out_of_core_fixed_obstacles() {
    Scenario sc;
    sc.name = "out-of-core fixed obstacles";
    sc.segs = full_rows(3, 2000, 2000, 237);
    for(int r = 0; r < 3; ++r) {
        for(int c = 0; c < 2; ++c) {
            sc.nodes.push_back({2000.0 + c * 800,
                                2000.0 + r * SITE_H, 400, SITE_H, false});
        }
    }
    sc.num_movable = (int)sc.nodes.size();
    // I/O blocks outside the standard-cell rows must not be clamped onto the
    // boundary rows and fabricated into obstacles.
    sc.nodes.push_back({2000, -10000, 20000, 5000, true});
    sc.nodes.push_back({-10000, 3400, 5000, SITE_H, true});
    sc.nodes.push_back({2000, 2000.0 + 3 * SITE_H, 20000, 5000, true});
    return sc;
}

Scenario make_padding_threshold_identity() {
    Scenario sc;
    sc.name = "80 percent adjacent padding";
    sc.segs = full_rows(1, 2000, 2000, 40);
    const double xs[] = {2000, 2600, 3200, 3800, 4400, 4800};
    for(double x : xs) {
        sc.nodes.push_back({x, 2000, 400, SITE_H, false});
    }
    sc.num_movable = (int)sc.nodes.size();
    sc.padding = 1.0;
    return sc;
}

Scenario make_padding_below_threshold() {
    Scenario sc;
    sc.name = "below 80 percent adjacent padding";
    sc.segs = full_rows(1, 2000, 2000, 40);
    const double xs[] = {2000, 2600, 3200, 3800, 4200, 4600};
    for(double x : xs) {
        sc.nodes.push_back({x, 2000, 400, SITE_H, false});
    }
    sc.num_movable = (int)sc.nodes.size();
    sc.padding = 1.0;
    sc.require_padding = true;
    return sc;
}

Scenario make_sparse_rows(int missing, bool legal_input) {
    Scenario sc;
    sc.name = std::to_string(missing) + " missing rows, " +
              (legal_input ? "legal input unchanged" : "gap cells and overlap");
    const int base_x = 2000, base_y = 2300;
    const int real_rows[] = {0, 1, missing + 2, missing + 3};
    for(int r : real_rows) {
        add_segmented_row(sc.segs, r, base_x, base_y, 40, 5200, 6800);
        sc.nodes.push_back({double(base_x), double(base_y + r * SITE_H),
                            400, SITE_H, false});
        sc.nodes.push_back({double(base_x + 800),
                            double(base_y + r * SITE_H), 400, SITE_H, false});
    }
    sc.num_movable = (int)sc.nodes.size();
    sc.padding = 1;
    sc.require_padding = true;
    if(!legal_input) {
        sc.nodes[1].x = sc.nodes[0].x;
        sc.nodes[2].y = base_y + 2.0 * SITE_H;
        sc.nodes[3].y = base_y + (missing / 2 + 2.0) * SITE_H;
        sc.nodes[4].x = 5600;  // also inside an x gap on a real row
        sc.nodes[5].x = 5600;
    }
    // The adapter's ROW iteration order is not necessarily sorted.
    std::reverse(sc.segs.begin(), sc.segs.end());
    return sc;
}

Scenario make_sparse_obstacles() {
    Scenario sc = make_sparse_rows(205, false);
    sc.name = "205 missing rows with overlapping and off-grid obstacles";
    sc.nodes.push_back({3050, 2300.0 + SITE_H / 2.0, 800,
                        207.0 * SITE_H, true});
    sc.nodes.push_back({3400, 2300.0 + SITE_H, 1200,
                        206.0 * SITE_H, true});
    sc.nodes.push_back({2000, -10000, 20000, 5000, true});
    sc.nodes.push_back({-10000, 2300, 5000, SITE_H, true});
    return sc;
}

Scenario make_sparse_overfull(bool fragmented) {
    Scenario sc;
    sc.name = fragmented ? "sparse fragmented capacity" : "sparse insufficient capacity";
    sc.segs = {{2000, 2300, 2}, {3200, 2300, 2},
               {2000, 2300 + 206 * SITE_H, 2},
               {3200, 2300 + 206 * SITE_H, 2}};
    if(fragmented) {
        // Total free width is 1600, but no segment can fit this 600-wide cell.
        sc.nodes.push_back({2200, 2300, 600, SITE_H, false});
    }
    else {
        for(int i = 0; i < 5; ++i) {
            sc.nodes.push_back({2000, 2300, 400, SITE_H, false});
        }
    }
    sc.num_movable = (int)sc.nodes.size();
    return sc;
}

Scenario make_fractional_obstacle() {
    Scenario sc;
    sc.name = "sub-DBU obstacle crossing row boundary";
    sc.segs = full_rows(2, 2000, 2000, 40);
    sc.nodes = {{2000, 2000, 400, SITE_H, false},
                {2000, 3400, 400, SITE_H, false},
                {2050, 3399.75, 100, 0.5, true}};
    sc.num_movable = 2;
    return sc;
}

Scenario make_straddling_segments() {
    Scenario sc;
    sc.name = "candidate straddles adjacent ROW segments";
    sc.segs = {{2000, 2300, 2}, {2400, 2300, 2},
               {2000, 2300 + 2 * SITE_H, 2},
               {2400, 2300 + 2 * SITE_H, 2}};
    sc.nodes = {{2100, 2300, 600, SITE_H, false}};
    sc.num_movable = 1;
    return sc;
}

void run_expect_identity(const Scenario& sc) {
    printf("scenario: %s\n", sc.name.c_str());
    std::vector<LegalizeNode> nodes = sc.nodes;
    std::vector<double> out_x;
    std::vector<int> out_row;
    int base_x = 0, base_y = 0, num_rows = 0;
    int rc = ecc::dpLegalizeCore(sc.segs, SITE_W, SITE_H, nodes,
                                 sc.num_movable, sc.padding, out_x, out_row,
                                 base_x, base_y, num_rows);
    check_nodes_unchanged(sc.nodes, nodes);
    check(rc == 0, "core should handle an already-legal placement");
    if(rc != 0) {
        return;
    }
    verify(sc, nodes, sc.num_movable, out_x, out_row, base_x, base_y,
           num_rows);
    for(int i = 0; i < sc.num_movable; ++i) {
        check(out_x[i] == nodes[i].x, "legal cell x must be preserved");
        check(out_row[i] ==
                  (int)std::llround((nodes[i].y - base_y) / SITE_H),
              "legal cell row must be preserved");
    }
}

// Fail without publishing partial results. Only dense rows may use DPL.
void run_expect_failure(const Scenario& sc, int site_w = SITE_W,
                         int site_h = SITE_H) {
    printf("scenario: %s (expect failure)\n", sc.name.c_str());
    std::vector<LegalizeNode> nodes = sc.nodes;
    std::vector<double> out_x = {123.5, 456.5};
    std::vector<int> out_row = {-7, 99};
    int base_x = -12, base_y = -34, num_rows = -56;
    int rc = ecc::dpLegalizeCore(sc.segs, site_w, site_h, nodes,
                                 sc.num_movable, sc.padding, out_x, out_row,
                                 base_x, base_y, num_rows);
    check(rc == -1, "illegal/unsupported placement must fail cleanly");
    check(out_x == std::vector<double>({123.5, 456.5}) &&
              out_row == std::vector<int>({-7, 99}) &&
              base_x == -12 && base_y == -34 && num_rows == -56,
          "failure must leave every output unchanged");
    check_nodes_unchanged(sc.nodes, nodes);
}

void run_malformed_inputs() {
    const Scenario valid = make_sparse_rows(1, true);
    run_expect_failure(valid, 0, SITE_H);
    run_expect_failure(valid, SITE_W, 0);
    run_expect_failure(valid, -SITE_W, SITE_H);
    run_expect_failure(valid, SITE_W, -SITE_H);
    for(int which = 0; which < 21; ++which) {
        Scenario sc = valid;
        sc.name = "malformed input " + std::to_string(which);
        switch(which) {
        case 0: sc.segs.clear(); break;
        case 1: sc.segs[0].origin_y += 1; break;
        case 2: sc.segs[0].origin_x += 1; break;
        case 3: sc.segs[0].site_count = 0; break;
        case 4: sc.segs[0].site_count = -1; break;
        case 5: sc.segs[0].site_count = std::numeric_limits<int>::max(); break;
        case 6: sc.segs[0].origin_y = std::numeric_limits<int>::max(); break;
        case 7: sc.num_movable = -1; break;
        case 8: sc.num_movable = (int)sc.nodes.size() + 1; break;
        case 9: sc.nodes[0].x = std::numeric_limits<double>::quiet_NaN(); break;
        case 10: sc.nodes[0].y = std::numeric_limits<double>::infinity(); break;
        case 11: sc.nodes[0].w = 0; break;
        case 12: sc.nodes[0].h = -1; break;
        case 13: sc.nodes[0].w += 1; break;
        case 14: sc.nodes[0].h *= 2; break;
        case 15: sc.nodes[0].fixed = true; break;
        case 16: sc.padding = -1; break;
        case 17: sc.padding = std::numeric_limits<double>::infinity(); break;
        case 18: sc.padding = std::numeric_limits<double>::quiet_NaN(); break;
        case 19: sc.nodes[0].x = 1e30; break;
        case 20:
            sc.nodes.push_back({2000, 2300, 400,
                                std::numeric_limits<double>::quiet_NaN(), true});
            break;
        }
        run_expect_failure(sc);
    }
}

}  // namespace

int main() {
    run(make_basic());
    run(make_two_site_padding());
    run(make_macro_row());
    run(make_macro_inside_gap());
    run(make_macro_inside_gap_with_padding());
    run(make_padding_relaxation());
    run(make_hold_buffer_burst());
    run(make_overfull());  // greedy relocates the excess cell to row 1
    run_expect_failure(make_true_overfull());
    run(make_offgrid_macro());
    run_expect_identity(make_out_of_core_fixed_obstacles());
    run_expect_identity(make_padding_threshold_identity());
    run(make_padding_below_threshold());
    for(int missing : {1, 205}) {
        run_expect_identity(make_sparse_rows(missing, true));
        run(make_sparse_rows(missing, false));
    }
    run(make_sparse_obstacles());
    run_expect_failure(make_sparse_overfull(false));
    run_expect_failure(make_sparse_overfull(true));
    run_expect_failure(make_straddling_segments());
    run(make_fractional_obstacle());
    Scenario offgrid = make_sparse_rows(205, false);
    offgrid.name = "off-grid movable coordinates with fractional padding";
    offgrid.nodes[0].x += 50.5;
    offgrid.nodes[0].y += 700.25;
    offgrid.nodes[1].x = -400;
    offgrid.nodes[1].y = -800;
    offgrid.padding = 1.5;
    run(offgrid);
    Scenario empty = make_sparse_rows(205, true);
    empty.name = "sparse rows without movable cells";
    empty.nodes.clear();
    empty.num_movable = 0;
    run_expect_identity(empty);
    Scenario negative = make_sparse_rows(205, true);
    negative.name = "sparse grid with negative origins remains unchanged";
    for(auto& seg : negative.segs) {
        seg.origin_x -= 20000;
        seg.origin_y -= 500000;
    }
    for(auto& node : negative.nodes) {
        node.x -= 20000;
        node.y -= 500000;
    }
    run_expect_identity(negative);
    run_malformed_inputs();
    if(failures == 0) {
        printf("ALL LEGALIZE TESTS PASSED\n");
        return 0;
    }
    printf("%d FAILURES\n", failures);
    return 1;
}
