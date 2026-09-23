#include "legalize_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "greedy_legalize/src/function_cpu.h"
#include "abacus_legalize/src/abacus_legalize_cpu.h"

namespace ecc {

int dpLegalizeCore(const std::vector<LegalizeRowSeg>& segs, int site_w,
                   int site_h, std::vector<LegalizeNode>& nodes,
                   int num_movable, double padding_sites,
                   std::vector<double>& out_x, std::vector<int>& out_row,
                   int& base_x, int& base_y, int& num_rows_out) {
    if(segs.empty()) {
        printf("dp-legalize: no rows, inapplicable\n");
        return -1;
    }
    base_x = segs[0].origin_x;
    base_y = segs[0].origin_y;
    for(auto& row : segs) {
        base_x = std::min(base_x, row.origin_x);
        base_y = std::min(base_y, row.origin_y);
    }
    int max_right = 0;
    for(auto& row : segs) {
        max_right = std::max(max_right,
                             row.origin_x - base_x + row.site_count * site_w);
    }
    // Segmented rows (macros splitting rows) are fine for the bin-based
    // ops; only require all segments to share one site grid.
    int max_y = base_y;
    for(auto& row : segs) {
        if((row.origin_x - base_x) % site_w != 0 ||
           (row.origin_y - base_y) % site_h != 0) {
            printf("dp-legalize: row segment at (%d,%d) off site grid, "
                   "inapplicable\n",
                   row.origin_x, row.origin_y);
            return -1;
        }
        max_y = std::max(max_y, row.origin_y);
    }
    const int num_rows = (max_y - base_y) / site_h + 1;
    // Every row y must be covered by at least one segment.
    {
        std::vector<char> seen(num_rows, 0);
        for(auto& row : segs) {
            seen[(row.origin_y - base_y) / site_h] = 1;
        }
        for(int r = 0; r < num_rows; ++r) {
            if(!seen[r]) {
                printf("dp-legalize: missing row at y=%d, inapplicable\n",
                       base_y + r * site_h);
                return -1;
            }
        }
    }
    num_rows_out = num_rows;

    std::vector<double> sx, sy, px, py, wts;
    const double core_xh = double(base_x + max_right);
    const double core_yh = double(base_y + num_rows * site_h);
    auto push_node = [&](LegalizeNode& nd) {
        double cx = std::max(double(base_x), std::min(nd.x, core_xh - nd.w));
        double cy = std::max(double(base_y), std::min(nd.y, core_yh - nd.h));
        sx.push_back(nd.w);
        sy.push_back(nd.h);
        px.push_back(cx);
        py.push_back(cy);
        wts.push_back(1.0);
    };
    for(auto& nd : nodes) {
        push_node(nd);
    }
    // Row-segment gaps (macro halos, row cuts) are invisible to the
    // continuous bin model; add each gap as a fake fixed obstacle node so
    // the ops never place cells there.
    {
        std::vector<std::vector<std::pair<int, int>>> seg_ints(num_rows);
        for(auto& row : segs) {
            int rid = (row.origin_y - base_y) / site_h;
            seg_ints[rid].push_back(
                {row.origin_x, row.origin_x + row.site_count * site_w});
        }
        const int core_x0 = base_x;
        const int core_x1 = base_x + max_right;
        for(int r = 0; r < num_rows; ++r) {
            auto& ivs = seg_ints[r];
            std::sort(ivs.begin(), ivs.end());
            int cursor = core_x0;
            for(auto& iv : ivs) {
                if(iv.first > cursor && iv.first - cursor >= site_w) {
                    sx.push_back(iv.first - cursor);
                    sy.push_back(site_h);
                    px.push_back(cursor);
                    py.push_back(base_y + r * site_h);
                    wts.push_back(1.0);
                }
                cursor = std::max(cursor, iv.second);
            }
            if(core_x1 - cursor >= site_w) {
                sx.push_back(core_x1 - cursor);
                sy.push_back(site_h);
                px.push_back(cursor);
                py.push_back(base_y + r * site_h);
                wts.push_back(1.0);
            }
        }
    }
    const int num_nodes = (int)sx.size();
    if(num_movable == 0) {
        return 0;
    }

    std::vector<double> x(px), y(py);
    DreamPlace::LegalizationDB<double> db;
    memset(&db, 0, sizeof(db));
    db.init_x = px.data();
    db.init_y = py.data();
    db.node_size_x = sx.data();
    db.node_size_y = sy.data();
    db.node_weights = wts.data();
    db.x = x.data();
    db.y = y.data();
    db.xl = base_x;
    db.yl = base_y;
    db.xh = core_xh;
    db.yh = core_yh;
    db.site_width = site_w;
    db.row_height = site_h;
    db.bin_size_x = db.xh - db.xl;
    db.bin_size_y = site_h;
    db.num_bins_x = 1;
    db.num_bins_y = num_rows;
    db.num_sites_x = max_right / site_w;
    db.num_sites_y = num_rows;
    db.num_nodes = num_nodes;
    db.num_movable_nodes = num_movable;
    db.num_regions = 0;

    DreamPlace::greedyLegalizationCPU(
        db, px.data(), py.data(), sx.data(), sy.data(), x.data(), y.data(),
        db.xl, db.yl, db.xh, db.yh, db.site_width, db.row_height,
        db.num_bins_x, db.num_bins_y, num_nodes, num_movable);

    // Guard: a genuinely overfull row (movable + fixed width beyond row
    // capacity) would trip the vendored abacus' internal assert. Detect and
    // let the caller fall back to DPL instead of moving cells around.
    {
        std::vector<double> row_mov(num_rows, 0.0), row_fix(num_rows, 0.0);
        auto row_of = [&](double yy) {
            return std::max(
                0, std::min(num_rows - 1,
                            (int)llround((yy - base_y) / site_h)));
        };
        for(int i = 0; i < num_movable; ++i) {
            row_mov[row_of(y[i])] += sx[i];
        }
        for(int j = num_movable; j < num_nodes; ++j) {
            int rid_lo =
                std::max(0, (int)floor((py[j] - base_y) / site_h));
            int rid_hi = std::min(
                num_rows - 1,
                (int)floor((py[j] + sy[j] - 1 - base_y) / site_h));
            rid_lo = std::min(rid_lo, num_rows - 1);
            for(int rid = rid_lo; rid <= rid_hi; ++rid) {
                row_fix[rid] += sx[j];
            }
        }
        const double row_cap = double(max_right);
        for(int r = 0; r < num_rows; ++r) {
            if(row_mov[r] + row_fix[r] > row_cap) {
                printf("dp-legalize: row %d overfull (mov %.0f + fix %.0f > "
                       "%.0f), inapplicable\n",
                       r, row_mov[r], row_fix[r], row_cap);
                return -1;
            }
        }
    }

    DreamPlace::abacusLegalizationCPU(
        px.data(), py.data(), sx.data(), sy.data(), wts.data(), x.data(),
        y.data(), db.xl, db.yl, db.xh, db.yh, db.site_width, db.row_height,
        db.num_bins_x, db.num_bins_y, num_nodes, num_movable);


    const double pad_dbu = padding_sites * site_w;
    const std::vector<double> opx = x;
    // Post-hoc placement padding: per row, shift cells right to open
    // dp_padding-site gaps (fixed insts are obstacles). Rows that cannot
    // fit degrade to gap 0, keeping the ops' feasible positions.
    if(pad_dbu > 0) {
        std::vector<std::vector<int>> row_cells(num_rows);
        std::vector<std::vector<std::pair<double, double>>> fxd(num_rows);
        for(int i = 0; i < num_movable; ++i) {
            int rid = std::max(
                0, std::min(num_rows - 1,
                            (int)llround((y[i] - base_y) / site_h)));
            row_cells[rid].push_back(i);
        }
        for(int j = num_movable; j < num_nodes; ++j) {
            // Multi-row-height fixed insts (macros) block every row their
            // footprint covers, not just the origin row.
            int rid_lo = (int)floor((py[j] - base_y) / site_h);
            int rid_hi = (int)floor((py[j] + sy[j] - 1 - base_y) / site_h);
            rid_lo = std::max(0, std::min(num_rows - 1, rid_lo));
            rid_hi = std::max(0, std::min(num_rows - 1, rid_hi));
            for(int rid = rid_lo; rid <= rid_hi; ++rid) {
                fxd[rid].push_back({px[j], px[j] + sx[j]});
            }
        }
        int degraded_rows = 0;
        for(int r = 0; r < num_rows; ++r) {
            auto& cids = row_cells[r];
            auto& fiv = fxd[r];
            if(cids.empty()) {
                continue;
            }
            std::sort(cids.begin(), cids.end(),
                      [&](int a, int b) { return x[a] < x[b]; });
            std::sort(fiv.begin(), fiv.end());
            for(int attempt = 0; attempt < 2; ++attempt) {
                const double gap = attempt == 0 ? pad_dbu : 0.0;
                double prev_end = base_x;
                size_t fi = 0;
                bool overflow = false;
                for(int ci : cids) {
                    const double w = sx[ci];
                    double cx = std::max(attempt == 0 ? x[ci] : opx[ci],
                                         prev_end);
                    while(fi < fiv.size() && fiv[fi].second <= cx) {
                        ++fi;
                    }
                    while(fi < fiv.size() && cx + w > fiv[fi].first) {
                        cx = fiv[fi].second;
                        ++fi;
                        while(fi < fiv.size() && fiv[fi].second <= cx) {
                            ++fi;
                        }
                    }
                    if(cx + w > core_xh + 1e-9) {
                        overflow = true;
                        break;
                    }
                    x[ci] = cx;
                    prev_end = cx + w + gap;
                }
                if(!overflow) {
                    if(attempt > 0) {
                        ++degraded_rows;
                    }
                    break;
                }
            }
        }
        if(degraded_rows > 0) {
            printf("dp-legalize: %d rows degraded to zero padding\n",
                   degraded_rows);
        }
    }


    out_x.resize(num_movable);
    out_row.resize(num_movable);
    for(int i = 0; i < num_movable; ++i) {
        int rid = (int)llround((y[i] - base_y) / site_h);
        out_row[i] = std::max(0, std::min(num_rows - 1, rid));
        out_x[i] = llround(x[i]);
    }
    return 0;
}

}  // namespace ecc
