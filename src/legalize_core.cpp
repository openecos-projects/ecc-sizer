#include "legalize_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

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

    const double core_xh = double(base_x + max_right);
    const double core_yh = double(base_y + num_rows * site_h);
    using Interval = std::pair<double, double>;
    std::vector<std::vector<Interval>> row_obstacles(num_rows);
    auto add_obstacle = [&](int rid_lo, int rid_hi, double x0, double x1) {
        if(rid_hi < 0 || rid_lo >= num_rows) {
            return;
        }
        x0 = std::max(x0, double(base_x));
        x1 = std::min(x1, core_xh);
        if(x1 <= x0) {
            return;
        }
        rid_lo = std::max(0, std::min(num_rows - 1, rid_lo));
        rid_hi = std::max(0, std::min(num_rows - 1, rid_hi));
        for(int rid = rid_lo; rid <= rid_hi; ++rid) {
            row_obstacles[rid].push_back({x0, x1});
        }
    };
    auto clamp_node_x = [&](const LegalizeNode& nd) {
        return std::max(double(base_x), std::min(nd.x, core_xh - nd.w));
    };
    auto clamp_node_y = [&](const LegalizeNode& nd) {
        return std::max(double(base_y), std::min(nd.y, core_yh - nd.h));
    };

    // Fixed instances and row gaps are both obstacles. Build their union per
    // row before handing the geometry to DreamPlace so no fixed nodes overlap.
    for(int j = num_movable; j < (int)nodes.size(); ++j) {
        const LegalizeNode& nd = nodes[j];
        // Fixed I/O pads and the pad ring can sit outside the standard-cell
        // row area. Clamping them into the core fabric creates fictitious
        // obstacles along the boundary rows and cascades cells across the
        // design. Preserve their physical coordinates and discard only those
        // that do not intersect the row area.
        const int rid_lo = (int)floor((nd.y - base_y) / site_h);
        const int rid_hi =
            (int)floor((nd.y + nd.h - 1 - base_y) / site_h);
        add_obstacle(rid_lo, rid_hi, nd.x, nd.x + nd.w);
    }

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
                add_obstacle(r, r, cursor, iv.first);
            }
            cursor = std::max(cursor, iv.second);
        }
        if(core_x1 - cursor >= site_w) {
            add_obstacle(r, r, cursor, core_x1);
        }
    }

    std::vector<double> row_obstacle_width(num_rows, 0.0);
    for(int r = 0; r < num_rows; ++r) {
        auto& obstacles = row_obstacles[r];
        std::sort(obstacles.begin(), obstacles.end(),
                  [](const Interval& lhs, const Interval& rhs) {
                      return lhs.first < rhs.first ||
                             (lhs.first == rhs.first &&
                              lhs.second < rhs.second);
                  });
        std::vector<Interval> merged;
        for(const Interval& obstacle : obstacles) {
            if(merged.empty() || obstacle.first > merged.back().second) {
                merged.push_back(obstacle);
            }
            else {
                merged.back().second =
                    std::max(merged.back().second, obstacle.second);
            }
        }
        obstacles.swap(merged);
        for(const Interval& obstacle : obstacles) {
            row_obstacle_width[r] += obstacle.second - obstacle.first;
        }
    }

    const double pad_dbu = padding_sites * site_w;

    // Preserve an already-legal placement when its padding compliance is
    // sufficient. Site/row legality and non-overlap are mandatory; padding is
    // allowed to be best effort, matching the legalizer's padding behavior.
    constexpr int kPaddingComplianceNumerator = 4;
    constexpr int kPaddingComplianceDenominator = 5;
    bool input_legal = true;
    std::vector<std::vector<int>> input_row_cells(num_rows);
    for(int i = 0; i < num_movable && input_legal; ++i) {
        const LegalizeNode& nd = nodes[i];
        const long long x = std::llround(nd.x);
        const long long y = std::llround(nd.y);
        const long long w = std::llround(nd.w);
        if(std::fabs(nd.x - double(x)) > 1e-9 ||
           std::fabs(nd.y - double(y)) > 1e-9 ||
           std::fabs(nd.w - double(w)) > 1e-9 ||
           std::fabs(nd.h - double(site_h)) > 1e-9 || w <= 0 ||
           (x - base_x) % site_w != 0 || w % site_w != 0 ||
           (y - base_y) % site_h != 0) {
            input_legal = false;
            break;
        }
        const long long rid_ll = (y - base_y) / site_h;
        if(rid_ll < 0 || rid_ll >= num_rows) {
            input_legal = false;
            break;
        }
        const int rid = (int)rid_ll;
        const auto& segs_at_y = seg_ints[rid];
        auto seg_it = std::upper_bound(
            segs_at_y.begin(), segs_at_y.end(), x,
            [](long long value, const std::pair<int, int>& seg) {
                return value < seg.first;
            });
        if(seg_it == segs_at_y.begin()) {
            input_legal = false;
            break;
        }
        --seg_it;
        if(x < seg_it->first || x + w > seg_it->second) {
            input_legal = false;
            break;
        }
        input_row_cells[rid].push_back(i);
    }
    for(int r = 0; r < num_rows && input_legal; ++r) {
        auto& cell_ids = input_row_cells[r];
        std::sort(cell_ids.begin(), cell_ids.end(), [&](int lhs, int rhs) {
            return nodes[lhs].x < nodes[rhs].x;
        });
        const auto& obstacles = row_obstacles[r];
        size_t obstacle_idx = 0;
        double previous_end = 0.0;
        bool has_previous = false;
        for(int cell_id : cell_ids) {
            const double x0 = nodes[cell_id].x;
            const double x1 = x0 + nodes[cell_id].w;
            while(obstacle_idx < obstacles.size() &&
                  obstacles[obstacle_idx].second <= x0 + 1e-9) {
                ++obstacle_idx;
            }
            if(obstacle_idx < obstacles.size() &&
               obstacles[obstacle_idx].first < x1 - 1e-9) {
                input_legal = false;
                break;
            }
            if(has_previous && x0 < previous_end - 1e-9) {
                input_legal = false;
                break;
            }
            previous_end = x1;
            has_previous = true;
        }
    }

    int padding_pair_count = 0;
    int padding_pair_compliant = 0;
    if(input_legal) {
        struct RowInterval {
            double x0;
            double x1;
            bool movable;
        };
        std::vector<std::vector<RowInterval>> row_intervals(num_rows);
        for(int r = 0; r < num_rows; ++r) {
            for(int cell_id : input_row_cells[r]) {
                row_intervals[r].push_back(
                    {nodes[cell_id].x,
                     nodes[cell_id].x + nodes[cell_id].w, true});
            }
        }
        for(int j = num_movable; j < (int)nodes.size(); ++j) {
            const LegalizeNode& nd = nodes[j];
            const double x0 = std::max(double(base_x), nd.x);
            const double x1 =
                std::min(core_xh, nd.x + nd.w);
            if(x1 <= x0) {
                continue;
            }
            const int rid_lo =
                (int)floor((nd.y - base_y) / site_h);
            const int rid_hi =
                (int)floor((nd.y + nd.h - 1 - base_y) / site_h);
            for(int rid = std::max(0, rid_lo);
                rid <= std::min(num_rows - 1, rid_hi); ++rid) {
                row_intervals[rid].push_back({x0, x1, false});
            }
        }
        for(auto& intervals : row_intervals) {
            std::sort(intervals.begin(), intervals.end(),
                      [](const RowInterval& lhs, const RowInterval& rhs) {
                          return lhs.x0 < rhs.x0 ||
                                 (lhs.x0 == rhs.x0 &&
                                  lhs.x1 < rhs.x1);
                      });
            std::vector<RowInterval> merged;
            for(const RowInterval& interval : intervals) {
                if(merged.empty() ||
                   interval.x0 >= merged.back().x1 - 1e-9) {
                    merged.push_back(interval);
                }
                else {
                    merged.back().x1 =
                        std::max(merged.back().x1, interval.x1);
                    merged.back().movable =
                        merged.back().movable || interval.movable;
                }
            }
            for(size_t i = 1; i < merged.size(); ++i) {
                if(!merged[i - 1].movable && !merged[i].movable) {
                    continue;
                }
                ++padding_pair_count;
                const double gap = merged[i].x0 - merged[i - 1].x1;
                if(gap + 1e-9 >= pad_dbu) {
                    ++padding_pair_compliant;
                }
            }
        }
    }
    const bool padding_compliant =
        padding_pair_count == 0 ||
        padding_pair_compliant * kPaddingComplianceDenominator >=
            padding_pair_count * kPaddingComplianceNumerator;
    if(input_legal && padding_compliant) {
        out_x.resize(num_movable);
        out_row.resize(num_movable);
        for(int i = 0; i < num_movable; ++i) {
            out_x[i] = nodes[i].x;
            out_row[i] = (int)llround((nodes[i].y - base_y) / site_h);
        }
        printf("dp-legalize: input already legal, preserving placement "
               "(padding %d/%d)\n",
               padding_pair_compliant, padding_pair_count);
        return 0;
    }
    if(input_legal) {
        printf("dp-legalize: input padding compliance %d/%d is below %d%%\n",
               padding_pair_compliant, padding_pair_count,
               kPaddingComplianceNumerator * 100 /
                   kPaddingComplianceDenominator);
    }

    std::vector<double> sx, sy, px, py, wts;
    auto push_node = [&](const LegalizeNode& nd) {
        double cx = clamp_node_x(nd);
        double cy = clamp_node_y(nd);
        sx.push_back(nd.w);
        sy.push_back(nd.h);
        px.push_back(cx);
        py.push_back(cy);
        wts.push_back(1.0);
    };
    for(int i = 0; i < num_movable; ++i) {
        push_node(nodes[i]);
    }
    for(int r = 0; r < num_rows; ++r) {
        for(const Interval& obstacle : row_obstacles[r]) {
            sx.push_back(obstacle.second - obstacle.first);
            sy.push_back(site_h);
            px.push_back(obstacle.first);
            py.push_back(base_y + r * site_h);
            wts.push_back(1.0);
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
        std::vector<double> row_mov(num_rows, 0.0);
        auto row_of = [&](double yy) {
            return std::max(
                0, std::min(num_rows - 1,
                            (int)llround((yy - base_y) / site_h)));
        };
        for(int i = 0; i < num_movable; ++i) {
            row_mov[row_of(y[i])] += sx[i];
        }
        const double row_cap = double(max_right);
        for(int r = 0; r < num_rows; ++r) {
            if(row_mov[r] + row_obstacle_width[r] > row_cap) {
                printf("dp-legalize: row %d overfull (mov %.0f + fix %.0f > "
                       "%.0f), inapplicable\n",
                       r, row_mov[r], row_obstacle_width[r], row_cap);
                return -1;
            }
        }
    }

    DreamPlace::abacusLegalizationCPU(
        px.data(), py.data(), sx.data(), sy.data(), wts.data(), x.data(),
        y.data(), db.xl, db.yl, db.xh, db.yh, db.site_width, db.row_height,
        db.num_bins_x, db.num_bins_y, num_nodes, num_movable);


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
