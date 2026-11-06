#include "legalize_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <utility>

#include "greedy_legalize/src/function_cpu.h"
#include "abacus_legalize/src/abacus_legalize_cpu.h"

namespace ecc {

int dpLegalizeCore(const std::vector<LegalizeRowSeg>& segs, int site_w,
                   int site_h, std::vector<LegalizeNode>& nodes,
                   int num_movable, double padding_sites,
                   std::vector<double>& out_x, std::vector<int>& out_row,
                   int& base_x_out, int& base_y_out, int& num_rows_out) {
    const int int_max = std::numeric_limits<int>::max();
    const int int_min = std::numeric_limits<int>::min();
    if(segs.empty() || site_w <= 0 || site_h <= 0 ||
       num_movable < 0 || nodes.size() > (size_t)int_max ||
       (size_t)num_movable > nodes.size() ||
       !std::isfinite(padding_sites) || padding_sites < 0 ||
       padding_sites > double(int_max) / site_w) {
        printf("dp-legalize: invalid rows, site size, node count or padding\n");
        return -1;
    }
    int base_x = segs[0].origin_x;
    int base_y = segs[0].origin_y;
    for(auto& row : segs) {
        base_x = std::min(base_x, row.origin_x);
        base_y = std::min(base_y, row.origin_y);
    }
    long long max_right_ll = 0;
    // Segmented rows (macros splitting rows) are fine for the bin-based
    // ops; only require all segments to share one site grid.
    int max_y = base_y;
    for(auto& row : segs) {
        const long long right =
            (long long)row.origin_x + (long long)row.site_count * site_w;
        if(row.site_count <= 0 || right > int_max ||
           (long long)row.origin_y + site_h > int_max ||
           ((long long)row.origin_x - base_x) % site_w != 0 ||
           ((long long)row.origin_y - base_y) % site_h != 0) {
            printf("dp-legalize: row segment at (%d,%d) off site grid, "
                   "empty or out of range, inapplicable\n",
                   row.origin_x, row.origin_y);
            return -1;
        }
        max_right_ll = std::max(max_right_ll, right - base_x);
        max_y = std::max(max_y, row.origin_y);
    }
    const long long height = (long long)max_y - base_y + site_h;
    if(max_right_ll > int_max || height > int_max) {
        printf("dp-legalize: row grid exceeds supported coordinate range\n");
        return -1;
    }
    const int max_right = (int)max_right_ll;
    const int num_rows = (int)(height / site_h);

    for(size_t i = 0; i < nodes.size(); ++i) {
        const auto& nd = nodes[i];
        if(!std::isfinite(nd.x) || !std::isfinite(nd.y) ||
           !std::isfinite(nd.w) || !std::isfinite(nd.h) ||
           nd.x < int_min || nd.y < int_min || nd.w <= 0 || nd.h <= 0 ||
           nd.w > int_max || nd.h > int_max ||
           nd.x + nd.w > int_max || nd.y + nd.h > int_max ||
           nd.fixed != (i >= (size_t)num_movable)) {
            printf("dp-legalize: invalid node %zu\n", i);
            return -1;
        }
        if(i < (size_t)num_movable &&
           (nd.h != site_h || nd.w > max_right ||
            std::fabs(nd.w - std::round(nd.w)) > 1e-9 ||
            (long long)std::llround(nd.w) % site_w != 0)) {
            printf("dp-legalize: unsupported movable dimensions at %zu\n", i);
            return -1;
        }
    }

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
        // A partially blocked site is unusable by an aligned cell.
        x0 = base_x + std::floor((x0 - base_x) / site_w) * site_w;
        x1 = base_x + std::ceil((x1 - base_x) / site_w) * site_w;
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
    auto covered_rows = [&](const LegalizeNode& nd) {
        const double y0 = std::max(nd.y, double(base_y));
        const double y1 = std::min(nd.y + nd.h, core_yh);
        if(y1 <= y0) {
            return std::make_pair(0, -1);
        }
        return std::make_pair((int)std::floor((y0 - base_y) / site_h),
                              (int)std::ceil((y1 - base_y) / site_h) - 1);
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
        const auto covered = covered_rows(nd);
        add_obstacle(covered.first, covered.second, nd.x, nd.x + nd.w);
    }

    std::vector<std::vector<std::pair<int, int>>> seg_ints(num_rows);
    for(auto& row : segs) {
        int rid = (row.origin_y - base_y) / site_h;
        seg_ints[rid].push_back(
            {row.origin_x,
             (int)((long long)row.origin_x + (long long)row.site_count * site_w)});
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
            // An absent y stays at its physical grid index and is blocked
            // across the full width, never fabricated as a usable ROW.
            add_obstacle(r, r, cursor, core_x1);
        }
    }

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
    }

    const double pad_dbu = std::ceil(padding_sites) * site_w;

    // Preserve an already-legal placement when its padding compliance is
    // sufficient. Site/row legality and non-overlap are mandatory; padding is
    // allowed to be best effort, matching the legalizer's padding behavior.
    constexpr int kPaddingComplianceNumerator = 4;
    constexpr int kPaddingComplianceDenominator = 5;
    auto placement_legal = [&](const std::vector<double>& xx,
                               const std::vector<double>& yy,
                               std::vector<std::vector<int>>& row_cells) {
        row_cells.assign(num_rows, {});
        for(int i = 0; i < num_movable; ++i) {
            if(!std::isfinite(xx[i]) || !std::isfinite(yy[i]) ||
               xx[i] < base_x || xx[i] + nodes[i].w > core_xh ||
               yy[i] < base_y || yy[i] + site_h > core_yh) {
                return false;
            }
            const long long x = std::llround(xx[i]);
            const long long y = std::llround(yy[i]);
            if(std::fabs(xx[i] - double(x)) > 1e-9 ||
               std::fabs(yy[i] - double(y)) > 1e-9 ||
               (x - base_x) % site_w != 0 ||
               (y - base_y) % site_h != 0) {
                return false;
            }
            const long long rid = (y - base_y) / site_h;
            if(rid < 0 || rid >= num_rows) {
                return false;
            }
            const auto& segs_at_y = seg_ints[rid];
            if(!std::any_of(segs_at_y.begin(), segs_at_y.end(),
                            [&](const std::pair<int, int>& seg) {
                                return x >= seg.first &&
                                       x + nodes[i].w <= seg.second;
                            })) {
                return false;
            }
            row_cells[rid].push_back(i);
        }
        for(int r = 0; r < num_rows; ++r) {
            auto& cell_ids = row_cells[r];
            std::sort(cell_ids.begin(), cell_ids.end(), [&](int lhs, int rhs) {
                return xx[lhs] < xx[rhs];
            });
            const auto& obstacles = row_obstacles[r];
            size_t obstacle_idx = 0;
            double previous_end = base_x;
            for(int cell_id : cell_ids) {
                const double x0 = xx[cell_id];
                const double x1 = x0 + nodes[cell_id].w;
                while(obstacle_idx < obstacles.size() &&
                      obstacles[obstacle_idx].second <= x0 + 1e-9) {
                    ++obstacle_idx;
                }
                if((obstacle_idx < obstacles.size() &&
                    obstacles[obstacle_idx].first < x1 - 1e-9) ||
                   x0 < previous_end - 1e-9) {
                    return false;
                }
                previous_end = x1;
            }
        }
        return true;
    };
    auto publish = [&](const std::vector<double>& xx,
                       const std::vector<double>& yy) {
        std::vector<double> result_x(xx.begin(), xx.begin() + num_movable);
        std::vector<int> result_row(num_movable);
        for(int i = 0; i < num_movable; ++i) {
            result_row[i] = (int)std::llround((yy[i] - base_y) / site_h);
        }
        out_x.swap(result_x);
        out_row.swap(result_row);
        base_x_out = base_x;
        base_y_out = base_y;
        num_rows_out = num_rows;
    };
    std::vector<double> input_x, input_y;
    for(int i = 0; i < num_movable; ++i) {
        input_x.push_back(nodes[i].x);
        input_y.push_back(nodes[i].y);
    }
    std::vector<std::vector<int>> input_row_cells;
    const bool input_legal = placement_legal(input_x, input_y, input_row_cells);

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
            const auto covered = covered_rows(nd);
            for(int rid = covered.first; rid <= covered.second; ++rid) {
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
        (long long)padding_pair_compliant * kPaddingComplianceDenominator >=
            (long long)padding_pair_count * kPaddingComplianceNumerator;
    if(input_legal && padding_compliant) {
        publish(input_x, input_y);
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
    if(sx.size() > (size_t)int_max) {
        printf("dp-legalize: too many obstacle fragments\n");
        return -1;
    }
    const int num_nodes = (int)sx.size();

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

    // Greedy can leave overlaps for abacus to repair. Check the capacity of
    // each free interval in abacus' center ordering before it can assert.
    std::vector<std::vector<int>> row_cells(num_rows);
    for(int i = 0; i < num_movable; ++i) {
        if(!std::isfinite(x[i]) || !std::isfinite(y[i]) ||
           x[i] < base_x || x[i] + sx[i] > core_xh ||
           y[i] < base_y || y[i] + site_h > core_yh ||
           std::fabs(y[i] - std::round(y[i])) > 1e-9 ||
           (std::llround(y[i]) - base_y) % site_h != 0) {
            printf("dp-legalize: greedy returned invalid coordinates\n");
            return -1;
        }
        const int rid = (int)((std::llround(y[i]) - base_y) / site_h);
        if(rid < 0 || rid >= num_rows || seg_ints[rid].empty()) {
            printf("dp-legalize: greedy left a cell on an absent row\n");
            return -1;
        }
        row_cells[rid].push_back(i);
    }
    for(int r = 0; r < num_rows; ++r) {
        auto& cids = row_cells[r];
        auto center = [&](int i) { return x[i] + sx[i] / 2; };
        std::sort(cids.begin(), cids.end(), [&](int a, int b) {
            return center(a) < center(b) || (center(a) == center(b) && a < b);
        });
        size_t ci = 0;
        double left = base_x;
        for(const auto& obstacle : row_obstacles[r]) {
            const double fixed_center = (obstacle.first + obstacle.second) / 2;
            double width = 0;
            // Movables precede fixed nodes when their centers coincide.
            while(ci < cids.size() && center(cids[ci]) <= fixed_center) {
                width += sx[cids[ci++]];
            }
            if(width > obstacle.first - left) {
                printf("dp-legalize: row %d has an overfull free interval\n", r);
                return -1;
            }
            left = obstacle.second;
        }
        double width = 0;
        while(ci < cids.size()) {
            width += sx[cids[ci++]];
        }
        if(width > core_xh - left) {
            printf("dp-legalize: row %d has an overfull free interval\n", r);
            return -1;
        }
    }

    DreamPlace::abacusLegalizationCPU(
        px.data(), py.data(), sx.data(), sy.data(), wts.data(), x.data(),
        y.data(), db.xl, db.yl, db.xh, db.yh, db.site_width, db.row_height,
        db.num_bins_x, db.num_bins_y, num_nodes, num_movable);

    if(!placement_legal(x, y, row_cells)) {
        printf("dp-legalize: abacus did not produce a legal placement\n");
        return -1;
    }

    const std::vector<double> opx = x;
    // Post-hoc placement padding: per row, shift cells right to open
    // dp_padding-site gaps (fixed insts are obstacles). Rows that cannot
    // fit degrade to gap 0, keeping the ops' feasible positions.
    if(pad_dbu > 0) {
        int degraded_rows = 0;
        for(int r = 0; r < num_rows; ++r) {
            auto& cids = row_cells[r];
            const auto& fiv = row_obstacles[r];
            if(cids.empty()) {
                continue;
            }
            bool placed = false;
            for(int attempt = 0; attempt < 2; ++attempt) {
                const double gap = attempt == 0 ? pad_dbu : 0.0;
                double prev_end = base_x;
                size_t fi = 0;
                bool overflow = false;
                for(int ci : cids) {
                    const double w = sx[ci];
                    double cx = std::max(opx[ci], prev_end);
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
                    placed = true;
                    if(attempt > 0) {
                        ++degraded_rows;
                    }
                    break;
                }
            }
            if(!placed) {
                for(int ci : cids) {
                    x[ci] = opx[ci];
                }
                ++degraded_rows;
            }
        }
        if(degraded_rows > 0) {
            printf("dp-legalize: %d rows degraded to zero padding\n",
                   degraded_rows);
        }
    }


    if(!placement_legal(x, y, row_cells)) {
        printf("dp-legalize: padded result is not legal; discarding result\n");
        return -1;
    }
    publish(x, y);
    return 0;
}

}  // namespace ecc
