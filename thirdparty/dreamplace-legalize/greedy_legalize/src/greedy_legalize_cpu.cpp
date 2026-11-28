/**
 * @file   greedy_legalize_cpu.cpp
 * @author Yibo Lin
 * @date   Oct 2018
 */
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include "function_cpu.h"

DREAMPLACE_BEGIN_NAMESPACE

template <typename T>
int greedyLegalizationCPU(
        const LegalizationDB<T>& db, 
        const T* init_x, const T* init_y, 
        const T* node_size_x, const T* node_size_y, 
        T* x, T* y, 
        const T xl, const T yl, const T xh, const T yh, 
        const T site_width, const T row_height, 
        int num_bins_x, int num_bins_y, 
        const int num_nodes, 
        const int num_movable_nodes
        )
{
    float milliseconds = 0; 

    auto run_single_pass = [&](const T* base_x,
                               const T* base_y,
                               T alpha,
                               T beta,
                               bool lr_flag,
                               bool width_first,
                               bool snap_to_edge,
                               std::vector<T>& cand_x,
                               std::vector<T>& cand_y) -> int {
        cand_x.assign(init_x, init_x + num_nodes);
        cand_y.assign(init_y, init_y + num_nodes);
        std::copy(base_x, base_x + num_movable_nodes, cand_x.begin());
        std::copy(base_y, base_y + num_movable_nodes, cand_y.begin());

        int pass_bins_x = 1;
        int pass_bins_y = 1;
        T bin_size_x = div((xh - xl), pass_bins_x);
        T bin_size_y = div((yh - yl), pass_bins_y);
        bin_size_y = std::max(
            (T)(ceilDiv(bin_size_y, row_height) * row_height), (T)row_height);
        pass_bins_y = ceilDiv((yh - yl), bin_size_y);

        // bin dimension in y direction for blanks is row_height
        T blank_bin_size_y = row_height;
        int blank_num_bins_y = floorDiv((yh - yl), blank_bin_size_y);
        dreamplacePrint(kDEBUG, "%s blank_num_bins_y = %d\n",
                        "Standard cell legalization", blank_num_bins_y);

        std::vector<std::vector<int> > bin_cells(pass_bins_x * pass_bins_y);
        std::vector<std::vector<int> > bin_cells_copy(pass_bins_x * pass_bins_y);

        distributeCells2BinsCPU(
            db, cand_x.data(), cand_y.data(), node_size_x, node_size_y,
            bin_size_x, bin_size_y, xl, yl, xh, yh, pass_bins_x, pass_bins_y,
            num_nodes, num_movable_nodes, bin_cells);

        std::vector<std::vector<int> > bin_fixed_cells(pass_bins_x * pass_bins_y);
        distributeFixedCells2BinsCPU(
            db, init_x, init_y, node_size_x, node_size_y, bin_size_x, bin_size_y,
            xl, yl, xh, yh, pass_bins_x, pass_bins_y, num_nodes,
            num_movable_nodes, bin_fixed_cells);

        std::vector<std::vector<Blank<T> > > bin_blanks(pass_bins_x *
                                                        blank_num_bins_y);
        std::vector<std::vector<Blank<T> > > bin_blanks_copy(pass_bins_x *
                                                             blank_num_bins_y);
        distributeBlanks2BinsCPU(
            init_x, init_y, node_size_x, node_size_y, bin_fixed_cells,
            bin_size_x, bin_size_y, blank_bin_size_y, xl, yl, xh, yh,
            site_width, row_height, pass_bins_x, pass_bins_y, blank_num_bins_y,
            bin_blanks);

        int num_unplaced_cells_host = 0;
        int min_unplaced_node_size_x_host = 0;
        int num_iters =
            floor(log((T)std::min(pass_bins_x, pass_bins_y)) / log(2.0)) + 1;
        for (int iter = 0; iter < num_iters; ++iter)
        {
            dreamplacePrint(kDEBUG, "%s iteration %d with %dx%d bins\n",
                            "Standard cell legalization", iter, pass_bins_x,
                            pass_bins_y);
            num_unplaced_cells_host = 0;
            dreamplacePrint(kDEBUG, "%s #bin_blanks\n",
                            "Standard cell legalization");
            countBinObjects(bin_blanks);

            milliseconds = clock();
            legalizeBinCPU<T>(
                init_x, init_y, node_size_x, node_size_y, bin_blanks, bin_cells,
                cand_x.data(), cand_y.data(), pass_bins_x, pass_bins_y,
                blank_num_bins_y, bin_size_x, bin_size_y, blank_bin_size_y,
                site_width, row_height, xl, yl, xh, yh, alpha, beta, lr_flag,
                &num_unplaced_cells_host, width_first, snap_to_edge);
            milliseconds = (clock() - milliseconds) / CLOCKS_PER_SEC * 1000;
            dreamplacePrint(kINFO, "%s legalizeBin takes %.3f ms\n",
                            "Standard cell legalization", milliseconds);
            dreamplacePrint(kDEBUG, "%s num_unplaced_cells = %d\n",
                            "Standard cell legalization",
                            num_unplaced_cells_host);

            if (num_unplaced_cells_host == 0 || iter + 1 == num_iters)
            {
                break;
            }

            milliseconds = clock();
            min_unplaced_node_size_x_host = floorDiv((xh - xl), site_width);
            minNodeSizeCPU(bin_cells, node_size_x, node_size_y, site_width,
                           row_height, pass_bins_x, pass_bins_y,
                           &min_unplaced_node_size_x_host);
            milliseconds = (clock() - milliseconds) / CLOCKS_PER_SEC * 1000;
            dreamplacePrint(kINFO, "%s minNodeSize takes %.3f ms\n",
                            "Standard cell legalization", milliseconds);

            int dst_num_bins_x = (pass_bins_x >> 1) + (pass_bins_x & 1);
            int dst_num_bins_y = (pass_bins_y >> 1) + (pass_bins_y & 1);
            int scale_ratio_x = (pass_bins_x == dst_num_bins_x)
                                    ? 1
                                    : pass_bins_x / dst_num_bins_x;
            int scale_ratio_y = (pass_bins_y == dst_num_bins_y)
                                    ? 1
                                    : pass_bins_y / dst_num_bins_y;

            resizeBinObjectsCPU(bin_cells_copy, dst_num_bins_x, dst_num_bins_y);
            mergeBinCellsCPU(bin_cells, pass_bins_x, pass_bins_y,
                             bin_cells_copy, dst_num_bins_x, dst_num_bins_y,
                             scale_ratio_x, scale_ratio_y);
            resizeBinObjectsCPU(bin_blanks_copy, dst_num_bins_x,
                                blank_num_bins_y);
            mergeBinBlanksCPU(bin_blanks, pass_bins_x, blank_num_bins_y,
                              bin_blanks_copy, dst_num_bins_x, blank_num_bins_y,
                              scale_ratio_x,
                              min_unplaced_node_size_x_host * site_width);

            pass_bins_x = dst_num_bins_x;
            pass_bins_y = dst_num_bins_y;
            bin_size_x = bin_size_x * 2;
            bin_size_y = bin_size_y * 2;

            std::swap(bin_cells, bin_cells_copy);
            std::swap(bin_blanks, bin_blanks_copy);
        }
        return num_unplaced_cells_host;
    };

    auto compute_displacement = [&](const std::vector<T>& cand_x,
                                    const std::vector<T>& cand_y) -> double {
        double disp = 0.0;
        for (int k = 0; k < num_movable_nodes; ++k)
        {
            disp += std::fabs(double(cand_x[k]) - double(init_x[k])) +
                    std::fabs(double(cand_y[k]) - double(init_y[k]));
        }
        return disp;
    };

    std::vector<T> best_x(x, x + num_nodes);
    std::vector<T> best_y(y, y + num_nodes);
    int best_unplaced = num_movable_nodes + 1;
    double best_disp = DBL_MAX;
    bool best_lr = false;

    std::vector<T> cand_x;
    std::vector<T> cand_y;
    auto try_candidate = [&](const T* base_x,
                             const T* base_y,
                             T alpha,
                             T beta,
                             bool lr_flag,
                             bool width_first,
                             bool snap_to_edge) {
        int unplaced = run_single_pass(base_x, base_y, alpha, beta, lr_flag,
                                       width_first, snap_to_edge, cand_x,
                                       cand_y);
        double disp = compute_displacement(cand_x, cand_y);
        if (unplaced < best_unplaced ||
            (unplaced == best_unplaced && disp < best_disp))
        {
            best_unplaced = unplaced;
            best_disp = disp;
            best_lr = lr_flag;
            best_x.swap(cand_x);
            best_y.swap(cand_y);
        }
    };

    // 1. Evaluate independent right-to-left and left-to-right sweeps from
    //    (init_x, init_y) and keep the one with fewer unplaced cells /
    //    smaller total displacement.
    try_candidate(x, y, (T)1.0, (T)4.0, /*lr_flag=*/false,
                  /*width_first=*/false, /*snap_to_edge=*/false);
    try_candidate(x, y, (T)1.0, (T)4.0, /*lr_flag=*/true,
                  /*width_first=*/false, /*snap_to_edge=*/false);

    // 2. If a valid sweep succeeded, also test one reverse-direction midpoint
    //    refinement (alpha = 0.5) and keep it only if it stays legal and
    //    strictly reduces total displacement.
    if (best_unplaced == 0)
    {
        const std::vector<T> seed_x = best_x;
        const std::vector<T> seed_y = best_y;
        try_candidate(seed_x.data(), seed_y.data(), (T)0.5, (T)4.0, !best_lr,
                      /*width_first=*/false, /*snap_to_edge=*/false);
    }

    // 3. Fallback for dense layouts with wide buffers: only when position-order
    //    sweeps leave unplaced cells due to blank fragmentation, pack widest
    //    cells first (then with edge-snapping if needed).
    if (best_unplaced > 0)
    {
        try_candidate(x, y, (T)1.0, (T)4.0, /*lr_flag=*/false,
                      /*width_first=*/true, /*snap_to_edge=*/false);
        try_candidate(x, y, (T)1.0, (T)4.0, /*lr_flag=*/true,
                      /*width_first=*/true, /*snap_to_edge=*/false);
    }
    if (best_unplaced > 0)
    {
        try_candidate(x, y, (T)1.0, (T)4.0, /*lr_flag=*/false,
                      /*width_first=*/true, /*snap_to_edge=*/true);
        try_candidate(x, y, (T)1.0, (T)4.0, /*lr_flag=*/true,
                      /*width_first=*/true, /*snap_to_edge=*/true);
    }

    std::copy(best_x.begin(), best_x.begin() + num_movable_nodes, x);
    std::copy(best_y.begin(), best_y.begin() + num_movable_nodes, y);
    return best_unplaced;
}

// Provide out-of-line definitions for callers that only see the
// declaration in function_cpu.h. Recent GCC versions fully inline the
// implicit instantiations used by the wrappers below in this translation
// unit, leaving no symbol for the linker.
template int greedyLegalizationCPU<double>(
        const LegalizationDB<double>& db, 
        const double* init_x, const double* init_y, 
        const double* node_size_x, const double* node_size_y, 
        double* x, double* y, 
        const double xl, const double yl, const double xh, const double yh, 
        const double site_width, const double row_height, 
        int num_bins_x, int num_bins_y, 
        const int num_nodes, 
        const int num_movable_nodes);

template int greedyLegalizationCPU<float>(
        const LegalizationDB<float>& db, 
        const float* init_x, const float* init_y, 
        const float* node_size_x, const float* node_size_y, 
        float* x, float* y, 
        const float xl, const float yl, const float xh, const float yh, 
        const float site_width, const float row_height, 
        int num_bins_x, int num_bins_y, 
        const int num_nodes, 
        const int num_movable_nodes);

int instantiateGreedyLegalizationCPU(
        const LegalizationDB<float>& db, 
        const float* init_x, const float* init_y, 
        const float* node_size_x, const float* node_size_y, 
        float* x, float* y, 
        const float xl, const float yl, const float xh, const float yh, 
        const float site_width, const float row_height, 
        int num_bins_x, int num_bins_y, 
        const int num_nodes, 
        const int num_movable_nodes
        )
{
    return greedyLegalizationCPU(
            db, 
            init_x, init_y, 
            node_size_x, node_size_y, 
            x, y, 
            xl, yl, xh, yh, 
            site_width, row_height, 
            num_bins_x, num_bins_y, 
            num_nodes, 
            num_movable_nodes
            );
}

int instantiateGreedyLegalizationCPU(
        const LegalizationDB<double>& db, 
        const double* init_x, const double* init_y, 
        const double* node_size_x, const double* node_size_y, 
        double* x, double* y, 
        const double xl, const double yl, const double xh, const double yh, 
        const double site_width, const double row_height, 
        int num_bins_x, int num_bins_y, 
        const int num_nodes, 
        const int num_movable_nodes
        )
{
    return greedyLegalizationCPU(
            db, 
            init_x, init_y, 
            node_size_x, node_size_y, 
            x, y, 
            xl, yl, xh, yh, 
            site_width, row_height, 
            num_bins_x, num_bins_y, 
            num_nodes, 
            num_movable_nodes
            );
}

DREAMPLACE_END_NAMESPACE
