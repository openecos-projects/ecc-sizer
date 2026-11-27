# ECC Sizer 评测

## ISPD26（2026-11-25）

### 算法策略

本轮使用 AES、JPEG、Ariane 及其 v2 的 ASAP7 输入。各流程独立从原始设计开始，使用 32 个 STA 线程；初始列仅做合法化，不执行时序修复。六设计共 30 项独立评测全部通过。

| 流程 | 本轮策略 |
|---|---|
| drv+setup | 先修复 slew、电容和 fanout 违例，再沿关键路径修复 setup；使用换型、缓冲和负载降型候选，经 STA 验收后提交。采用默认配置，未配置 VT 后缀，不启用 VT 切换；不调用 Path、VG 或 GOBI。 |
| VG | 按电气违例选网，在 placement Steiner 树上用 CPU Van Ginneken 动态规划传播、合并和剪枝候选，生成完整缓冲树后统一应用与验证。 |
| GOBI | 按电气违例选网，在 GPU forest 上并行求解多个网的缓冲树；本轮使用 GPU 1，六例均为 CUDA，无 fallback。完整结果见 [GJ2026](gj2026.md)。 |
| Path | 从关键 setup/recovery 路径提取共享网，生成旁支隔离、关键负载缓冲和整网多级树候选；以预测全局 WNS/TNS 排序，预测收益相同时结合受影响路径、电气风险、buffer 数量和面积比较。实测反馈用于校准预测及复用拒绝证据。 |

Path 的接受、排序与停止分别控制：

- **接受**：使用全局时序指标；额外绝对、相对收益门槛均为 0，但仍要求超过 STA 数值容差的严格改善，并满足 hold、DRV 等非退化检查。本轮不设面积增长上限。
- **搜索**：`max_paths=16`、`max_candidates=8`、`batch_size=1`；启用自适应搜索、拒绝证据复用和时序预检查，关闭端点覆盖与分组调度。不调用 VG/GOBI，也不把跨逻辑门的多个网拼成一棵树。
- **停止**：续搜额度 `search_patience=32`，每次试验消耗 1；接受后按 WNS/TNS 的实测进展补回额度，上限仍为 32。`progress_fraction=0.05` 是续搜补偿尺度，不是接受门槛；微小改善不会直接重置完整额度。试验次数、刷新次数均无硬上限，算法不设墙钟时间上限。
- **验证**：搜索阶段采用增量寄生更新与增量电气分析；阶段末统一合法化和完整验证，失败则回滚整阶段。最终输出由独立进程重新加载评估。

### 评测结果

| 设计 | 初始输入 | drv+setup | VG | Path |
|---|---|---|---|---|
| AES | WNS -1.025906 ns<br>TNS -278.964 ns<br>时间 —<br>buffer 0 | WNS -0.101187 ns (90.14%)<br>TNS -32.886 ns (88.21%)<br>算法时间 13.14 s<br>进程时间 18.84 s<br>buffer 11 | WNS -0.120080 ns (88.30%)<br>TNS -35.153 ns (87.40%)<br>算法时间 1.83 s<br>进程时间 7.11 s<br>buffer 283 | WNS -0.097584 ns (90.49%)<br>TNS -19.597 ns (92.98%)<br>算法时间 8.90 s<br>进程时间 14.42 s<br>buffer 26 (-257) |
| AES v2 | WNS -1.435193 ns<br>TNS -252.872 ns<br>时间 —<br>buffer 0 | WNS -0.495422 ns (65.48%)<br>TNS -116.215 ns (54.04%)<br>算法时间 15.56 s<br>进程时间 21.05 s<br>buffer 94 | WNS -0.489287 ns (65.91%)<br>TNS -108.583 ns (57.06%)<br>算法时间 4.07 s<br>进程时间 9.73 s<br>buffer 524 | WNS -0.403171 ns (71.91%)<br>TNS -99.348 ns (60.71%)<br>算法时间 26.27 s<br>进程时间 32.35 s<br>buffer 46 (-478) |
| JPEG | WNS -9.348061 ns<br>TNS -28185.798 ns<br>时间 —<br>buffer 0 | WNS -0.366657 ns (96.08%)<br>TNS -960.734 ns (96.59%)<br>算法时间 41.52 s<br>进程时间 52.97 s<br>buffer 222 | WNS -0.276766 ns (97.04%)<br>TNS -418.623 ns (98.51%)<br>算法时间 44.77 s<br>进程时间 55.76 s<br>buffer 12677 | WNS -0.220082 ns (97.65%)<br>TNS -353.323 ns (98.75%)<br>算法时间 58.17 s<br>进程时间 67.74 s<br>buffer 251 (-12426) |
| JPEG v2 | WNS -33.269903 ns<br>TNS -94154.998 ns<br>时间 —<br>buffer 0 | WNS -4.248329 ns (87.23%)<br>TNS -12090.647 ns (87.16%)<br>算法时间 24.54 s<br>进程时间 35.29 s<br>buffer 186 | WNS -4.820918 ns (85.51%)<br>TNS -12352.581 ns (86.88%)<br>算法时间 27.51 s<br>进程时间 38.08 s<br>buffer 8518 | WNS -4.250474 ns (87.22%)<br>TNS -10709.497 ns (88.63%)<br>算法时间 23.29 s<br>进程时间 33.10 s<br>buffer 109 (-8409) |
| Ariane | WNS -188.529020 ns<br>TNS -3368791.055 ns<br>时间 —<br>buffer 0 | WNS -1.792823 ns (99.05%)<br>TNS -24398.202 ns (99.28%)<br>算法时间 159.20 s<br>进程时间 187.50 s<br>buffer 2405 | WNS -1.262066 ns (99.33%)<br>TNS -9213.542 ns (99.73%)<br>算法时间 339.72 s<br>进程时间 368.38 s<br>buffer 34122 | WNS -2.852753 ns (98.49%)<br>TNS -28733.274 ns (99.15%)<br>算法时间 185.77 s<br>进程时间 212.35 s<br>buffer 385 (-33737) |
| Ariane v2 | WNS -23.191897 ns<br>TNS -231271.645 ns<br>时间 —<br>buffer 0 | WNS -1.800890 ns (92.23%)<br>TNS -21001.488 ns (90.92%)<br>算法时间 276.47 s<br>进程时间 307.44 s<br>buffer 3196 | WNS -1.134665 ns (95.11%)<br>TNS -8949.656 ns (96.13%)<br>算法时间 171.90 s<br>进程时间 201.33 s<br>buffer 43920 | WNS -2.698872 ns (88.36%)<br>TNS -38510.310 ns (83.35%)<br>算法时间 177.27 s<br>进程时间 206.28 s<br>buffer 287 (-43633) |

### 数据结论

以下中位数按设计分别计算；WNS/TNS 改善率只统计初始存在对应违例的设计，算法时间取各修复阶段时间之和。电气清零要求 slew、cap、fanout 三项计数均为 0。独立评测通过表示输出可加载且通过放置、供电等检查，不表示 setup 或 DRV 已清零。

| 算法 | WNS / TNS 改善中位数 | 新增 buffer 中位数 | 面积增长中位数 | 算法时间中位数 | 电气清零 |
|---|---:|---:|---:|---:|---:|
| drv+setup | 91.19% / 89.57% | 204 | 0.43% | 33.03 s | 2/6 |
| VG | 91.70% / 91.76% | 10597.5 | 10.88% | 36.14 s | 2/6 |
| GOBI | 91.91% / 93.00% | 9242 | 10.35% | 7.85 s | 2/6 |
| Path | 89.43% / 90.80% | 180 | 0.32% | 42.22 s | 1/6 |

- **Path 在 AES/JPEG 系列中取得较好的时序与修改规模平衡。** 四个设计的 TNS 都是四种算法中最优，AES、AES v2、JPEG 的 WNS 也最优；JPEG v2 的 drv+setup WNS 略好（−4.2483 对 −4.2505 ns）。Path 六设计总新增 1104 个 buffer，相比 VG 的 100044 个减少 98.90%，但这不代表同等最终时序下的节省。
- **Ariane 暴露了 Path 的收敛不足。** Ariane / Ariane v2 的 Path TNS 为 −28733 / −38510 ns，而 GOBI 为 −9084 / −7717 ns。Path 仅插入 385 / 287 个 buffer，GOBI 为 43331 / 54971 个；这是明显的质量与修改规模取舍，不能仅凭相对初始超过 98% 的 TNS 改善判定 Ariane 已修好。
- **GOBI 在这组大规模插入问题上体现了求解吞吐优势。** 配对的 VG/GOBI 求解时间比中位数为 64.11，范围 3.46～138.85；算法阶段时间比中位数为 4.52。两者算法及输出树不同，这些比值不是相同算法的纯 GPU 加速比。GOBI 六设计总插入数反而比 VG 多 18.76%，速度优势未自动转化为更小修改规模。
- **drv+setup 是修改规模适中的常规修复参照。** 六设计 WNS/TNS 均改善，面积增长中位数 0.43%；但没有取得最优 TNS，且 Ariane v2 算法时间为 276.47 s，高于 Path 的 177.27 s 和 GOBI 的 31.81 s。不能预设常规修复一定更快。
- **目前没有一种算法完成 setup 清零。** 六设计所有修复结果仍有 setup 违例，Path 六例均因续搜额度耗尽停止，日志明确表示搜索未穷尽。下一步应同时评估残余负裕量、电气违例和 buffer 成本，而不是继续压低接受门槛或只追求插入更少。

原始记录：[ispd26-benchmark-20261125](../results/ispd26-benchmark-20261125/)。

<!-- ecc-ics55-benchmark -->
## ECC / ICS55（2026-11-25）

本轮评测 68 个 post-placement 快照，分别运行初始、drv+setup、VG、GOBI、Path。已完成 340/340 项，独立验证通过 340 项。

采用各快照元数据指定的 TT 1.2 V / 25°C Liberty 与 cworst 技术 LEF；使用 MET2 的 LEF RC 做 placement 寄生估计，不是布线后提取或签核结果。保留原 SDC、理想时钟和工艺 dont-use；最大 fanout 为 32，STA 使用 32 线程。Path 使用与本页 ASAP7 评测相同的阶段末验证、增量分析及续搜策略，其余流程采用默认策略；不启用 VT 后缀映射。各流程统一使用工艺元数据指定的 BUFX8H7L、BUFX12H7L、BUFX16H7L、BUFX20H7L 四种 buffer，以满足当前 GOBI 最多 32 种 buffer 的限制。

### ecc_pr329_1011

| 设计 | 初始输入 | drv+setup | VG | Path |
|---|---|---|---|---|
| BM64 | WNS -1.232218 ns<br>TNS -434.579 ns<br>时间 —<br>buffer 0 | WNS -1.201616 ns (2.48%)<br>TNS -425.246 ns (2.15%)<br>算法时间 33.14 s<br>进程时间 40.65 s<br>buffer 41 | WNS -1.231775 ns (0.04%)<br>TNS -428.704 ns (1.35%)<br>算法时间 1.40 s<br>进程时间 9.08 s<br>buffer 110 | WNS -1.118426 ns (9.23%)<br>TNS -407.353 ns (6.26%)<br>算法时间 16.45 s<br>进程时间 23.92 s<br>buffer 21 (-89) |
| PPU | WNS -0.711682 ns<br>TNS -79.282 ns<br>时间 —<br>buffer 0 | WNS -0.559134 ns (21.43%)<br>TNS -59.854 ns (24.51%)<br>算法时间 8.06 s<br>进程时间 15.60 s<br>buffer 124 | WNS -0.674429 ns (5.23%)<br>TNS -63.020 ns (20.51%)<br>算法时间 1.10 s<br>进程时间 8.94 s<br>buffer 307 | WNS -0.456658 ns (35.83%)<br>TNS -69.915 ns (11.82%)<br>算法时间 6.79 s<br>进程时间 14.65 s<br>buffer 39 (-268) |
| aes | WNS 0.000000 ns<br>TNS 0.000 ns<br>时间 —<br>buffer 0 | WNS 0.000000 ns (—)<br>TNS 0.000 ns (—)<br>算法时间 2.74 s<br>进程时间 11.89 s<br>buffer 4 | WNS 0.000000 ns (—)<br>TNS 0.000 ns (—)<br>算法时间 1.42 s<br>进程时间 9.48 s<br>buffer 12 | WNS 0.000000 ns (—)<br>TNS 0.000 ns (—)<br>算法时间 1.48 s<br>进程时间 10.45 s<br>buffer 0 (-12) |
| arm9 | WNS -2.335113 ns<br>TNS -440.989 ns<br>时间 —<br>buffer 0 | WNS -2.244168 ns (3.89%)<br>TNS -421.819 ns (4.35%)<br>算法时间 50.87 s<br>进程时间 60.72 s<br>buffer 4 | WNS -2.334963 ns (0.01%)<br>TNS -383.207 ns (13.10%)<br>算法时间 2.92 s<br>进程时间 12.09 s<br>buffer 697 | WNS -2.144515 ns (8.16%)<br>TNS -400.007 ns (9.29%)<br>算法时间 27.11 s<br>进程时间 36.51 s<br>buffer 35 (-662) |
| blake2s | WNS -2.340643 ns<br>TNS -699.708 ns<br>时间 —<br>buffer 0 | WNS -2.157899 ns (7.81%)<br>TNS -645.289 ns (7.78%)<br>算法时间 18.74 s<br>进程时间 27.39 s<br>buffer 304 | WNS -2.272149 ns (2.93%)<br>TNS -654.704 ns (6.43%)<br>算法时间 2.58 s<br>进程时间 11.29 s<br>buffer 1204 | WNS -2.216629 ns (5.30%)<br>TNS -666.212 ns (4.79%)<br>算法时间 20.15 s<br>进程时间 28.86 s<br>buffer 28 (-1176) |
| can_top | WNS -10.533285 ns<br>TNS -19225.993 ns<br>时间 —<br>buffer 0 | WNS -5.172443 ns (50.89%)<br>TNS -15837.099 ns (17.63%)<br>算法时间 244.01 s<br>进程时间 263.41 s<br>buffer 1314 | WNS -2.114794 ns (79.92%)<br>TNS -12271.247 ns (36.17%)<br>算法时间 17.83 s<br>进程时间 37.40 s<br>buffer 21282 | WNS -9.284428 ns (11.86%)<br>TNS -19215.050 ns (0.06%)<br>算法时间 56.86 s<br>进程时间 75.42 s<br>buffer 26 (-21256) |
| double_fpu | WNS -1.731038 ns<br>TNS -366.855 ns<br>时间 —<br>buffer 0 | WNS -1.679528 ns (2.98%)<br>TNS -365.208 ns (0.45%)<br>算法时间 95.54 s<br>进程时间 108.04 s<br>buffer 2 | WNS -1.725815 ns (0.30%)<br>TNS -348.994 ns (4.87%)<br>算法时间 4.90 s<br>进程时间 16.80 s<br>buffer 872 | WNS -1.731038 ns (0.00%)<br>TNS -366.855 ns (0.00%)<br>算法时间 25.47 s<br>进程时间 37.63 s<br>buffer 0 (-872) |
| eth_top | WNS -1.778487 ns<br>TNS -326.988 ns<br>时间 —<br>buffer 0 | WNS -1.429397 ns (19.63%)<br>TNS -151.005 ns (53.82%)<br>算法时间 39.90 s<br>进程时间 50.65 s<br>buffer 479 | WNS -1.436217 ns (19.24%)<br>TNS -190.509 ns (41.74%)<br>算法时间 4.02 s<br>进程时间 14.15 s<br>buffer 1670 | WNS -1.414198 ns (20.48%)<br>TNS -173.226 ns (47.02%)<br>算法时间 18.22 s<br>进程时间 28.22 s<br>buffer 25 (-1645) |
| fft | WNS -1.232389 ns<br>TNS -68.392 ns<br>时间 —<br>buffer 0 | WNS -1.195651 ns (2.98%)<br>TNS -65.341 ns (4.46%)<br>算法时间 23.51 s<br>进程时间 31.98 s<br>buffer 35 | WNS -1.231916 ns (0.04%)<br>TNS -65.509 ns (4.22%)<br>算法时间 1.85 s<br>进程时间 10.13 s<br>buffer 87 | WNS -1.232389 ns (0.00%)<br>TNS -68.392 ns (0.00%)<br>算法时间 16.02 s<br>进程时间 24.27 s<br>buffer 0 (-87) |
| picorv32a | WNS -1.816927 ns<br>TNS -92.608 ns<br>时间 —<br>buffer 0 | WNS -1.734833 ns (4.52%)<br>TNS -88.623 ns (4.30%)<br>算法时间 11.40 s<br>进程时间 18.91 s<br>buffer 43 | WNS -1.808253 ns (0.48%)<br>TNS -91.999 ns (0.66%)<br>算法时间 1.49 s<br>进程时间 9.03 s<br>buffer 226 | WNS -1.684740 ns (7.28%)<br>TNS -89.189 ns (3.69%)<br>算法时间 8.81 s<br>进程时间 16.30 s<br>buffer 32 (-194) |
| poly1305 | WNS -2.120734 ns<br>TNS -316.018 ns<br>时间 —<br>buffer 0 | WNS -2.007480 ns (5.34%)<br>TNS -299.376 ns (5.27%)<br>算法时间 47.03 s<br>进程时间 58.64 s<br>buffer 261 | WNS -2.087611 ns (1.56%)<br>TNS -304.105 ns (3.77%)<br>算法时间 3.78 s<br>进程时间 14.79 s<br>buffer 708 | WNS -2.044739 ns (3.58%)<br>TNS -311.057 ns (1.57%)<br>算法时间 24.45 s<br>进程时间 35.89 s<br>buffer 31 (-677) |
| salsa20 | WNS -1.869553 ns<br>TNS -259.958 ns<br>时间 —<br>buffer 0 | WNS -1.603267 ns (14.24%)<br>TNS -225.948 ns (13.08%)<br>算法时间 42.70 s<br>进程时间 51.04 s<br>buffer 172 | WNS -1.777896 ns (4.90%)<br>TNS -239.947 ns (7.70%)<br>算法时间 2.78 s<br>进程时间 10.99 s<br>buffer 1163 | WNS -1.551532 ns (17.01%)<br>TNS -227.290 ns (12.57%)<br>算法时间 14.65 s<br>进程时间 22.86 s<br>buffer 32 (-1131) |
| sha256 | WNS -1.602234 ns<br>TNS -82.339 ns<br>时间 —<br>buffer 0 | WNS -1.586804 ns (0.96%)<br>TNS -81.594 ns (0.90%)<br>算法时间 29.72 s<br>进程时间 36.81 s<br>buffer 0 | WNS -1.602234 ns (0.00%)<br>TNS -82.339 ns (0.00%)<br>算法时间 2.33 s<br>进程时间 9.28 s<br>buffer 0 | WNS -1.498761 ns (6.46%)<br>TNS -77.420 ns (5.97%)<br>算法时间 8.63 s<br>进程时间 15.55 s<br>buffer 32 (+32) |
| sm4 | WNS -0.136764 ns<br>TNS -1.229 ns<br>时间 —<br>buffer 0 | WNS -0.027347 ns (80.00%)<br>TNS -0.073 ns (94.07%)<br>算法时间 94.65 s<br>进程时间 114.51 s<br>buffer 3 | WNS 0.000000 ns (100.00%)<br>TNS 0.000 ns (100.00%)<br>算法时间 9.36 s<br>进程时间 29.02 s<br>buffer 847 | WNS 0.000000 ns (100.00%)<br>TNS 0.000 ns (100.00%)<br>算法时间 12.75 s<br>进程时间 32.18 s<br>buffer 3 (-844) |
| stage_b_ysyx_23060170 | WNS -0.817717 ns<br>TNS -167.522 ns<br>时间 —<br>buffer 0 | WNS -0.791706 ns (3.18%)<br>TNS -128.858 ns (23.08%)<br>算法时间 18.00 s<br>进程时间 24.61 s<br>buffer 37 | WNS -0.817717 ns (0.00%)<br>TNS -167.522 ns (0.00%)<br>算法时间 1.30 s<br>进程时间 7.78 s<br>buffer 0 | WNS -0.817717 ns (0.00%)<br>TNS -167.522 ns (0.00%)<br>算法时间 6.19 s<br>进程时间 12.69 s<br>buffer 0 (+0) |
| stage_b_ysyx_23060203 | WNS -0.344087 ns<br>TNS -109.608 ns<br>时间 —<br>buffer 0 | WNS -0.289700 ns (15.81%)<br>TNS -51.341 ns (53.16%)<br>算法时间 13.19 s<br>进程时间 20.25 s<br>buffer 187 | WNS -0.343997 ns (0.03%)<br>TNS -106.773 ns (2.59%)<br>算法时间 1.14 s<br>进程时间 7.78 s<br>buffer 351 | WNS -0.344087 ns (0.00%)<br>TNS -109.608 ns (0.00%)<br>算法时间 7.26 s<br>进程时间 13.79 s<br>buffer 0 (-351) |
| stage_b_ysyx_23060229 | WNS -0.885994 ns<br>TNS -92.346 ns<br>时间 —<br>buffer 0 | WNS -0.754288 ns (14.87%)<br>TNS -82.847 ns (10.29%)<br>算法时间 20.76 s<br>进程时间 27.50 s<br>buffer 43 | WNS -0.865014 ns (2.37%)<br>TNS -82.839 ns (10.29%)<br>算法时间 1.07 s<br>进程时间 7.73 s<br>buffer 210 | WNS -0.719835 ns (18.75%)<br>TNS -86.481 ns (6.35%)<br>算法时间 8.06 s<br>进程时间 14.79 s<br>buffer 30 (-180) |
| stage_b_ysyx_23060246 | WNS -0.679517 ns<br>TNS -93.798 ns<br>时间 —<br>buffer 0 | WNS -0.657111 ns (3.30%)<br>TNS -88.428 ns (5.72%)<br>算法时间 13.57 s<br>进程时间 20.71 s<br>buffer 43 | WNS -0.679517 ns (0.00%)<br>TNS -93.798 ns (0.00%)<br>算法时间 1.59 s<br>进程时间 8.59 s<br>buffer 0 | WNS -0.679517 ns (0.00%)<br>TNS -93.798 ns (0.00%)<br>算法时间 6.34 s<br>进程时间 13.34 s<br>buffer 0 (+0) |
| stage_b_ysyx_24070003 | WNS -1.296683 ns<br>TNS -253.504 ns<br>时间 —<br>buffer 0 | WNS -1.245700 ns (3.93%)<br>TNS -233.621 ns (7.84%)<br>算法时间 7.05 s<br>进程时间 13.64 s<br>buffer 55 | WNS -1.296683 ns (0.00%)<br>TNS -253.504 ns (0.00%)<br>算法时间 1.38 s<br>进程时间 7.94 s<br>buffer 0 | WNS -1.166303 ns (10.05%)<br>TNS -214.038 ns (15.57%)<br>算法时间 7.84 s<br>进程时间 14.60 s<br>buffer 35 (+35) |
| stage_b_ysyx_24100012 | WNS -0.233955 ns<br>TNS -13.990 ns<br>时间 —<br>buffer 0 | WNS -0.186607 ns (20.24%)<br>TNS -11.308 ns (19.17%)<br>算法时间 9.83 s<br>进程时间 16.60 s<br>buffer 35 | WNS -0.224030 ns (4.24%)<br>TNS -12.068 ns (13.73%)<br>算法时间 0.93 s<br>进程时间 7.58 s<br>buffer 83 | WNS -0.140645 ns (39.88%)<br>TNS -8.925 ns (36.20%)<br>算法时间 10.29 s<br>进程时间 16.85 s<br>buffer 48 (-35) |
| stage_b_ysyx_24110017 | WNS -0.619937 ns<br>TNS -161.924 ns<br>时间 —<br>buffer 0 | WNS -0.564082 ns (9.01%)<br>TNS -127.994 ns (20.95%)<br>算法时间 11.76 s<br>进程时间 18.45 s<br>buffer 19 | WNS -0.619937 ns (0.00%)<br>TNS -161.924 ns (0.00%)<br>算法时间 1.38 s<br>进程时间 7.94 s<br>buffer 0 | WNS -0.526496 ns (15.07%)<br>TNS -123.916 ns (23.47%)<br>算法时间 8.18 s<br>进程时间 14.85 s<br>buffer 36 (+36) |
| stage_b_ysyx_25010008 | WNS -0.692032 ns<br>TNS -106.139 ns<br>时间 —<br>buffer 0 | WNS -0.615118 ns (11.11%)<br>TNS -97.915 ns (7.75%)<br>算法时间 6.41 s<br>进程时间 13.08 s<br>buffer 75 | WNS -0.690719 ns (0.19%)<br>TNS -100.228 ns (5.57%)<br>算法时间 1.02 s<br>进程时间 7.83 s<br>buffer 136 | WNS -0.564427 ns (18.44%)<br>TNS -99.047 ns (6.68%)<br>算法时间 7.18 s<br>进程时间 13.89 s<br>buffer 36 (-100) |
| stage_b_ysyx_25010030 | WNS -1.236192 ns<br>TNS -239.946 ns<br>时间 —<br>buffer 0 | WNS -0.924210 ns (25.24%)<br>TNS -196.719 ns (18.02%)<br>算法时间 7.10 s<br>进程时间 13.74 s<br>buffer 120 | WNS -1.168120 ns (5.51%)<br>TNS -232.812 ns (2.97%)<br>算法时间 1.01 s<br>进程时间 7.63 s<br>buffer 291 | WNS -0.986785 ns (20.18%)<br>TNS -196.462 ns (18.12%)<br>算法时间 7.29 s<br>进程时间 13.99 s<br>buffer 35 (-256) |
| stage_b_ysyx_25020037 | WNS -1.201967 ns<br>TNS -113.200 ns<br>时间 —<br>buffer 0 | WNS -1.012451 ns (15.77%)<br>TNS -86.868 ns (23.26%)<br>算法时间 6.78 s<br>进程时间 13.59 s<br>buffer 43 | WNS -1.053000 ns (12.39%)<br>TNS -91.004 ns (19.61%)<br>算法时间 1.12 s<br>进程时间 7.89 s<br>buffer 140 | WNS -0.959455 ns (20.18%)<br>TNS -89.086 ns (21.30%)<br>算法时间 6.98 s<br>进程时间 13.69 s<br>buffer 35 (-105) |
| stage_b_ysyx_25040129 | WNS -0.512748 ns<br>TNS -60.266 ns<br>时间 —<br>buffer 0 | WNS -0.405405 ns (20.93%)<br>TNS -54.081 ns (10.26%)<br>算法时间 19.66 s<br>进程时间 26.42 s<br>buffer 35 | WNS -0.428744 ns (16.38%)<br>TNS -54.049 ns (10.32%)<br>算法时间 0.84 s<br>进程时间 7.58 s<br>buffer 39 | WNS -0.299849 ns (41.52%)<br>TNS -41.602 ns (30.97%)<br>算法时间 9.47 s<br>进程时间 16.40 s<br>buffer 40 (+1) |
| stage_d_ysyx_24080018 | WNS -1.033713 ns<br>TNS -93.006 ns<br>时间 —<br>buffer 0 | WNS -0.906832 ns (12.27%)<br>TNS -83.185 ns (10.56%)<br>算法时间 8.26 s<br>进程时间 15.10 s<br>buffer 126 | WNS -1.020125 ns (1.31%)<br>TNS -90.612 ns (2.57%)<br>算法时间 1.12 s<br>进程时间 8.03 s<br>buffer 244 | WNS -0.860754 ns (16.73%)<br>TNS -78.279 ns (15.83%)<br>算法时间 7.78 s<br>进程时间 14.59 s<br>buffer 36 (-208) |
| stage_d_ysyx_24090003 | WNS -0.091004 ns<br>TNS -0.709 ns<br>时间 —<br>buffer 0 | WNS 0.000000 ns (100.00%)<br>TNS 0.000 ns (100.00%)<br>算法时间 2.79 s<br>进程时间 9.89 s<br>buffer 54 | WNS 0.000000 ns (100.00%)<br>TNS 0.000 ns (100.00%)<br>算法时间 1.02 s<br>进程时间 8.08 s<br>buffer 133 | WNS 0.000000 ns (100.00%)<br>TNS 0.000 ns (100.00%)<br>算法时间 1.90 s<br>进程时间 9.09 s<br>buffer 3 (-130) |
| stage_d_ysyx_24090010 | WNS -1.415648 ns<br>TNS -1037.568 ns<br>时间 —<br>buffer 0 | WNS -1.127627 ns (20.35%)<br>TNS -911.492 ns (12.15%)<br>算法时间 9.31 s<br>进程时间 16.05 s<br>buffer 59 | WNS -1.205115 ns (14.87%)<br>TNS -916.103 ns (11.71%)<br>算法时间 1.26 s<br>进程时间 7.99 s<br>buffer 613 | WNS -1.052890 ns (25.62%)<br>TNS -938.165 ns (9.58%)<br>算法时间 8.68 s<br>进程时间 15.60 s<br>buffer 32 (-581) |
| stage_d_ysyx_25010009 | WNS -1.072164 ns<br>TNS -992.766 ns<br>时间 —<br>buffer 0 | WNS -0.973409 ns (9.21%)<br>TNS -869.569 ns (12.41%)<br>算法时间 15.15 s<br>进程时间 21.81 s<br>buffer 84 | WNS -1.048628 ns (2.20%)<br>TNS -916.235 ns (7.71%)<br>算法时间 1.31 s<br>进程时间 8.23 s<br>buffer 691 | WNS -0.970899 ns (9.44%)<br>TNS -917.038 ns (7.63%)<br>算法时间 12.16 s<br>进程时间 19.21 s<br>buffer 33 (-658) |
| stage_d_ysyx_25020042 | WNS -0.287175 ns<br>TNS -10.891 ns<br>时间 —<br>buffer 0 | WNS -0.138914 ns (51.63%)<br>TNS -2.725 ns (74.98%)<br>算法时间 7.01 s<br>进程时间 13.60 s<br>buffer 94 | WNS -0.253648 ns (11.67%)<br>TNS -6.295 ns (42.20%)<br>算法时间 0.83 s<br>进程时间 7.39 s<br>buffer 318 | WNS -0.012768 ns (95.55%)<br>TNS -0.017 ns (99.84%)<br>算法时间 24.40 s<br>进程时间 31.08 s<br>buffer 91 (-227) |
| stage_d_ysyx_25070194 | WNS -0.615241 ns<br>TNS -267.880 ns<br>时间 —<br>buffer 0 | WNS -0.500538 ns (18.64%)<br>TNS -243.664 ns (9.04%)<br>算法时间 6.46 s<br>进程时间 13.19 s<br>buffer 60 | WNS -0.573901 ns (6.72%)<br>TNS -249.818 ns (6.74%)<br>算法时间 0.93 s<br>进程时间 7.68 s<br>buffer 220 | WNS -0.615241 ns (0.00%)<br>TNS -267.880 ns (0.00%)<br>算法时间 10.05 s<br>进程时间 16.45 s<br>buffer 0 (-220) |
| stage_d_ysyx_25080201 | WNS -0.927642 ns<br>TNS -451.112 ns<br>时间 —<br>buffer 0 | WNS -0.799875 ns (13.77%)<br>TNS -417.293 ns (7.50%)<br>算法时间 4.64 s<br>进程时间 10.99 s<br>buffer 80 | WNS -0.909685 ns (1.94%)<br>TNS -434.512 ns (3.68%)<br>算法时间 0.70 s<br>进程时间 7.03 s<br>buffer 208 | WNS -0.787779 ns (15.08%)<br>TNS -425.581 ns (5.66%)<br>算法时间 6.40 s<br>进程时间 12.84 s<br>buffer 35 (-173) |
| tpu | WNS -1.145222 ns<br>TNS -539.243 ns<br>时间 —<br>buffer 0 | WNS -0.982697 ns (14.19%)<br>TNS -494.990 ns (8.21%)<br>算法时间 79.30 s<br>进程时间 93.17 s<br>buffer 133 | WNS -1.001185 ns (12.58%)<br>TNS -484.436 ns (10.16%)<br>算法时间 5.83 s<br>进程时间 18.91 s<br>buffer 1401 | WNS -0.891043 ns (22.19%)<br>TNS -493.414 ns (8.50%)<br>算法时间 28.41 s<br>进程时间 41.95 s<br>buffer 20 (-1381) |
| trng | WNS -0.368908 ns<br>TNS -57.647 ns<br>时间 —<br>buffer 0 | WNS -0.226837 ns (38.51%)<br>TNS -14.159 ns (75.44%)<br>算法时间 48.03 s<br>进程时间 56.28 s<br>buffer 220 | WNS -0.333971 ns (9.47%)<br>TNS -34.298 ns (40.50%)<br>算法时间 2.37 s<br>进程时间 10.54 s<br>buffer 1116 | WNS -0.210881 ns (42.84%)<br>TNS -25.106 ns (56.45%)<br>算法时间 18.65 s<br>进程时间 26.82 s<br>buffer 7 (-1109) |
| y_huff | WNS -0.115413 ns<br>TNS -10.845 ns<br>时间 —<br>buffer 0 | WNS -0.074556 ns (35.40%)<br>TNS -8.004 ns (26.19%)<br>算法时间 26.06 s<br>进程时间 33.25 s<br>buffer 9 | WNS -0.115413 ns (0.00%)<br>TNS -8.414 ns (22.41%)<br>算法时间 1.14 s<br>进程时间 7.98 s<br>buffer 35 | WNS -0.038505 ns (66.64%)<br>TNS -5.610 ns (48.27%)<br>算法时间 10.76 s<br>进程时间 17.73 s<br>buffer 35 (+0) |
| ysyx_210134 | WNS -3.570492 ns<br>TNS -2347.400 ns<br>时间 —<br>buffer 0 | WNS -3.188167 ns (10.71%)<br>TNS -1801.113 ns (23.27%)<br>算法时间 93.20 s<br>进程时间 107.69 s<br>buffer 1054 | WNS -2.994813 ns (16.12%)<br>TNS -1836.906 ns (21.75%)<br>算法时间 8.96 s<br>进程时间 23.08 s<br>buffer 5073 | WNS -2.915434 ns (18.35%)<br>TNS -2083.095 ns (11.26%)<br>算法时间 30.79 s<br>进程时间 44.74 s<br>buffer 36 (-5037) |

<!-- analysis:ecc_pr329_1011 -->
#### 数据结论

本批 36 个设计中，AES 初始 setup 已无违例；下表时序改善中位数统计其余 35 个设计，其他中位数统计全部 36 个。最优案例数采用独占最优统计，并列不计入某一种算法。

| 算法 | WNS / TNS 改善中位数 | 新增 buffer 中位数 | 面积增长中位数 | 算法时间中位数 | 电气清零 |
|---|---:|---:|---:|---:|---:|
| drv+setup | 14.19% / 12.15% | 57 | 0.91% | 16.58 s | 15/36 |
| VG | 2.37% / 7.70% | 235 | 3.67% | 1.38 s | 19/36 |
| GOBI | 2.41% / 6.22% | 384 | 4.86% | 1.44 s | 20/36 |
| Path | 16.73% / 9.29% | 32 | 0.24% | 9.76 s | 1/36 |

- **Path 的优势集中在 WNS 和低修改成本。** 23/36 个设计独占最优 WNS，总新增 buffer 为 927 个，对比 VG 的 40487 个减少 97.71%；但只有 10/36 个设计独占最优 TNS，少于 drv+setup 的 18/36。少量关键路径修复尚不能替代对更多违例端点的处理。
- **drv+setup 的改善覆盖更稳定。** 35 个初始 setup 违例设计均改善，Path 为 29 个。Path 有 7 例最终指标与初始一致：AES 本来已满足 setup，另外 6 例因最终合法化后的 DRV 严重度检查失败而整阶段回滚。这是可从日志直接定位的收益损失，值得优先改进最终布局可接受性。
- **VG/GOBI 更适合本批电气修复，但 GOBI 没有普遍加速。** 二者分别有 19/36、20/36 个设计电气清零；VG 在 26/36 个设计上求解更快，求解时间中位数为 0.068 s，GOBI 为 0.166 s。GOBI 在 31/36 个设计中插入更多 buffer；两者又各有 5 个设计因整批方案未通过验收而回滚，快速求解不等于必有收益。
- **PPA 代价以 Path 最小。** Path 面积增长中位数 0.24%，估算总功耗增长中位数约 0.004%；drv+setup、VG、GOBI 的功耗增长中位数分别为 0.30%、1.06%、1.57%。Path 算法时间仍是 VG 的配对中位 6.43 倍，节省面积并非免费。
- **新增 setup 清零案例仍少。** drv+setup 新清零 1 例，VG、GOBI、Path 各 2 例；其余结果不能按“评测通过”当作时序收敛。
<!-- /analysis:ecc_pr329_1011 -->

### ecc_pr342_1018

| 设计 | 初始输入 | drv+setup | VG | Path |
|---|---|---|---|---|
| BM64 | WNS -1.212289 ns<br>TNS -426.472 ns<br>时间 —<br>buffer 0 | WNS -1.165004 ns (3.90%)<br>TNS -410.777 ns (3.68%)<br>算法时间 26.65 s<br>进程时间 33.75 s<br>buffer 58 | WNS -1.199212 ns (1.08%)<br>TNS -418.603 ns (1.85%)<br>算法时间 1.22 s<br>进程时间 8.43 s<br>buffer 119 | WNS -1.129085 ns (6.86%)<br>TNS -408.363 ns (4.25%)<br>算法时间 9.81 s<br>进程时间 16.67 s<br>buffer 23 (-96) |
| PPU | WNS -0.682617 ns<br>TNS -58.788 ns<br>时间 —<br>buffer 0 | WNS -0.598073 ns (12.39%)<br>TNS -31.047 ns (47.19%)<br>算法时间 14.37 s<br>进程时间 21.10 s<br>buffer 120 | WNS -0.622461 ns (8.81%)<br>TNS -32.298 ns (45.06%)<br>算法时间 1.16 s<br>进程时间 7.78 s<br>buffer 321 | WNS -0.475558 ns (30.33%)<br>TNS -51.717 ns (12.03%)<br>算法时间 5.99 s<br>进程时间 12.69 s<br>buffer 37 (-284) |
| aes | WNS -0.106772 ns<br>TNS -0.222 ns<br>时间 —<br>buffer 0 | WNS 0.000000 ns (100.00%)<br>TNS 0.000 ns (100.00%)<br>算法时间 4.47 s<br>进程时间 11.81 s<br>buffer 4 | WNS -0.106772 ns (0.00%)<br>TNS -0.222 ns (0.00%)<br>算法时间 1.90 s<br>进程时间 9.13 s<br>buffer 0 | WNS 0.000000 ns (100.00%)<br>TNS 0.000 ns (100.00%)<br>算法时间 1.56 s<br>进程时间 8.93 s<br>buffer 2 (+2) |
| arm9 | WNS -2.215335 ns<br>TNS -401.002 ns<br>时间 —<br>buffer 0 | WNS -2.126414 ns (4.01%)<br>TNS -360.320 ns (10.14%)<br>算法时间 25.64 s<br>进程时间 34.70 s<br>buffer 99 | WNS -2.193477 ns (0.99%)<br>TNS -367.056 ns (8.47%)<br>算法时间 2.40 s<br>进程时间 10.98 s<br>buffer 661 | WNS -2.081434 ns (6.04%)<br>TNS -393.225 ns (1.94%)<br>算法时间 17.44 s<br>进程时间 26.52 s<br>buffer 29 (-632) |
| blake2s | WNS -2.308432 ns<br>TNS -697.683 ns<br>时间 —<br>buffer 0 | WNS -2.220567 ns (3.81%)<br>TNS -660.652 ns (5.31%)<br>算法时间 15.97 s<br>进程时间 24.41 s<br>buffer 288 | WNS -2.238008 ns (3.05%)<br>TNS -662.197 ns (5.09%)<br>算法时间 2.84 s<br>进程时间 11.19 s<br>buffer 1185 | WNS -2.238659 ns (3.02%)<br>TNS -678.677 ns (2.72%)<br>算法时间 15.96 s<br>进程时间 24.08 s<br>buffer 32 (-1153) |
| double_fpu | WNS -1.721917 ns<br>TNS -367.107 ns<br>时间 —<br>buffer 0 | WNS -1.676349 ns (2.65%)<br>TNS -344.771 ns (6.08%)<br>算法时间 48.58 s<br>进程时间 60.39 s<br>buffer 356 | WNS -1.720513 ns (0.08%)<br>TNS -347.454 ns (5.35%)<br>算法时间 4.95 s<br>进程时间 16.89 s<br>buffer 853 | WNS -1.529508 ns (11.17%)<br>TNS -361.301 ns (1.58%)<br>算法时间 20.47 s<br>进程时间 32.16 s<br>buffer 30 (-823) |
| eth_top | WNS -2.049052 ns<br>TNS -233.401 ns<br>时间 —<br>buffer 0 | WNS -1.693991 ns (17.33%)<br>TNS -182.906 ns (21.63%)<br>算法时间 55.81 s<br>进程时间 65.49 s<br>buffer 531 | WNS -1.573869 ns (23.19%)<br>TNS -170.850 ns (26.80%)<br>算法时间 3.81 s<br>进程时间 13.23 s<br>buffer 1654 | WNS -1.805507 ns (11.89%)<br>TNS -209.177 ns (10.38%)<br>算法时间 36.16 s<br>进程时间 45.94 s<br>buffer 15 (-1639) |
| fft | WNS -1.193399 ns<br>TNS -67.689 ns<br>时间 —<br>buffer 0 | WNS -1.168043 ns (2.12%)<br>TNS -63.766 ns (5.80%)<br>算法时间 16.48 s<br>进程时间 25.22 s<br>buffer 45 | WNS -1.188034 ns (0.45%)<br>TNS -64.131 ns (5.26%)<br>算法时间 1.91 s<br>进程时间 10.19 s<br>buffer 91 | WNS -1.159366 ns (2.85%)<br>TNS -65.755 ns (2.86%)<br>算法时间 14.41 s<br>进程时间 22.85 s<br>buffer 31 (-60) |
| picorv32a | WNS -1.735914 ns<br>TNS -92.273 ns<br>时间 —<br>buffer 0 | WNS -1.685998 ns (2.88%)<br>TNS -90.854 ns (1.54%)<br>算法时间 11.01 s<br>进程时间 18.50 s<br>buffer 39 | WNS -1.735914 ns (0.00%)<br>TNS -92.273 ns (0.00%)<br>算法时间 2.27 s<br>进程时间 9.84 s<br>buffer 0 | WNS -1.636272 ns (5.74%)<br>TNS -90.420 ns (2.01%)<br>算法时间 9.66 s<br>进程时间 17.10 s<br>buffer 29 (+29) |
| poly1305 | WNS -2.124006 ns<br>TNS -317.149 ns<br>时间 —<br>buffer 0 | WNS -2.028361 ns (4.50%)<br>TNS -298.566 ns (5.86%)<br>算法时间 25.74 s<br>进程时间 37.01 s<br>buffer 247 | WNS -2.094744 ns (1.38%)<br>TNS -305.133 ns (3.79%)<br>算法时间 3.86 s<br>进程时间 15.01 s<br>buffer 686 | WNS -2.034101 ns (4.23%)<br>TNS -311.757 ns (1.70%)<br>算法时间 23.68 s<br>进程时间 34.63 s<br>buffer 33 (-653) |
| salsa20 | WNS -1.844546 ns<br>TNS -250.162 ns<br>时间 —<br>buffer 0 | WNS -1.595180 ns (13.52%)<br>TNS -215.742 ns (13.76%)<br>算法时间 24.42 s<br>进程时间 33.14 s<br>buffer 178 | WNS -1.759951 ns (4.59%)<br>TNS -230.057 ns (8.04%)<br>算法时间 2.57 s<br>进程时间 11.23 s<br>buffer 1186 | WNS -1.495511 ns (18.92%)<br>TNS -217.627 ns (13.01%)<br>算法时间 15.30 s<br>进程时间 24.09 s<br>buffer 34 (-1152) |
| sha256 | WNS -1.702695 ns<br>TNS -96.026 ns<br>时间 —<br>buffer 0 | WNS -1.603871 ns (5.80%)<br>TNS -76.725 ns (20.10%)<br>算法时间 16.15 s<br>进程时间 23.30 s<br>buffer 236 | WNS -1.702695 ns (0.00%)<br>TNS -96.026 ns (0.00%)<br>算法时间 2.54 s<br>进程时间 9.56 s<br>buffer 0 | WNS -1.548450 ns (9.06%)<br>TNS -80.470 ns (16.20%)<br>算法时间 10.43 s<br>进程时间 17.39 s<br>buffer 36 (+36) |
| stage_b_ysyx_23060170 | WNS -0.815100 ns<br>TNS -179.809 ns<br>时间 —<br>buffer 0 | WNS -0.789446 ns (3.15%)<br>TNS -132.637 ns (26.23%)<br>算法时间 18.11 s<br>进程时间 24.65 s<br>buffer 33 | WNS -0.815100 ns (0.00%)<br>TNS -132.727 ns (26.18%)<br>算法时间 0.89 s<br>进程时间 7.50 s<br>buffer 130 | WNS -0.738862 ns (9.35%)<br>TNS -177.920 ns (1.05%)<br>算法时间 6.15 s<br>进程时间 12.74 s<br>buffer 32 (-98) |
| stage_b_ysyx_23060203 | WNS -0.326484 ns<br>TNS -91.128 ns<br>时间 —<br>buffer 0 | WNS -0.285727 ns (12.48%)<br>TNS -52.177 ns (42.74%)<br>算法时间 6.49 s<br>进程时间 13.54 s<br>buffer 51 | WNS -0.326484 ns (0.00%)<br>TNS -91.128 ns (0.00%)<br>算法时间 1.58 s<br>进程时间 8.37 s<br>buffer 0 | WNS -0.283709 ns (13.10%)<br>TNS -81.112 ns (10.99%)<br>算法时间 6.73 s<br>进程时间 13.36 s<br>buffer 32 (+32) |
| stage_b_ysyx_23060229 | WNS -0.886317 ns<br>TNS -95.537 ns<br>时间 —<br>buffer 0 | WNS -0.715509 ns (19.27%)<br>TNS -84.510 ns (11.54%)<br>算法时间 10.88 s<br>进程时间 17.59 s<br>buffer 52 | WNS -0.742376 ns (16.24%)<br>TNS -84.858 ns (11.18%)<br>算法时间 0.95 s<br>进程时间 7.63 s<br>buffer 223 | WNS -0.690086 ns (22.14%)<br>TNS -90.305 ns (5.48%)<br>算法时间 8.06 s<br>进程时间 14.81 s<br>buffer 35 (-188) |
| stage_b_ysyx_23060246 | WNS -0.684805 ns<br>TNS -94.888 ns<br>时间 —<br>buffer 0 | WNS -0.651769 ns (4.82%)<br>TNS -91.642 ns (3.42%)<br>算法时间 8.54 s<br>进程时间 15.31 s<br>buffer 6 | WNS -0.684790 ns (0.00%)<br>TNS -91.427 ns (3.65%)<br>算法时间 0.97 s<br>进程时间 7.73 s<br>buffer 98 | WNS -0.684805 ns (0.00%)<br>TNS -94.888 ns (0.00%)<br>算法时间 7.11 s<br>进程时间 13.94 s<br>buffer 0 (-98) |
| stage_b_ysyx_24070003 | WNS -1.365867 ns<br>TNS -280.632 ns<br>时间 —<br>buffer 0 | WNS -1.261468 ns (7.64%)<br>TNS -245.981 ns (12.35%)<br>算法时间 12.35 s<br>进程时间 18.92 s<br>buffer 60 | WNS -1.364392 ns (0.11%)<br>TNS -277.783 ns (1.02%)<br>算法时间 0.91 s<br>进程时间 7.48 s<br>buffer 120 | WNS -1.226435 ns (10.21%)<br>TNS -238.559 ns (14.99%)<br>算法时间 8.16 s<br>进程时间 15.10 s<br>buffer 35 (-85) |
| stage_b_ysyx_24100012 | WNS -0.204991 ns<br>TNS -12.074 ns<br>时间 —<br>buffer 0 | WNS -0.166010 ns (19.02%)<br>TNS -8.498 ns (29.62%)<br>算法时间 12.05 s<br>进程时间 18.70 s<br>buffer 36 | WNS -0.194781 ns (4.98%)<br>TNS -9.214 ns (23.69%)<br>算法时间 0.87 s<br>进程时间 7.43 s<br>buffer 92 | WNS -0.131646 ns (35.78%)<br>TNS -8.874 ns (26.51%)<br>算法时间 9.19 s<br>进程时间 15.69 s<br>buffer 41 (-51) |
| stage_b_ysyx_24110017 | WNS -0.857841 ns<br>TNS -160.122 ns<br>时间 —<br>buffer 0 | WNS -0.549972 ns (35.89%)<br>TNS -137.781 ns (13.95%)<br>算法时间 8.43 s<br>进程时间 15.00 s<br>buffer 14 | WNS -0.647265 ns (24.55%)<br>TNS -152.300 ns (4.89%)<br>算法时间 0.78 s<br>进程时间 7.28 s<br>buffer 49 | WNS -0.508393 ns (40.74%)<br>TNS -133.554 ns (16.59%)<br>算法时间 7.59 s<br>进程时间 14.14 s<br>buffer 39 (-10) |
| stage_b_ysyx_25010008 | WNS -0.724143 ns<br>TNS -104.779 ns<br>时间 —<br>buffer 0 | WNS -0.626524 ns (13.48%)<br>TNS -94.135 ns (10.16%)<br>算法时间 6.02 s<br>进程时间 12.74 s<br>buffer 74 | WNS -0.724143 ns (0.00%)<br>TNS -104.779 ns (0.00%)<br>算法时间 1.56 s<br>进程时间 8.28 s<br>buffer 0 | WNS -0.576115 ns (20.44%)<br>TNS -95.234 ns (9.11%)<br>算法时间 7.29 s<br>进程时间 14.04 s<br>buffer 36 (+36) |
| stage_b_ysyx_25010030 | WNS -1.200231 ns<br>TNS -236.482 ns<br>时间 —<br>buffer 0 | WNS -1.113817 ns (7.20%)<br>TNS -223.803 ns (5.36%)<br>算法时间 8.39 s<br>进程时间 15.90 s<br>buffer 242 | WNS -1.176125 ns (2.01%)<br>TNS -229.191 ns (3.08%)<br>算法时间 0.97 s<br>进程时间 7.59 s<br>buffer 303 | WNS -1.010479 ns (15.81%)<br>TNS -196.064 ns (17.09%)<br>算法时间 6.47 s<br>进程时间 13.04 s<br>buffer 31 (-272) |
| stage_b_ysyx_25020037 | WNS -1.198730 ns<br>TNS -103.412 ns<br>时间 —<br>buffer 0 | WNS -1.047061 ns (12.65%)<br>TNS -84.988 ns (17.82%)<br>算法时间 7.42 s<br>进程时间 14.09 s<br>buffer 43 | WNS -1.114310 ns (7.04%)<br>TNS -89.944 ns (13.02%)<br>算法时间 0.91 s<br>进程时间 7.53 s<br>buffer 112 | WNS -0.992986 ns (17.16%)<br>TNS -83.059 ns (19.68%)<br>算法时间 6.04 s<br>进程时间 12.74 s<br>buffer 36 (-76) |
| stage_b_ysyx_25040129 | WNS -0.480924 ns<br>TNS -60.626 ns<br>时间 —<br>buffer 0 | WNS -0.397580 ns (17.33%)<br>TNS -54.943 ns (9.37%)<br>算法时间 17.58 s<br>进程时间 24.13 s<br>buffer 31 | WNS -0.415628 ns (13.58%)<br>TNS -56.176 ns (7.34%)<br>算法时间 0.80 s<br>进程时间 7.48 s<br>buffer 42 | WNS -0.316107 ns (34.27%)<br>TNS -42.337 ns (30.17%)<br>算法时间 8.61 s<br>进程时间 15.20 s<br>buffer 43 (+1) |
| stage_d_ysyx_24080018 | WNS -1.014504 ns<br>TNS -97.155 ns<br>时间 —<br>buffer 0 | WNS -0.898992 ns (11.39%)<br>TNS -88.766 ns (8.64%)<br>算法时间 22.03 s<br>进程时间 28.85 s<br>buffer 134 | WNS -0.981062 ns (3.30%)<br>TNS -94.343 ns (2.89%)<br>算法时间 4.50 s<br>进程时间 19.86 s<br>buffer 240 | WNS -0.874882 ns (13.76%)<br>TNS -86.176 ns (11.30%)<br>算法时间 7.70 s<br>进程时间 14.66 s<br>buffer 36 (-204) |
| stage_d_ysyx_24090003 | WNS 0.000000 ns<br>TNS 0.000 ns<br>时间 —<br>buffer 0 | WNS 0.000000 ns (—)<br>TNS 0.000 ns (—)<br>算法时间 3.66 s<br>进程时间 10.74 s<br>buffer 44 | WNS 0.000000 ns (—)<br>TNS 0.000 ns (—)<br>算法时间 1.05 s<br>进程时间 8.19 s<br>buffer 140 | WNS 0.000000 ns (—)<br>TNS 0.000 ns (—)<br>算法时间 1.02 s<br>进程时间 7.98 s<br>buffer 0 (-140) |
| stage_d_ysyx_24090010 | WNS -1.349715 ns<br>TNS -994.511 ns<br>时间 —<br>buffer 0 | WNS -1.117931 ns (17.17%)<br>TNS -934.100 ns (6.07%)<br>算法时间 6.84 s<br>进程时间 13.69 s<br>buffer 52 | WNS -1.224880 ns (9.25%)<br>TNS -945.988 ns (4.88%)<br>算法时间 1.20 s<br>进程时间 7.93 s<br>buffer 621 | WNS -0.979772 ns (27.41%)<br>TNS -897.037 ns (9.80%)<br>算法时间 11.25 s<br>进程时间 18.25 s<br>buffer 39 (-582) |
| stage_d_ysyx_25010009 | WNS -1.135752 ns<br>TNS -924.755 ns<br>时间 —<br>buffer 0 | WNS -0.921999 ns (18.82%)<br>TNS -830.604 ns (10.18%)<br>算法时间 8.54 s<br>进程时间 15.76 s<br>buffer 103 | WNS -1.030859 ns (9.24%)<br>TNS -863.566 ns (6.62%)<br>算法时间 1.35 s<br>进程时间 8.13 s<br>buffer 673 | WNS -0.921936 ns (18.83%)<br>TNS -877.655 ns (5.09%)<br>算法时间 11.77 s<br>进程时间 18.46 s<br>buffer 18 (-655) |
| stage_d_ysyx_25020042 | WNS -0.314875 ns<br>TNS -11.140 ns<br>时间 —<br>buffer 0 | WNS -0.137543 ns (56.32%)<br>TNS -4.297 ns (61.43%)<br>算法时间 6.50 s<br>进程时间 12.99 s<br>buffer 92 | WNS -0.224450 ns (28.72%)<br>TNS -6.561 ns (41.10%)<br>算法时间 0.82 s<br>进程时间 7.23 s<br>buffer 323 | WNS -0.002258 ns (99.28%)<br>TNS -0.004 ns (99.96%)<br>算法时间 24.86 s<br>进程时间 31.28 s<br>buffer 98 (-225) |
| stage_d_ysyx_25070194 | WNS -0.624030 ns<br>TNS -273.115 ns<br>时间 —<br>buffer 0 | WNS -0.508564 ns (18.50%)<br>TNS -246.138 ns (9.88%)<br>算法时间 17.17 s<br>进程时间 23.86 s<br>buffer 60 | WNS -0.610041 ns (2.24%)<br>TNS -253.350 ns (7.24%)<br>算法时间 0.78 s<br>进程时间 7.18 s<br>buffer 223 | WNS -0.624030 ns (0.00%)<br>TNS -273.115 ns (0.00%)<br>算法时间 9.84 s<br>进程时间 16.27 s<br>buffer 0 (-223) |
| stage_d_ysyx_25080201 | WNS -0.904166 ns<br>TNS -467.397 ns<br>时间 —<br>buffer 0 | WNS -0.819600 ns (9.35%)<br>TNS -433.355 ns (7.28%)<br>算法时间 7.79 s<br>进程时间 14.14 s<br>buffer 151 | WNS -0.903603 ns (0.06%)<br>TNS -450.249 ns (3.67%)<br>算法时间 0.82 s<br>进程时间 7.33 s<br>buffer 211 | WNS -0.835584 ns (7.59%)<br>TNS -451.443 ns (3.41%)<br>算法时间 9.01 s<br>进程时间 15.45 s<br>buffer 20 (-191) |
| trng | WNS -0.215793 ns<br>TNS -24.729 ns<br>时间 —<br>buffer 0 | WNS -0.151413 ns (29.83%)<br>TNS -16.961 ns (31.41%)<br>算法时间 48.06 s<br>进程时间 56.12 s<br>buffer 223 | WNS -0.187964 ns (12.90%)<br>TNS -17.626 ns (28.73%)<br>算法时间 2.36 s<br>进程时间 10.19 s<br>buffer 1027 | WNS -0.012155 ns (94.37%)<br>TNS -0.012 ns (99.95%)<br>算法时间 45.47 s<br>进程时间 53.44 s<br>buffer 116 (-911) |
| y_huff | WNS -0.108522 ns<br>TNS -9.907 ns<br>时间 —<br>buffer 0 | WNS -0.093892 ns (13.48%)<br>TNS -8.466 ns (14.54%)<br>算法时间 25.09 s<br>进程时间 32.12 s<br>buffer 4 | WNS -0.108522 ns (0.00%)<br>TNS -9.907 ns (0.00%)<br>算法时间 1.81 s<br>进程时间 8.73 s<br>buffer 0 | WNS -0.048121 ns (55.66%)<br>TNS -5.022 ns (49.31%)<br>算法时间 10.47 s<br>进程时间 17.46 s<br>buffer 34 (+34) |

<!-- analysis:ecc_pr342_1018 -->
#### 数据结论

本批 32 个设计中，stage_d_ysyx_24090003 初始 setup 已无违例；时序改善中位数统计其余 31 个设计，其他中位数统计全部 32 个。两批有 32 个同名设计，但 DEF/Verilog/SDC 输入组合均不同，因此分别报告，不视为 68 个互相独立的设计样本。

| 算法 | WNS / TNS 改善中位数 | 新增 buffer 中位数 | 面积增长中位数 | 算法时间中位数 | 电气清零 |
|---|---:|---:|---:|---:|---:|
| drv+setup | 12.48% / 10.18% | 60 | 1.13% | 13.36 s | 13/32 |
| VG | 2.01% / 5.09% | 175.5 | 2.85% | 1.28 s | 20/32 |
| GOBI | 1.49% / 5.03% | 357.5 | 4.84% | 1.28 s | 20/32 |
| Path | 13.76% / 10.38% | 32.5 | 0.36% | 9.42 s | 0/32 |

- **Path 的 WNS 优势再次出现，但 TNS 优势并不全面。** 24/32 个设计独占最优 WNS，12/32 个独占最优 TNS；drv+setup 则有 15/32 个独占最优 TNS。Path 总新增 1052 个 buffer，相比 VG 的 11383 个减少 90.76%，面积增长中位数只有 0.36%。
- **drv+setup 对总违例负担的控制更均衡。** 31 个初始 setup 违例设计均改善；Path 改善 29 个，其中两个设计最终因 DRV 严重度检查失败而回滚。虽然 VG/GOBI 电气清零设计更多，本批 drv+setup 的残余 slew/cap 总数为 833/193，低于 VG 的 7802/540、GOBI 的 3119/321。因此“多少设计清零”和“总残余违例”需要同时看。
- **VG 与 GOBI 的算法阶段时间相近，VG 的修改规模更小。** VG 在 26/32 个设计上求解更快，求解时间中位数为 0.064 s，GOBI 为 0.154 s；GOBI 在 26/32 个设计中插入更多 buffer，两者各有 6 例整批方案回滚。本批数据不支持默认优先使用 GOBI。
- **Path 不能承担完整 DRV 修复职责。** 32 个最终结果均仍有电气违例；它的功耗增长中位数约 0.009%，但算法时间是 VG 的配对中位 7.08 倍。低面积、低功耗扰动的收益与搜索时间、残余违例之间仍需权衡。
- **新增 setup 清零只有 AES。** drv+setup 与 Path 都将其清零，VG/GOBI 没有新增清零案例；多数设计仍需后续修复。
<!-- /analysis:ecc_pr342_1018 -->

原始记录：[ecc-ics55-benchmark-20261125](../results/ecc-ics55-benchmark-20261125/)。GOBI 的完整指标见结果目录的 `tables.md`。

复现命令：

```sh
python3 scripts/benchmark_ecc.py --output results/ecc-ics55-benchmark-20261125 --update-docs
```
<!-- /ecc-ics55-benchmark -->

## PBBI / ISPD26 AES、JPEG（2026-11-25）

当前 PBBI 使用文献 [6] 的 BELT 解析预测、OpenSTA 全网 AT/RAT，以及 [16] 的 PSP、SqP、300 项库查找。已删除固定 128 状态截断；分叉采用流式精确支配合并，未增加近似规则。预缓冲电阻倍率为 1（PSP），固定门尺寸、最多 64 条 setup 路径，逐路径真实增量 RC/STA 验收，阶段末统一合法化与完整验证。预测仍有负 slack 时，以初始预测 WNS 平移规划 RAT；这仍是工程适配，不等于论文 §6 先确定最小可达延迟的实验步骤。

本节只新运行 PBBI；初始、VG、Path 直接引用 `ispd26-benchmark-20261125`。两项 PBBI 使用相同最终二进制、32 线程、原始输入与 dont-use；源码尚未提交。公式对应、已实现内容和缺项见[算法说明](buffer-algorithms.md)。

| 设计 | 初始输入 | VG | Path | PBBI |
|---|---|---|---|---|
| AES | WNS -1.025906 ns<br>TNS -278.964 ns<br>算法时间 —<br>进程时间 —<br>buffer 0 | WNS -0.120080 ns (88.30%)<br>TNS -35.153 ns (87.40%)<br>算法时间 1.832 s<br>进程时间 7.11 s<br>buffer 283 | WNS -0.097584 ns (90.49%)<br>TNS -19.597 ns (92.98%)<br>算法时间 8.905 s<br>进程时间 14.42 s<br>buffer 26 | WNS -0.170687 ns (83.36%)<br>TNS -45.817 ns (83.58%)<br>算法时间 825.602 s<br>进程时间 830.77 s<br>buffer 189 |
| JPEG | WNS -9.348061 ns<br>TNS -28185.798 ns<br>算法时间 —<br>进程时间 —<br>buffer 0 | WNS -0.276766 ns (97.04%)<br>TNS -418.623 ns (98.51%)<br>算法时间 44.769 s<br>进程时间 55.76 s<br>buffer 12677 | WNS -0.220082 ns (97.65%)<br>TNS -353.323 ns (98.75%)<br>算法时间 58.174 s<br>进程时间 67.74 s<br>buffer 251 | WNS -0.379284 ns (95.94%)<br>TNS -841.941 ns (97.01%)<br>算法时间 212.173 s<br>进程时间 221.29 s<br>buffer 523 |

### 数据结论

- **公式与前沿实现通过验证，但整体工程收益尚未达标。** 两设计均通过独立重载、供电及布局检查，hold WNS/TNS 为 0。当前 PBBI 的时序、数量和算法时间均未优于既有 Path；不能把更贴合论文直接解释为 PPA 改善。
- **AES 相对旧 PBBI：** WNS/TNS 从 -0.285239/-65.461 ns 变为 -0.170687/-45.817 ns；buffer 从 18 变为 189，算法时间从 3.475 秒变为 825.602 秒。slew/cap/fanout 为 0/0/0。
- **JPEG 相对旧 PBBI：** WNS/TNS 从 -0.801157/-1566.211 ns 变为 -0.379284/-841.941 ns；buffer 从 686 变为 523，算法时间从 65.614 秒变为 212.173 秒。slew/cap/fanout 为 141/0/3。
- **AES 的主要问题是长尾与过于乐观的规划目标。** 初始 BELT 预测 WNS 为 -0.067015 ns。第 4 次试验接受后只有 14 个 buffer，增量 WNS/TNS 已达到 -0.169916/-47.952941 ns；随后 buffer 大幅增长，合法化前 WNS 不变。应先对齐论文的可达目标测定，而不是再次增加收益门槛、放松预测或限制候选数量。
- **JPEG 的数量下降不等于面积下降。** 当前面积约 4799.605 µm²，旧版约 4796.368 µm²；buffer 型号不同，数量下降约 24% 而面积略增。本文 buffer-only 实验以数量为成本，不应改称面积最优。
- **前沿不再受到固定数量截断。** JPEG 前沿峰值为 31931，PSP/SqP 分别移除 2047311/116885 个状态。索引和流式合并仅改变精确支配检查及中间集合存储；全枚举 Pareto 和笛卡尔积对照通过。`solve_seconds` 包含规划中的预测 STA 更新与 DP，不能当作纯 DP 内核时间。

| 设计 | PBBI 面积（µm²） | 相对初始面积 | PBBI 总功耗（mW） | 相对初始总功耗 |
|---|---:|---:|---:|---:|
| AES | 1326.430 | +2.108% | 68.105 | -0.553% |
| JPEG | 4799.605 | +2.404% | 112.512 | -42.514% |

测试：12 项 Sizer CTest 全部通过，覆盖论文 437.0 ps 障碍算例、阻塞旁支、分段不变性、短障碍、PSP/SqP、负载相关库查找、无候选数量上限、精确支配与全枚举一致性，以及 STA annotation 恢复和整阶段回滚。修改文件 clang-tidy 无项目告警。功耗采用相同 STA 活动率假设；耗时为共享机器单次运行。

原始记录：[AES](../results/ispd26-pbbi-paper-benchmark-20261125/)、[JPEG](../results/ispd26-pbbi-paper-jpeg-benchmark-20261125/)，验证日志位于 `validation/`。中间失败和复测不保留；[旧 PBBI](../results/ispd26-pbbi-benchmark-20261125/) 暂留作收益对照，等待确认后再决定清理。传统对照来源：[原有 benchmark](../results/ispd26-benchmark-20261125/)。

```sh
python3 scripts/benchmark_ispd26.py --output results/ispd26-pbbi-paper-benchmark-20261125 \
  --designs aes_cipher_top --profiles pbbi --threads 32 \
  --dont-use 'BUFx*_L' --dont-use 'BUFx*_SL' --dont-use 'HB*_L' --dont-use 'HB*_SL'
python3 scripts/benchmark_ispd26.py --output results/ispd26-pbbi-paper-jpeg-benchmark-20261125 \
  --designs jpeg_encoder --profiles pbbi --threads 32 \
  --dont-use 'BUFx*_L' --dont-use 'BUFx*_SL' --dont-use 'HB*_L' --dont-use 'HB*_SL'
```

## 算法选择与后续重点

| 目标 | 本轮数据支持的选择 | 主要限制 |
|---|---|---|
| 改善最差路径，同时控制 buffer 和面积 | 优先评估 Path | 对 TNS/DRV 的覆盖有限；最终合法化可能撤销全部收益 |
| 同时处理电气违例和较广范围的 setup 违例 | drv+setup 作为常规参照 | 单元试探与 STA/合法化成本较高，不保证最快或清零 |
| 快速处理较小规模的电气修复 | 优先评估 VG CPU | 整网、整批树方案可能插入过多或整体回滚 |
| 大规模多网缓冲树求解 | 评估 GOBI CUDA | 优势依赖规模；ICS55 上未呈现普遍加速，buffer/面积常高于 VG |

这些是当前配置下的观察，不是算法在所有工艺上的固定排名。ICS55 统一限制为工艺指定的四种 buffer，ISPD26 使用不同的候选集；输入违例分布、工艺及 RC 模型也不同，不能只用两组结果的差异归因于工艺或 GPU。功耗来自当前 STA 活动率假设下的估算；耗时来自共享主机单次运行，不能据小幅时间差宣称稳定加速。

优先优化顺序建议如下：

1. **先解决 Path 最终合法化后的收益丢失。** ICS55 两批共有 8 次整阶段回滚。先定位位移、寄生与 DRV 严重度的变化，再评估局部合法化、检查点或保留可接受编辑子集；目前数据尚不能断言哪种实现最优。
2. **扩大有效 TNS/DRV 覆盖，同时保留数量约束。** WNS 变好而 TNS 不占优说明只追踪少量最差路径存在局限。可对照评估端点覆盖、带电气收益的候选排序，以及 `path,drv,setup` 的组合；本轮各流程独立运行，尚未验证组合一定优于单流程。
3. **让 VG/GOBI 的执行粒度与方案成本更可控。** ICS55 中两者各有 11 次整批回滚，GOBI 的 buffer 数也普遍更高。优先比较同一可接受收益下的插入数、面积与批次划分，再评估按实际树规模选择 CPU/GPU；不能根据设计实例总数直接定阈值。
4. **保留严格改善验收，按有效收益控制长尾。** 当前数据既显示微小修改有价值，也显示搜索仍会留下大量违例。应追踪每次提交的 WNS/TNS/DRV 收益、buffer 成本和验证耗时，而不是恢复固定百分比接受门槛或把零违例作为唯一停止条件。
