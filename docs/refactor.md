# 当前重构状态

## 已收敛的结构

OpenDB/STA 是设计与时序的单一来源。旧 sizing 搜索、LUT、自定义 timer、重复 Liberty 导入、env/cmd 入口及旧 PBBI 已移除。按 DAC 2005 论文重新实现的 PBBI 首版使用独立路径树与 DP，接入共享 Buffer 执行层；实现范围见[缓冲器插入](buffer-algorithms.md)。其他功能通过 DRV、Setup、Buffer、Power、ORD、Fanout、Hold 和 Preplace 保留。流程、算法、动作和运行时依赖的边界见 [架构](modules-and-classes.md)。

Path/VG/GOBI 共用 buffer 执行框架。Path 的候选、排序和反馈归入算法对象；repair 负责资源约束、编辑、实测验收和回滚。公共配置独立放在 `common`，运行时保护对象及刷新回调不序列化。数量上限统一为可选值，0 是零额度。

Path 保留严格正收益验收，按实测进展补回续搜额度，预测饱和时优先受影响路径裕量。增量 RC 后先使 delay cone 失效再计算 required time。路径采样先复制值再做可能刷新 STA 的查询。耗时只用于诊断，已删除搜索时间限制。

## 工程入口与清理

- 只使用 `build/`，默认构建 VG/GOBI CUDA，主机 C++20、CUDA C++17。
- `benchmark_ispd26.py`：默认评测六设计的初始 / drv+setup / VG / GOBI / Path，逐项保存命令、输入、日志和独立评估结果，成功后删除 DEF/Verilog/ODB；内置终端、年度报告及对照表打印，`--report-only` 可读取已有结果重新生成表格。
- `benchmark_ecc.py`：读取 ECC/ICS55 两批 post-placement 快照及工艺元数据，复用独立评估和报告代码；按工艺配置统一 buffer 候选集，逐设计更新 benchmark 文档。
- `check_cpp.py`：基于当前编译数据库运行 clangd/clang-tidy。
- 删除旧格式专用的 repair regression、VG pipeline、VG CUDA 对照脚本及两个 env Tcl evaluator；这些入口依赖已删除的启动器，独立评测统一由 benchmark 完成。
- 删除对应的历史回归策略 JSON 与 Python 测试，合并重复的旧别名拒绝断言为通用未知输入测试；删除无引用的旧 DEF 生成脚本和固定型号 dont-use 清单。真实 STA、ECO 回滚、约束传递、供电、布局和 CUDA 求解测试保留。
- docs 按主题维护当前版本，合并 repair/ORD、Path 理论与树说明，删除旧 PBBI 文档、旧评测专题及 archive 中的旧工作树与构建产物。

2026-11-25 已完成[六设计评测](../results/ispd26-benchmark-20261125/)，新日志、指标和报告统一使用当前流程名称。全部指标与上一轮一致，旧备份已清理。

## 验证与后续重点

六设计评测状态与结果见[Benchmark](benchmark.md)，GPU/Path 年度表见 [GJ2026](gj2026.md)。本轮构建和测试日志在同一结果目录的 validation 中。

后续优化重点是大设计候选生成与 STA 刷新开销，以及候选收益和 buffer 成本的权衡。小树优先、混合型号树和额外缓存的历史实验并未证明稳定收益，没有作为当前策略保留。正收益停止策略不保证 setup/DRV 清零，修复结果须同时检查时序、电气约束和改动数量。

本轮构建及 CTest 通过；六设计五种流程共 30 项独立评测全部通过。GOBI 六例均使用 CUDA，无 fallback。Ariane v2 的 Path 算法时间为 177.268 秒，完整进程为 206.28 秒。构建与测试日志位于结果目录的 `validation/`，与备份的逐项对比见 `comparison-with-backup.json`。

## PBBI（2026-11-25）

使用独立跨网路径树、固定门节点和成本感知 DP，执行复用 Buffer 的 ECO 与刷新框架。已补齐全网络缓冲感知 AT/RAT、预测时序选路、逐路径真实增量 RC/STA 更新及未处理网收尾。预测延迟通过 OpenSTA 临时 annotation 实现，返回规划结果前完整恢复，不另建 timer。逐路径实测不退化验收和阶段末完整验证共同保护网表与布局。不可达 SDC 下的成本优化使用初始全网预测 slack 作为规划目标，不改变原 SDC，也不新增数值收益门槛或墙钟时间限制。

新增集成测试覆盖预测标注恢复、保留用户已有 annotation、真实时序恢复、未采样网收尾、增量执行及最终失败时的整阶段回滚。入口为 `--flow pbbi`；前沿数量上限已移除，改为论文的近似开关及 PSP/APSP 电阻倍率。

联合 gate/sink sizing、PI 网缓冲候选尚未实现。预测已改用引用文献 [6] 的 BELT，前沿采用 [16] 的 PSP/APSP、SqP 和库查找；不再使用树 DP 作为预测器或固定数量截断。详细对应和下一步优先级见[缓冲器插入](buffer-algorithms.md)。评测只运行 AES/JPEG 的 PBBI，其他对照复用原有 benchmark，不保留复测副本。

当前模型与剪枝版本的 12 项 CTest、clang-tidy 和 AES/JPEG 独立重载评测通过。最终算法时间为 825.602/212.173 秒，新增 buffer 为 189/523。相比旧 PBBI 时序改善，但耗时仍明显增加；AES 长尾增加了大量 buffer，JPEG 的数量下降也未带来面积下降。当前无全面工程优势，不提交代码。详细结论见 [Benchmark](benchmark.md)。
