# 模块边界与调用关系

主机代码使用 C++20，CUDA 使用 C++17。OpenDB 保存网表与布局，OpenSTA 保存约束、模型和时序状态；算法中的图、树和缓存只是当前设计的临时视图。

| 层 | 模块 | 职责 |
|---|---|---|
| 应用 | `application.cpp` | CLI、阶段组合、运行时回调、日志与输出 |
| 会话 | `design_session.*` | OpenROAD 生命周期、标准输入输出、RC 初始化与刷新 |
| 分析 | `design_analysis.*`, `parasitics.h` | 全局/局部指标、预测模型、placement RC 和 STA 失效管理 |
| 流程 | `repair/drv.*`, `setup.*`, `buffer.*` | 候选执行、资源约束、验收、提交与回滚 |
| 算法 | `algorithms/path.*`, `van_ginneken.*`, `gobi.*`, `pbbi.*`, `cell_candidates.*` | 路径调度、树求解、换型候选；不编辑网表 |
| 动作 | `actions/netlist.*`, `placement.*`, `legalize_core.*` | 插入、换型、事务、合法化；不判断收益 |
| 参数与依赖 | `repair/config.*`, `runtime.h` | 可序列化策略与运行时对象分开管理 |

## 流程和动作

默认 `--flow drv,setup`。DRV 处理电容、slew、fanout；Setup 处理负 setup slack，可使用换型/VT、旁支缓冲、负载降型。Setup 不自动调用 Path、VG、GOBI 或 PBBI。`--flow path,drv,setup` 依次作用于前一阶段已提交的设计。

`repair/drv.*` 也聚合 ORD 驱动遍历、独立 fanout 修复、power 与时序恢复。ORD 借鉴 OpenROAD 的 levelized driver 遍历，但编辑和验收由 Sizer 实现。`hold`、`preplace`、`preplace-gain` 使用 OpenROAD 专用流程。

`repair/buffer.*` 统一执行 Path/VG/GOBI/PBBI：`BufferProposal` 表示计划，`BufferTrial` 执行一个 ECO，`BufferExecution` 管理刷新和代次。算法保留不同的候选、求解和调度策略，不通过继承强迫 GPU forest 与 STA 路径使用同一数据结构。

`pbbi::Planner` 在规划期间安装并恢复预测延迟，复用 OpenSTA 传播全网 AT/RAT。返回的每条路径都不持有 STA Path 对象；执行层更新真实 RC/STA 后再次调用 Planner。已处理网使用实际延迟，其余网继续使用预测。最终合法化前原有单元保持不动，预测几何才可复用。

## 对象与生命周期

- `Settings` 保存阶段策略；`Options` 加入公共策略；`Runtime` 保存回调、保护对象。`RepairConfig::common` 独立于 DRV，不再让 Setup/Path 从 DRV 复制公共参数。
- `DesignMetrics` 是统一指标快照；`BufferTrialResult` 是一次 ECO 的结果；`BufferRepairResult` 是整个阶段结果。接受次数计 ECO，buffer 数计实例。
- `BufferTreePlan` 为确定的物理编辑，包含型号、坐标、负载及子节点；子节点先于父节点。`applyBufferTree` 先检查完整拓扑，再执行，不自行判断收益或提交事务。
- `EcoTransaction` 独占 journal；子提交合入父事务，父回滚恢复全部子编辑和位移。最终合法化失败会撤销整个 provisional 阶段。
- STA 的路径对象不能跨会更新搜索状态的查询保存；采样时先复制 pin/slack，再进行预测和其它 slack 查询，避免使用失效的 `PathEnd`。每次拓扑更新都使相关快照失效。
- 增量 RC 更新后，在首次 required-time 传播之前使受影响 delay cone 失效，防止先用旧 delay 计算指标，再重复全量验证。回滚后也必须恢复 STA。

`special_net.*` 在加载和插入后保护外部 PAD 网、补齐供电连接；`lef_layer_rc.*` 在输入未设置 RC 时，从已加载 LEF 层属性补默认 RC。它们不是另外的文件解析器。LEF/DEF/Verilog/Liberty/SDC/SPEF 均由 OpenROAD/OpenSTA 读取。

## 开发入口

单目录构建、测试、静态检查命令见[项目 README](../README.md)。测试覆盖真实 STA、ECO 回滚、布局、约束、算法和 CUDA。`scripts/check_cpp.py` 读取 `build/` 的编译数据库；`scripts/benchmark_ispd26.py` 负责设计级独立验证。
