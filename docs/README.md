# Sizer 文档

当前版本使用一个 `build/`，默认构建 VG/GOBI CUDA 支持。快速开始见[项目 README](../README.md)。

| 主题 | 文档 |
|---|---|
| 模块、流程、动作与对象生命周期 | [架构](modules-and-classes.md) |
| 当前配置、默认值与组合流程 | [配置](repair-config.md) |
| Path、VG、GOBI、PBBI 的算法与执行 | [缓冲器插入](buffer-algorithms.md) |
| ASAP7 与 ECC/ICS55 的策略、结果及评测命令 | [Benchmark](benchmark.md) |
| 研究背景、任务成果、GPU 与 Path 评测 | [GJ2026 年度总结](gj2026.md) |
| 重构现状、验证与后续重点 | [重构](refactor.md) |

文档描述当前实现。[2026-11-25 六设计评测](../results/ispd26-benchmark-20261125/)已完成，共 30 项全部通过。上一轮备份已清理；ECC/ICS55 评测使用独立结果目录。benchmark 按日期记录策略与结果。原始研究资料保留在 `papers/`，不代表当前实现包含论文中的全部方法。

PBBI 的 AES/JPEG 对照见 [Benchmark](benchmark.md)，PBBI 单独运行，对照复用原有 benchmark；算法范围与尚未实现的论文部分见[缓冲器插入](buffer-algorithms.md)。
