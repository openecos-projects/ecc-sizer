# 配置与流程组合

通常只需提供设计输入 Tcl。默认执行 `drv,setup`；指定 `--flow path`、`vg`、`gobi` 可单独运行，`--flow path,drv,setup` 可按顺序组合。表格中的 `drv+setup` 对应 `--flow drv,setup`。

配置通过 `--set key=value` 或 `--config file` 提供。先读取配置文件，再应用命令行覆盖；未知键和非法值在读设计前报错。运行时 STA/ODB 对象及刷新回调不属于配置。

| 参数 | 默认 | 含义 |
|---|---|---|
| `common.actions` | resize,buffer,load_downsize | 允许的动作；resize 包含配置好的 VT 换型 |
| `drv.max_inserted_buffers` / `setup.max_inserted_buffers` | unlimited | 各自阶段提交的插入数上限 |
| `buffer.max_inserted_buffers` | unlimited | Path/VG/GOBI/PBBI 每阶段提交数量上限 |
| `buffer.path.max_trials` | unlimited | 累计试验数上限 |
| `buffer.path.max_refreshes` | unlimited | 实际刷新次数上限，保留回滚刷新余量 |
| `buffer.path.search_patience` | 32 | 续搜额度，每次试验消耗 1；unlimited 关闭此启发式 |
| `buffer.path.progress_fraction` | 0.05 | 剩余违例改善 5% 获得 1 额度，补回后不超过初始额度 |
| `buffer.path.max_area_growth` | 0.02 | 允许增长的面积比例 |
| `buffer.path.min_relative_improvement` | 0 | 可选的单步相对收益门槛 |
| `buffer.path.min_improvement_ns` | 0 | 可选的额外绝对收益门槛，另有数值容差 |
| `buffer.path.max_paths` / `max_candidates` | 16 / 8 | 每类检查采样端点数 / 每网主候选数 |
| `buffer.path.placement` / `verification` | trial / trial | 每试验合法化/完整验证，或阶段末合法化/完整验证 |
| `buffer.path.incremental_parasitics` / `incremental_analysis` | false / false | 增量 RC 与局部电气检查 |
| `buffer.path.adaptive_search` / `reuse_rejections` | false / false | 反馈调度与经过依赖验证的失败复用 |
| `buffer.pbbi.max_paths` | 64 | PBBI 在全网缓冲感知 STA 上选择的 setup 路径数；未处理网另行收尾 |
| `buffer.pbbi.approximate` | true | 使用引用文献 [16] 的 PSP/APSP、SqP 和库查找；false 保留完整 Pareto 前沿，仅用于小树对照 |
| `buffer.pbbi.prebuffer_resistance_scale` | 1 | 预缓冲电阻相对上游最小电阻的倍率；1 为 PSP，>1 为 APSP；不得小于 1 |
| `buffer.vg.backend` | cpu | VG 求解后端，可选 cuda；构建 CUDA 不会改变此默认值 |
| `buffer.vg.scope` | drv | 选网范围，setup 也纳入负 slack 网 |
| `buffer.vg.max_nets` / `buffer.gobi.max_nets` | unlimited | 收集网数上限 |

所有可选上限用 `unlimited` 表示无上限，0 是实际零额度；动作是否启用由 `common.actions` 决定。拒绝的 ECO 不消耗已提交插入数量。串联阶段分别计数。Path 没有按墙钟时间停止的配置或代码，性能计时只用于日志。

默认 Path 接受超过数值容差的 WNS 或 TNS 正收益，并检查另一时序指标、hold、DRV 和面积。微小收益只补回少量续搜额度；已经损失后又恢复的指标不会重复赚取额度。修复到零违例也作为进展。停止并不证明已穷尽搜索或能够清零。

六设计评测采用 `placement=final`、`verification=final`、增量分析、失败复用，并解除面积上限；这些设置内置在 benchmark 的 Path profile，未修改 Sizer 通用默认值。完整参数保存在每个结果的 `result.json` 与可直接执行的 `run.sh`。

```ini
[common]
actions=resize,buffer,load_downsize
[buffer.path]
search_patience=32
progress_fraction=0.05
```

C++ 接口使用秒；名称带 `_ns` 的文本参数使用纳秒。详细字段和校验规则见 [`repair/config.h`](../src/repair/config.h) 与 [`config.cpp`](../src/repair/config.cpp)。
