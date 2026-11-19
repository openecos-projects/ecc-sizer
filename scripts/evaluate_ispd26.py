#!/usr/bin/env python3
"""Reproduce the six-design CPU / CUDA / Path work-summary experiment.

Run from any directory. Inputs are read-only; each flow starts from the
original DEF. CPU/CUDA/Path are buffer-only; optional native and native-path
flows enable Native design repair (including resizing) before Path. The
profiles below reproduce the archived VG/GBI and explicit-tree Path settings,
using the selected current binaries. Historical timings are not guaranteed.
No time, trial, refresh, buffer-count or area budget is imposed on Path.
Jobs run sequentially, so GPU tasks do not compete for memory.

Examples:
  python3 scripts/evaluate_ispd26.py --dry-run
  python3 scripts/evaluate_ispd26.py --output results/ispd26-repro
  python3 scripts/evaluate_ispd26.py --output results/ispd26-repro --resume
  python3 scripts/evaluate_ispd26.py --designs aes_cipher_top --algorithms gbi path
  python3 scripts/evaluate_ispd26.py --algorithms native native-path --output results/native-path

Each case reuses evaluate_fanout.py for independent OpenROAD PPA evaluation,
placement/supply checks and the contest structural check. This does not replace
the additional connectivity/fault-injection audit archived with the report.
Outputs: manifest.json, status.json, report.md, summary.csv, and per-algorithm
summary.json plus cases/<design>/ logs, effective configuration and netlists.
Completed runs also print an aligned result table to the terminal.
Requires the external ISPD26 ASAP7 inputs/checker and a CUDA-enabled Sizer for GBI.
"""

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time
import unicodedata


ROOT = Path(__file__).resolve().parents[1]
DESIGNS = {
    "aes_cipher_top": "AES", "aes_cipher_top_v2": "AES v2",
    "jpeg_encoder": "JPEG", "jpeg_encoder_v2": "JPEG v2",
    "ariane": "Ariane", "ariane_v2": "Ariane v2",
}
FLAGS = {"vg": "--repair-vg", "gbi": "--repair-gbi", "path": "--path-from-input",
         "native": "--repair-design", "native-path": "--repair-setup"}
LABELS = {"vg": "CPU", "gbi": "CUDA", "path": "Path",
          "native": "Native design", "native-path": "Native design → Path",
          "path-reference": "Path（同二进制存档）"}
# Explicit experiment parameters, not new algorithm defaults.
COMMON = {
    "common.max_fanout": "64", "common.margin": "0.1",
    "common.recovered_slew_fraction": "0.25",
    "common.overload_group_fraction": "0.5",
    "tolerances.wns_absolute_ns": "0.000001",
    "tolerances.tns_absolute_ns": "0.000001",
    "tolerances.tns_relative": "0.000001", "tolerances.drv_severity": "0.000001",
    "power.enabled": "false", "pbbi.enabled": "false",
}
PROFILES = {
    "vg": {
        "vg.backend": "cpu", "vg.scope": "drv", "vg.max_buffers": "0",
        "vg.max_nets": "0", "vg.max_group_loads": "0", "vg.min_gain": "0",
        "vg.require_drv_reduction": "true",
    },
    "gbi": {
        "gbi.backend": "cuda", "gbi.max_buffers": "0", "gbi.max_nets": "0",
        "gbi.require_drv_reduction": "true",
    },
    "path": {
        "design.max_passes": "0", "setup.path_buffer_enabled": "true",
        "setup.path_buffer_only": "true", "setup.path_buffer_strategy": "feedback",
        "setup.path_buffer_recovery": "true", "setup.path_buffer_tree_candidates": "true",
        "setup.path_buffer_gain_scope": "global", "setup.path_buffer_fanout_repair": "true",
        "setup.path_buffer_max_paths": "16", "setup.path_buffer_max_candidates": "8",
        "setup.path_buffer_batch_size": "1", "setup.path_buffer_max_trials": "0",
        "setup.path_buffer_max_consecutive_rejections": "0",
        "setup.path_buffer_max_seconds": "0", "setup.path_buffer_max_refreshes": "0",
        "setup.path_buffer_max_inserted_buffers": "0", "setup.path_buffer_max_area_growth": "-1",
        "setup.path_buffer_min_relative_improvement": "0.001",
        "setup.min_improvement_ns": "0.0001", "setup.defer_rollback_refresh": "true",
    },
}
NATIVE = {
    "design.max_passes": "8", "design.max_inserted_buffers": "0",
    "design.max_load_downsizes_per_pass": "64",
    "common.allow_gate_resize": "true", "common.allow_load_downsize": "true",
    "common.load_downsize_slack_margin_ns": "0", "common.vt_suffixes": "none",
}
PROFILES["native"] = NATIVE
PROFILES["native-path"] = dict(PROFILES["path"], **NATIVE)
METRICS = ("setup_wns_ns", "setup_tns_ns", "hold_wns_ns", "hold_tns_ns",
           "slew_violations", "cap_violations", "fanout_violations", "area_um2",
           "total_power_w", "inserted_sizer_buffers", "unplaced",
           "placement_checked", "supply_pin_errors")
TABLE_LABELS = {"vg": "CPU", "gbi": "CUDA", "path": "Path",
                "native": "Native design", "native-path": "Native design → Path",
                "path-reference": "Path（存档）"}


def display_width(value):
    return sum(0 if unicodedata.combining(char) else
               2 if unicodedata.east_asian_width(char) in ("W", "F") else 1
               for char in value)


def print_result_table(args, grouped):
    print("\nISPD26 评测结果")
    print("S/C/F = Slew/Cap/Fanout；功耗单位为 mW；完整流程耗时为 Sizer 进程时间。")
    print("CPU/CUDA 为不同算法，CUDA 括号内的耗时比是 CPU/CUDA，不代表同算法硬件加速。")
    header = ["设计", "状态", "WNS (ns)", "TNS (ns)", "电气违例 S/C/F",
              "面积 (µm²)", "功耗 (mW)", "新增 buffer", "求解耗时 (s)", "完整流程 (s)"]
    groups = []

    def metric_values(metrics):
        drv = "/".join(str(int(metrics[key + "_violations"]))
                       for key in ("slew", "cap", "fanout"))
        return [f"{metrics['setup_wns_ns']:.4f}", f"{metrics['setup_tns_ns']:.2f}", drv,
                f"{metrics['area_um2']:.2f}", f"{metrics['total_power_w'] * 1000:.3f}"]

    def improvement(before, after, key, digits=2):
        if before[key] == 0:
            return "—"
        percent = 100 * (abs(before[key]) - abs(after[key])) / abs(before[key])
        if abs(percent) < 0.5 * 10 ** -digits:
            percent = 0
        return f"{percent:.{digits}f}%"

    for design in args.designs:
        records = {algorithm: {row["mode"]: row for row in rows if row["design"] == design}
                   for algorithm, rows in grouped.items()}
        baseline = next((value["baseline"] for value in records.values() if "baseline" in value), None)
        if baseline is None:
            continue
        before = baseline["metrics"]
        group = [[DESIGNS[design], "初始"] + metric_values(before) + ["0", "—", "—"]]
        algorithms = list(args.algorithms)
        if args.path_reference:
            algorithms.append("path-reference")
        for algorithm in algorithms:
            result = records.get(algorithm, {}).get("sizer")
            if result is None:
                continue
            metrics = result["metrics"]
            topology_key = "design_topology" if algorithm == "native" else f"{algorithm}_topology"
            topology = result.get(topology_key, {})
            if algorithm in ("vg", "gbi"):
                solve = topology.get("solve_ms")
                solve_text = f"{solve / 1000:.3f}" if solve is not None else "—"
            elif algorithm == "native":
                elapsed = topology.get("elapsed_ms")
                solve_text = f"{elapsed / 1000:.3f}" if elapsed is not None else "—"
            else:
                solve_text = "—"
            runtime = f"{result['sizer_seconds']:.2f}"
            if algorithm == "gbi" and "vg" in records and "sizer" in records["vg"]:
                cpu = records["vg"]["sizer"]
                cpu_solve = cpu.get("vg_topology", {}).get("solve_ms", 0) / 1000
                cuda_solve = (topology.get("solve_ms") or 0) / 1000
                if cuda_solve > 0 and solve_text != "—":
                    solve_text += f" ({cpu_solve / cuda_solve:.2f}×)"
                if result["sizer_seconds"] > 0:
                    runtime += f" ({cpu['sizer_seconds'] / result['sizer_seconds']:.2f}×)"
            group.append(["", TABLE_LABELS[algorithm]] + metric_values(metrics) +
                         [str(int(metrics["inserted_sizer_buffers"])), solve_text, runtime])

            if algorithm in ("path", "path-reference"):
                path_before = before
                gain_label = "Path 改善"
            elif algorithm == "native-path":
                path_before = result["stage_metrics"]["design"]["metrics"]
                gain_label = "Path 阶段改善"
            else:
                continue
            drv_gain = "/".join(improvement(path_before, metrics, key)
                                for key in ("slew_violations", "cap_violations", "fanout_violations"))
            group.append(["", gain_label, improvement(path_before, metrics, "setup_wns_ns"),
                          improvement(path_before, metrics, "setup_tns_ns"), drv_gain,
                          improvement(path_before, metrics, "area_um2", 4),
                          improvement(path_before, metrics, "total_power_w", 4), "—", "—", "—"])
        groups.append(group)

    if not groups:
        print("没有已完成的评测结果。")
        return
    rows = [header] + [row for group in groups for row in group]
    widths = [max(display_width(row[index]) for row in rows) for index in range(len(header))]

    def border(left, middle, right, fill):
        return left + middle.join(fill * (width + 2) for width in widths) + right

    def line(row):
        cells = []
        for index, (value, width) in enumerate(zip(row, widths)):
            padding = " " * (width - display_width(value))
            cells.append(" " + (value + padding if index < 2 else padding + value) + " ")
        return "│" + "│".join(cells) + "│"

    print(border("┌", "┬", "┐", "─"))
    print(line(header))
    print(border("╞", "╪", "╡", "═"))
    for index, group in enumerate(groups):
        if index:
            print(border("├", "┼", "┤", "─"))
        for row in group:
            print(line(row))
    print(border("└", "┴", "┘", "─"))


def stamp():
    return datetime.now(timezone.utc).isoformat()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temp.replace(path)


def parse_overrides(values):
    result = {}
    for item in values:
        key, separator, value = item.partition("=")
        if not separator or not key or not value or key in result:
            raise ValueError("repair overrides require unique KEY=VALUE entries: " + item)
        if key == "runtime.sta_threads":
            raise ValueError("Use --threads for the STA thread count")
        result[key] = value
    return result


def config(algorithm, threads, overrides=None):
    result = dict(COMMON, **PROFILES[algorithm])
    result.update(overrides or {})
    result["runtime.sta_threads"] = str(threads)
    return result


def command(args, algorithm, design):
    out = args.output / algorithm / "cases" / design
    cmd = [sys.executable, "-u", str(ROOT / "scripts/evaluate_fanout.py"),
           "--benchmark", str(args.benchmark), "--equiv-dir", str(args.equiv_dir),
           "--designs", design, FLAGS[algorithm], "--max-fanout", "64",
           "--sizer", str(args.sizer), "--openroad", str(args.openroad),
           "--dont-use-list", str(args.dont_use_list), "--output", str(out)]
    if algorithm == "path":
        cmd.append("--stage-snapshots")
    if algorithm in ("native", "native-path"):
        cmd.append("--stage-metrics")
    for key, value in config(algorithm, args.threads, getattr(args, "repair_overrides", None)).items():
        cmd += ["--repair-set", key + "=" + value]
    return cmd


def provenance(args, jobs):
    required = [args.sizer, args.openroad, args.dont_use_list,
                args.equiv_dir / "netlist_equiv_check.py",
                args.equiv_dir / "asap7_equivalent_cell_list.csv"]
    for name in args.designs:
        required += [args.benchmark / "design" / name / f for f in ("contest.def", "contest.sdc")]
    platform = args.benchmark / "platform/ASAP7"
    required += [platform / "util/setRC.tcl", platform / "lef/asap7_tech_1x_201209.lef"]
    if not list((platform / "lib").glob("asap7sc7p5t_*_TT_*.lib")):
        raise ValueError("Missing ASAP7 TT Liberty libraries: " + str(platform))
    for p in required:
        if not p.is_file():
            raise ValueError("Missing required file: " + str(p))
    for p in (args.sizer, args.openroad):
        if not os.access(p, os.X_OK):
            raise ValueError("Not executable: " + str(p))
    # Hash inputs/checker/scripts as well as binaries, including on resume.
    files = set(required)
    files.update(p for p in platform.rglob("*") if p.suffix in (".lib", ".lef", ".tcl") and p.is_file())
    files.update(p for p in args.equiv_dir.rglob("*") if p.suffix in (".py", ".tcl", ".csv") and p.is_file())
    for name in args.designs:
        files.update((args.benchmark / "design" / name).glob("*.tcl"))
    files.update(ROOT / "scripts" / n for n in (
        "evaluate_ispd26.py", "evaluate_fanout.py",
        "evaluate_fanout.tcl", "check_repair_regression.py"))
    if args.path_reference:
        reference_run = args.path_reference / "run.json"
        recorded = json.loads(reference_run.read_text())
        if recorded["binary_sha256"] != sha256(args.sizer):
            raise ValueError("Path reference requires the exact same Sizer binary")
        files.add(reference_run)
        for name in args.designs:
            summary = args.path_reference / "cases" / name / "summary.json"
            validate(summary.parent, "path", name, args.threads)
            files.add(summary)
    source_files = sorted(p for p in (ROOT / "src").iterdir()
                          if p.suffix in (".cpp", ".h", ".cu") and p.is_file())
    git_head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    return {
        "profile": "ispd26-work-summary-six-v1", "git_head": git_head,
        "python": sys.version, "threads": args.threads, "concurrency": 1,
        "designs": args.designs, "algorithms": args.algorithms,
        "path_reference": str(args.path_reference) if args.path_reference else None,
        "profiles": {a: config(a, args.threads, getattr(args, "repair_overrides", None)) for a in args.algorithms}, "jobs": jobs,
        "files_sha256": {str(p): sha256(p) for p in sorted(files)},
        "sources_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in source_files},
        "environment": {k: os.environ.get(k) for k in ("CUDA_VISIBLE_DEVICES", "LD_LIBRARY_PATH", "OMP_NUM_THREADS")},
    }


def validate(case, algorithm, design, threads, overrides=None):
    rows = json.loads((case / "summary.json").read_text())
    if len(rows) != 2 or {r.get("mode") for r in rows} != {"baseline", "sizer"}:
        raise ValueError("Incomplete baseline/sizer result: " + str(case))
    indexed = {r["mode"]: r for r in rows}
    base, final = indexed["baseline"], indexed["sizer"]
    for row in rows:
        if row["design"] != design or row["input_fingerprint"] != base["input_fingerprint"]:
            raise ValueError("Input/design mismatch: " + str(case))
        for key in METRICS:
            if not math.isfinite(row["metrics"][key]):
                raise ValueError("Nonfinite metric " + key)
        if any(row["metrics"][k] != v for k, v in
               (("placement_checked", 1), ("unplaced", 0), ("supply_pin_errors", 0))):
            raise ValueError("Placement/supply validation failed: " + str(case))
    native = algorithm in ("native", "native-path")
    if final.get("contest_structural_check") != "passed":
        raise ValueError("Missing structural check")
    if final["gate_resize"] != native or final["repair_design"] != native or final["repair_pbbi"]:
        raise ValueError("Unexpected sizing/design/PBBI stage")
    effective = final["effective_repair_config"]
    for key, expected in config(algorithm, threads, overrides).items():
        actual = effective.get(key)
        if actual == expected:
            continue
        try:
            equal = math.isclose(float(actual), float(expected), rel_tol=1e-6, abs_tol=1e-15)
        except (ValueError, TypeError):
            equal = False
        if not equal:
            raise ValueError(f"Configuration mismatch: {key}: {actual} != {expected}")
    if algorithm == "path" and final["path_startpoint"] != "raw_input":
        raise ValueError("Path must start from the original input")
    if native:
        stages = final["stage_metrics"]
        required = ["input", "design"] + (["setup"] if algorithm == "native-path" else [])
        for name in required:
            stage = stages[name]
            if stage.get("contest_structural_check") != "passed":
                raise ValueError("Unverified native stage: " + name)
            for key in METRICS:
                if not math.isfinite(stage["metrics"][key]):
                    raise ValueError("Nonfinite native stage metric: " + key)
            for key, expected in (("placement_checked", 1), ("unplaced", 0), ("supply_pin_errors", 0)):
                if stage["metrics"][key] != expected:
                    raise ValueError("Invalid placement/supplies in stage " + name)
        for key in METRICS:
            if stages[required[-1]]["metrics"][key] != final["metrics"][key]:
                raise ValueError("Final stage/reloaded output mismatch: " + key)
        if final["repair_setup"] != (algorithm == "native-path"):
            raise ValueError("Unexpected setup stage")
    if algorithm == "native-path":
        if final["path_startpoint"] != "after_native_design":
            raise ValueError("Native→Path must start from the design stage")
        if any(t.get("metrics_complete") == 0 for t in final["path_trials"] if t["accepted"]):
            raise ValueError("Path accepted an edit without full electrical metrics")
        phase_verification = effective.get("setup.path_buffer_verification") == "final"
        if phase_verification and effective.get("setup.path_buffer_placement") != "final":
            raise ValueError("Final verification requires final placement and phase rollback")
        if effective.get("setup.path_buffer_incremental_parasitics") == "true" and not phase_verification:
            if any(t.get("full_verified") != 1 for t in final["path_trials"] if t["accepted"]):
                raise ValueError("Incremental Path accepted an edit without full verification")
        if effective.get("setup.path_buffer_placement") == "final":
            topology = final["setup_topology"]
            trials = final["path_trials"]
            if any(t.get("placement_provisional") != 1 for t in trials):
                raise ValueError("Final-DP trial must be marked provisional")
            provisional = sum(t["buffers"] for t in trials if t["accepted"])
            if topology.get("path_buffer_provisional_buffers") != provisional:
                raise ValueError("Provisional buffer count mismatch")
            accepted = topology.get("path_buffer_final_placement_accepted") == 1
            restored = topology.get("path_buffer_final_placement_rolled_back") == 1
            if provisional and (topology.get("path_buffer_final_placement_attempted") != 1 or accepted == restored):
                raise ValueError("Missing final placement acceptance or rollback")
            start, end = stages["design"]["metrics"], final["metrics"]
            expected_buffers = provisional if accepted else 0
            if end["inserted_sizer_buffers"] - start["inserted_sizer_buffers"] != expected_buffers:
                raise ValueError("Final placement retained unexpected buffers")
            for key in ("setup_wns_ns", "setup_tns_ns", "hold_wns_ns", "hold_tns_ns"):
                tolerance = float(effective["tolerances.wns_absolute_ns"])
                if "tns" in key:
                    tolerance = max(float(effective["tolerances.tns_absolute_ns"]),
                                    abs(start[key]) * float(effective["tolerances.tns_relative"]))
                if end[key] + tolerance < start[key]:
                    raise ValueError("Final placement degraded " + key)
            for key in ("slew_violations", "cap_violations", "fanout_violations"):
                if end[key] > start[key]:
                    raise ValueError("Final placement degraded " + key)
        for key in ("resized_cells", "vt_swaps", "downsized_loads"):
            if final["setup_topology"].get(key) != 0:
                raise ValueError("Path performed a non-buffer edit: " + key)
    if algorithm in ("vg", "gbi") and not final[algorithm + "_topology"]:
        raise ValueError("Missing algorithm completion counters")
    return rows


def stage_comparison(args, grouped):
    comparisons = []
    for algorithm in ("path", "native-path", "path-reference"):
        for row in grouped.get(algorithm, []):
            if row["mode"] != "sizer":
                continue
            base = next(r["metrics"] for r in grouped[algorithm]
                        if r["design"] == row["design"] and r["mode"] == "baseline")
            before = row["stage_metrics"]["design"]["metrics"] if algorithm == "native-path" else base
            after = row["metrics"]
            setup = row["setup_topology"]
            trials = row["path_trials"]
            reasons = Counter(reason for trial in trials for reason in trial["rejection_reasons"])
            accepted = sum(bool(t["accepted"]) for t in trials)
            rejected = len(trials) - accepted
            provisional_accepted = sum(bool(t["accepted"]) and bool(t.get("placement_provisional")) for t in trials)
            final_rolled_back = setup.get("path_buffer_final_placement_rolled_back") == 1
            native = row.get("design_topology", {}) if algorithm == "native-path" else {}
            effective = row["effective_repair_config"]
            relative = float(effective["setup.path_buffer_min_relative_improvement"])
            absolute = float(effective["setup.min_improvement_ns"])
            comparisons.append(dict(
                design=row["design"], flow=algorithm, archived=algorithm == "path-reference",
                gain_scope=effective["setup.path_buffer_gain_scope"],
                placement_mode=row["effective_repair_config"].get("setup.path_buffer_placement", "trial"),
                provisional_accepted_trials=provisional_accepted,
                max_consecutive_rejections=int(effective.get("setup.path_buffer_max_consecutive_rejections", 0)),
                stopped_on_rejections=bool(setup.get("path_buffer_stopped_on_rejections", 0)),
                final_placement_rolled_back=final_rolled_back,
                initial_metrics=base, path_start_metrics=before, final_metrics=after,
                path_wns_gain_ns=after["setup_wns_ns"]-before["setup_wns_ns"],
                path_tns_gain_ns=after["setup_tns_ns"]-before["setup_tns_ns"],
                initial_wns_gain_ns=after["setup_wns_ns"]-base["setup_wns_ns"],
                initial_tns_gain_ns=after["setup_tns_ns"]-base["setup_tns_ns"],
                path_inserted_buffers=after["inserted_sizer_buffers"]-before["inserted_sizer_buffers"],
                native_inserted_buffers=native.get("inserted_buffers", 0),
                native_resized_gates=native.get("resized_gates", 0),
                native_resized_buffers=native.get("resized_buffers", 0),
                native_downsized_loads=native.get("downsized_loads", 0),
                native_seconds=native.get("elapsed_ms", 0)/1000,
                path_seconds=setup["elapsed_ms"]/1000, sizer_seconds=row["sizer_seconds"],
                attempted_trials=len(trials), accepted_trials=0 if final_rolled_back else accepted,
                rejected_trials=rejected,
                rejection_rate=rejected/len(trials) if trials else None,
                rejection_reasons=dict(reasons),
                fanout_precheck_rejections=setup.get("path_buffer_fanout_precheck_rejections", 0),
                refreshes=setup.get("refreshes"), rollback_refreshes=setup.get("rollback_refreshes"),
                refresh_seconds=setup.get("refresh_ms", 0)/1000,
                approximate_wns_gain_floor_ns=max(absolute, abs(before["setup_wns_ns"])*relative),
                approximate_tns_gain_floor_ns=max(absolute, abs(before["setup_tns_ns"])*relative)))
    save(args.output / "stage-comparison.json", comparisons)
    if not comparisons:
        return []
    lines = ["", "## Path 阶段增量与成本", "",
             "Native design 使用既有 8 轮配置、不限 buffer 数；允许 resize/load downsize，关闭功耗恢复与 PBBI。",
             "Path 保持 buffer-only，关闭时间、试探、刷新、buffer 数和面积增长预算。",
             "上述为默认实验配置；repair-set 可覆盖。连续拒绝提前停止默认关闭，实际配置及是否触发见 stage-comparison.json。提前停止不表示候选耗尽。",
             "Native 前置后相对收益门槛随 Path 起点变化，门槛另列；不能把拒绝率变化全部归因于扇出。",
             "表中门槛为初始全局参考值。current_global 每次接受后按剩余全局负裕量更新，endpoints 按本次受影响采样端点更新；两者的逐次门槛不能用初始全局值代替。",
             "Native 单独运行与串联中的 design 阶段均独立评估；两者完成时要求 PPA 完全一致。",
             "设计/Path 阶段耗时取进程日志，Sizer 为完整进程；不含外部独立评估。", "",
             "final placement 模式的逐次接受为临时接受；最终指标来自 DP 后独立重载。整轮回滚时保留数为零，逐次拒绝率仍仅描述搜索阶段。", "",
             "| 设计 | 流程 | Path 起点 WNS/TNS (ns) | Path ΔWNS/ΔTNS (ns) | Path 新增 buffer | Native resize gate/buffer/downsize | Native 阶段 (s) | Path 阶段 (s) | Sizer (s) |",
             "|---|---|---|---|---:|---|---:|---:|---:|"]
    if args.path_reference:
        lines[2:2] = ["Path 存档对照使用相同 Sizer 二进制，输入指纹与初始 PPA逐例核对；运行批次/并发负载不同，耗时仅作参考，不作为受控加速比。", ""]
    for r in comparisons:
        m = r["path_start_metrics"]
        lines.append(f"| {DESIGNS[r['design']]} | {LABELS[r['flow']]} | {m['setup_wns_ns']:.6f} / {m['setup_tns_ns']:.3f} | {r['path_wns_gain_ns']:.6f} / {r['path_tns_gain_ns']:.3f} | {int(r['path_inserted_buffers'])} | {int(r['native_resized_gates'])}/{int(r['native_resized_buffers'])}/{int(r['native_downsized_loads'])} | {r['native_seconds']:.2f} | {r['path_seconds']:.2f} | {r['sizer_seconds']:.2f} |")
    lines += ["", "| 设计 | 流程 | 起点 S/C/F | 最终 S/C/F | 试探/接受/拒绝 | 拒绝率 | no_setup_gain | fanout 拒绝/预筛除 | 收益尺度 | 起始全局参考门槛 WNS/TNS (ns) |",
              "|---|---|---|---|---|---:|---:|---|---|---|"]
    for r in comparisons:
        drv = lambda m: '/'.join(str(int(m[k+'_violations'])) for k in ('slew', 'cap', 'fanout'))
        rate = f"{r['rejection_rate']*100:.1f}%" if r['rejection_rate'] is not None else "—"
        lines.append(f"| {DESIGNS[r['design']]} | {LABELS[r['flow']]} | {drv(r['path_start_metrics'])} | {drv(r['final_metrics'])} | {r['attempted_trials']}/{r['accepted_trials']}/{r['rejected_trials']} | {rate} | {r['rejection_reasons'].get('no_setup_gain',0)} | {r['rejection_reasons'].get('fanout',0)}/{int(r['fanout_precheck_rejections'])} | {r['gain_scope']} | {r['approximate_wns_gain_floor_ns']:.6f}/{r['approximate_tns_gain_floor_ns']:.3f} |")
    return lines


def collect(args, state):
    grouped = {a: [] for a in args.algorithms}
    if args.path_reference:
        grouped["path-reference"] = [r for d in args.designs for r in
                                    validate(args.path_reference / "cases" / d, "path", d, args.threads)]
    baselines = {}
    for job in state:
        if job["status"] != "completed":
            continue
        a, d = job["algorithm"], job["design"]
        rows = validate(args.output / a / "cases" / d, a, d, args.threads, getattr(args, "repair_overrides", None))
        base = next(r for r in rows if r["mode"] == "baseline")
        reference = (base["input_fingerprint"], {k: base["metrics"][k] for k in METRICS})
        if d in baselines and baselines[d] != reference:
            raise ValueError("Different initial PPA/input between algorithms: " + d)
        baselines[d] = reference
        grouped[a].extend(rows)
    # Check imported references too; never compare different inputs or stages.
    for rows in grouped.values():
        for row in rows:
            if row["mode"] != "baseline":
                continue
            d = row["design"]
            reference = (row["input_fingerprint"], {k: row["metrics"][k] for k in METRICS})
            if d in baselines and baselines[d] != reference:
                raise ValueError("Different initial PPA/input between algorithms: " + d)
            baselines[d] = reference
    native = {r["design"]: r for r in grouped.get("native", []) if r["mode"] == "sizer"}
    for row in grouped.get("native-path", []):
        if row["mode"] == "sizer" and row["design"] in native:
            for key in METRICS:
                if native[row["design"]]["metrics"][key] != row["stage_metrics"]["design"]["metrics"][key]:
                    raise ValueError("Native-alone and pipeline design-stage differ: " + key)
    table = []
    for a, rows in grouped.items():
        folder = args.output / a
        folder.mkdir(exist_ok=True)
        save(folder / "summary.json", rows)
        for row in rows:
            m = row["metrics"]
            solve = row.get(a + "_topology", {}).get("solve_ms") if a != "path" else None
            table.append(dict(design=row["design"], algorithm=a, mode=row["mode"],
                              **{k: m[k] for k in METRICS},
                              solve_seconds=solve / 1000 if solve is not None else "",
                              sizer_seconds=row["sizer_seconds"]))
    if table:
        with (args.output / "summary.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(table[0]))
            writer.writeheader(); writer.writerows(table)
    lines = ["# ISPD26 六设计复现结果", "",
             "各流程从相同原始输入独立运行；Native design → Path 在同一 OpenDB/STA 中串联，只有前置 design 阶段允许 sizing。",
             "仅列已完成并通过独立 PPA/结构检查的结果，阶段失败见 status.json。",
             "单场景 placement RC、默认活动率估计功耗；不是布线签核。",
             "CPU/CUDA 为不同算法，耗时比不是同算法的纯硬件加速比。", "",
             "| 设计 | 流程 | WNS (ns) | TNS (ns) | S/C/F | 面积 (µm²) | 功耗 (mW) | buffer | 求解 (s) | Sizer (s) |",
             "|---|---|---:|---:|---|---:|---:|---:|---:|---:|"]
    for d in args.designs:
        shown_base = False
        for a in grouped:
            for row in grouped[a]:
                if row["design"] != d or (row["mode"] == "baseline" and shown_base):
                    continue
                baseline = row["mode"] == "baseline"
                shown_base |= baseline
                m = row["metrics"]
                label = "初始" if baseline else LABELS[a]
                solve = row.get(a + "_topology", {}).get("solve_ms") if a != "path" else None
                solve_text = f"{solve / 1000:.3f}" if solve is not None else "—"
                runtime = "—" if baseline else f"{row['sizer_seconds']:.2f}"
                drv = "/".join(str(int(m[k + "_violations"])) for k in ("slew", "cap", "fanout"))
                lines.append(f"| {DESIGNS[d]} | {label} | {m['setup_wns_ns']:.6f} | {m['setup_tns_ns']:.3f} | {drv} | {m['area_um2']:.3f} | {m['total_power_w']*1000:.3f} | {int(m['inserted_sizer_buffers'])} | {solve_text} | {runtime} |")
    lines += stage_comparison(args, grouped)
    (args.output / "report.md").write_text("\n".join(lines) + "\n")
    return grouped


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    contest = ROOT.parent / "ISPD26-Contest"
    parser.add_argument("--benchmark", type=Path, default=contest / "gbi_benchmark/ISPD26")
    parser.add_argument("--equiv-dir", type=Path, default=contest / "equiv_check")
    parser.add_argument("--sizer", type=Path, default=ROOT / "build-gbi-cuda3/src/Sizer")
    parser.add_argument("--openroad", type=Path, default=ROOT / "build/bin/openroad")
    parser.add_argument("--dont-use-list", type=Path, default=ROOT / "tests/data/asap7_buffer_r_only.dont_use")
    parser.add_argument("--output", type=Path, default=ROOT / "results/ispd26-work-summary-repro")
    parser.add_argument("--designs", nargs="+", choices=DESIGNS, default=list(DESIGNS))
    parser.add_argument("--algorithms", nargs="+", choices=FLAGS, default=["vg", "gbi", "path"])
    parser.add_argument("--path-reference", type=Path,
                        help="reuse archived Path cases; requires its exact Sizer binary and matching inputs/config")
    parser.add_argument("--threads", type=int, default=32, help="STA threads (report: 32)")
    parser.add_argument("--repair-set", action="append", default=[], metavar="KEY=VALUE",
                        help="explicit configuration override; recorded and checked for controlled ablations")
    parser.add_argument("--dry-run", action="store_true", help="print commands only; do not create outputs")
    parser.add_argument("--resume", action="store_true", help="skip validated completed cases; preserve failed attempts")
    args = parser.parse_args()
    args.repair_overrides = parse_overrides(args.repair_set)
    if args.threads < 1 or len(set(args.designs)) != len(args.designs) or len(set(args.algorithms)) != len(args.algorithms):
        parser.error("threads must be positive; designs and algorithms must not contain duplicates")
    for name in ("benchmark", "equiv_dir", "sizer", "openroad", "dont_use_list", "output"):
        setattr(args, name, getattr(args, name).expanduser().resolve())
    if args.path_reference:
        if args.repair_overrides:
            parser.error("--path-reference uses the archived fixed profile; run overridden profiles separately")
        args.path_reference = args.path_reference.expanduser().resolve()
        if "path" in args.algorithms:
            parser.error("choose a fresh path run or --path-reference, not both")
    for source in (args.benchmark, args.equiv_dir):
        if args.output == source or source in args.output.parents:
            parser.error("output must be separate from read-only benchmark/checker inputs")
    jobs = [dict(algorithm=a, design=d, command=command(args, a, d))
            for d in args.designs for a in args.algorithms]
    if args.dry_run:
        for job in jobs:
            print(shlex.join(job["command"]))
        return 0
    identity = provenance(args, jobs)
    manifest = args.output / "manifest.json"
    if args.resume:
        previous = json.loads(manifest.read_text())
        if previous["identity"] != identity:
            raise ValueError("Resume input/binary/source/config differs; use a fresh output directory")
        state = json.loads((args.output / "status.json").read_text())
    else:
        if args.output.exists() and any(args.output.iterdir()):
            raise ValueError("Output is not empty; use --resume or a fresh directory")
        args.output.mkdir(parents=True, exist_ok=True)
        save(manifest, dict(created=stamp(), identity=identity))
        state = [dict(algorithm=j["algorithm"], design=j["design"], status="pending") for j in jobs]
        save(args.output / "status.json", state)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    cache = ROOT / ".cache"
    for key, subdir in (("TMPDIR", "tmp"), ("TMP", "tmp"), ("TEMP", "tmp"),
                        ("XDG_CACHE_HOME", "xdg"), ("CUDA_CACHE_PATH", "cuda")):
        folder = cache / subdir; folder.mkdir(parents=True, exist_ok=True)
        env[key] = str(folder)
    collect(args, state)  # Validate completed cases again before resuming.
    for job, record in zip(jobs, state):
        if record["status"] == "completed":
            print(f"skip {job['algorithm']}/{job['design']}: completed", flush=True)
            continue
        case = args.output / job["algorithm"] / "cases" / job["design"]
        if case.exists():
            archive = args.output / "attempts" / (job["algorithm"] + "-" + job["design"] + "-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f"))
            archive.parent.mkdir(exist_ok=True); shutil.move(str(case), str(archive))
        case.mkdir(parents=True)
        record.update(status="running", started=stamp())
        save(args.output / "status.json", state)
        print(f"run {job['algorithm']}/{job['design']}", flush=True)
        start = time.monotonic(); process = None
        try:
            with (case / "driver.log").open("w") as log:
                process = subprocess.Popen(job["command"], cwd=ROOT, env=env,
                                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                code = process.wait()
            if code:
                raise RuntimeError(f"Evaluator exited {code}; see {case / 'driver.log'}")
            validate(case, job["algorithm"], job["design"], args.threads, args.repair_overrides)
            record.update(status="completed", finished=stamp(), elapsed_seconds=time.monotonic()-start)
            collect(args, state)
        except BaseException as error:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
            record.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                          finished=stamp(), error=str(error))
            save(args.output / "status.json", state)
            raise
        record.pop("error", None)
        save(args.output / "status.json", state)
        print(f"completed {job['algorithm']}/{job['design']}", flush=True)
    grouped = collect(args, state)
    print_result_table(args, grouped)
    print(f"\nResults: {args.output / 'report.md'}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
