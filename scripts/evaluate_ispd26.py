#!/usr/bin/env python3
"""Reproduce the six-design CPU / CUDA / Path work-summary experiment.

Run from any directory. Inputs are read-only; every algorithm starts from the
original DEF, with no preceding Native design repair or gate sizing. The
profiles below reproduce the archived VG/GBI and explicit-tree Path settings,
using the selected current binaries. Historical timings are not guaranteed.
No time, trial, refresh, buffer-count or area budget is imposed on Path.
Jobs run sequentially, so GPU tasks do not compete for memory.

Examples:
  python3 scripts/evaluate_ispd26.py --dry-run
  python3 scripts/evaluate_ispd26.py --output results/ispd26-repro
  python3 scripts/evaluate_ispd26.py --output results/ispd26-repro --resume
  python3 scripts/evaluate_ispd26.py --designs aes_cipher_top --algorithms gbi path

Each case reuses evaluate_fanout.py for independent OpenROAD PPA evaluation,
placement/supply checks and the contest structural check. This does not replace
the additional connectivity/fault-injection audit archived with the report.
Outputs: manifest.json, status.json, report.md, summary.csv, and per-algorithm
summary.json plus cases/<design>/ logs, effective configuration and netlists.
Requires the external ISPD26 ASAP7 inputs/checker and a CUDA-enabled Sizer for GBI.
"""

import argparse
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


ROOT = Path(__file__).resolve().parents[1]
DESIGNS = {
    "aes_cipher_top": "AES", "aes_cipher_top_v2": "AES v2",
    "jpeg_encoder": "JPEG", "jpeg_encoder_v2": "JPEG v2",
    "ariane": "Ariane", "ariane_v2": "Ariane v2",
}
FLAGS = {"vg": "--repair-vg", "gbi": "--repair-gbi", "path": "--path-from-input"}
LABELS = {"vg": "CPU", "gbi": "CUDA", "path": "Path"}
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
        "setup.path_buffer_max_seconds": "0", "setup.path_buffer_max_refreshes": "0",
        "setup.path_buffer_max_inserted_buffers": "0", "setup.path_buffer_max_area_growth": "-1",
        "setup.path_buffer_min_relative_improvement": "0.001",
        "setup.min_improvement_ns": "0.0001", "setup.defer_rollback_refresh": "true",
    },
}
METRICS = ("setup_wns_ns", "setup_tns_ns", "hold_wns_ns", "hold_tns_ns",
           "slew_violations", "cap_violations", "fanout_violations", "area_um2",
           "total_power_w", "inserted_sizer_buffers", "unplaced",
           "placement_checked", "supply_pin_errors")


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


def config(algorithm, threads):
    return dict(COMMON, **PROFILES[algorithm], **{"runtime.sta_threads": str(threads)})


def command(args, algorithm, design):
    out = args.output / algorithm / "cases" / design
    cmd = [sys.executable, "-u", str(ROOT / "scripts/evaluate_fanout.py"),
           "--benchmark", str(args.benchmark), "--equiv-dir", str(args.equiv_dir),
           "--designs", design, FLAGS[algorithm], "--max-fanout", "64",
           "--sizer", str(args.sizer), "--openroad", str(args.openroad),
           "--dont-use-list", str(args.dont_use_list), "--output", str(out)]
    if algorithm == "path":
        cmd.append("--stage-snapshots")
    for key, value in config(algorithm, args.threads).items():
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
    source_files = sorted(p for p in (ROOT / "src").iterdir()
                          if p.suffix in (".cpp", ".h", ".cu") and p.is_file())
    git_head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    return {
        "profile": "ispd26-work-summary-six-v1", "git_head": git_head,
        "python": sys.version, "threads": args.threads, "concurrency": 1,
        "designs": args.designs, "algorithms": args.algorithms,
        "profiles": {a: config(a, args.threads) for a in args.algorithms}, "jobs": jobs,
        "files_sha256": {str(p): sha256(p) for p in sorted(files)},
        "sources_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in source_files},
        "environment": {k: os.environ.get(k) for k in ("CUDA_VISIBLE_DEVICES", "LD_LIBRARY_PATH", "OMP_NUM_THREADS")},
    }


def validate(case, algorithm, design, threads):
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
    if final.get("contest_structural_check") != "passed" or final["gate_resize"]:
        raise ValueError("Missing structural check or unexpected gate resizing")
    if final["repair_design"] or final["repair_pbbi"]:
        raise ValueError("Unexpected preceding design repair or PBBI stage")
    effective = final["effective_repair_config"]
    for key, expected in config(algorithm, threads).items():
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
    if algorithm in ("vg", "gbi") and not final[algorithm + "_topology"]:
        raise ValueError("Missing algorithm completion counters")
    return rows


def collect(args, state):
    grouped = {a: [] for a in args.algorithms}
    baselines = {}
    for job in state:
        if job["status"] != "completed":
            continue
        a, d = job["algorithm"], job["design"]
        rows = validate(args.output / a / "cases" / d, a, d, args.threads)
        base = next(r for r in rows if r["mode"] == "baseline")
        reference = (base["input_fingerprint"], {k: base["metrics"][k] for k in METRICS})
        if d in baselines and baselines[d] != reference:
            raise ValueError("Different initial PPA/input between algorithms: " + d)
        baselines[d] = reference
        grouped[a].extend(rows)
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
             "从原始输入分别运行 CPU、CUDA、Path；无 Native design 前置阶段。",
             "仅列已完成并通过独立 PPA/结构检查的结果，阶段失败见 status.json。",
             "单场景 placement RC、默认活动率估计功耗；不是布线签核。",
             "CPU/CUDA 为不同算法，耗时比不是同算法的纯硬件加速比。", "",
             "| 设计 | 流程 | WNS (ns) | TNS (ns) | S/C/F | 面积 (µm²) | 功耗 (mW) | buffer | 求解 (s) | Sizer (s) |",
             "|---|---|---:|---:|---|---:|---:|---:|---:|---:|"]
    for d in args.designs:
        shown_base = False
        for a in args.algorithms:
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
    (args.output / "report.md").write_text("\n".join(lines) + "\n")


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
    parser.add_argument("--algorithms", nargs="+", choices=FLAGS, default=list(FLAGS))
    parser.add_argument("--threads", type=int, default=32, help="STA threads (report: 32)")
    parser.add_argument("--dry-run", action="store_true", help="print commands only; do not create outputs")
    parser.add_argument("--resume", action="store_true", help="skip validated completed cases; preserve failed attempts")
    args = parser.parse_args()
    if args.threads < 1 or len(set(args.designs)) != len(args.designs) or len(set(args.algorithms)) != len(args.algorithms):
        parser.error("threads must be positive; designs and algorithms must not contain duplicates")
    for name in ("benchmark", "equiv_dir", "sizer", "openroad", "dont_use_list", "output"):
        setattr(args, name, getattr(args, name).expanduser().resolve())
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
            validate(case, job["algorithm"], job["design"], args.threads)
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
    print(f"Results: {args.output / 'report.md'}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
