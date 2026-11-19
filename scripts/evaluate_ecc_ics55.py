#!/usr/bin/env python3
"""Run Sizer repair on local snapshots of ECC ics55 placement results.

The ECC benchmark is deliberately copied into the repository's ignored
``benchmark/ecc_ics55_inputs`` directory before a run.  The source workspace
is read only; generated Sizer/OpenROAD files go below ``results/``.
The default comparison uses the common ``post_pl`` designs of two runs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
from collections import defaultdict


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_ROOT = ROOT / "benchmark" / "ecc_ics55_inputs"
DEFAULT_OUTPUT = ROOT / "results" / "ecc-ics55"
TCL = Path(__file__).with_suffix(".tcl").resolve()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read_json(path: Path):
    return json.loads(path.read_text())


def find_source_designs(source: Path, stage: str) -> dict[str, dict]:
    dataset = read_json(source / "dataset.json")
    specs = {item["name"]: item for item in dataset["designs"]}
    result = {}
    for name, spec in specs.items():
        stage_dir = source / "workspaces" / name / "artifacts" / "ecc" / stage
        required = [stage_dir / x for x in ("design.def", "design.v", "constraints.sdc")]
        metadata = source / "workspaces" / name / "artifacts" / "ecc" / "post_rt" / "stage_metadata.json"
        if all(path.is_file() for path in required) and metadata.is_file():
            result[name] = {"spec": spec, "stage_dir": stage_dir, "metadata": metadata}
    return result


def snapshot_source(source: Path, destination: Path, stage: str) -> dict:
    designs = find_source_designs(source, stage)
    destination.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source / "dataset.json", destination / "dataset.json")
    copied = {}
    for name, item in designs.items():
        target = destination / "workspaces" / name / "artifacts" / "ecc" / stage
        target.mkdir(parents=True, exist_ok=True)
        for filename in ("design.def", "design.v", "constraints.sdc"):
            source_file = item["stage_dir"] / filename
            target_file = target / filename
            shutil.copy2(source_file, target_file, follow_symlinks=True)
        # post_pl metadata is often empty; post_rt metadata contains the
        # effective ics55 library/LEF configuration and is copied locally.
        meta_target = destination / "workspaces" / name / "artifacts" / "ecc" / "post_rt"
        meta_target.mkdir(parents=True, exist_ok=True)
        shutil.copy2(item["metadata"], meta_target / "stage_metadata.json")
        copied[name] = {
            "design": name,
            "source": str(item["stage_dir"].resolve()),
            "files": {
                filename: sha256(target / filename)
                for filename in ("design.def", "design.v", "constraints.sdc")
            },
        }
    manifest = {
        "source": str(source.resolve()),
        "stage": stage,
        "copied_design_count": len(copied),
        "designs": copied,
    }
    (destination / "snapshot_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def metadata_config(metadata: Path) -> dict:
    data = read_json(metadata)
    config = data.get("effective_flow_config", {})
    if not config:
        raise RuntimeError(f"missing effective_flow_config in {metadata}")
    libs = config.get("liberty_corners", {}).get("tt", {}).get("lib_paths", [])
    lefs = list(config.get("lef_paths", []))
    tlef = config.get("interconnect_rc_corners", {}).get("cworst", {}).get("tlef_path")
    if tlef and tlef not in lefs:
        lefs.insert(0, tlef)
    if not libs or not lefs:
        raise RuntimeError(f"incomplete ics55 library configuration in {metadata}")
    return {
        "libs": [str(Path(x).resolve()) for x in libs],
        "lefs": [str(Path(x).resolve()) for x in lefs],
        "rc_layer": "MET2",
        "pdk": config.get("process_node", "ics55"),
        "rc_corner": data.get("effective_flow_config", {}).get("interconnect_rc_corner", "cworst"),
    }


def liberty_buffer_cells(path: Path) -> list[dict]:
    """Extract ordinary one-input buffer candidates from a Liberty file.

    The ECC ics55 library also contains tri-state buffers.  They are useful
    library cells, but are not valid ECO insertion candidates for this
    evaluator.  This small parser deliberately uses only top-level cell
    blocks and the positive-unate ``function: \"A\"`` marker, so the runner
    does not need a Liberty Python dependency.
    """
    text = path.read_text(errors="replace")
    cells = []
    matches = list(re.finditer(r"^  cell \(([^)]+)\) \{", text, re.MULTILINE))
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        name = match.group(1)
        block = text[match.start():end]
        if not (name.startswith("BUF") and not name.startswith("TBUF")):
            continue
        if "three_state" in block or not re.search(r'function\s*:\s*"A"', block):
            continue
        area_match = re.search(r"\barea\s*:\s*([0-9.eE+-]+)", block)
        cap_match = re.search(r"\bmax_capacitance\s*:\s*([0-9.eE+-]+)", block)
        cells.append({
            "name": name,
            "area": float(area_match.group(1)) if area_match else float("inf"),
            "max_capacitance": float(cap_match.group(1)) if cap_match else 0.0,
        })
    return cells


def write_ics55_buffer_filter(libs: list[str], path: Path, limit: int) -> dict:
    """Keep a balanced strength sweep and exclude every other buffer master.

    The three ics55 LLSC libraries expose the same drive-strength sweep for
    H7H/H7L/H7R.  A balanced quota per suffix preserves small, medium and
    large choices in each flavor while satisfying the CUDA kernel's 32-slot
    constant-memory limit.  All tri-state/other buffer-like masters are
    explicitly excluded as well.
    """
    regular = {}
    all_buffer_names = set()
    for lib in libs:
        text = Path(lib).read_text(errors="replace")
        matches = list(re.finditer(r"^  cell \(([^)]+)\) \{", text, re.MULTILINE))
        for index, match in enumerate(matches):
            end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
            name = match.group(1)
            block = text[match.start():end]
            if not (name.startswith("BUF") or name.startswith("INV") or name.startswith("DLY")):
                continue
            all_buffer_names.add(name)
            if name.startswith("BUF") and not name.startswith("TBUF") \
                    and "three_state" not in block \
                    and re.search(r'function\s*:\s*"A"', block):
                area_match = re.search(r"\barea\s*:\s*([0-9.eE+-]+)", block)
                regular[name] = float(area_match.group(1)) if area_match else float("inf")
    if not regular:
        raise RuntimeError("no ordinary ics55 buffer cells found in Liberty files")
    if limit < 1:
        raise ValueError("buffer candidate limit must be positive")

    def group(name: str) -> str:
        match = re.search(r"(H7[HLR])$", name)
        return match.group(1) if match else "other"

    groups = defaultdict(list)
    for name, area in regular.items():
        groups[group(name)].append((name, area))
    group_names = sorted(groups)
    keep = []
    remaining = min(limit, len(regular))
    for group_index, group_name in enumerate(group_names):
        values = sorted(groups[group_name], key=lambda item: (item[0], item[1]))
        quota = min(len(values), remaining // (len(group_names) - group_index))
        remaining -= quota
        if quota == 0:
            continue
        if quota == 1:
            indexes = [len(values) // 2]
        else:
            indexes = sorted({round(i * (len(values) - 1) / (quota - 1)) for i in range(quota)})
        keep.extend(values[index][0] for index in indexes)
    keep = sorted(set(keep))
    excluded = sorted(all_buffer_names - set(keep))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(excluded) + "\n")
    return {"limit": limit, "candidate_count": len(keep), "candidates": keep,
            "excluded_count": len(excluded), "excluded": excluded,
            "selection": "balanced H7H/H7L/H7R drive-strength sweep; deterministic coverage, not global PPA search"}


def run(command: list[str], log: Path, env: dict[str, str]) -> float:
    log.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    with log.open("w") as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, env=env, check=True)
    return time.monotonic() - start


def parse_metrics(path: Path) -> dict[str, float]:
    return {key: float(value) for key, value in
            (line.split("\t") for line in path.read_text().splitlines())}


def run_one(args, version: str, inputs: Path, name: str, spec: dict, output: Path) -> dict:
    source_stage = inputs / "workspaces" / name / "artifacts" / "ecc" / args.stage
    metadata = inputs / "workspaces" / name / "artifacts" / "ecc" / "post_rt" / "stage_metadata.json"
    cfg = metadata_config(metadata)
    design_output = output / version / name
    design_output.mkdir(parents=True, exist_ok=True)
    baseline_output = design_output / "baseline"
    baseline_output.mkdir(exist_ok=True)
    base_env = dict(os.environ,
                    LEF_FILES="\n".join(cfg["lefs"]),
                    LIB_FILES="\n".join(cfg["libs"]),
                    RC_LAYER=cfg["rc_layer"],
                    STA_THREADS=str(args.sta_threads),
                    INPUT_SDC=str((source_stage / "constraints.sdc").resolve()),
                    OUTPUT_DIR=str(baseline_output.resolve()),
                    INPUT_DEF=str((source_stage / "design.def").resolve()))
    openroad = Path(args.openroad).resolve()
    baseline_seconds = run([str(openroad), "-no_init", "-exit", str(TCL)],
                           baseline_output / "evaluation.log", base_env)
    result = {"version": version, "design": name, "mode": "baseline",
              "evaluation_seconds": baseline_seconds,
              "metrics": parse_metrics(baseline_output / "metrics.tsv")}
    if args.mode == "baseline":
        return result

    repair = design_output / "sizer"
    repair.mkdir(exist_ok=True)
    env_file = repair / "sizer.env"
    env_file.write_text("-home " + str(ROOT) + "\n" +
                        "-libpath " + str(Path(cfg["libs"][0]).parent) + "\n" +
                        "-num_vt 1\n" +
                        "".join(f"-lib {Path(lib).name}\n" for lib in cfg["libs"]) +
                        "".join(f"-lef {lef}\n" for lef in cfg["lefs"]))
    top = spec["top"]
    command_file = repair / "sizer.cmd"
    flag = {"native": "-sizer_repair_setup",
            "vg": "-sizer_repair_vg",
            "gobi": "-sizer_repair_gobi"}[args.mode]
    command_text = (
        f"-top {top}\n-def {source_stage / 'design.def'}\n"
        f"-sdc {source_stage / 'constraints.sdc'}\n-noSPEF\n-thread 1\n"
        f"-min_route_layer MET2\n-max_route_layer MET5\n{flag} {args.max_fanout}\n"
        f"-sizer_repair_set runtime.sta_threads={args.sta_threads}\n")
    if args.buffer_filter:
        command_text += f"-dont_use_list {args.buffer_filter}\n"
    command_text += (
        f"-def_out_path {repair / 'repaired.def'}\n"
        f"-verilog_out_path {repair / 'repaired.v'}\n")
    command_file.write_text(command_text)
    sizer_env = dict(os.environ)
    sizer_seconds = run([str(Path(args.sizer).resolve()), "-env", str(env_file), "-f", str(command_file)],
                        repair / "sizer.log", sizer_env)
    repaired_output = repair / "evaluation"
    repaired_output.mkdir(exist_ok=True)
    eval_env = dict(base_env, INPUT_DEF=str((repair / "repaired.def").resolve()),
                    OUTPUT_DIR=str(repaired_output.resolve()))
    eval_seconds = run([str(openroad), "-no_init", "-exit", str(TCL)],
                       repaired_output / "evaluation.log", eval_env)
    result.update({"mode": args.mode, "sizer_seconds": sizer_seconds,
                   "evaluation_seconds": baseline_seconds + eval_seconds,
                   "repair_metrics": parse_metrics(repaired_output / "metrics.tsv"),
                   "sizer_log": str((repair / "sizer.log").resolve())})
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pr329", type=Path, required=True)
    parser.add_argument("--pr342", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, default=DEFAULT_INPUT_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sizer", default=str(ROOT / "build-gbi-cuda3/src/Sizer"))
    parser.add_argument("--openroad", default=str(ROOT / "build/bin/openroad"))
    parser.add_argument("--stage", default="post_pl", choices=["post_pl"])
    parser.add_argument("--mode", choices=["baseline", "native", "vg", "gobi"], default="native")
    parser.add_argument("--max-fanout", type=int, default=32)
    parser.add_argument("--buffer-candidates", type=int, default=32,
                        help="maximum VG/GOBI buffer masters; ics55 defaults to 32")
    parser.add_argument("--sta-threads", type=int, default=4)
    parser.add_argument("--designs", nargs="*", help="design names; default is the common input set")
    parser.add_argument("--copy-input", action="store_true", help="copy source input snapshots locally before running")
    args = parser.parse_args()
    if args.max_fanout < 2 or args.sta_threads < 1 or args.buffer_candidates < 1:
        parser.error("max-fanout must be >= 2, sta-threads positive, and buffer-candidates positive")
    args.output.mkdir(parents=True, exist_ok=True)
    if args.copy_input:
        for version, source in (("ecc_pr329_1011", args.pr329), ("ecc_pr342_1018", args.pr342)):
            snapshot_source(source.resolve(), (args.input_root / version).resolve(), args.stage)
    inputs = {"ecc_pr329_1011": args.input_root / "ecc_pr329_1011",
              "ecc_pr342_1018": args.input_root / "ecc_pr342_1018"}
    available = [find_source_designs(path, args.stage) for path in inputs.values()]
    names = set(available[0]) & set(available[1])
    if args.designs:
        names &= set(args.designs)
    if not names:
        raise SystemExit("no common local post_pl designs; use --copy-input first")
    args.buffer_filter = ""
    buffer_selection = None
    if args.mode in {"vg", "gobi"}:
        first_name = sorted(names)[0]
        first_metadata = inputs["ecc_pr329_1011"] / "workspaces" / first_name / "artifacts" / "ecc" / "post_rt" / "stage_metadata.json"
        cfg = metadata_config(first_metadata)
        filter_path = args.output / "ics55-buffer-excluded.list"
        buffer_selection = write_ics55_buffer_filter(cfg["libs"], filter_path, args.buffer_candidates)
        args.buffer_filter = str(filter_path.resolve())
        (args.output / "ics55-buffer-selection.json").write_text(
            json.dumps(buffer_selection, indent=2) + "\n")
    specs = read_json(inputs["ecc_pr329_1011"] / "dataset.json")["designs"]
    specs = {item["name"]: item for item in specs}
    manifest = {"mode": args.mode, "stage": args.stage, "max_fanout": args.max_fanout,
                "buffer_candidates": args.buffer_candidates,
                "buffer_selection": buffer_selection,
                "sta_threads": args.sta_threads, "designs": sorted(names),
                "inputs": {key: str(value.resolve()) for key, value in inputs.items()},
                "sizer": str(Path(args.sizer).resolve()), "openroad": str(Path(args.openroad).resolve())}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    rows = []
    for name in sorted(names):
        for version in inputs:
            try:
                row = run_one(args, version, inputs[version], name, specs[name], args.output)
                rows.append(row)
                (args.output / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
                print(json.dumps(row), flush=True)
            except Exception as exc:
                row = {"version": version, "design": name, "mode": args.mode,
                       "status": "failed", "error": str(exc)}
                rows.append(row)
                (args.output / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
                print(json.dumps(row), flush=True)


if __name__ == "__main__":
    main()
