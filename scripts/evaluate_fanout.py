#!/usr/bin/env python3
"""Run real ISPD26 fanout experiments inside the Sizer development container.

Inputs stay unchanged. External contest data/checker are not vendored. Results
include reloaded-DEF timing with benchmark RC and the contest structural check;
this is a placement estimate, not routed signoff or formal equivalence.
"""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

from check_repair_regression import input_fingerprint


def run(command, log, env):
    start = time.monotonic()
    with log.open("w") as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                       env=env, check=True)
    return time.monotonic() - start


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--equiv-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--designs", nargs="+",
                        default=["aes_cipher_top", "jpeg_encoder"])
    parser.add_argument("--max-fanout", type=int, default=64)
    parser.add_argument("--buffer-cell", default="",
                        help="explicit Sizer buffer cell (default: auto)")
    parser.add_argument("--repair-design", action="store_true",
                        help="evaluate native cap/slew/fanout repair instead of fanout-only")
    parser.add_argument("--repair-setup", action="store_true",
                        help="run native DRV followed by setup repair")
    parser.add_argument("--repair-gbi", action="store_true",
                        help="run CPU VG/GBI; combine with --repair-setup for GBI -> design -> setup")
    parser.add_argument("--stage-metrics", action="store_true",
                        help="save and independently evaluate each native repair stage")
    parser.add_argument("--setup-trials", type=int)
    parser.add_argument("--setup-passes", type=int)
    parser.add_argument("--repair-config", type=Path)
    parser.add_argument("--repair-set", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--legalize", action="store_true",
                        help="legalize baseline/reference and check all placements (automatic for native repair)")
    parser.add_argument("--no-gate-resize", action="store_true",
                        help="disable ordinary gate resizing for native repair ablation")
    parser.add_argument("--sizer", default="/build/src/Sizer")
    parser.add_argument("--openroad", default="/build/bin/openroad")
    parser.add_argument("--reference", action="store_true",
                        help="also run repair_design and GBI buffer-only setup")
    args = parser.parse_args()
    if args.repair_setup:
        args.repair_design = True
    if any(value is not None and value < 0 for value in (args.setup_trials, args.setup_passes)):
        parser.error("setup budgets must be nonnegative")
    if args.max_fanout < 2:
        parser.error("--max-fanout must be at least 2")
    args.output.mkdir(parents=True, exist_ok=True)
    platform = args.benchmark.resolve() / "platform"
    script = Path(__file__).with_suffix(".tcl").resolve()
    summary = []
    for name in args.designs:
        design = args.benchmark.resolve() / "design" / name
        out = args.output.resolve() / name
        out.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, PLATFORM_DIR=str(platform),
                   DESIGN_PATH=str(design), MAX_FANOUT=str(args.max_fanout),
                   EQUIV_DIR=str(args.equiv_dir.resolve()),
                   CHECK_PLACEMENT=str(int(args.legalize or args.repair_design or args.repair_gbi)),
                   NATIVE_REPAIR=str(int(args.repair_design or args.repair_gbi)))
        libdir = platform / "ASAP7" / "lib"
        libs = sorted(libdir.glob("asap7sc7p5t_*_TT_*.lib"))
        libs.append(libdir / "sram_asap7_64x256_1rw.lib")
        lefdir = platform / "ASAP7" / "lef"
        lefs = [lefdir / "asap7_tech_1x_201209.lef"]
        lefs += sorted(lefdir.glob("asap7sc7p5t_28_*_1x_220121a.lef"))
        lefs.append(lefdir / "sram_asap7_64x256_1rw.lef")
        fingerprint = input_fingerprint(design, libs + lefs + [platform / "ASAP7/util/setRC.tcl"])
        env_file = out / "sizer.env"
        env_file.write_text(f"-home /src\n-libpath {libdir}\n-num_vt 1\n"
                            + "".join(f"-lib {p.name}\n" for p in libs)
                            + "".join(f"-lef {p}\n" for p in lefs)
                            + f"-setRCFile {platform}/ASAP7/util/setRC.tcl\n")
        modes = ["baseline", "sizer"]
        if args.reference:
            modes += ["openroad_design", "openroad_buffer"]
        for mode in modes:
            dest = out / mode
            dest.mkdir(parents=True, exist_ok=True)
            input_def = design / "contest.def"
            runtime = None
            topology = {}
            gbi_topology = {}
            design_topology = {}
            setup_topology = {}
            stage_metrics = {}
            effective_config = {}
            if mode == "sizer":
                cmd_file = out / "sizer.cmd"
                input_def = dest / "repaired.def"
                buffer_option = (f"-buffer_cell {args.buffer_cell}\n"
                                 if args.buffer_cell else "")
                if args.no_gate_resize:
                    buffer_option += "-sizer_repair_no_gate_resize\n"
                if args.repair_gbi:
                    repair_flag = "-sizer_repair_gbi"
                else:
                    repair_flag = "-sizer_repair_design" if args.repair_design else "-sizer_fanout_repair"
                if args.repair_setup:
                    repair_flag = "-sizer_repair_setup"
                    if args.setup_trials is not None:
                        buffer_option += f"-sizer_setup_trials {args.setup_trials}\n"
                    if args.setup_passes is not None:
                        buffer_option += f"-sizer_setup_passes {args.setup_passes}\n"
                if args.repair_gbi and args.repair_design:
                    buffer_option += f"-sizer_repair_gbi {args.max_fanout}\n"
                    repair_flag = "-sizer_repair_setup" if args.repair_setup else "-sizer_repair_design"
                if args.stage_metrics:
                    buffer_option += f"-sizer_repair_stage_prefix {dest}/stage\n"
                if args.repair_config:
                    buffer_option += f"-sizer_repair_config {json.dumps(str(args.repair_config.resolve()), ensure_ascii=False)}\n"
                for entry in args.repair_set:
                    buffer_option += f"-sizer_repair_set {entry}\n"
                cmd_file.write_text(
                    f"-top {name}\n-def {design}/contest.def\n"
                    f"-sdc {design}/contest.sdc\n-noSPEF\n-thread 1\n"
                    "-min_route_layer M1\n-max_route_layer M7\n"
                    f"{repair_flag} {args.max_fanout}\n"
                    + buffer_option +
                    f"-def_out_path {input_def}\n"
                    f"-verilog_out_path {dest}/repaired.v\n")
                runtime = run([args.sizer, "-env", str(env_file), "-f",
                               str(cmd_file)], dest / "sizer.log", env)
                log_text = (dest / "sizer.log").read_text()
                effective = re.search(r"Sizer repair configuration begin\n(.*?)Sizer repair configuration end", log_text, re.S)
                if effective:
                    (dest / "effective-repair.ini").write_text(effective.group(1))
                    effective_config = dict(line.split("=", 1) for line in effective.group(1).splitlines())
                    # Independently reloaded STA must use the same fanout limit.
                    configured_fanout = re.search(r"^common.max_fanout=(\d+)$", effective.group(1), re.M)
                    if int(configured_fanout.group(1)) != args.max_fanout:
                        raise RuntimeError("repair config fanout differs from --max-fanout evaluation constraint")
                if args.repair_gbi:
                    pattern = r"Sizer GBI repair: (.*)"
                else:
                    pattern = r"Sizer design repair: (.*)" if args.repair_design else r"Sizer fanout repair complete: (.*)"
                match = re.search(pattern, log_text)
                if match is None:
                    raise RuntimeError(f"missing Sizer completion: {dest}")
                topology = {k: float(v) for k, v in
                            re.findall(r"(\w+)=([0-9.eE+-]+)", match.group(1))}
                for enabled, label, target in [
                        (args.repair_gbi, "GBI", gbi_topology),
                        (args.repair_design, "design", design_topology)]:
                    if enabled:
                        stage_match = re.search(rf"Sizer {label} repair: (.*)", log_text)
                        if stage_match is None:
                            raise RuntimeError(f"missing {label} completion: {dest}")
                        target.update({k: float(v) for k, v in
                                       re.findall(r"(\w+)=([0-9.eE+-]+)", stage_match.group(1))})
                if args.repair_setup:
                    setup_match = re.search(r"Sizer setup repair: (.*)", log_text)
                    if setup_match is None:
                        raise RuntimeError(f"missing setup completion: {dest}")
                    setup_topology = {k: float(v) for k, v in
                                      re.findall(r"(\w+)=([0-9.eE+-]+)", setup_match.group(1))}
            power_topology = {}
            if mode == "sizer":
                power_match = re.search(r"Sizer power repair: (.*)", log_text)
                if power_match:
                    power_topology = {k: float(v) for k, v in
                                      re.findall(r"(\w+)=([0-9.eE+-]+)", power_match.group(1))}
            eval_env = dict(env, INPUT_DEF=str(input_def), EVAL_MODE=mode,
                            OUTPUT_DIR=str(dest))
            elapsed = run([args.openroad, "-no_init", "-exit", str(script)],
                          dest / "evaluation.log", eval_env)
            metrics = {}
            for line in (dest / "metrics.tsv").read_text().splitlines():
                key, value = line.split("\t")
                metrics[key] = float(value)
            if mode == "sizer" and args.stage_metrics:
                stages = ["input"]
                if args.repair_gbi:
                    stages.append("gbi")
                if args.repair_design:
                    stages.append("design")
                if args.repair_setup:
                    stages.append("setup")
                for stage in stages:
                    stage_def = dest / f"stage.{stage}.def"
                    if not stage_def.is_file():
                        raise RuntimeError(f"missing stage snapshot: {stage_def}")
                    stage_dest = dest / "stages" / stage
                    stage_dest.mkdir(parents=True, exist_ok=True)
                    stage_env = dict(env, INPUT_DEF=str(stage_def), EVAL_MODE="sizer",
                                     OUTPUT_DIR=str(stage_dest))
                    stage_elapsed = run([args.openroad, "-no_init", "-exit", str(script)],
                                        stage_dest / "evaluation.log", stage_env)
                    values = dict((k, float(v)) for k, v in
                                  (line.split("\t") for line in
                                   (stage_dest / "metrics.tsv").read_text().splitlines()))
                    run(["python3", str(args.equiv_dir / "netlist_equiv_check.py"),
                         "--pre_opt", str(out / "baseline"), "--post_opt", str(stage_dest),
                         "--equiv_cells", str(args.equiv_dir / "asap7_equivalent_cell_list.csv")],
                        stage_dest / "equivalence.log", env)
                    stage_metrics[stage] = dict(metrics=values, evaluation_seconds=stage_elapsed,
                                               contest_structural_check="passed")
            row = dict(design=name, mode=mode, power_topology=power_topology, input_fingerprint=fingerprint, max_fanout=args.max_fanout,
                       buffer_cell=args.buffer_cell or "auto",
                       repair_design=args.repair_design,
                       repair_gbi=args.repair_gbi,
                       repair_overrides=args.repair_set,
                       gate_resize=((not args.repair_gbi or args.repair_design) and effective_config.get("common.allow_gate_resize",
                                    str(not args.no_gate_resize).lower()) == "true"),
                       effective_repair_config=effective_config,
                       placement_checked=args.legalize or args.repair_design or args.repair_gbi,
                       evaluation_seconds=elapsed, sizer_seconds=runtime,
                       topology=topology, setup_topology=setup_topology,
                       gbi_topology=gbi_topology, design_topology=design_topology,
                       stage_metrics=stage_metrics,
                       repair_setup=args.repair_setup, metrics=metrics)
            if mode != "baseline":
                run(["python3", str(args.equiv_dir / "netlist_equiv_check.py"),
                     "--pre_opt", str(out / "baseline"), "--post_opt", str(dest),
                     "--equiv_cells", str(args.equiv_dir /
                                          "asap7_equivalent_cell_list.csv")],
                    dest / "equivalence.log", env)
                row["contest_structural_check"] = "passed"
            summary.append(row)
            (args.output / "summary.json").write_text(
                json.dumps(summary, indent=2) + "\n")
            print(json.dumps(row), flush=True)


if __name__ == "__main__":
    main()
