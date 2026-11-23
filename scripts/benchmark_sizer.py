#!/usr/bin/env python3
"""Run the current Sizer on ISPD26 inputs and independently reload each output.

Keep logs, input scripts, metrics and provenance; discard generated DEF/V/ODB
once evaluated. Each profile starts from the same original placement.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
PROFILES = {
    "baseline": ("legalize", {}),
    "drv-setup": ("drv,setup", {}),
    "vg": ("vg", {}),
    "path": ("path", {}),
    "path-drv-setup": ("path,drv,setup", {}),
}


def digest(path):
    with path.open("rb") as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
        return digest.hexdigest()


def tcl(value):
    # Double-quoted Tcl word, including paths with whitespace/metacharacters.
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('$', '\\$').replace('[', '\\[') + '"'


def loader(benchmark, design, input_def, threads, dont_use=()):
    restrictions = ""
    if dont_use:
        restrictions = "set_dont_use [list " + " ".join(tcl(p) for p in dont_use) + "]\n"
    return f"""set platform_dir {tcl(benchmark / 'platform')}
set design_path {tcl(benchmark / 'design' / design)}
source $design_path/lib_setup.tcl
foreach lef $lefs {{ read_lef $lef }}
foreach lib $libbest {{ read_liberty $lib }}
{restrictions}read_def {tcl(input_def)}
read_sdc $design_path/contest.sdc
set_ideal_network [all_clocks]
source $rc_file
set_thread_count {threads}
set_max_fanout 64 [current_design]
"""


METRICS = r"""
check_placement -verbose
estimate_parasitics -placement
set_cmd_units -time ns -capacitance pF
# Initializes fanout checking before OpenSTA's low-level count query.
report_check_types -max_slew -max_capacitance -max_fanout -digits 6
set block [ord::get_db_block]
set area 0.0
set unplaced 0
set added 0
set supply_errors 0
foreach inst [$block getInsts] {
    set master [$inst getMaster]
    set area [expr {$area + [$block dbuToMicrons [$master getWidth]] * [$block dbuToMicrons [$master getHeight]]}]
    if {![$inst isPlaced]} { incr unplaced }
    if {[string match sizer_*_buf* [$inst getName]]} { incr added }
    foreach pin [$inst getITerms] {
        set type [$pin getSigType]
        if {$type in {POWER GROUND}} {
            set net [$pin getNet]
            if {$net eq "NULL" || [$net getSigType] ne $type} { incr supply_errors }
        }
    }
}
lassign [sta::design_power [sta::cmd_scene]] internal switching leakage total
foreach {key value} [list area_um2 $area instances [llength [$block getInsts]] \
    nets [llength [$block getNets]] unplaced $unplaced supply_pin_errors $supply_errors \
    inserted_buffers $added total_power_w $total leakage_power_w $leakage \
    setup_wns_ns [sta::worst_negative_slack -max] \
    setup_tns_ns [sta::total_negative_slack -max] \
    hold_wns_ns [sta::worst_negative_slack -min] \
    hold_tns_ns [sta::total_negative_slack -min] \
    slew_violations [sta::max_slew_violation_count] \
    cap_violations [sta::max_capacitance_violation_count] \
    fanout_violations [sta::max_fanout_violation_count]] {
    puts "BENCH_METRIC $key $value"
}
report_checks -path_delay max -digits 6
"""


def execute(command, log, timeout):
    start = time.monotonic()
    with log.open("w") as stream:
        try:
            result = subprocess.run(command, cwd=ROOT, stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=timeout)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = "timeout"
    return {"command": list(map(str, command)), "returncode": code,
            "seconds": time.monotonic() - start}


def run_case(args, design, profile, identity):
    directory = args.output / design / profile
    directory.mkdir(parents=True, exist_ok=True)
    result_file = directory / "result.json"
    if result_file.exists():
        old = json.loads(result_file.read_text())
        if old["identity"] == identity and old["ok"]:
            print(f"REUSE {design}/{profile}", flush=True)
            return old
        raise RuntimeError(f"Refusing to overwrite an existing run: {directory}")
    flow, config = PROFILES[profile]
    config = {**config, **dict(item.split("=", 1) for item in args.set)}
    source = args.benchmark / "design" / design / "contest.def"
    prefix = directory / "repaired"
    input_script = directory / "input.tcl"
    input_script.write_text(loader(args.benchmark, design, source, args.threads, args.dont_use))
    command = [str(args.sizer), str(input_script), "--flow", flow,
               "--threads", str(args.threads), "--output", str(prefix),
               "--log-level", "debug"]
    for key, value in config.items():
        command.extend(["--set", f"{key}={value}"])
    result = {"identity": identity, "design": design, "profile": profile,
              "flow": flow, "overrides": config, "ok": False}
    print(f"RUN {design}/{profile}", flush=True)
    result["sizer"] = execute(command, directory / "sizer.log", args.timeout)
    if result["sizer"]["returncode"] == 0:
        script = directory / "evaluate.tcl"
        # A fresh process of the same binary, terminating from the Tcl loader
        # before the application runs repair stages. No re-legalization here.
        body = loader(args.benchmark, design, prefix.with_suffix('.def'), args.threads, args.dont_use) + METRICS
        script.write_text("if {[catch {\n" + body + "\n} message options]} {\n"
                          "    puts stderr [dict get $options -errorinfo]\n    exit 1\n}\nexit 0\n")
        result["evaluation"] = execute([str(args.sizer), str(script)],
                                       directory / "evaluate.log", args.timeout)
        log = (directory / "evaluate.log").read_text()
        result["metrics"] = {key: float(value) for key, value in
                             re.findall(r"^BENCH_METRIC (\w+) (\S+)$", log, re.M)}
        result["ok"] = (result["evaluation"]["returncode"] == 0 and
                        len(result["metrics"]) == 15 and
                        result["metrics"]["unplaced"] == 0 and
                        result["metrics"]["supply_pin_errors"] == 0)
        for suffix in (".def", ".v", ".odb"):
            prefix.with_suffix(suffix).unlink(missing_ok=True)
    result_file.write_text(json.dumps(result, indent=2) + "\n")
    print(f"{'PASS' if result['ok'] else 'FAIL'} {design}/{profile} "
          f"{result['sizer']['seconds']:.1f}s {result.get('metrics', {})}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--sizer", type=Path, default=ROOT / "build-gbi-cuda3/src/Sizer")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--designs", nargs="+", required=True)
    parser.add_argument("--profiles", nargs="+", choices=PROFILES,
                        default=["baseline", "drv-setup", "vg", "path"])
    parser.add_argument("--threads", type=int, default=32)
    parser.add_argument("--timeout", type=float, default=1800)
    parser.add_argument("--set", action="append", default=[])
    parser.add_argument("--dont-use", action="append", default=[],
                        help="Liberty cell pattern excluded through OpenROAD set_dont_use; repeatable")
    args = parser.parse_args()
    for key in ("benchmark", "sizer", "output"):
        setattr(args, key, getattr(args, key).resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    identity = {"binary_sha256": digest(args.sizer), "threads": args.threads,
                "overrides": args.set, "dont_use": args.dont_use,
                "benchmark": str(args.benchmark),
                "runner_sha256": digest(Path(__file__))}
    manifest = {"identity": identity,
                "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                "design_inputs": {design: {p.name: digest(p) for p in
                    (args.benchmark / "design" / design).iterdir()
                    if p.suffix in (".def", ".v", ".sdc", ".tcl")} for design in args.designs},
                "platform_inputs": {str(p.relative_to(args.benchmark)): digest(p)
                    for p in (args.benchmark / "platform/ASAP7").rglob("*") if p.is_file()},
                "profiles": {name: PROFILES[name] for name in args.profiles}}
    manifest_path = args.output / "manifest.json"
    if manifest_path.exists() and json.loads(manifest_path.read_text()) != manifest:
        raise RuntimeError("Use a new output directory for a different experiment")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output / "source.diff").write_bytes(subprocess.check_output(["git", "diff", "HEAD", "--", "src", "tests", "README.md"], cwd=ROOT))
    results = [run_case(args, design, profile, identity)
               for design in args.designs for profile in args.profiles]
    (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    return not all(result["ok"] for result in results)


if __name__ == "__main__":
    raise SystemExit(main())
