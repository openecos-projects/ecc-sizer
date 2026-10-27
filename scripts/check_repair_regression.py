#!/usr/bin/env python3
"""Check independently reloaded native-repair results against a versioned policy.

Negative slack is allowed. A passing run must improve on its input, preserve
hold/DRV/placement, and meet the saved quality floor. Runtime gating is optional
because it requires comparable hardware/load; phase counters remain in summary.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def input_fingerprint(design, platform_files):
    digest = hashlib.sha256()
    for path in sorted(platform_files, key=lambda p: p.name):
        digest.update(path.name.encode() + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return dict(def_sha256=hashlib.sha256((design / "contest.def").read_bytes()).hexdigest(),
                sdc_sha256=hashlib.sha256((design / "contest.sdc").read_bytes()).hexdigest(),
                platform_sha256=digest.hexdigest())


def check_power(row):
    """Require reloaded power to agree with the accepted native power stage."""
    config = row.get("effective_repair_config", {})
    if config.get("power.enabled") != "true":
        return []
    name, errors = row.get("design", "unknown"), []
    stats = row.get("power_topology", {})
    metrics = row.get("metrics", {})
    for kind in ("leakage", "total"):
        before, after = stats.get(f"{kind}_before_w"), stats.get(f"{kind}_after_w")
        reloaded = metrics.get(f"{kind}_power_w")
        if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
                   for v in (before, after, reloaded)):
            errors.append(f"{name}: missing/nonpositive/nonfinite {kind} power")
            continue
        # Native console values have six significant digits; reload has full precision.
        if not math.isclose(after, reloaded, rel_tol=1e-5, abs_tol=1e-15):
            errors.append(f"{name}: reloaded {kind} power differs from native result")
        if config.get("power.objective", "leakage") == kind and reloaded > before * (1 + 1e-5):
            errors.append(f"{name}: power objective regressed")
    for key, budget_key in (("attempted_batches", "power.max_trials"),
                            ("accepted_batches", "power.max_passes")):
        value = stats.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            errors.append(f"{name}: invalid power counter {key}")
        elif value > int(config[budget_key]):
            errors.append(f"{name}: power budget exceeded: {key}")
    return errors


def check(rows, policy, reference=None, max_runtime_ratio=None):
    errors = []
    if policy.get("version") != 1:
        return ["unsupported regression policy version"]
    expected = policy["designs"]
    indexed = {}
    for row in rows:
        key = (row.get("design"), row.get("mode"))
        if key in indexed:
            errors.append(f"duplicate result: {key}")
        indexed[key] = row
    tol = policy["tolerances"]
    reference_index = {(r["design"], r["mode"]): r for r in (reference or [])}
    if max_runtime_ratio is not None and (not math.isfinite(max_runtime_ratio) or max_runtime_ratio <= 0):
        return ["runtime ratio must be finite and positive"]
    for name, rule in expected.items():
        base = indexed.get((name, "baseline"))
        final = indexed.get((name, "sizer"))
        if base is None or final is None:
            errors.append(f"{name}: missing baseline/sizer result")
            continue
        for label, row in (("baseline", base), ("sizer", final)):
            if row.get("input_fingerprint") != rule["input_fingerprint"]:
                errors.append(f"{name}/{label}: benchmark/platform fingerprint mismatch")
            if row.get("max_fanout") != rule["max_fanout"]:
                errors.append(f"{name}/{label}: fanout constraint mismatch")
            if not row.get("repair_setup") or not row.get("placement_checked"):
                errors.append(f"{name}/{label}: native setup and placement checks required")
        keys = ["setup_wns_ns", "setup_tns_ns", "hold_wns_ns", "hold_tns_ns",
                "area_um2", "unplaced", "placement_checked", "supply_pin_errors",
                "slew_violations", "cap_violations", "fanout_violations"]
        metrics = [base.get("metrics", {}), final.get("metrics", {})]
        valid = True
        for label, data in zip(("baseline", "sizer"), metrics):
            for key in keys:
                value = data.get(key)
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    errors.append(f"{name}/{label}: missing/nonfinite {key}")
                    valid = False
            if data.get("placement_checked") != 1 or data.get("unplaced") != 0 or data.get("supply_pin_errors") != 0:
                errors.append(f"{name}/{label}: placement/supply check failed")
        if not valid:
            continue
        errors.extend(check_power(final))
        before, after = metrics
        for key, value in rule["baseline_metrics"].items():
            epsilon = tol["time_ns"] if key.endswith("_ns") else abs(value) * tol["area_relative"]
            if abs(before[key] - value) > epsilon:
                errors.append(f"{name}: baseline {key} changed")
        for key in ("setup_wns_ns", "setup_tns_ns"):
            if after[key] <= before[key] + rule["min_improvement_ns"]:
                errors.append(f"{name}: {key} did not improve on input")
            floor = rule["quality_floor"][key]
            epsilon = max(tol["time_ns"], abs(floor) * tol["tns_relative"]) if "tns" in key else tol["time_ns"]
            if after[key] + epsilon < floor:
                errors.append(f"{name}: {key} below saved quality floor")
        for key in ("hold_wns_ns", "hold_tns_ns"):
            epsilon = max(tol["time_ns"], abs(before[key]) * tol["tns_relative"]) if "tns" in key else tol["time_ns"]
            if after[key] + epsilon < before[key]:
                errors.append(f"{name}: {key} regressed")
        for key in ("slew_violations", "cap_violations", "fanout_violations"):
            if after[key] < 0 or after[key] > rule["max_drv_violations"]:
                errors.append(f"{name}: {key} exceeds DRV target")
        if before["area_um2"] <= 0 or after["area_um2"] <= 0 or after["area_um2"] > before["area_um2"] * (1 + rule["max_area_growth"]):
            errors.append(f"{name}: invalid area or area budget exceeded")
        if final.get("contest_structural_check") != "passed":
            errors.append(f"{name}: structural check did not pass")
        if max_runtime_ratio is not None:
            previous = reference_index.get((name, "sizer"), {})
            old, new = previous.get("sizer_seconds"), final.get("sizer_seconds")
            if previous.get("input_fingerprint") != final.get("input_fingerprint"):
                errors.append(f"{name}: runtime reference input mismatch")
            if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in (old, new)):
                errors.append(f"{name}: missing/invalid runtime reference")
            elif new > old * max_runtime_ratio:
                errors.append(f"{name}: runtime ratio {new / old:.3f} > {max_runtime_ratio}")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--policy", type=Path, default=Path(__file__).resolve().parents[1] / "tests/data/repair_regression.json")
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--max-runtime-ratio", type=float)
    args = parser.parse_args()
    errors = check(json.loads(args.summary.read_text()), json.loads(args.policy.read_text()),
                   json.loads(args.reference.read_text()) if args.reference else None, args.max_runtime_ratio)
    for error in errors:
        print(error)
    print("repair regression: " + ("FAILED" if errors else "PASSED"))
    raise SystemExit(bool(errors))


if __name__ == "__main__":
    main()
