#!/usr/bin/env python3
"""Exercise regression guards without the external benchmark or EDA runtime."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("repair_regression", ROOT / "scripts/check_repair_regression.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RepairRegressionTest(unittest.TestCase):
    def setUp(self):
        self.policy = json.loads((ROOT / "tests/data/repair_regression.json").read_text())
        self.rows = []
        for name, rule in self.policy["designs"].items():
            for mode in ("baseline", "sizer"):
                metrics = dict(rule["baseline_metrics"], hold_wns_ns=0, hold_tns_ns=0,
                               slew_violations=0, cap_violations=0, fanout_violations=0,
                               unplaced=0, placement_checked=1, supply_pin_errors=0)
                if mode == "sizer":
                    metrics.update(rule["quality_floor"])
                self.rows.append(dict(design=name, mode=mode, metrics=metrics, max_fanout=64,
                                      input_fingerprint=rule["input_fingerprint"], repair_setup=True,
                                      placement_checked=True, contest_structural_check="passed", sizer_seconds=10))

    def test_negative_slack_is_allowed(self):
        self.assertEqual(module.check(self.rows, self.policy), [])

    def test_rejects_missing_duplicate_and_changed_input(self):
        self.assertTrue(module.check(self.rows[:-1], self.policy))
        self.assertTrue(module.check(self.rows + [self.rows[0]], self.policy))
        self.rows[0]["input_fingerprint"] = {}
        self.assertTrue(module.check(self.rows, self.policy))

    def test_quality_and_safety_guards(self):
        for key, value in (("setup_wns_ns", -100), ("setup_tns_ns", -1e9),
                           ("hold_wns_ns", -1), ("hold_tns_ns", -1),
                           ("area_um2", 1e12), ("area_um2", -1),
                           ("slew_violations", 1), ("cap_violations", 1), ("fanout_violations", 1),
                           ("unplaced", 1), ("supply_pin_errors", 1), ("placement_checked", 0),
                           ("setup_wns_ns", float("nan")), ("setup_tns_ns", float("inf"))):
            with self.subTest(key=key, value=value):
                rows = copy.deepcopy(self.rows)
                rows[1]["metrics"][key] = value
                self.assertTrue(module.check(rows, self.policy))
        self.rows[1]["contest_structural_check"] = "failed"
        self.assertTrue(module.check(self.rows, self.policy))

    def test_no_gain_fails_even_if_quality_floor_relaxed(self):
        name = self.rows[1]["design"]
        self.policy["designs"][name]["quality_floor"] = dict(setup_wns_ns=-100, setup_tns_ns=-1e9)
        self.rows[1]["metrics"] = dict(self.rows[0]["metrics"])
        self.assertTrue(module.check(self.rows, self.policy))

    def test_power_reload_and_budget_guards(self):
        row = copy.deepcopy(self.rows[1])
        row["effective_repair_config"] = {"power.enabled": "true", "power.objective": "leakage",
                                          "power.max_trials": "24", "power.max_passes": "6"}
        row["power_topology"] = dict(leakage_before_w=2e-6, leakage_after_w=1e-6,
                                     total_before_w=1e-3, total_after_w=1.1e-3,
                                     accepted_batches=1, attempted_batches=2)
        row["metrics"].update(leakage_power_w=1e-6, total_power_w=1.1e-3)
        self.assertEqual(module.check_power(row), []) # Leakage objective allows dynamic growth.
        for key, value in (("leakage_power_w", 0), ("total_power_w", float("nan")),
                           ("leakage_power_w", 3e-6), ("total_power_w", 0.5e-3)):
            bad = copy.deepcopy(row)
            bad["metrics"][key] = value
            self.assertTrue(module.check_power(bad))
        row["power_topology"]["attempted_batches"] = 25
        self.assertTrue(module.check_power(row))
        row["power_topology"]["attempted_batches"] = 2
        row["effective_repair_config"]["power.objective"] = "total"
        self.assertTrue(module.check_power(row))


    def test_runtime_guard_is_optional_and_checks_reference(self):
        self.assertEqual(module.check(self.rows, self.policy, self.rows, 1.1), [])
        self.assertTrue(module.check(self.rows, self.policy, self.rows, 0.5))
        self.assertTrue(module.check(self.rows, self.policy, None, 1.1))
        self.assertTrue(module.check(self.rows, self.policy, self.rows, float("nan")))
        self.assertTrue(module.check(self.rows, self.policy, self.rows, -1))


if __name__ == "__main__":
    unittest.main()
