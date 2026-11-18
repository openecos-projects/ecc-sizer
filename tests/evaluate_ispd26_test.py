#!/usr/bin/env python3
"""Check experiment attribution without requiring the external EDA inputs."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("evaluate_ispd26", ROOT / "scripts/evaluate_ispd26.py")
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)


class NativePathComparisonTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        self.design = "aes_cipher_top"
        self.args = SimpleNamespace(output=self.output, algorithms=["native", "native-path"],
                                    designs=[self.design], threads=32, path_reference=None)
        self.base = dict.fromkeys(evaluation.METRICS, 0)
        self.base.update(setup_wns_ns=-10, setup_tns_ns=-100, placement_checked=1,
                         area_um2=100, total_power_w=0.1, fanout_violations=2)
        self.design_metrics = dict(self.base, setup_wns_ns=-2, setup_tns_ns=-20,
                                   inserted_sizer_buffers=4, fanout_violations=0)
        self.final = dict(self.design_metrics, setup_wns_ns=-1, setup_tns_ns=-10,
                          inserted_sizer_buffers=6)

    def rows(self, algorithm):
        baseline = dict(design=self.design, mode="baseline", metrics=self.base,
                        input_fingerprint={"def": "same-input"}, sizer_seconds=0)
        stages = {name: dict(metrics=metrics, contest_structural_check="passed")
                  for name, metrics in (("input", self.base), ("design", self.design_metrics))}
        pipeline = algorithm == "native-path"
        if pipeline:
            stages["setup"] = dict(metrics=self.final, contest_structural_check="passed")
        result = dict(baseline, mode="sizer", metrics=self.final if pipeline else self.design_metrics,
                      gate_resize=True, repair_design=True, repair_pbbi=False, repair_setup=pipeline,
                      effective_repair_config=evaluation.config(algorithm, 32),
                      contest_structural_check="passed", stage_metrics=stages,
                      path_startpoint="after_native_design", sizer_seconds=8,
                      design_topology=dict(elapsed_ms=2000, inserted_buffers=4, resized_gates=3),
                      setup_topology=dict(elapsed_ms=4000, resized_cells=0, vt_swaps=0,
                                          downsized_loads=0, refresh_ms=3000),
                      path_trials=[dict(accepted=True, rejection_reasons=[]),
                                   dict(accepted=False, rejection_reasons=["no_setup_gain"])])
        return copy.deepcopy([baseline, result])

    def write(self, algorithm, rows=None):
        case = self.output / algorithm / "cases" / self.design
        case.mkdir(parents=True, exist_ok=True)
        (case / "summary.json").write_text(json.dumps(rows or self.rows(algorithm)))
        return case

    def collect(self):
        evaluation.collect(self.args, [dict(algorithm=a, design=self.design, status="completed")
                                       for a in self.args.algorithms])

    def test_separates_native_gain_from_path_gain(self):
        for algorithm in self.args.algorithms:
            self.write(algorithm)
        self.collect()
        row, = json.loads((self.output / "stage-comparison.json").read_text())
        self.assertEqual(row["initial_wns_gain_ns"], 9)
        self.assertEqual(row["path_wns_gain_ns"], 1)
        self.assertEqual(row["path_inserted_buffers"], 2)
        self.assertEqual(row["native_inserted_buffers"], 4)
        self.assertEqual(row["native_resized_gates"], 3)
        self.assertEqual(row["rejection_rate"], 0.5)
        self.assertEqual(row["path_seconds"], 4)
        self.assertEqual(row["approximate_wns_gain_floor_ns"], 0.002)

    def test_ablation_overrides_are_validated_without_changing_thresholds(self):
        overrides = evaluation.parse_overrides(["setup.path_buffer_calibrate=false"])
        original = evaluation.config("native-path", 32)
        changed = evaluation.config("native-path", 32, overrides)
        self.assertEqual(changed["setup.path_buffer_calibrate"], "false")
        for key in ("setup.path_buffer_min_relative_improvement", "setup.min_improvement_ns"):
            self.assertEqual(changed[key], original[key])
        rows = self.rows("native-path")
        rows[1]["effective_repair_config"] = changed
        case = self.write("native-path", rows)
        evaluation.validate(case, "native-path", self.design, 32, overrides)
        rows[1]["effective_repair_config"]["setup.path_buffer_calibrate"] = "true"
        self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "Configuration mismatch"):
            evaluation.validate(case, "native-path", self.design, 32, overrides)

    def test_incremental_path_requires_full_verification_of_every_acceptance(self):
        overrides = {"setup.path_buffer_incremental_parasitics": "true"}
        rows = self.rows("native-path")
        rows[1]["effective_repair_config"] = evaluation.config("native-path", 32, overrides)
        case = self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "without full verification"):
            evaluation.validate(case, "native-path", self.design, 32, overrides)
        rows[1]["path_trials"][0]["full_verified"] = 1
        self.write("native-path", rows)
        evaluation.validate(case, "native-path", self.design, 32, overrides)

    def test_invalid_or_duplicate_overrides_are_rejected(self):
        for values in (["missing_value"], ["a="], ["=1"], ["a=1", "a=2"], ["runtime.sta_threads=8"]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                evaluation.parse_overrides(values)

    def test_final_placement_requires_final_acceptance_and_legal_result(self):
        overrides = {"setup.path_buffer_placement": "final"}
        rows = self.rows("native-path")
        final = rows[1]
        final["effective_repair_config"] = evaluation.config("native-path", 32, overrides)
        for trial in final["path_trials"]:
            trial.update(placement_provisional=1, buffers=2)
        final["setup_topology"].update(path_buffer_provisional_buffers=2,
                                       path_buffer_final_placement_attempted=1)
        case = self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "Missing final placement"):
            evaluation.validate(case, "native-path", self.design, 32, overrides)
        final["setup_topology"]["path_buffer_final_placement_accepted"] = 1
        self.write("native-path", rows)
        evaluation.validate(case, "native-path", self.design, 32, overrides)
        final["metrics"]["setup_wns_ns"] = -3
        final["stage_metrics"]["setup"]["metrics"] = final["metrics"]
        self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "Final placement degraded"):
            evaluation.validate(case, "native-path", self.design, 32, overrides)

    def test_final_placement_rollback_does_not_count_provisional_buffers(self):
        overrides = {"setup.path_buffer_placement": "final"}
        rows = self.rows("native-path")
        final = rows[1]
        final["effective_repair_config"] = evaluation.config("native-path", 32, overrides)
        for trial in final["path_trials"]:
            trial.update(placement_provisional=1, buffers=2)
        final["setup_topology"].update(path_buffer_provisional_buffers=2,
                                       path_buffer_final_placement_attempted=1,
                                       path_buffer_final_placement_rolled_back=1)
        case = self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "retained unexpected buffers"):
            evaluation.validate(case, "native-path", self.design, 32, overrides)
        final["metrics"] = copy.deepcopy(self.design_metrics)
        final["stage_metrics"]["setup"]["metrics"] = final["metrics"]
        self.write("native-path", rows)
        evaluation.validate(case, "native-path", self.design, 32, overrides)
        evaluation.stage_comparison(self.args, {"native-path": rows})
        row, = json.loads((self.output / "stage-comparison.json").read_text())
        self.assertEqual(row["accepted_trials"], 0)
        self.assertEqual(row["provisional_accepted_trials"], 1)
        self.assertEqual(row["path_inserted_buffers"], 0)

    def test_final_verification_requires_full_phase_acceptance(self):
        overrides = {"setup.path_buffer_incremental_parasitics": "true",
                     "setup.path_buffer_verification": "final",
                     "setup.path_buffer_placement": "final"}
        rows = self.rows("native-path")
        final = rows[1]
        final["effective_repair_config"] = evaluation.config("native-path", 32, overrides)
        for trial in final["path_trials"]:
            trial.update(placement_provisional=1, buffers=2, metrics_complete=1, full_verified=0)
        final["setup_topology"].update(path_buffer_provisional_buffers=2,
                                      path_buffer_final_placement_attempted=1)
        case = self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "Missing final placement"):
            evaluation.validate(case, "native-path", self.design, 32, overrides)
        final["setup_topology"]["path_buffer_final_placement_accepted"] = 1
        self.write("native-path", rows)
        evaluation.validate(case, "native-path", self.design, 32, overrides)
        overrides["setup.path_buffer_placement"] = "trial"
        final["effective_repair_config"] = evaluation.config("native-path", 32, overrides)
        self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "phase rollback"):
            evaluation.validate(case, "native-path", self.design, 32, overrides)

    def test_timing_only_trial_cannot_be_reported_as_accepted(self):
        rows = self.rows("native-path")
        rows[1]["path_trials"][0]["metrics_complete"] = 0
        case = self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "without full electrical metrics"):
            evaluation.validate(case, "native-path", self.design, 32)

    def test_rejects_different_initial_input_or_ppa(self):
        self.write("native")
        for change in ("input", "ppa"):
            with self.subTest(change=change):
                rows = self.rows("native-path")
                if change == "input":
                    for row in rows:
                        row["input_fingerprint"] = {"def": "other-input"}
                else:
                    rows[0]["metrics"]["setup_wns_ns"] = -11
                self.write("native-path", rows)
                with self.assertRaisesRegex(ValueError, "Different initial"):
                    self.collect()

    def test_rejects_different_native_checkpoint(self):
        self.write("native")
        rows = self.rows("native-path")
        rows[1]["stage_metrics"]["design"]["metrics"]["setup_wns_ns"] = -3
        self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "Native-alone and pipeline"):
            self.collect()

    def test_rejects_sizing_during_path(self):
        for key in ("resized_cells", "vt_swaps", "downsized_loads"):
            with self.subTest(key=key):
                rows = self.rows("native-path")
                rows[1]["setup_topology"][key] = 1
                case = self.write("native-path", rows)
                with self.assertRaisesRegex(ValueError, "non-buffer edit"):
                    evaluation.validate(case, "native-path", self.design, 32)

    def test_rejects_final_reload_mismatch(self):
        rows = self.rows("native-path")
        rows[1]["metrics"] = dict(rows[1]["metrics"], setup_tns_ns=-11)
        case = self.write("native-path", rows)
        with self.assertRaisesRegex(ValueError, "reloaded output mismatch"):
            evaluation.validate(case, "native-path", self.design, 32)

    def test_rejects_unchecked_stage_and_wrong_startpoint(self):
        for change in ("unchecked", "raw_input"):
            with self.subTest(change=change):
                rows = self.rows("native-path")
                if change == "unchecked":
                    rows[1]["stage_metrics"]["design"]["contest_structural_check"] = "failed"
                else:
                    rows[1]["path_startpoint"] = "raw_input"
                case = self.write("native-path", rows)
                with self.assertRaises(ValueError):
                    evaluation.validate(case, "native-path", self.design, 32)


if __name__ == "__main__":
    unittest.main()
