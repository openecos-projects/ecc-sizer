#!/usr/bin/env python3
"""Exercise the real CLI, OpenROAD loader, stage composition and ODB output."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

BINARY = Path(sys.argv.pop(1)).resolve()
ROOT = Path(__file__).resolve().parents[1]
TECH = ROOT / "thirdparty/OpenROAD/test/Nangate45/Nangate45.lef"
LIB = ROOT / "thirdparty/OpenROAD/test/Nangate45/Nangate45_typ.lib"
SDC = ROOT / "tests/data/fanout_repair_sta.sdc"


class ApplicationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="sizer [input] ")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.script = self.work / "design {input}.tcl"
        self.script.write_text(f"""
read_lef {{{TECH}}}
read_liberty {{{LIB}}}
read_verilog {{{ROOT / 'tests/data/fanout_repair_sta.v'}}}
link_design top
add_global_connection -net VDD -inst_pattern .* -pin_pattern VDD -power
add_global_connection -net VSS -inst_pattern .* -pin_pattern VSS -ground
global_connect
initialize_floorplan -die_area {{0 0 100 100}} -core_area {{5 5 95 95}} \\
    -site FreePDK45_38x28_10R_NP_162NW_34O
set x 10000
foreach inst [[ord::get_db_block] getInsts] {{
    $inst setLocation $x 10000
    $inst setPlacementStatus PLACED
    incr x 2000
}}
set y 10000
foreach term [[ord::get_db_block] getBTerms] {{
    set pin [odb::dbBPin_create $term]
    odb::dbBox_create $pin [[ord::get_db_tech] findLayer metal2] 10000 $y 10100 [expr {{$y + 100}}]
    $pin setPlacementStatus PLACED
    incr y 2000
}}
read_sdc {{{SDC}}}
set_wire_rc -signal -layer metal2
set_wire_rc -clock -layer metal2
set_max_fanout 4 [current_design]
""")

    def run_sizer(self, *args, ok=True):
        result = subprocess.run(
            [str(BINARY), *map(str, args)], cwd=self.work,
            env=dict(os.environ, QT_QPA_PLATFORM="offscreen"),
            capture_output=True, text=True, timeout=180,
        )
        output = result.stdout + result.stderr
        if ok:
            self.assertEqual(result.returncode, 0, output)
        else:
            self.assertNotEqual(result.returncode, 0, output)
        return output

    def test_help_and_invalid_arguments(self):
        self.assertIn("--flow", self.run_sizer("--help"))
        for args in (("--threads",), ("--wat",),
                     (self.script, "--flow", "pbbi"),
                     (self.script, "--flow", "sizing"),
                     (self.script, "--flow", "native"),
                     (self.script, "--flow", "design"),
                     (self.script, "--legalizer", "missing"),
                     ("-env", "old.env", "-f", "old.cmd"),
                     ("legacy", "-env", "old.env"),
                     (self.script, "--set", "pbbi.enabled=true"),
                     (self.script, "--set", "setup.path_buffer_only=true"),
                     (self.script, "--set", "power.enabled=true"),
                     (self.script, "--flow", "drv,"),
                     (self.script, "--set", "common.max_fanout=1"),
                     ("--def", "missing.def")):
            self.run_sizer(*args, ok=False)

    def test_checked_tcl_failure_stops_before_output(self):
        self.script.write_text('error "deliberate input failure"\n')
        output = self.run_sizer(self.script, "--threads", "1", ok=False)
        self.assertIn("deliberate input failure", output)
        self.assertFalse(list(self.work.glob("*.odb")))

    def test_invalid_flow_is_rejected_before_loading_design(self):
        self.script.write_text('error "input must not execute"\n')
        for flow in ("drv,preplace", "preplace,setup", "path,preplace-gain",
                     "preplace-gain,drv", "preplace,preplace"):
            with self.subTest(flow=flow):
                output = self.run_sizer(self.script, "--flow", flow, ok=False)
                self.assertIn("independent flow", output)
                self.assertNotIn("input must not execute", output)
        for flow in ("preplace", "preplace-gain", "path", "vg", "gobi"):
            for rc in (("--spef", "missing.spef"), ("--global-routing",)):
                with self.subTest(flow=flow, rc=rc):
                    output = self.run_sizer(self.script, "--flow", flow, *rc,
                                            ok=False)
                    self.assertIn("zero wire RC" if flow.startswith("preplace")
                                  else "placement RC", output)
                    self.assertNotIn("input must not execute", output)
        self.assertFalse(list(self.work.glob("*.odb")))

    def test_drv_setup_default_and_odb_round_trip(self):
        prefix = self.work / "drv setup result"
        log = self.work / "sizer.log"
        output = self.run_sizer(self.script, "--threads", "1", "--output", prefix,
                                "--log", log)
        self.assertLess(output.index("Starting drv"), output.index("Starting setup"))
        self.assertIn("setup finished", log.read_text())
        for suffix in (".def", ".v", ".odb"):
            self.assertTrue(Path(str(prefix) + suffix).is_file())
        next_prefix = self.work / "roundtrip"
        output = self.run_sizer("--odb", str(prefix) + ".odb", "--lib", LIB,
                                "--sdc", SDC, "--flow", "vg,path,power",
                                "--threads", "1", "--output", next_prefix)
        self.assertIn("Placement RC from LEF", output)
        self.assertLess(output.index("Starting vg"), output.index("Starting path"))
        self.assertLess(output.index("Starting path"), output.index("Starting power"))
        self.assertTrue(Path(str(next_prefix) + ".odb").is_file())
        self.run_sizer("--odb", str(prefix) + ".odb", "--lib", LIB, "--sdc", SDC,
                       "--output", prefix, ok=False)

    def test_composed_goals_with_explicit_actions_and_limits(self):
        output = self.run_sizer(
            self.script, "--flow", "path,drv,setup", "--threads", "1",
            "--set", "common.actions=resize",
            "--set", "drv.max_inserted_buffers=0",
            "--set", "setup.max_inserted_buffers=unlimited",
            "--set", "buffer.max_inserted_buffers=2",
            "--output", self.work / "composed")
        self.assertLess(output.index("Starting path"), output.index("Starting drv"))
        self.assertLess(output.index("Starting drv"), output.index("Starting setup"))
        self.assertTrue((self.work / "composed.odb").is_file())

    def test_path_trial_logging_respects_level_and_reports_measured_gains(self):
        self.script.write_text(self.script.read_text().replace(
            "fanout_repair_sta.v", "gate_repair_sta.v") + """
create_clock -name clk -period 0.5 [get_ports clk]
set_input_delay 0 -clock clk [get_ports {in enable}]
set_output_delay 0.49 -clock clk [get_ports out0]
""")
        for level in ("debug", "info", "off"):
            with self.subTest(level=level):
                log = self.work / f"{level}.log"
                output = self.run_sizer(
                    self.script, "--flow", "path", "--threads", "1",
                    "--log-level", level, "--log", log,
                    "--set", "buffer.path.max_area_growth=1", "--set", "buffer.path.max_trials=3",
                    "--output", self.work / level)
                if level == "debug":
                    self.assertIn("accepted=true", output)
                    self.assertIn("accepted=false", output)
                    self.assertIn("reasons=[no_setup_gain", output)
                    self.assertIn("global gain WNS=+", output)
                    self.assertRegex(output, r"global gain WNS=[+-]\d+\.\d+ns TNS=[+-]\d+\.\d+ns")
                    self.assertIn("objective gain WNS=", log.read_text())
                else:
                    self.assertNotIn("Path trial", output)
                    self.assertNotIn("Path trial", log.read_text())
                if level != "off":
                    self.assertEqual(output.count("Path search started:"), 1)
                    self.assertEqual(output.count("Path search finished:"), 1)
                    self.assertIn("Path commit:", output)
                    self.assertLess(output.index("Path search started:"),
                                    output.index("Path commit:"))
                    self.assertLess(output.index("Path commit:"),
                                    output.index("Path search finished:"))
                else:
                    self.assertNotIn("Path search", output)
                    self.assertNotIn("Path commit:", output)
                if level == "debug":
                    self.assertIn("Path candidates: wave=", output)
                else:
                    self.assertNotIn("Path candidates:", output)
                self.assertNotIn("Path progress:", output)

    def test_fanout_and_def_input(self):
        prefix = self.work / "fanout"
        output = self.run_sizer(self.script, "--threads", "1", "--flow", "fanout",
                                "--output", prefix)
        self.assertIn("Fanout repair inserted", output)
        output = self.run_sizer("--def", str(prefix) + ".def", "--lef", TECH,
                                "--lib", LIB, "--sdc", SDC, "--threads", "1",
                                "--flow", "ord,legalize", "--log-level", "warn",
                                "--output", self.work / "direct")
        self.assertNotIn("Starting ord", output)

    def test_hold_and_unplaced_preplacement(self):
        output = self.run_sizer(self.script, "--flow", "hold", "--threads", "1",
                                "--output", self.work / "hold")
        self.assertIn("hold finished", output)
        # Preplacement must work without rows/locations or a wire RC model.
        self.script.write_text(f"""
read_lef {{{TECH}}}
read_liberty {{{LIB}}}
read_verilog {{{ROOT / 'tests/data/fanout_repair_sta.v'}}}
link_design top
read_sdc {{{SDC}}}
""")
        for stage in ("preplace", "preplace-gain"):
            output = self.run_sizer(self.script, "--flow", stage, "--threads", "1",
                                    "--output", self.work / stage)
            self.assertIn(f"{stage} finished", output)
            self.assertTrue((self.work / f"{stage}.odb").is_file())

    def test_spef_and_global_routing_hold(self):
        spef = self.work / "input parasitics.spef"
        with self.script.open("a") as script:
            script.write(f"estimate_parasitics -placement -spef_file {{{spef}}}\n")
        self.run_sizer(self.script, "--flow", "hold", "--threads", "1",
                       "--output", self.work / "seed")
        original = spef.read_bytes()
        # Reload the seed design directly, without overwriting the SPEF input.
        output = self.run_sizer("--odb", self.work / "seed.odb", "--lib", LIB,
                                "--sdc", SDC, "--spef", spef, "--flow", "hold",
                                "--threads", "1", "--output", self.work / "postroute")
        self.assertIn("hold finished", output)
        self.assertEqual(spef.read_bytes(), original)
        extracted_script = self.work / "extracted.tcl"
        extracted_script.write_text(f"""
read_liberty {{{LIB}}}
read_db {{{self.work / 'seed.odb'}}}
read_sdc {{{SDC}}}
read_spef {{{spef}}}
""")
        output = self.run_sizer(extracted_script, "--flow", "hold", "--threads", "1",
                                "--output", self.work / "script-spef")
        self.assertIn("hold finished", output)
        output = self.run_sizer(extracted_script, "--flow", "drv,setup,power",
                                "--threads", "1", "--output", self.work / "drv-setup-spef")
        self.assertIn("setup finished", output)
        self.assertIn("power finished", output)
        self.assertEqual(spef.read_bytes(), original)
        self.assertIn("placement RC", self.run_sizer(extracted_script, "--flow", "path", ok=False))
        self.assertIn("zero wire RC", self.run_sizer(extracted_script, "--flow", "preplace", ok=False))
        with self.script.open("a") as script:
            script.write("make_tracks\nset_routing_layers -signal metal2-metal6 -clock metal2-metal6\n")
        output = self.run_sizer(self.script, "--global-routing", "--flow", "drv,setup,power,hold",
                                "--threads", "1", "--output", self.work / "routed")
        self.assertIn("hold finished", output)

    def test_power_uses_loaded_design_and_abacus(self):
        with self.script.open("a") as script:
            script.write("set_input_delay 0 -clock clk [get_ports in]\n"
                         "set_output_delay 1 -clock clk [get_ports out*]\n")
        output = self.run_sizer(self.script, "--flow", "drv,setup,power", "--legalizer", "abacus", "--threads", "1",
                                "--output", self.work / "drv-setup-power")
        self.assertIn("power finished", output)
        self.assertTrue((self.work / "drv-setup-power.odb").is_file())


if __name__ == "__main__":
    unittest.main()
