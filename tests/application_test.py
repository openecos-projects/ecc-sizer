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
                     (self.script, "--flow", "native,"),
                     (self.script, "--set", "common.max_fanout=1"),
                     ("--def", "missing.def")):
            self.run_sizer(*args, ok=False)

    def test_checked_tcl_failure_stops_before_output(self):
        self.script.write_text('error "deliberate input failure"\n')
        output = self.run_sizer(self.script, "--threads", "1", ok=False)
        self.assertIn("deliberate input failure", output)
        self.assertFalse(list(self.work.glob("*.odb")))

    def test_native_default_and_odb_round_trip(self):
        prefix = self.work / "native result"
        log = self.work / "sizer.log"
        output = self.run_sizer(self.script, "--threads", "1", "--output", prefix,
                                "--log", log)
        self.assertIn("Starting native", output)
        self.assertIn("native finished", log.read_text())
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


if __name__ == "__main__":
    unittest.main()
