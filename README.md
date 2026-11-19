<!-- SPDX-License-Identifier: Apache-2.0 -->
# ECC Sizer

ECC Sizer repairs a placed design using OpenDB and OpenSTA. The primary
baselines are **Native**, **VG** and **Path**. **GOBI** is retained as an
additional CUDA algorithm. PBBI is outside the new application flow.

## Build

Initialize the OpenROAD submodule and its nested dependencies:

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF
cmake --build build -j 8
ctest --test-dir build --output-on-failure -R '^ecc_sizer_'
```

Use CMake 3.24+ and a C++20 compiler, with the dependencies required by the
vendored OpenROAD (Tcl, Boost, Eigen, OR-Tools, Qt, etc.). GTest and a Python
interpreter are required when `BUILD_TESTING=ON`. The optional CUDA solvers
use CUDA C++17:

```sh
cmake -S . -B build -DENABLE_GOBI_CUDA=ON -DENABLE_VG_CUDA=ON
```

GOBI requires a usable CUDA device; requesting it without one is an error.
VG defaults to CPU and can use CUDA with `--set vg.backend=cuda`; its existing
CPU fallback policy remains available. Profiling is optional via
`ENABLE_PROFILER`, which requires gperftools.

## Run

A single OpenROAD input script is sufficient. It loads the libraries, physical
design and constraints using standard `read_*`/`link_design` commands:

```sh
build/src/Sizer design.tcl
build/src/Sizer design.tcl --flow vg
build/src/Sizer design.tcl --flow path
build/src/Sizer design.tcl --flow gobi
build/src/Sizer design.tcl --flow vg,native,path,power --output results/repaired
```

Alternatively, pass the input files directly:

```sh
build/src/Sizer --def design.def --lef tech.lef --lef cells.lef \
  --lib cells.lib --sdc design.sdc
build/src/Sizer --odb design.odb --lib cells.lib --sdc design.sdc
```

Repeat `--lef` and `--lib` as needed, with the technology LEF first. Input Tcl
paths are relative to the working directory. Direct DEF/ODB input requires
Liberty and SDC files; ODB does not contain the Liberty timing models.

The default `native` flow runs electrical repair followed by native setup
repair. `path` invokes Path directly, without implicitly running Native first.
Use `--flow design,path` to measure Path after native electrical repair, or
`--flow native,path` to run it after both native phases. Stages share the
current design; inputs are never reloaded between stages.

| Stage | Behavior |
| --- | --- |
| native | Native electrical repair, then native setup repair |
| design / setup | Either native phase independently |
| vg | CPU/CUDA Van Ginneken buffer search |
| path | STA-verified path buffer search |
| gobi | GOBI CUDA buffer search |
| ord | OpenROAD-style driver traversal using Sizer edits/acceptance |
| power | Native power recovery, including supported resizing/VT moves |
| fanout | Standalone fanout buffer insertion |
| legalize | Legalize the current placement and refresh timing |

All new flows require placed-design data with core rows and one STA scene.
They use placement parasitics. Existing wire RC is retained; if absent,
physical RC is derived from the lowest usable routing layer in the LEF.
Use `--rc rc.tcl` when the technology needs an explicit wire model. An SDC
fanout constraint is never relaxed by the application's default limit.
Use OpenROAD's `set_dont_use` and `set_dont_touch` in input Tcl for exclusions.

Output defaults to `<input-stem>.sized.def`, `.v` and `.odb`; `--output PREFIX`
sets their location. Input files must not be reused as the output prefix.

## Configuration and logging

Ordinary runs need no env/cmd or tuning files. `--threads` defaults to the
available hardware concurrency. Algorithm budgets, electrical margins,
acceptance tolerances and experimental policies remain optional through
`--config FILE` and `--set KEY=VALUE`. Configuration files are loaded first;
explicit overrides then win regardless of their position on the command line.
Unknown keys and malformed values fail before design loading.

For example, `--set common.max_fanout=32` overrides the default fanout fallback;
`--set setup.path_buffer_max_trials=0` removes the optional Path trial budget.
The C++ API uses seconds; text options ending in `_ns` use nanoseconds.
VT moves still require explicitly configured library suffixes; cell naming
conventions are not guessed.

New application diagnostics use spdlog. `--log-level debug|info|warn|error|off`
controls them and `--log FILE` adds a file sink. OpenROAD/STA diagnostics use
OpenROAD's logger and its own controls. Tcl failures are checked and stop the
flow instead of allowing acceptance with stale timing.

## Source organization

- `application.cpp`: CLI, stage ordering, shared repair services and output.
- `design_io.*`: OpenROAD input/output, checked Tcl calls, RC defaults and
  supply connectivity. There is no additional LEF/Liberty/DEF/SDC parser here.
- `design_edit.*`: OpenDB buffer insertion and cell-replacement actions. This
  layer has no Sizer, Resizer, timing, candidate-ranking or acceptance policy.
- `repair_*.{h,cpp,cu}`: algorithms, derived analysis data, acceptance checks,
  placement integration and optional CUDA solvers. Native planning produces
  a `NetRepairPlan` before applying edits inside the caller's ECO transaction.
- `sizer.cpp`, `ckt.cpp` and related files: retained sizing and other existing
  flows. They build as `sizer_legacy`; algorithms build as `sizer_algorithms`
  without a dependency on that library. The application links both.

OpenDB remains the authoritative topology/placement store, and OpenSTA owns
constraints/timing. Algorithm graphs and caches are temporary views. Plans
must be rebuilt after intervening topology changes or rollback.

The original sizing, preplace, hold, postroute/SPEF, global-routing RC and
other non-PBBI functionality has **not** been removed. Those implementations
remain accessible via `Sizer legacy -env FILE -f FILE`, including the main
branch's legalization changes. Their old parser/output conventions remain
isolated there; they are not yet migrated to the new entry point. Existing
experiment scripts are not a compatibility contract for the new CLI.

`tests/` contains algorithm, ECO/rollback, legalization and real application
loading/composition tests. `benchmark/`, `results/` and build directories are
local data/output. Do not remove them as source cleanup without inspecting
their contents.
