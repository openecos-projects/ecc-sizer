<!-- SPDX-License-Identifier: Apache-2.0 -->
# ECC Sizer

ECC Sizer repairs a placed design using OpenDB and OpenSTA. The primary
baselines are **Native**, **VG** and **Path**. **GOBI** is retained as an
additional CUDA algorithm. PBBI has been removed.

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
CPU fallback policy remains available.

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
| hold | OpenROAD hold repair with legalization and a second repair pass |
| preplace / preplace-gain | Independent zero-wire-RC repair of an unplaced design |
| sizing | Retained cell-sizing search, using the loaded OpenROAD design |

Repair uses one STA scene. Placed flows require core rows; preplace does not.
Native/ORD/VG/GOBI/Path use placement parasitics. Existing wire RC is retained;
if absent, physical RC is derived from the lowest usable LEF routing layer.
Sizing and hold also support `--spef FILE` (or `read_spef` in input Tcl) for
extracted/postroute parasitics and `--global-routing` for routing estimates. Supply routing setup/tracks in
input Tcl for global routing. Input Tcl controls propagated/ideal clocks.
Use `--rc rc.tcl` when the technology needs an explicit wire model. An SDC
fanout constraint is never relaxed by the application's default limit.
Use OpenROAD's `set_dont_use` and `set_dont_touch` in input Tcl for exclusions.

Output defaults to `<input-stem>.sized.def`, `.v` and `.odb`; `--output PREFIX`
sets their location. Input files must not be reused as the output prefix.

## Configuration and logging

The env/cmd format and `legacy` entry have been removed. Ordinary runs need no tuning files. `--threads` defaults to the
available hardware concurrency. Algorithm budgets, electrical margins,
acceptance tolerances and experimental policies remain optional through
`--config FILE` and `--set KEY=VALUE`. Configuration files are loaded first;
explicit overrides then win regardless of their position on the command line.
Unknown keys and malformed values fail before design loading.

For example, `--set common.max_fanout=32` overrides the default fanout fallback;
`--set path.max_trials=0` removes the optional Path trial budget.
The C++ API uses seconds; text options ending in `_ns` use nanoseconds.
VT moves still require explicitly configured library suffixes; cell naming
conventions are not guessed.

Application, repair and retained sizing diagnostics use spdlog. `--log-level debug|info|warn|error|off`
controls them and `--log FILE` adds a file sink. OpenROAD/STA diagnostics use
OpenROAD's logger and its own controls. Tcl failures are checked and stop the
flow instead of allowing acceptance with stale timing.

## Source organization

- `application.cpp`: CLI, stage ordering, shared repair services and output.
- `design_io.*`: OpenROAD input/output, checked Tcl calls, RC defaults and
  supply connectivity. There is no additional LEF/Liberty/DEF/SDC parser here.
- `design_edit.*`: buffer insertion, master replacement, legalization and scoped
  ECO transactions. Child commits merge into their parent; uncommitted edits
  roll back on scope exit. Algorithms explicitly control timing refresh.
- `repair_drv.*`: Native and ORD electrical repair, standalone fanout repair,
  load downsizing and power recovery. Their acceptance policies remain explicit.
- `repair_timing.*`: native setup repair and timing interfaces. VG, GOBI and
  Path retain their separate, substantial solver implementations; Path has
  independent `path.*` settings and never runs implicitly inside setup repair.
- `repair_backend.*`: read-only electrical analysis, candidate planning and
  acceptance helpers. Implementations are compiled once per backend library.
- `repair_config.*`: typed algorithm policies and a single key/field registry
  for strict parsing, unit conversion and serialization. `[path]` is supported;
  `gobi.backend` has been removed because GOBI always uses CUDA.
- `cell_sizing.*`, `sizing_model.*`, `liberty_import.cpp`: retained cell-sizing
  search and its derived analysis arrays. Liberty models come from loaded STA
  objects. These build as `sizer_search`, separate from `sizer_algorithms`.
  Placement, global-routing and extracted RC use an explicit mode; the old
  external timer launchers and their unused configuration have been removed.

OpenDB remains the authoritative topology/placement store, and OpenSTA owns
constraints/timing. Algorithm graphs and caches are temporary views. Plans
must be rebuilt after intervening topology changes or rollback.

Input and output now use the same entry for all retained stages, including
sizing, preplace, hold and postroute. Old env/cmd experiment launchers and
submission templates have been deleted. Existing result checkers and OpenROAD
Tcl evaluators remain useful for stored data; historical experiment formats
are not a compatibility contract.

`tests/` contains algorithm, ECO/rollback, legalization and real application
loading/composition tests. `benchmark/`, `results/` and build directories are
local data/output. Do not remove them as source cleanup without inspecting
their contents.
