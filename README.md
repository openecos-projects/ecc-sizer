<!-- SPDX-License-Identifier: Apache-2.0 -->
# ECC Sizer

ECC Sizer repairs placed designs using OpenDB and OpenSTA. It provides
DRV/Setup repair, Path and Van Ginneken buffer search, and GOBI GPU buffer search.
See [docs](docs/README.md) for architecture, algorithms and benchmark results.

## Build and test

Use CMake 3.24+, a C++20 compiler, CUDA, and the dependencies of the vendored
OpenROAD (Tcl, Boost, Eigen, OR-Tools, Qt, etc.). CUDA sources use C++17.
GTest and Python are needed for tests.

```sh
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF
cmake --build build -j 8
ctest --test-dir build --output-on-failure -R '^ecc_sizer_'
```

Use one `build/` directory. VG and GOBI CUDA support are enabled by default.
On a machine without the CUDA toolkit, configure the same directory with
`-DENABLE_VG_CUDA=OFF -DENABLE_GOBI_CUDA=OFF`. GOBI requires a usable GPU;
VG defaults to its CPU solver even in a CUDA build. Select its GPU solver with
`--set buffer.vg.backend=cuda`; VG can fall back to CPU if unavailable.

## Run

An OpenROAD Tcl script loads LEF/Liberty, the design and SDC with standard
`read_*`/`link_design` commands:

```sh
build/src/Sizer design.tcl
build/src/Sizer design.tcl --flow vg
build/src/Sizer design.tcl --flow gobi
build/src/Sizer design.tcl --flow path
build/src/Sizer design.tcl --flow path,drv,setup,power --output results/repaired
```

Direct input is also supported:

```sh
build/src/Sizer --def design.def --lef tech.lef --lef cells.lef \
  --lib cells.lib --sdc design.sdc --rc rc.tcl
build/src/Sizer --odb design.odb --lib cells.lib --sdc design.sdc
```

Repeat `--lef` and `--lib` as needed, technology LEF first. ODB does not contain
Liberty models. Tcl paths are relative to the working directory. Output defaults
to `<input-stem>.sized.def`, `.v` and `.odb`; `--output PREFIX` selects another
location. Do not reuse an input file as the output prefix.

| Flow | Behavior |
|---|---|
| drv | Electrical repair with resize/VT, buffers and load downsizing |
| setup | Setup repair with resize/VT, side-branch buffering and load downsizing |
| path | Path-driven buffer candidate search with measured STA feedback |
| vg | CPU/CUDA Van Ginneken tree search |
| gobi | GOBI CUDA forest search |
| ord | OpenROAD-style driver traversal with Sizer edits and acceptance |
| power | Resizing/VT power search with bounded DRV/setup recovery |
| fanout | Standalone fanout buffering |
| legalize | Placement legalization and timing refresh |
| hold | OpenROAD hold repair, legalization and a second repair pass |
| preplace / preplace-gain | Independent zero-wire-RC repair before placement |

The default is `drv,setup`. Stages run in order on the previously committed
design. Setup does not implicitly call Path, VG or GOBI. Buffer algorithms share
execution infrastructure but keep distinct search strategies. VG's
`buffer.vg.scope=setup` also considers negative-slack nets; GOBI selects DRV nets.

Repair uses one STA scene. Placed flows require core rows; preplace does not.
Path/VG/GOBI require placement RC. DRV, Setup, ORD, Power and Hold also support
`--spef FILE` or `read_spef`, and `--global-routing` with routing setup in Tcl.
Existing RC is retained; when missing it is derived from a usable LEF layer.
Use `--rc rc.tcl` for explicit RC. Input Tcl controls ideal/propagated clocks and
`set_dont_use`/`set_dont_touch`. SDC fanout limits are not relaxed by defaults.

OpenDP is the default legalizer. `--legalizer abacus` supports uniform
single-height row grids and fixed obstacles, falling back to OpenDP for
unsupported layouts. Both protect fixed/DNT cells and check placement.
Power recovery and final-placement Buffer search use outer ECO transactions:
failed validation restores the whole phase, including committed child edits.

## Configuration and logs

Ordinary runs need no tuning file. `--config FILE` and `--set KEY=VALUE` are
optional; command-line overrides apply after configuration files. Unknown keys
fail before loading the design. `--threads N` controls STA threads.

```sh
build/src/Sizer design.tcl --flow setup --set common.actions=resize
build/src/Sizer design.tcl --flow drv,setup --set setup.max_inserted_buffers=12
build/src/Sizer design.tcl --flow path --set buffer.max_inserted_buffers=unlimited
```

`common.actions` selects any subset of `resize,buffer,load_downsize`, or `none`.
Resize includes configured VT changes; VT suffixes must be specified explicitly.
Insertion caps are per stage; `unlimited` means no cap and `0` permits none.
Rejected ECOs do not consume committed insertion counts. VG/GOBI apply complete
tree plans, skipping those that do not fit.

Path defaults to 32 search credits. A trial spends one; new WNS or TNS progress
earns fractional credits, capped at 32. Small accepted gains do not reset the
whole budget. There is no elapsed-time stopping condition. Placement and full
verification default to each trial. The benchmark profile instead selects
final placement/verification and incremental analysis. See
[configuration](docs/repair-config.md) for current settings.

Sizer diagnostics use spdlog: `--log-level debug|info|warn|error|off`, `--log FILE`.
Path reports accepted edits and phase events at info level; debug adds rejection
reasons, gains, candidates and search credits. VG/GOBI report backend and solve
cost. OpenROAD/STA use their own logger. Tcl failures stop the flow.

## Six-design benchmark

```sh
python3 scripts/benchmark_ispd26.py \
  --benchmark /path/to/ISPD26 --output results/ispd26-run
python3 scripts/print_ispd26_results.py results/ispd26-run
python3 scripts/print_ispd26_results.py results/ispd26-run --markdown
```

The default runs AES, JPEG, Ariane and their v2 designs, each with baseline,
DRV+Setup, VG CPU, GOBI CUDA and Path. Every repaired output is independently
reloaded for placement, supply, forbidden-cell and STA/PPA checks. Successful
runs discard generated DEF/V/ODB and keep scripts, logs and JSON. Failed cases
retain their outputs. Identical completed cases can be reused by rerunning the
same command. `--dry-run` writes input scripts and executable commands only.

Each task has `run.sh` for manual invocation. `comparison.md` contains the
Native/VG/Path summary; `tables.txt` and `tables.md` contain GPU and Path tables.
Native in report tables means `--flow drv,setup`. There is no native CLI alias.
The process timeout belongs to the benchmark runner, not the search algorithm.

## Code checks and architecture

```sh
python3 scripts/check_cpp.py --build-dir build --tool clang-tidy
python3 scripts/check_cpp.py --build-dir build --tool clangd
```

Use `--executable /usr/lib/llvm-18/bin/clang-tidy` or `clangd` if needed. The
checker reads the actual compilation database and translates NVCC flags for
Clang's CUDA parser. It does not replace compilation or GPU runtime tests.

[Architecture](docs/modules-and-classes.md) separates application/session,
analysis, repair, algorithms and actions. OpenDB is the topology source of truth;
OpenSTA owns models and timing. Algorithms expose read-only solving/candidate
APIs. Actions apply edits; repair owns STA acceptance, transactions and rollback.
