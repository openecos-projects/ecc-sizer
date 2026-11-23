<!-- SPDX-License-Identifier: Apache-2.0 -->
# ECC Sizer

ECC Sizer repairs a placed design using OpenDB and OpenSTA. The primary
baselines are **DRV + Setup**, **VG** and **Path**. **GOBI** is retained as an
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
VG defaults to CPU and can use CUDA with `--set buffer.vg.backend=cuda`; its existing
CPU fallback policy remains available.

## C++ checks

Generate the compilation database and check project sources with LLVM 18
`clang-tidy` and `clangd`:

```sh
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
python3 scripts/check_cpp.py --build-dir build --tool clang-tidy
python3 scripts/check_cpp.py --build-dir build --tool clangd
```

Use `--executable /usr/lib/llvm-18/bin/clang-tidy` (or `clangd`) if the tool
is not on PATH, and `--jobs N` to limit concurrency. The script checks every
unique project translation unit in `src/` from that build, without changing
files; it excludes third-party sources and test translation units. Enable both
CUDA solvers when configuring to include their `.cu` files. For those files,
the script translates NVCC flags for Clang's parser, preserving C++17 and the
build's include paths/defines; normal C++ files use the actual C++20 commands.
Clang parsing does not replace an NVCC build or CUDA runtime tests.

The root `.clang-tidy` enables focused correctness, performance and modernization
checks, including mandatory control-flow braces. `.clang-format` provides one
format for all project sources; this tree was formatted with clang-format 22.

## Run

A single OpenROAD input script is sufficient. It loads the libraries, physical
design and constraints using standard `read_*`/`link_design` commands:

```sh
build/src/Sizer design.tcl
build/src/Sizer design.tcl --flow vg
build/src/Sizer design.tcl --flow path
build/src/Sizer design.tcl --flow gobi
build/src/Sizer design.tcl --flow path,drv,setup,power --output results/repaired
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

The default flow is `drv,setup`: electrical repair followed by setup repair.
Stages can be ordered independently: `--flow path,drv,setup` first applies Path,
then repairs electrical violations and setup timing. Every stage sees the
preceding committed design. `native` and `design` are no longer stage names.
The benchmark runner uses `drv-setup` for `drv,setup` and `path-drv-setup` for
`path,drv,setup`; these profile names describe ordered stage combinations.

The flow retains `path`, `vg` and `gobi`, including mixed sequences such as
`--flow path,vg,setup`. All three use the shared buffer repair architecture;
there is no separate `buffer` stage or strategy switch. Setup does not invoke
these algorithms implicitly. VG setup scope (`--set buffer.vg.scope=setup`)
changes eligible nets and solver scoring; GOBI currently collects electrically
violating nets. Path iterates with measured feedback; VG/GOBI retain one-shot
whole-tree acceptance. Proposal application, insertion-budget checks,
preparation, refresh and rollback handling are shared.

| Stage | Behavior |
| --- | --- |
| drv | Electrical repair using resize/VT, buffer insertion and load downsizing |
| setup | Setup repair using resize/VT, side-branch buffering and load downsizing |
| path | STA-verified path buffer search |
| vg | CPU/CUDA Van Ginneken buffer search |
| gobi | GOBI CUDA buffer search |
| ord | OpenROAD-style driver traversal using Sizer edits/acceptance |
| power | Cell resizing/VT power search with bounded DRV/setup recovery |
| fanout | Standalone fanout buffer insertion |
| legalize | Legalize the current placement and refresh timing |
| hold | OpenROAD hold repair with legalization and a second repair pass |
| preplace / preplace-gain | Independent zero-wire-RC repair of an unplaced design |

Repair uses one STA scene. Placed flows require core rows; preplace does not.
VG/GOBI/Path use placement parasitics. Existing wire RC is retained;
if absent, physical RC is derived from the lowest usable LEF routing layer.
DRV, setup, ORD, power and hold also support `--spef FILE` (or `read_spef` in input Tcl) for
extracted/postroute parasitics and `--global-routing` for routing estimates. Supply routing setup/tracks in
input Tcl for global routing. Input Tcl controls propagated/ideal clocks.
Use `--rc rc.tcl` when the technology needs an explicit wire model. An SDC
fanout constraint is never relaxed by the application's default limit.
Use OpenROAD's `set_dont_use` and `set_dont_touch` in input Tcl for exclusions.

Output defaults to `<input-stem>.sized.def`, `.v` and `.odb`; `--output PREFIX`
sets their location. Input files must not be reused as the output prefix.

OpenROAD/OpenSTA own LEF, DEF, Verilog, Liberty, SDC and SPEF parsing,
cell timing models, parasitic networks and timing propagation. Algorithms
query these live objects; there is no copied Liberty LUT, RC import graph,
custom timer or timing-report parser.

Use `--flow drv,setup,power` for electrical/setup repair followed by power search.
Power tries slack-budgeted moves first. A larger saving can provisionally
violate setup/DRV and invoke DRV/setup recovery inside the same ECO transaction.
The final design must reduce the chosen STA power objective without increasing
area or degrading setup, hold or electrical checks. Rejection and exceptions
undo both the power move and any committed recovery children. All recovery
trials count against `power.max_trials`; there is no second search engine.

`--legalizer abacus` retains the vendored DreamPlace greedy/Abacus legalizer
for uniform single-height row grids and fixed obstacles. OpenDP remains the
default and handles unsupported layouts, regions, blockages and padding.
Both choices protect fixed/DNT cells and validate placement with OpenROAD.

## Configuration and logging

The env/cmd format and `legacy` entry have been removed. Ordinary runs need no tuning files. `--threads` defaults to the
available hardware concurrency. Algorithm budgets, electrical margins,
acceptance tolerances and experimental policies remain optional through
`--config FILE` and `--set KEY=VALUE`. Configuration files are loaded first;
explicit overrides then win regardless of their position on the command line.
Unknown keys and malformed values fail before design loading.

For example, `--set common.max_fanout=32` overrides the default fanout fallback;
`--set buffer.path.max_trials=unlimited` removes the optional Path trial budget.
Insertion limits are optional for every algorithm: `drv.max_inserted_buffers`,
`setup.max_inserted_buffers` and `buffer.max_inserted_buffers` default to `unlimited` (`std::nullopt` in C++). A number is an
actual cap: `0` permits zero insertions and `12` permits at most twelve. Limits
count committed insertions in that stage; rejected ECOs do not consume them.
VG/GOBI apply complete tree plans and skip plans that do not fit.

Select allowed Sizer repair actions separately with `common.actions`:
`resize,buffer,load_downsize` (default), any subset, or `none`. `resize` includes
buffer/gate drive changes and configured VT moves; `load_downsize` independently
permits shrinking side loads. The setting applies to DRV/ORD, setup, power and
Path/VG/GOBI; explicit OpenROAD hold/preplace and standalone fanout stages keep
their own behavior. For example:

```sh
# Setup through cell replacements only; no buffer insertion or load downsizing.
build/src/Sizer design.tcl --flow setup --set common.actions=resize
# Allow all default actions, with an explicit setup insertion cap.
build/src/Sizer design.tcl --flow drv,setup --set setup.max_inserted_buffers=12
# No insertion count cap; other timing, area and search constraints still apply.
build/src/Sizer design.tcl --flow path --set buffer.max_inserted_buffers=unlimited
```

The former `common.allow_gate_resize` / `common.allow_load_downsize` switches
are replaced by this action list. Configurations use the `drv.*` section;
`design.*` is removed. Old zero-means-unlimited configurations must use
`unlimited` explicitly. Path scheduling and VG candidate solving remain separate
implementations; selecting a VG solver inside Path is not yet exposed.

The C++ API uses seconds; text options ending in `_ns` use nanoseconds.
VT moves still require explicitly configured library suffixes; cell naming
conventions are not guessed.

Application, repair and Abacus diagnostics use spdlog. `--log-level debug|info|warn|error|off`
controls them and `--log FILE` adds a file sink. OpenROAD/STA diagnostics use
OpenROAD's logger and its own controls. Tcl failures are checked and stop the
flow instead of allowing acceptance with stale timing.

Path logs search start/end, accepted commits and final placement/verification
outcomes at info level. Logs are triggered by these events, without a timer or
per-candidate traversal output. `provisional=true` means final legalization and
verification are still pending. Debug logs add each candidate wave's counts,
batch-to-single retry decisions, each measured trial's rejection reasons, signed
global and objective WNS/TNS gains in ns, required gains, candidate net/master,
planned buffer count, and placement/refresh/measurement times.

GOBI debug reports show pre-pruning solution slots, peak layer slots, frontier
reserved capacity, traceback history and savings against retaining load/slack
for all layers. Negative savings indicate reservation overhead on small graphs.
These capacity estimates exclude allocator/reallocation overhead and other GPU
arrays. Separate device-wide used/free/total memory snapshots include CUDA and
other users of the device; they are not GOBI's allocation totals. All sizes use MiB.

## Source organization

- `application.cpp`: CLI and flow validation, session initialization, stage
  execution and output. `ApplicationFlow` owns shared services and callback lifetimes.
- `design_session.*`: OpenROAD session lifetime, input/output, checked Tcl calls, RC defaults and
  supply connectivity. There is no additional LEF/Liberty/DEF/SDC parser here.
  `special_net.*` protects external PAD nets and extends wildcard supply connections
  after insertion; `lef_layer_rc.*` converts already-loaded layer properties and
  fills missing RC during initialization.
- `actions/netlist.*`: buffer insertion, master replacement and scoped ECO
  transactions. Child commits merge into their parent; uncommitted edits roll
  back on scope exit. Algorithms explicitly control timing refresh.
- `actions/placement.*`: OpenDB placement adaptation, protected cells and
  legalization. `actions/legalize_core.*` owns standalone greedy/Abacus geometry.
- `repair/drv.*`: DRV and ORD electrical strategies, standalone fanout repair,
  and power recovery. DRV owns the replacement-versus-buffer fallback policy.
- `repair/setup.*`: setup candidates and STA acceptance. Path, VG and GOBI
  remain independent strategies, not implicit calls inside setup.
- `algorithms/path.*`: path candidates, forecasts, feedback and queue scheduling,
  grouped in one header and implementation. `PathPolicy` ranks candidates
  and learns from measurements; `PathSchedule` owns queue and batch decisions.
  Neither edits the design or decides acceptance.
- `repair/buffer.*`: `BufferRepairer` executes Path/VG/GOBI proposals through
  a shared `BufferTrial` and `BufferExecution` lifecycle. Algorithms own search
  decisions; repair owns edits, acceptance, budgets, STA restoration and final
  legalization. `BufferRepairResult` contains common execution metrics plus
  separate Path search statistics; it does not inherit algorithm state.
- `algorithms/van_ginneken.*`, `gobi.*`: read-only buffer-tree solvers; CUDA
  implementations live beside their algorithms.
- `design_analysis.*`: shared design metrics, OpenDB/STA queries, incremental
  analysis and model evaluation; no dependency on a repair goal or edit module.
- `algorithms/cell_candidates.*`: shared resize/VT and load-downsize candidates.
- `parasitics.h`: placement RC estimation shared by the application and algorithms.
- `repair/config.*`: typed policies, optional insertion limits, complete-plan
  budget selection and a single registry for parsing and serialization.
  `RepairConfig::common` is independent of DRV and all stage settings.
- `repair/runtime.h`: per-run callbacks and design-object exclusions, passed
  separately from configuration. Solvers receive only `DesignConstraints`.
  Buffer callbacks live in `BufferRuntime`; the required refresh callback remains an
  explicit argument of `repair`/`run`. Copying or loading config carries no hooks
  or OpenDB object pointers.

The placement geometry builds as `sizer_placement`; actions and algorithms
build as `sizer_algorithms`, directly linked by the application.

Algorithms expose search separately from execution. `VanGinneken::solve` and
`Gobi::solve` take caller-selected net snapshots and return buffer-tree plans;
`PathSearch::generateCandidates` returns an initial path candidate frontier.
These calls do not edit the netlist, open an ECO or invoke placement/refresh
callbacks. `BufferRepairer::run` combines the Path scheduling policy with ECO
execution; action permissions, insertion limits, STA acceptance and rollback
remain enforced by the runner. The CLI uses these runners; Path does not
automatically invoke VG.

Path combines path-gain forecasts with measured trial feedback to calibrate
later forecasts, diversify candidate families and defer failed variants.
There is one Path strategy; the old `path.strategy` selector and sequential
implementation have been removed. Equal WNS/TNS forecasts prefer fewer buffers
before secondary ranking criteria. Rejection history is scoped to the design
generation and structural context; committed edits reopen affected opportunities.
Optional adaptive exploration also audits empirical failures before suspending
similar candidates. Forecasts guide scheduling, never replace full STA
acceptance, and are not treated as mathematical bounds for pruning.

All three algorithms use `BufferTreePlan` from `actions/netlist.h`: each node
specifies its master, location, direct loads and child indices. Children precede
parents, and a plan can contain multiple roots. `applyBufferTree` validates the
whole topology before editing, supports different masters within a tree, and
leaves acceptance and transaction ownership to its caller. Callers can compose
their own scheduling and solving with these APIs; plans and net snapshots must
be regenerated after changes to the design or STA scene. VG/GOBI results share
`DesignMetrics` with DRV, setup and Path.

The former sizing engine has been removed. Its active strategies map to the
shared repair implementation as follows:

| Former responsibility | Current implementation |
| --- | --- |
| Capacitance/slew repair, drive/VT moves | `repair/drv`, shared cell candidates |
| Negative-slack cell search, side-load relief | `repair/setup` and STA validation |
| Leakage reduction followed by timing recovery | Compound trials in `power` |
| Best-design snapshots and undo | Scoped OpenDB ECO transactions and strict acceptance |
| DreamPlace/Abacus placement | Shared placement adapter and `legalize_core` |
| Liberty/RC import, custom interpolation/timing | OpenROAD/OpenSTA objects and APIs |

Disabled random kicks, nominal multi-start scheduling, unused exponential
path enumeration and technology-specific LUT patches were not carried over.
VG, Path and GOBI retain their independent algorithms; the obsolete `sizing`
entry is removed rather than maintained as a compatibility alias.

OpenDB remains the authoritative topology/placement store, and OpenSTA owns
constraints/timing. Algorithm graphs and caches are temporary views. Plans
must be rebuilt after intervening topology changes or rollback.

Input and output now use the same entry for all retained stages, including
DRV/setup, power, preplace, hold and postroute. Old env/cmd experiment launchers and
submission templates have been deleted. Existing result checkers and OpenROAD
Tcl evaluators remain useful for stored data; historical experiment formats
are not a compatibility contract.

`tests/` contains algorithm, ECO/rollback, legalization and real application
loading/composition tests. `benchmark/`, `results/` and build directories are
local data/output. Do not remove them as source cleanup without inspecting
their contents.

## API vocabulary

Repair workflows use `*Repairer::run`; read-only tree solvers use `solve`, and
Path uses `generateCandidates`. Edit primitives use `applyBufferTree` and
`replaceCell` and never decide timing acceptance. `Settings` hold a stage's
serializable policy, `Options` add common policy, and `Runtime` carries live
constraints and callbacks. No runtime state is copied into `RepairConfig`.

A `Candidate` is a ranked alternative; a `Plan` describes edits against an
unchanged design snapshot; `PathTrialResult` records one measured ECO; a
`RepairResult` describes the completed workflow. Trial counts are batches,
not buffer counts. `rechecked_full_rc` means an additional authoritative RC
check occurred; false does not imply that the normal refresh was incremental.

Use `scripts/benchmark_sizer.py --help` for the current ISPD26 runner. It
loads via OpenROAD Tcl, records input/binary hashes and logs, independently
reloads the final design to check placement and PPA, and removes generated
DEF/Verilog/ODB after evaluation. Historical experiment documents/configurations
are not instructions for the current CLI.

Path execution limits (`max_trials`, `max_seconds`, `max_refreshes`,
`max_consecutive_rejections`), setup's `max_trials`, both stages'
`max_area_growth`, and VG/GOBI `max_nets` also accept `unlimited`. Zero is an
actual cap, including zero area growth. Path has no default total trial cap:
it stops after 32 consecutive rejected single-candidate trials, resetting that
count only after an accepted improvement. This heuristic bounds an unproductive
tail; it does not prove optimality. Setup defaults to 24 trials; Path time/refresh
limits and VG/GOBI net limits are absent. Path requires an absolute improvement
of 0.0001 ns and a relative WNS or TNS improvement of 0.1% of the initial global
violation, with timing/electrical non-regression guards. The default area growth
limit remains 2%. `--set buffer.path.max_consecutive_rejections=unlimited` disables
the stagnation limit; an explicit `buffer.path.max_trials` remains a total hard budget.
VG/GOBI's `require_drv_non_regression` guards electrical violation counts;
strict improvement may be timing or electrical, so it is not named “reduction.”
