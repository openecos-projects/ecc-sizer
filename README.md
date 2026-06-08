# ECC Sizer

ECC Sizer is a gate-sizing research prototype built on top of OpenROAD and
OpenSTA. It reads timing and physical-design inputs, runs sizing and repair
heuristics, and emits updated sizing results for benchmark-style gate-sizing
flows.

The current codebase is intended for research and reproducibility work rather
than as a polished standalone EDA product. It keeps close integration with a
forked OpenROAD tree through Git submodules.

## Repository Layout

- `src/` - ECC Sizer source code and the `Sizer` executable target.
- `submit/` - benchmark submission wrapper files and command templates.
- `test/` - lightweight regression script scaffolding.
- `etc/` - build, dependency, coverage, and helper scripts.
- `cmake/` - project-specific CMake find modules.
- `thirdparty/OpenROAD/` - OpenROAD submodule used by the build.

OpenROAD also contains nested submodules, including OpenSTA and ABC:

- `thirdparty/OpenROAD/src/sta`
- `thirdparty/OpenROAD/third-party/abc`

## Prerequisites

The project uses CMake and a C++ compiler with C++17 support for the top-level
project. OpenROAD may require newer C++ settings internally. The local build
also expects the OpenROAD dependency stack to be available, including Tcl,
Boost, Eigen, OR-Tools, gperftools profiler, and the libraries required by the
OpenROAD modules enabled in the submodule.

The existing build has been verified in this checkout with:

- GCC 11.4
- CMake
- Tcl 8.6
- gperftools profiler
- OpenROAD, OpenSTA, and ABC initialized as submodules

## Clone And Initialize Submodules

Clone the repository and initialize submodules recursively:

```bash
git clone --recursive <repo-url> ecc-sizer
cd ecc-sizer
git submodule update --init --recursive
```

If the repository was cloned without `--recursive`, run:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## Build

Configure and build with CMake:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target Sizer -j "$(nproc)"
```

The executable is generated at:

```text
build/src/Sizer
```

The helper script `etc/Build.sh` wraps the same CMake workflow and exposes a
few build options:

```bash
./etc/Build.sh -dir=build -threads="$(nproc)"
```

## Usage

The executable expects an environment file and a command file:

```bash
./build/src/Sizer -env <env_file> -f <cmd_file>
```

The repository includes example-style templates:

- `src/env_file`
- `src/cmd_file`
- `submit/env_base_file`
- `submit/cmd_base_file`

These files may contain benchmark-specific paths or contest-flow assumptions.
Before running on a new design, update the Liberty, LEF, Verilog, SDC, DEF,
SPEF, output, and top-module settings for your local benchmark environment.

## Submodule Notes

The top-level repository tracks OpenROAD as a submodule. OpenROAD then tracks
OpenSTA and ABC as nested submodules. For reproducible builds, keep the
submodule commits committed in the parent repositories rather than relying only
on local branch checkouts.

Useful status checks:

```bash
git submodule status --recursive
git -C thirdparty/OpenROAD status --short --branch
git -C thirdparty/OpenROAD/src/sta status --short --branch
git -C thirdparty/OpenROAD/third-party/abc status --short --branch
```

## License

The top-level repository is distributed under the MIT License. Some source files
retain BSD-3-Clause notices from prior academic gate-sizing code, and the
OpenROAD, OpenSTA, ABC, Boost, Tcl, gperftools, and other third-party
dependencies are governed by their own licenses. Review the license files in
the corresponding submodules and dependency packages before redistribution.
