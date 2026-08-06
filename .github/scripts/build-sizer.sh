#!/usr/bin/env bash

set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

export PATH="/usr/local/bin:${PATH}"
export LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib64:/opt/or-tools/lib:/opt/or-tools/lib64:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="/usr/local/lib:/usr/local/lib64:/opt/or-tools/lib:/opt/or-tools/lib64:${LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="/usr/local:/opt/or-tools:${CMAKE_PREFIX_PATH:-}"

openroad_deps_prefixes="${OPENROAD_DEPS_PREFIXES:-thirdparty/OpenROAD/etc/openroad_deps_prefixes.txt}"
cmake_dependency_args=()
cudd_dir="${CUDD_DIR:-/usr/local}"

if [[ -f "${openroad_deps_prefixes}" ]]; then
  read -r -a cmake_dependency_args < "${openroad_deps_prefixes}"
  for ((i = 0; i < ${#cmake_dependency_args[@]}; i++)); do
    if [[ "${cmake_dependency_args[i]}" == "-Dcudd_ROOT="* ]]; then
      cudd_dir="${cmake_dependency_args[i]#-Dcudd_ROOT=}"
    elif [[ "${cmake_dependency_args[i]}" == "-D" && "${cmake_dependency_args[i + 1]:-}" == cudd_ROOT=* ]]; then
      cudd_dir="${cmake_dependency_args[i + 1]#cudd_ROOT=}"
    fi
  done
fi

cmake -S . -B build \
  "${cmake_dependency_args[@]}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_TESTS=OFF \
  -DUSE_SYSTEM_BOOST=ON \
  -DUSE_SYSTEM_ABC=OFF \
  -DABC_SKIP_TESTS=ON \
  -DBUILD_PYTHON=OFF \
  -DUSE_SYSTEM_OPENSTA=OFF \
  -DOPENROAD_VERSION=v0.1.0-alpha \
  -DENABLE_PROFILER="${ENABLE_PROFILER:-OFF}" \
  -DCUDD_DIR="${cudd_dir}" \
  -DZLIB_HOME=/usr/lib/x86_64-linux-gnu \
  -DCMAKE_RULE_MESSAGES=OFF

cmake --build build --target Sizer -j "${NPROC:-$(nproc)}"
