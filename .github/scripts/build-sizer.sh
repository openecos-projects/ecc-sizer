#!/usr/bin/env bash

set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

export PATH="/usr/local/bin:${PATH}"
export LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib64:/opt/or-tools/lib:/opt/or-tools/lib64:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="/usr/local/lib:/usr/local/lib64:/opt/or-tools/lib:/opt/or-tools/lib64:${LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="/usr/local:/opt/or-tools:${CMAKE_PREFIX_PATH:-}"

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_TESTS=OFF \
  -DUSE_SYSTEM_BOOST=ON \
  -DUSE_SYSTEM_ABC=OFF \
  -DABC_SKIP_TESTS=ON \
  -DUSE_SYSTEM_OPENSTA=OFF \
  -DOPENROAD_VERSION=v0.1.0-alpha \
  -DZLIB_HOME=/usr/lib/x86_64-linux-gnu \
  -DCMAKE_RULE_MESSAGES=OFF

cmake --build build --target Sizer -j "${NPROC:-$(nproc)}"
