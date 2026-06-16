#!/usr/bin/env bash

set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

openroad_deps_prefixes="${OPENROAD_DEPS_PREFIXES:-thirdparty/OpenROAD/etc/openroad_deps_prefixes.txt}"

sudo ./thirdparty/OpenROAD/etc/DependencyInstaller.sh -all \
  -threads="${NPROC:-$(nproc)}" \
  -save-deps-prefixes="${openroad_deps_prefixes}"

sudo apt-get update
sudo apt-get install -y pax-utils
