#!/usr/bin/env bash

set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

binary="${BINARY_PATH:-build/src/Sizer}"
dist_dir="${DIST_DIR:-dist}"
package_name="${PACKAGE_NAME:-ecc-sizer}"
archive_name="${ARCHIVE_NAME:-ecc-sizer-linux-x64.tar.gz}"
package_root="${dist_dir}/${package_name}"
library_search_path="${SIZER_LIBRARY_PATH:-/usr/local/lib:/usr/local/lib64:/opt/or-tools/lib:/opt/or-tools/lib64}"
extra_library_roots="${SIZER_EXTRA_LIBRARY_ROOTS:-/opt/or-tools/lib:/opt/or-tools/lib64}"

if ! command -v lddtree >/dev/null 2>&1; then
  echo "lddtree is required; install pax-utils before packaging." >&2
  exit 1
fi

if [[ ! -x "${binary}" ]]; then
  echo "Sizer binary not found at ${binary}." >&2
  exit 1
fi

rm -rf "${dist_dir}"
mkdir -p "${package_root}/bin" "${package_root}/lib" "${package_root}/libexec"

cp "${binary}" "${package_root}/libexec/Sizer"

IFS=: read -r -a extra_library_root_array <<< "${extra_library_roots}"
for library_root in "${extra_library_root_array[@]}"; do
  if [[ -d "${library_root}" ]]; then
    while IFS= read -r -d '' lib; do
      cp -nL "${lib}" "${package_root}/lib/"
    done < <(find "${library_root}" -maxdepth 1 -type f \( -name "*.so" -o -name "*.so.*" \) -print0)
  fi
done

dependency_list="$(mktemp)"
needed_list="$(mktemp)"
available_list="$(mktemp)"
missing_list="$(mktemp)"
trap 'rm -f "${dependency_list}" "${needed_list}" "${available_list}" "${missing_list}"' EXIT
LD_LIBRARY_PATH="${library_search_path}:${LD_LIBRARY_PATH:-}" \
  lddtree -l "${package_root}/libexec/Sizer" > "${dependency_list}"

while IFS= read -r lib; do
  cp -nL "${lib}" "${package_root}/lib/"
done < <(tail -n +2 "${dependency_list}" | awk '/^\// { print }' | sort -u)

if [[ ! -f "${package_root}/lib/ld-linux-x86-64.so.2" && -f /lib64/ld-linux-x86-64.so.2 ]]; then
  cp -nL /lib64/ld-linux-x86-64.so.2 "${package_root}/lib/"
fi

for lib in \
  /lib/x86_64-linux-gnu/libnss_dns.so.2 \
  /lib/x86_64-linux-gnu/libnss_files.so.2 \
  /lib/x86_64-linux-gnu/libnss_compat.so.2 \
  /lib/x86_64-linux-gnu/libresolv.so.2 \
  /lib/x86_64-linux-gnu/libnss_hesiod.so.2
do
  if [[ -f "${lib}" ]]; then
    cp -nL "${lib}" "${package_root}/lib/"
  fi
done

if [[ -d /usr/share/tcltk/tcl8.6 ]]; then
  cp -a /usr/share/tcltk/tcl8.6 "${package_root}/lib/"
elif [[ -d /usr/lib/tcl8.6 ]]; then
  cp -a /usr/lib/tcl8.6 "${package_root}/lib/"
fi

install -m 0755 .github/scripts/sizer-wrapper.sh "${package_root}/bin/Sizer"

find "${package_root}/lib" -maxdepth 1 -type f -printf '%f\n' | sort -u > "${available_list}"
find "${package_root}/libexec" "${package_root}/lib" -maxdepth 1 -type f -print0 |
  while IFS= read -r -d '' elf; do
    readelf -h "${elf}" >/dev/null 2>&1 || continue
    readelf -d "${elf}" 2>/dev/null |
      awk '/NEEDED/ { name=$0; sub(/.*\[/, "", name); sub(/\].*/, "", name); print name }'
  done | sort -u > "${needed_list}"

while IFS= read -r soname; do
  if ! grep -Fxq "${soname}" "${available_list}"; then
    echo "${soname}" >> "${missing_list}"
  fi
done < "${needed_list}"

if [[ -s "${missing_list}" ]]; then
  echo "Missing bundled shared libraries:" >&2
  cat "${missing_list}" >&2
  exit 1
fi

"${package_root}/lib/ld-linux-x86-64.so.2" \
  --inhibit-cache \
  --inhibit-rpath "" \
  --library-path "${package_root}/lib" \
  --list \
  "${package_root}/libexec/Sizer" >/dev/null

tar --owner=root --group=root -czf "${dist_dir}/${archive_name}" -C "${dist_dir}" "${package_name}"
