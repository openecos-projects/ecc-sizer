#!/usr/bin/env bash

set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

binary="${BINARY_PATH:-build/src/Sizer}"
dist_dir="${DIST_DIR:-dist}"
package_name="${PACKAGE_NAME:-ecc-sizer}"
archive_name="${ARCHIVE_NAME:-ecc-sizer-linux-x64.tar.gz}"
package_root="${dist_dir}/${package_name}"

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

while IFS= read -r lib; do
  cp -nL "${lib}" "${package_root}/lib/" || true
done < <(lddtree -l "${package_root}/libexec/Sizer" | tail -n +2 | grep "^/" || true)

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

"${package_root}/bin/Sizer" -help || true

tar --owner=root --group=root -czf "${dist_dir}/${archive_name}" -C "${dist_dir}" "${package_name}"
