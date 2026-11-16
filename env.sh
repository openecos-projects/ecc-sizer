# Source from the repository root before builds or evaluations.
_ECC_SIZER_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export TMPDIR="$_ECC_SIZER_ROOT/.cache/tmp"
export TMP="$TMPDIR"
export TEMP="$TMPDIR"
export XDG_CACHE_HOME="$_ECC_SIZER_ROOT/.cache"
export CCACHE_DIR="$_ECC_SIZER_ROOT/.cache/ccache"
export CUDA_CACHE_PATH="$_ECC_SIZER_ROOT/.cache/cuda"
export TRITON_CACHE_DIR="$_ECC_SIZER_ROOT/.cache/triton"
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$TMPDIR" "$CCACHE_DIR" "$CUDA_CACHE_PATH" "$TRITON_CACHE_DIR"
unset _ECC_SIZER_ROOT
