#!/usr/bin/env bash
# Cross-build the Rapier FFI shim (native/rapier_shim) for every shipped platform and place the
# resulting libs in lib/native/<platform>/ (where src/util/physics/ffi.lua loads them).
#
# Builds run from a single host. Per-target toolchains:
#   - macOS arm64/x64 : rustup targets (native on a Mac)
#   - Linux x64       : cargo-zigbuild (zig as the cross-linker)
#   - Windows x64     : mingw-w64 (x86_64-pc-windows-gnu)
# A target whose toolchain is missing is SKIPPED with a warning rather than failing the whole run,
# so e.g. a Linux CI box can still produce the Linux/Windows libs.
#
# Usage: scripts/build-native.sh [target-substring]   # optional filter, e.g. "linux"
set -uo pipefail
cd "$(dirname "$0")/.."

crate=native/rapier_shim
filter="${1:-}"
failed=0

build_one() {
  local triple="$1" platdir="$2" libname="$3" method="$4"
  if [[ -n "$filter" && "$platdir" != *"$filter"* && "$triple" != *"$filter"* ]]; then
    return 0
  fi
  echo "==> $platdir ($triple)"
  if ! ( cd "$crate" && case "$method" in
      zig) cargo zigbuild --release --target "$triple" ;;
      *)   cargo build    --release --target "$triple" ;;
    esac ); then
    echo "   SKIP $platdir: build failed (toolchain missing?)" >&2
    failed=1
    return 0
  fi
  mkdir -p "lib/native/$platdir"
  cp "$crate/target/$triple/release/$libname" "lib/native/$platdir/$libname"
  echo "   placed lib/native/$platdir/$libname"
}

#         triple                       platform-dir   lib filename          linker
build_one aarch64-apple-darwin         macos-arm64    librapier_shim.dylib  cargo
build_one x86_64-apple-darwin          macos-x86_64   librapier_shim.dylib  cargo
build_one x86_64-unknown-linux-gnu     linux-x64      librapier_shim.so     zig
build_one x86_64-pc-windows-gnu        windows-x64    rapier_shim.dll       cargo

if [[ "$failed" -ne 0 ]]; then
  echo "native build finished with skips (see warnings above)." >&2
else
  echo "native build complete."
fi
