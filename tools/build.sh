#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
toolchain_dir="${LOST_SIGNAL_TOOLCHAIN_DIR:-$project_root/build/toolchain}"
cmake_minimum="3.25.0"
cmake_bootstrap="3.31.8"
build_client=ON
run_tests=OFF
build_type=RelWithDebInfo
clean_build=OFF

for argument in "$@"; do
 case "$argument" in
 --headless) build_client=OFF ;;
 --test) run_tests=ON ;;
 --debug) build_type=Debug ;;
 --clean) clean_build=ON ;;
 --help|-h)
 cat <<'EOF'
Usage: tools/build.sh [--headless] [--test] [--debug] [--clean]

Builds the project and downloads CMake plus pinned source dependencies when needed.
Environment overrides: CXX, LOST_SIGNAL_BUILD_DIR, LOST_SIGNAL_TOOLCHAIN_DIR.
EOF
 exit 0
 ;;
 *) echo "Unknown option: $argument" >&2; exit 2 ;;
 esac
done

if test -n "${LOST_SIGNAL_BUILD_DIR:-}"; then
 build_dir="$LOST_SIGNAL_BUILD_DIR"
elif test "$build_client" = ON; then
 build_dir="$project_root/build/release"
else
 build_dir="$project_root/build/headless"
fi

if test "$clean_build" = ON; then
 rm -rf "$build_dir"
fi

version_at_least() {
 test "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -n1)" = "$2"
}

cmake_bin="$(command -v cmake || true)"
if test -n "$cmake_bin"; then
 detected_version="$($cmake_bin --version | awk 'NR == 1 {print $3}')"
 if ! version_at_least "$detected_version" "$cmake_minimum"; then
 cmake_bin=""
 fi
fi

if test -z "$cmake_bin"; then
 system_name="$(uname -s)"
 machine_name="$(uname -m)"
 case "$system_name:$machine_name" in
 Linux:x86_64)
 package="cmake-$cmake_bootstrap-linux-x86_64"
 package_sha256="630615d8e98ac33eba7fbe472626dff5c899c85af3c024585ae109166a6909d0"
 ;;
 Linux:aarch64|Linux:arm64)
 package="cmake-$cmake_bootstrap-linux-aarch64"
 package_sha256="609735983e3bdf24b6ab379d918458d64196fe72b98226f62dd5e9fe7b2997cc"
 ;;
 Darwin:x86_64|Darwin:arm64)
 package="cmake-$cmake_bootstrap-macos-universal"
 package_sha256="d1449f969c54d5c00886d5b643340d493dfb3c81cb39ee29b35453395c11ebf7"
 ;;
 *)
 echo "CMake $cmake_minimum or newer is required on $system_name/$machine_name." >&2
 exit 1
 ;;
 esac
 cmake_home="$toolchain_dir/cmake-$cmake_bootstrap"
 cmake_bin="$cmake_home/bin/cmake"
 if ! test -x "$cmake_bin"; then
 if ! command -v curl >/dev/null 2>&1; then
 echo "curl is required once to download CMake $cmake_bootstrap." >&2
 exit 1
 fi
 echo "Bootstrapping CMake $cmake_bootstrap into $cmake_home"
 mkdir -p "$toolchain_dir" "$cmake_home"
 archive="$toolchain_dir/$package.tar.gz"
 curl -fL --retry 3 --retry-delay 2 \
 -o "$archive" \
 "https://cmake.org/files/v3.31/$package.tar.gz"
 if command -v sha256sum >/dev/null 2>&1; then
 printf '%s  %s\n' "$package_sha256" "$archive" | sha256sum --check --status
 elif command -v shasum >/dev/null 2>&1; then
 printf '%s  %s\n' "$package_sha256" "$archive" | shasum -a 256 --check --status
 else
 echo "sha256sum or shasum is required to verify the CMake download." >&2
 exit 1
 fi
 tar -xzf "$archive" --strip-components=1 -C "$cmake_home"
 fi
fi

compiler_supports_required_cxx20() {
 local compiler="$1"
 local probe_dir
 probe_dir="$(mktemp -d "${TMPDIR:-/tmp}/lost-signal-cxx-probe.XXXXXX")"
 cat >"$probe_dir/probe.cpp" <<'EOF'
#include <atomic>
int main() {
 std::atomic<int> value{0};
 value.notify_all();
 value.wait(1);
 return value.load();
}
EOF
 if "$compiler" -std=c++20 "$probe_dir/probe.cpp" -o "$probe_dir/probe" \
  >/dev/null 2>&1; then
  rm -rf "$probe_dir"
  return 0
 fi
 rm -rf "$probe_dir"
 return 1
}

if test -n "${CXX:-}"; then
 compiler_candidates=("$CXX")
else
 # Prefer the user's normal compiler when it satisfies the complete language
 # and library contract. The versioned GNU compiler is the verified release
 # toolchain on the development workstation; Clang is a compatibility path.
 compiler_candidates=(g++-13 clang++ c++)
fi

cxx_bin=""
for candidate in "${compiler_candidates[@]}"; do
 if command -v "$candidate" >/dev/null 2>&1 &&
  compiler_supports_required_cxx20 "$candidate"; then
  cxx_bin="$(command -v "$candidate")"
  break
 fi
done
if test -z "$cxx_bin"; then
 if test -n "${CXX:-}"; then
  echo "CXX=$CXX cannot compile the required C++20 atomic wait/notify API." >&2
 else
  echo "A compiler with C++20 atomic wait/notify support is required (GCC 11+, GCC 13 verified)." >&2
 fi
 exit 1
fi

if test -f "$build_dir/CMakeCache.txt"; then
 cached_cxx="$(sed -n 's/^CMAKE_CXX_COMPILER:FILEPATH=//p' "$build_dir/CMakeCache.txt" | head -n1)"
 if test -n "$cached_cxx" &&
  test "$(readlink -f "$cached_cxx")" != "$(readlink -f "$cxx_bin")"; then
  echo "Build directory $build_dir was configured with $cached_cxx, but this run selected $cxx_bin." >&2
  echo "Run again with --clean or choose a separate LOST_SIGNAL_BUILD_DIR." >&2
  exit 1
 fi
fi

echo "Configuring ${build_type} build in $build_dir"
echo "C++ compiler: $cxx_bin"
echo "CMake dependencies are pinned and downloaded automatically on first build."

CXX="$cxx_bin" "$cmake_bin" -S "$project_root" -B "$build_dir" \
 -DCMAKE_BUILD_TYPE="$build_type" \
 -DLOST_SIGNAL_BUILD_CLIENT="$build_client" \
 -DLOST_SIGNAL_BUILD_TESTS=ON
"$cmake_bin" --build "$build_dir" --parallel "${LOST_SIGNAL_JOBS:-2}"

if test "$run_tests" = ON; then
 "$cmake_bin" --build "$build_dir" --target test
fi

echo "Build complete: $build_dir"
if test "$build_client" = ON; then
 echo "Run: $build_dir/lost_signal"
else
 echo "Run: $build_dir/lost_signal_headless --recommended"
fi
