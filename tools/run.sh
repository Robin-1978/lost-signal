#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
headless=OFF

if test "${1:-}" = "--headless"; then
 headless=ON
 shift
fi

if test "$headless" = ON; then
 build_dir="${LOST_SIGNAL_BUILD_DIR:-$project_root/build/headless}"
 executable="$build_dir/lost_signal_headless"
 build_arguments=(--headless)
else
 build_dir="${LOST_SIGNAL_BUILD_DIR:-$project_root/build/release}"
 executable="$build_dir/lost_signal"
 build_arguments=()
fi

if ! test -x "$executable"; then
 "$project_root/tools/build.sh" "${build_arguments[@]}"
fi

exec "$executable" "$@"
