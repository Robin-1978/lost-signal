#!/usr/bin/env bash
# Format all C++ sources with the pinned clang-format style (.clang-format).
# Prefers a local clang-format-14/15 binary, then plain clang-format.
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

formatter=""
for candidate in clang-format-14 clang-format-15 clang-format; do
  if command -v "$candidate" >/dev/null 2>&1; then
    formatter="$candidate"
    break
  fi
done

if test -z "$formatter"; then
  cat >&2 <<'EOF'
No clang-format found. Install one of:
  sudo apt-get install clang-format-14
or unpack a single binary without root:
 apt-get download clang-format-14 && dpkg-deb -x clang-format-*.deb /tmp/lost-signal-cf14
 export PATH="/tmp/lost-signal-cf14/usr/bin:$PATH"
EOF
  exit 1
fi

cd "$project_root"
git ls-files 'src/**/*.cpp' 'src/**/*.hpp' 'src/*.cpp' 'src/*.hpp' 'tests/*.cpp' | xargs "$formatter" -i
git status -sb
