#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${BUILD_DIR:-${repo_dir}/build-release}"
stage_root="${STAGE_ROOT:-${repo_dir}/build/first-contact-package}"
stage_dir="${stage_root}/lost-signal-first-contact"
archive="${stage_root}/lost-signal-first-contact.tar.gz"

viewer="${build_dir}/sim_viewer"
mujoco="${build_dir}/lib/libmujoco.so.3.16.0"
raylib="${build_dir}/_deps/raylib-build/raylib/libraylib.so.550"

for required in "${viewer}" "${mujoco}" "${raylib}"; do
  if [[ ! -f "${required}" ]]; then
    printf 'missing release artifact: %s\n' "${required}" >&2
    printf 'build it with: cmake --build %s -j6\n' "${build_dir}" >&2
    exit 1
  fi
done

for required in \
  configs/experiences/first_contact.json \
  configs/experiences/zh-CN/first_contact.json \
  assets/fonts/NotoSansCJKsc-Regular.otf \
  assets/models/manifest.json \
  docs/first-contact/README.md \
  docs/first-contact/first_contact_hero_hook_zh.png; do
  if [[ ! -f "${repo_dir}/${required}" ]]; then
    printf 'missing delivery content: %s\n' "${required}" >&2
    exit 1
  fi
done

python3 -c 'import pathlib, shutil, sys; shutil.rmtree(pathlib.Path(sys.argv[1]), ignore_errors=True)' "${stage_root}"
mkdir -p "${stage_dir}/bin" "${stage_dir}/lib"
cp "${viewer}" "${stage_dir}/bin/sim_viewer"
cp "${mujoco}" "${stage_dir}/lib/"
cp "${raylib}" "${stage_dir}/lib/"
cp -a "${repo_dir}/assets" "${repo_dir}/configs" "${stage_dir}/"
mkdir -p "${stage_dir}/docs"
cp -a "${repo_dir}/docs/first-contact" "${stage_dir}/docs/"
cp "${repo_dir}/LICENSE" "${repo_dir}/NOTICE" \
  "${repo_dir}/THIRD_PARTY_NOTICES.md" "${stage_dir}/"

cat >"${stage_dir}/run-first-contact.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${here}"
export LD_LIBRARY_PATH="${here}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
args=("$@")
has_profile=false
for argument in "${args[@]}"; do
  if [[ "${argument}" == "--profile" ]]; then
    has_profile=true
    break
  fi
done
if [[ "${has_profile}" == false ]]; then
  args=(--profile "${here}/player-profile.json" "${args[@]}")
fi
exec "${here}/bin/sim_viewer" --play --3d --locale "${LOCALE:-zh-CN}" "${args[@]}"
EOF
chmod +x "${stage_dir}/run-first-contact.sh"

head="$(git -C "${repo_dir}" rev-parse HEAD)"
branch="$(git -C "${repo_dir}" rev-parse --abbrev-ref HEAD)"
dirty="clean"
if [[ -n "$(git -C "${repo_dir}" status --porcelain --untracked-files=normal)" ]]; then
  dirty="dirty"
fi
cat >"${stage_dir}/BUILD-INFO.txt" <<EOF
format=lost-signal-first-contact-package/1
branch=${branch}
head=${head}
worktree=${dirty}
built_at_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
launch=./run-first-contact.sh
EOF

(
  cd "${stage_dir}"
  find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum >SHA256SUMS
)

tar -C "${stage_root}" -czf "${archive}" lost-signal-first-contact
printf 'First Contact package: %s\n' "${archive}"
printf 'SHA-256: '
sha256sum "${archive}" | cut -d' ' -f1
