#!/usr/bin/env bash
# 기본 미교정 또는 명시 실장 프로필을 빌드만 한다. 장치 설치/flash 없음.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
if [[ $# -ne 3 && ( $# -ne 5 || ${4:-} != --installed-profile ) ]]; then
  printf 'usage: %s BUILD ST_SOURCE EVIDENCE [--installed-profile DIR]\n' "$0" >&2; exit 2
fi
extra=()
if [[ $# -eq 5 ]]; then extra=(--installed-profile "$5"); fi
exec bash "$task_root/scripts/build_runtime_preview.sh" "${1:?fresh build path}" \
  "${2:?pinned ST source}" "${3:?fresh evidence path}" firmware_runtime_drive "${extra[@]}"
