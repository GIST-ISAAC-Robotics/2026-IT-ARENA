#!/usr/bin/env bash
# 작업트리의 src/arena_autonomy·arena_vehicle_interface·arena_bringup, scripts, tests, config/tests를 격리 사본으로 고정한다.
# 인자: 사본 이름(~/arena_scratch/straight_refresh/<이름>), 목록·해시 출력 폴더(이 폴더 기준 상대). 작업트리는 읽기만 한다.
set -euo pipefail
NAME=$1; OUTDIR=$2
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R="$(cd "$HERE/../../../.." && pwd)"
O="$HERE/$OUTDIR"; mkdir -p "$O"
T="$HOME/arena_scratch/straight_refresh/$NAME"
cd "$R"
git ls-files -co --exclude-standard src/arena_autonomy src/arena_vehicle_interface src/arena_bringup scripts tests config/tests > "$O/base_file_list.txt"
{ echo "captured_utc $(date -u +%FT%TZ)"; echo "HEAD $(git rev-parse HEAD)"; git status --porcelain=v1; } > "$O/base_git_state.txt"
rm -rf "$T"; mkdir -p "$T"
tar cf - -T "$O/base_file_list.txt" | (cd "$T" && tar xf -)
(cd "$T" && sha256sum $(cat "$O/base_file_list.txt")) > "$O/base_sha256.txt"
wc -l < "$O/base_sha256.txt"
