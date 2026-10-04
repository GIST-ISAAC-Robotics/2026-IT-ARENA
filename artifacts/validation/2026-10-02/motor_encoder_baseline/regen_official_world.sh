#!/usr/bin/env bash
# 차량 설정 변경으로 낡은 공식 팀 파생 실행 월드의 입력 해시를 재생성한다(시뮬레이션 실행 아님).
# 공식 ZIP/assets는 읽기만 한다. 생성 전후 파일 해시와 assets 무결성을 기록한다.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
OUT="$REPO/artifacts/validation/2026-10-02/motor_encoder_baseline"
DEST="src/arena_gazebo/worlds/it_arena_official"
cd "$REPO" || exit 2
set +u
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"
set -u
export PYTHONDONTWRITEBYTECODE=1
(cd "$DEST" && find . -type f -print0 | sort -z | xargs -0 sha256sum) > "$OUT/official_world_before.sha256"
git status --porcelain -- assets > "$OUT/assets_status_before.txt"
python3 scripts/build_official_track.py > "$OUT/official_world_build.log" 2>&1
echo "build_exit=$?" | tee "$OUT/official_world_regen_verdict.txt"
python3 scripts/build_official_track.py --check >> "$OUT/official_world_build.log" 2>&1
echo "check_exit=$?" | tee -a "$OUT/official_world_regen_verdict.txt"
(cd "$DEST" && find . -type f -print0 | sort -z | xargs -0 sha256sum) > "$OUT/official_world_after.sha256"
git status --porcelain -- assets > "$OUT/assets_status_after.txt"
diff "$OUT/official_world_before.sha256" "$OUT/official_world_after.sha256" > "$OUT/official_world_hash_diff.txt"
echo "changed_files=$(grep -c '^>' "$OUT/official_world_hash_diff.txt")" | tee -a "$OUT/official_world_regen_verdict.txt"
grep '^>' "$OUT/official_world_hash_diff.txt" | awk '{print $3}' | tee -a "$OUT/official_world_regen_verdict.txt"
if cmp -s "$OUT/assets_status_before.txt" "$OUT/assets_status_after.txt" && [ ! -s "$OUT/assets_status_after.txt" ]; then
  echo "assets_unchanged=true" | tee -a "$OUT/official_world_regen_verdict.txt"
else
  echo "assets_unchanged=false" | tee -a "$OUT/official_world_regen_verdict.txt"
fi
