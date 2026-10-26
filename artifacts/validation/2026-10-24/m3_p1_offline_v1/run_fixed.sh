#!/usr/bin/env bash
# 고정 판 1회: 스크립트 사본·소스/입력 해시 보존 → 단위 회귀 → 저장 3기록 대조. ROS/Gazebo 없음.
set -u
REPO="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
BASE="$REPO/artifacts/validation/2026-10-24/m3_p1_offline_v1"
RUN="$BASE/${1:?run name required}"
SCREEN_ARGS="${2:-}"
if [ -e "$RUN" ]; then echo "refuse to overwrite $RUN" >&2; exit 91; fi
mkdir -p "$RUN" || exit 92
cp "$BASE/candidate.py" "$BASE/screen.py" "$BASE/test_candidate.py" "$RUN/" || exit 93
cd "$REPO" || exit 94
sha256sum "$RUN/candidate.py" "$RUN/screen.py" "$RUN/test_candidate.py" \
  src/arena_autonomy/arena_autonomy/local_follow.py src/arena_autonomy/arena_autonomy/local_path.py \
  src/arena_autonomy/arena_autonomy/occupancy_timing.py src/arena_autonomy/arena_autonomy/scene_motion.py \
  src/arena_autonomy/arena_autonomy/local_scene.py src/arena_autonomy/arena_autonomy/lidar_motion.py \
  src/arena_autonomy/arena_autonomy/follow_trace.py \
  artifacts/validation/2026-10-24/e3_onroad_v1/g1/follow_decisions.jsonl \
  artifacts/validation/2026-10-24/e3_onroad_v1/r0plus/follow_decisions.jsonl \
  artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl > "$RUN/hashes_before.txt"
python3 -m pytest -q -p no:cacheprovider "$RUN/test_candidate.py" > "$RUN/pytest.log" 2>&1
printf '%s\n' "$?" > "$RUN/pytest_exit.txt"
python3 -I "$RUN/screen.py" $SCREEN_ARGS > "$RUN/stdout.json" 2> "$RUN/stderr.log"
code=$?
printf '%s\n' "$code" > "$RUN/screen_exit.txt"
sha256sum -c "$RUN/hashes_before.txt" > "$RUN/hashes_after_check.txt" 2>&1
printf '%s\n' "$?" > "$RUN/hashes_after_exit.txt"
exit "$code"
