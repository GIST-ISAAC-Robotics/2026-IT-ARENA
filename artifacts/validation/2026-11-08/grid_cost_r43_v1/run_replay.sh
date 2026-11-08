#!/usr/bin/env bash
# r43 재생 일괄 실행. 사용: run_replay.sh [smoke]
# 순서: 이전 트리 준비 → before_1 → after_1 → after_2 → before_2 (계측 없는 ABBA) → capture_1(전수 대조) → compare.
# 각 단계는 env -i 로 PYTHON·ROS·AMENT 환경을 비우고 python3 -I -B 로 실행한다. ROS·Gazebo·Jetson 미사용.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-08/grid_cost_r43_v1
OUT=$A/raw; TREE=$HOME/arena_scratch/r43_before_tree; SMOKE=0
if [ "${1:-}" = smoke ]; then OUT=$A/raw_smoke; TREE=$HOME/arena_scratch/r43_before_tree_smoke; SMOKE=1; fi
mkdir -p "$OUT"
status=0
step() {
  local name=$1; shift
  env -i HOME="$HOME" PATH=/usr/bin:/bin R43_SMOKE=$SMOKE python3 -I -B $A/replay_r43.py "$@" > "$OUT/${name}.log" 2>&1
  local code=$?
  echo "ACTUAL_EXIT_CODE=$code" > "$OUT/${name}.exit"
  [ $code -eq 0 ] || status=1
  return $code
}
step prepare prepare "$TREE" || { echo "ACTUAL_EXIT_CODE=1" > "$OUT/run_all.exit"; exit 1; }
step before_1 run before before_1 "$TREE"
step after_1 run repo after_1
step after_2 run repo after_2
step before_2 run before before_2 "$TREE"
step capture_1 capture capture_1
step compare compare
echo "ACTUAL_EXIT_CODE=$status" > "$OUT/run_all.exit"
exit $status
