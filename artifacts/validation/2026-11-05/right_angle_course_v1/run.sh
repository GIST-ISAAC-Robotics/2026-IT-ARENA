#!/usr/bin/env bash
# 직각 코너 시험 월드 단독 1랩 1회. 사용: run.sh <이름> [run_current_solo.py 추가 인자]
# 격리 사본에서 기존 진입점(scripts/run_current_solo.py)을 그대로 실행한다. 재실행·자동 재시도 없음.
MAIN="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="$MAIN/artifacts/validation/2026-11-05/right_angle_course_v1"
WS="${WS_DIR:-$HOME/arena_scratch/right_angle_v1/ws}"
N="${1:?name}"; shift
CW="$A/${WORLD:-course_world}"
OUT="artifacts/right_angle/$N"
if [ -e "$A/$N" ] || [ -e "$A/${N}_sim.exit" ] || [ -e "$WS/$OUT" ]; then echo "existing $N output; no rerun" >&2; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running; run not started)" > "$A/${N}_sim.exit"; exit 3
fi
# 실행 전 확인: 시험 월드 설치, 사본 코드가 복사 당시 기록과 같은지, 공용 트리의 현재 해시.
{
  echo "args=$* world_dir=$CW"
  W="$WS/src/arena_gazebo/worlds/it_arena_official"
  for f in world.sdf scene.json centerline.csv; do
    a=$(sha256sum < "$W/$f" | cut -c1-64); b=$(sha256sum < "$CW/$f" | cut -c1-64)
    echo "world_$f installed=$a expected=$b match=$([ "$a" = "$b" ] && echo 1 || echo 0)"
  done
  ( cd "$WS" && find src scripts tests config -type f ! -path '*/__pycache__/*' ! -name '*.pyc' \
      ! -path 'src/arena_gazebo/worlds/it_arena_official/*' | LC_ALL=C sort | xargs -d '\n' sha256sum ) > "$A/${N}_copy_code_sha256.txt"
  grep -v ' src/arena_gazebo/worlds/it_arena_official/' "$A/isolated_copy_sha256.txt" > "$A/${N}_copy_code_expected.txt"
  echo "copy_code_unchanged_since_copy=$(cmp -s "$A/${N}_copy_code_sha256.txt" "$A/${N}_copy_code_expected.txt" && echo 1 || echo 0)"
  ( cd "$MAIN" && find src scripts tests config -type f ! -path '*/__pycache__/*' ! -name '*.pyc' \
      ! -path 'src/arena_gazebo/worlds/it_arena_official/*' | LC_ALL=C sort | xargs -d '\n' sha256sum ) > "$A/${N}_main_code_sha256.txt"
  echo "copy_code_equals_main_tree_now=$(cmp -s "$A/${N}_copy_code_sha256.txt" "$A/${N}_main_code_sha256.txt" && echo 1 || echo 0)"
  rm -f "$A/${N}_copy_code_expected.txt"
} > "$A/${N}_verify_before.log" 2>&1
if grep -q 'match=0' "$A/${N}_verify_before.log"; then
  echo "ACTUAL_EXIT_CODE=4 (test world not installed; run not started)" > "$A/${N}_sim.exit"; exit 4
fi
{ nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1; uname -r; } > "$A/${N}_host.log"
cd "$WS" || exit 2
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -c "import arena_autonomy.local_pursuit as m; from ament_index_python.packages import get_package_share_directory as g; print(m.__file__); print(g('arena_gazebo'))" >> "$A/${N}_host.log" 2>&1
start=$(date +%s)
echo "start_utc=$(date -u +%FT%TZ)" >> "$A/${N}_preflight.log"
python3 -B scripts/run_current_solo.py "$OUT" "$@" > "$A/${N}_console.log" 2>&1
code=$?
wall=$(( $(date +%s) - start ))
echo "end_utc=$(date -u +%FT%TZ)" >> "$A/${N}_preflight.log"
rsync -a "$WS/$OUT/" "$A/$N/"
echo "ACTUAL_EXIT_CODE=$code WALL_S=$wall" > "$A/${N}_sim.exit"
exit "$code"
