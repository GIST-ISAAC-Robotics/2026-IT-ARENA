#!/usr/bin/env bash
# 파일럿 단독 실행 1회와 회수. 인자: 이름 프로필.yaml. 재시도 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-04/lidar_mount_speed_v4
N=$1; P=$2
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running)" > "$A/${N}.exit"; exit 3
fi
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B "$A/run_solo.py" "$N" --profile "$P" --prepare-only >> "$A/${N}_preflight.log" 2>&1 || { echo "ACTUAL_EXIT_CODE=4 (prepare failed)" > "$A/${N}.exit"; exit 4; }
start=$(date +%s)
/usr/bin/python3 -B "$A/run_solo.py" "$N" --profile "$P"
run=$?
wall=$(( $(date +%s) - start ))
python3 -B artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py "$A/$N" 2.7777777778 > "$A/${N}_metrics.json" 2> "$A/${N}_post.log"; m=$?
python3 -B scripts/surface_contact_eval.py "$A/$N" --output "$A/${N}_surface.json" >> "$A/${N}_post.log" 2>&1; s=$?
python3 -B "$A/evaluate.py" solo "$N" >> "$A/${N}_post.log" 2>&1; e=$?
echo "ACTUAL_EXIT_CODE=$run METRICS=$m SURFACE=$s EVAL=$e WALL_S=$wall" > "$A/${N}.exit"
exit "$run"
