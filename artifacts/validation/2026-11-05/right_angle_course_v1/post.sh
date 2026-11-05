#!/usr/bin/env bash
# run.sh 뒤 회수. 사용: post.sh <이름>. 한 단계가 실패해도 나머지 근거를 남긴다. 판정선은 바꾸지 않는다.
MAIN="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="$MAIN/artifacts/validation/2026-11-05/right_angle_course_v1"
WS="$HOME/arena_scratch/right_angle_v1/ws"
N="${1:?name}"
R="$A/$N"
if [ ! -f "$A/${N}_sim.exit" ]; then echo "run not finished" >&2; exit 2; fi
if [ -e "$A/${N}.exit" ]; then echo "already collected" >&2; exit 4; fi
cd "$WS" || exit 2
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
m=-1; s=-1; mo=-1
if [ -f "$R/report.json" ]; then
  python3 -B "$MAIN/artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py" "$R" 2.7777777778 > "$A/${N}_metrics.json" 2> "$A/${N}_post.log"; m=$?
  python3 -B scripts/surface_contact_eval.py "$R" --output "$A/${N}_surface.json" >> "$A/${N}_post.log" 2>&1; s=$?
  python3 -B scripts/audit_motor_encoder_runtime.py "$R" --output "$A/${N}_motor_audit.json" >> "$A/${N}_post.log" 2>&1; mo=$?
fi
python3 -B "$A/analyze.py" "$R" "$A/${WORLD:-course_world}" "$A/${N}_analysis.json" > "$A/${N}_analysis.log" 2>&1; an=$?
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_postflight.log"; left=$?
echo "$(cat "$A/${N}_sim.exit") METRICS=$m SURFACE=$s MOTOR=$mo ANALYSIS=$an LEFTOVER_PGREP=$left" > "$A/${N}.exit"
exit 0
