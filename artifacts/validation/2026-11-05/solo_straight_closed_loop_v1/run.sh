#!/usr/bin/env bash
# 등록된 단독 1랩 1회 실행(--straight-budget, 시뮬만). 재실행·자동 재시도 없음. 회수는 post.sh.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-05/solo_straight_closed_loop_v1
N=run1
if [ ! -f "$A/registered.json" ]; then echo "not registered" >&2; exit 2; fi
if [ -e "$A/$N" ] || [ -e "$A/${N}_sim.exit" ]; then echo "existing $N output; no rerun" >&2; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running; run not started)" > "$A/${N}_sim.exit"; exit 3
fi
if ! /usr/bin/python3 -I -B "$A/verify.py" before > "$A/${N}_verify_before.log" 2>&1; then
  echo "ACTUAL_EXIT_CODE=4 (registered inputs changed; run not started)" > "$A/${N}_sim.exit"; exit 4
fi
{ nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1; uname -r; } > "$A/${N}_host.log"
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
start=$(date +%s)
echo "start_utc=$(date -u +%FT%TZ)" >> "$A/${N}_preflight.log"
bash "$ENV" /usr/bin/python3 -B scripts/run_current_solo.py "$A/$N" --straight-budget > "$A/${N}_console.log" 2>&1
code=$?
wall=$(( $(date +%s) - start ))
echo "end_utc=$(date -u +%FT%TZ)" >> "$A/${N}_preflight.log"
echo "ACTUAL_EXIT_CODE=$code WALL_S=$wall" > "$A/${N}_sim.exit"
exit "$code"
