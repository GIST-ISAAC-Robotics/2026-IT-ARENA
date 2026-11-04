#!/usr/bin/env bash
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-03/solo_straight_closed_loop_v1
R=$A/run1
worst=0
step() {
  local name=$1; shift
  "$@" > "$A/post_$name.log" 2>&1
  local rc=$?
  echo "$rc" > "$A/post_$name.exit"
  if [ "$rc" -ne 0 ]; then worst=1; fi
}
step retime python3 -B scripts/retime_autonomy_video.py "$R"
if [ -f "$R/lidar_third_person_simtime_1x.mp4" ]; then
  step video python3 -B scripts/inspect_autonomy_video.py "$R/lidar_third_person_simtime_1x.mp4" --output "$A/run1_video_review"
fi
python3 -B artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py "$R" 2.7777777778 > "$A/run1_metrics.json" 2>"$A/post_metrics.log"
rc=$?; echo "$rc" > "$A/post_metrics.exit"; if [ "$rc" -ne 0 ]; then worst=1; fi
step surface python3 -B scripts/surface_contact_eval.py "$R" --output "$A/run1_surface.json"
step motor python3 -B scripts/audit_motor_encoder_runtime.py "$R" --output "$A/run1_motor_audit.json"
step chain python3 -B "$A/speed_chain.py" "$R" "$A/run1_speed_chain.json"
step review python3 -B "$A/review.py"
echo "$worst" > "$A/post.exit"
exit "$worst"
