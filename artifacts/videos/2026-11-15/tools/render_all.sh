#!/usr/bin/env bash
# 2026-11-15 기록 영상 8개를 순서대로 1회씩 재렌더한다(소유 gz sim 하나씩). ONLY=<clip>로 하나만 실행할 수 있다.
# 요약: build/competition_recordings_20261115/logs/render_all.summary, 종료코드: render_all.exit
set -o pipefail
REPO="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
TOOLS="$REPO/artifacts/videos/2026-11-15/tools"
LOGS="$REPO/build/competition_recordings_20261115/logs"
mkdir -p "$LOGS"
SUMMARY="$LOGS/render_all.summary"
: > "$SUMMARY"
CLIPS=(
  "a_curved_course_r15|artifacts/validation/2026-11-05/solo_straight_closed_loop_v1/run1|6.0|42.0|--signal-inset"
  "b_right_angle_course_r25|artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1|6.0|38.8|--signal-inset"
  "c_right_angle_late_dev_r475|artifacts/validation/2026-11-14/solo_full_lap_r475_v1/run1|5.0|58.15|--signal-inset"
  "d1_overtake_medium_lead_r15m|artifacts/validation/2026-11-08/medium_pass_connected_v1/straight_moving_lead/run1|7.6|19.6|"
  "d2_bypass_stopped_car_r18|artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1|8.5|19.5|"
  "e1_signal_start_r475|artifacts/validation/2026-11-14/solo_full_lap_r475_v1/run1|2.4|12.0|--signal-inset"
  "e2_stop_restart_r40|artifacts/validation/2026-11-08/entry_width_transition_v1/run1|9.6|28.15|"
  "f_sensor_noise_delay_r38|artifacts/validation/2026-11-08/masked_motion_speed_v1/B_c1_injection/run1|5.4|68.45|--signal-inset"
)
# 미선정(교체됨): "d1_overtake_slow_lead_r13|artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1|7.6|19.0|"
STATUS=0
for spec in "${CLIPS[@]}"; do
  IFS='|' read -r NAME RUN T0 T1 EXTRA <<< "$spec"
  if [ -n "${ONLY:-}" ] && [ "$ONLY" != "$NAME" ]; then continue; fi
  echo "=== $NAME $(date -Is)" >> "$SUMMARY"
  # shellcheck disable=SC2086
  bash "$TOOLS/render_clip.sh" "$NAME" "$RUN" "$T0" "$T1" $EXTRA > "$LOGS/$NAME.console.log" 2>&1
  RC=$?
  tail -1 "$LOGS/$NAME/render_result.txt" >> "$SUMMARY" 2>/dev/null
  echo "rc=$RC" >> "$SUMMARY"
  [ "$RC" = 0 ] || STATUS=1
  [ "$RC" = 3 ] && break
done
POST=$(ps -eo pid,args | grep -E "gz sim|gz-sim|ruby .*gz" | grep -v -E "grep|render_all.sh|render_clip.sh" || true)
echo "final_scan_gz=[${POST:-none}] status=$STATUS $(date -Is)" >> "$SUMMARY"
echo "$STATUS" > "$LOGS/render_all.exit"
exit $STATUS
