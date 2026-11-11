#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-10/combined_c1_facility_v1
D=$A/two_actor_run1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [[ -e "$D" ]]; then exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)'; then exit 3; fi
bash "$ENV" /usr/bin/python3 -B "$A/run.py" > "$A/console.log" 2>&1
harness=$?
if [[ ! -f "$D/run1/report.json" ]]; then printf 'HARNESS=%s\n' "$harness" > "$A/early_failure.exit"; exit "$harness"; fi
step() {
 name=$1; shift
 bash "$ENV" /usr/bin/python3 -B "$@" > "$D/$name.log" 2>&1
 printf '%s\n' "$?" > "$D/$name.exit"
}
step surface scripts/surface_contact_eval.py "$D/run1" --output "$D/surface.json"
step continuity artifacts/validation/2026-11-10/facility_traffic_stability_v1/analyze_continuity.py "$D/run1" --output "$D/continuity.json"
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight_processes.txt"
leftover=$?
step assessment "$A/assess.py"
printf 'HARNESS=%s\nLEFTOVER_PGREP=%s\n' "$harness" "$leftover" > "$D/run1.exit"
exit "$harness"
