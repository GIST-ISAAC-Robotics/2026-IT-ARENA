#!/usr/bin/env bash
# 교통 장면 1회와 회수. 인자: 이름 장면(right_reapproach|close_cut_in|...) [프로필 yaml|-]. 자동 재시도 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-04/lidar_mount_adapted_v2
N=$1; CASE=$2; P=${3:--}
case "$CASE" in
  right_reapproach) CONFIG=artifacts/validation/2026-11-03/right_reapproach_v3/scenario.json ;;
  *) CONFIG=config/tests/official_traffic_v1.json ;;
esac
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/${N}_preflight.log"; then
  echo "ACTUAL_EXIT_CODE=3 (another simulation is running)" > "$A/${N}.exit"; exit 3
fi
OPTS=(); [ "$P" != "-" ] && OPTS+=(--local-sensor-config "$PWD/$A/profiles/$P")
[ -e "$A/$N" ] && exit 4
FILES=("$CONFIG" scripts/validate_g1_traffic.py src/arena_bringup/launch/simulation.launch.py
  src/arena_autonomy/arena_autonomy/{known_structure,partial_observation,local_follow,local_pursuit,lidar_safety}.py
  config/tests/lidar_mount_adapted_r2_sha256.json "$A/run_traffic.sh" "$A/evaluate.py")
[ "$P" != "-" ] && FILES+=("$A/profiles/$P")
sha256sum "${FILES[@]}" > "$A/${N}_inputs.sha256" || exit 4
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
start=$(date +%s)
bash "$ENV" /usr/bin/python3 -B scripts/validate_g1_traffic.py --mode g1g2 --handoff-reserve paced --handoff-yield-progress \
  --handoff-rear-sweep --official-signal-course --config "$PWD/$CONFIG" --case "$CASE" --seed 0 \
  --record-follow-decisions --handoff-observed-motion --side-departure-trial --current-lane-preference-trial \
  "${OPTS[@]}" --output "$PWD/$A/$N" > "$A/${N}_console.log" 2>&1
code=$?
wall=$(( $(date +%s) - start ))
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B scripts/surface_contact_eval.py "$A/$N" --output "$A/${N}_surface.json" > "$A/${N}_post.log" 2>&1; s=$?
python3 -B "$A/evaluate.py" traffic "$N" >> "$A/${N}_post.log" 2>&1; e=$?
sha256sum -c "$A/${N}_inputs.sha256" >> "$A/${N}_post.log"; integrity=$?
echo "ACTUAL_EXIT_CODE=$code SURFACE=$s EVAL=$e INTEGRITY=$integrity WALL_S=$wall" > "$A/${N}.exit"
exit "$code"
