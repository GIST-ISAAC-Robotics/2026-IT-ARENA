#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ARTIFACT="$ROOT/artifacts/validation/2026-10-17/g1_recent_observation_ros_v3"
RUN="$ARTIFACT/run"
cd "$ROOT"
mkdir -p "$RUN"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_before.sha256"
tar -czf "$RUN/source_snapshot_before.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_after.sha256"
  if diff -u "$RUN/source_hashes_before.sha256" "$RUN/source_hashes_after.sha256" > "$RUN/source_hash_diff.txt"; then echo SOURCE_UNCHANGED > "$RUN/source_hash_status.txt"; else echo SOURCE_CHANGED > "$RUN/source_hash_status.txt"; fi
  echo "EXIT_STATUS=$result" > "$RUN/runner_status.txt"
}
trap finish EXIT
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
{
  date --iso-8601=seconds
  printf 'ROS_DISTRO=%s\nPYTHONPATH=%s\n' "${ROS_DISTRO-}" "$PYTHONPATH"
  python3 -c 'import ackermann_msgs; print("ackermann_msgs="+ackermann_msgs.__file__)'
} > "$RUN/environment.log" 2>&1
colcon build --symlink-install --packages-select arena_autonomy arena_bringup > "$RUN/build.log" 2>&1
source install/local_setup.bash
python3 -B scripts/validate_pipeline_faults_ros.py \
  --controller-profile g1_recent_observation_candidate \
  --scenario full --wall-cap 600 --recent-check-mode evidence_expiry \
  --output "$RUN/validator_output" > "$RUN/validator.log" 2>&1
