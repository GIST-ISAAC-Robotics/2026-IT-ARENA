#!/usr/bin/env bash
set -eo pipefail

ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ARTIFACT="$ROOT/artifacts/validation/2026-10-17/g1_recent_observation_ros_v1"
RUN="$ARTIFACT/run"

cd "$ROOT"
mkdir -p "$RUN"

# Preserve the exact pre-run source/configuration inventory and hashes.
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_before.sha256"
find src scripts tests config -type f -printf '%P %s bytes\n' | sort > "$RUN/source_inventory_before.txt"
finalize() {
  status=$?
  find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_after.sha256"
  if diff -u "$RUN/source_hashes_before.sha256" "$RUN/source_hashes_after.sha256" > "$RUN/source_hash_diff.txt"; then
    printf 'SOURCE_HASHES_UNCHANGED\n' > "$RUN/source_hash_status.txt"
  else
    printf 'SOURCE_HASHES_CHANGED\n' > "$RUN/source_hash_status.txt"
  fi
  printf 'EXIT_STATUS=%s\n' "$status" > "$RUN/runner_status.txt"
}
trap finalize EXIT

source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1

{
  date --iso-8601=seconds
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO-}"
  printf 'PYTHONPATH=%s\n' "$PYTHONPATH"
  python3 -c 'import ackermann_msgs; print("ackermann_msgs=" + ackermann_msgs.__file__)'
  colcon version-check
} > "$RUN/environment.log" 2>&1

colcon build --symlink-install --packages-select arena_autonomy arena_bringup > "$RUN/build.log" 2>&1
source install/local_setup.bash
python3 -B scripts/validate_pipeline_faults_ros.py \
  --controller-profile g1_recent_observation_candidate \
  --scenario full \
  --wall-cap 600 \
  --output "$RUN/validator_output" > "$RUN/validator.log" 2>&1

