#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_integration_v2
previous=artifacts/validation/2026-10-23/integration_review_v1
test ! -e "$evidence/checks_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar --exclude=__pycache__ -czf "$evidence/source_snapshot.tar.gz" src scripts tests config
finish() {
  result=$?
  trap - EXIT
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || result=1
  printf '%s\n' "$result" > "$evidence/checks_exit.txt"
  exit "$result"
}
trap finish EXIT
sha256sum -c "$previous/binaries.sha256" > "$evidence/reused_binaries_check.log"
grep -E '  (src/arena_vehicle_interface/portable/|scripts/actuation_[^/]*\.cpp$)' "$previous/sources_before.sha256" \
 | sha256sum -c > "$evidence/reused_cpp_source_check.log"
export ARENA_RECEIVER_API_BINARY="$PWD/$previous/receiver_api_driver"
export ARENA_RECEIVER_PRODUCTION_API_BINARY="$PWD/$previous/receiver_production_api_driver"
export ARENA_RECEIVER_CPP_BINARY="$PWD/$previous/receiver_driver"
export ARENA_COMMAND_CPP_BINARY="$PWD/$previous/command_driver"
export ARENA_LINK_CPP_BINARY="$PWD/$previous/link_driver"
export ARENA_WIRE_CPP_BINARY="$PWD/$previous/wire_driver"
python3 -B -m pytest -q -p no:cacheprovider tests --junitxml="$evidence/junit.xml" > "$evidence/full_tests.log" 2>&1
