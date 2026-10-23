#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
root="$PWD"
evidence="$root/artifacts/validation/2026-10-23/installed_source_v1"
test ! -e "$evidence/exit.txt"
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
finish() {
  result=$?
  trap - EXIT
  cd "$root"
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_diff.txt" || result=1
  printf '%s\n' "$result" > "$evidence/exit.txt"
  exit "$result"
}
trap finish EXIT
(
  source install/local_setup.bash
  cd /tmp
  python3 -B "$evidence/audit.py" "$root" "$evidence/existing.json"
) > "$evidence/existing.log" 2>&1 || true
colcon list --base-paths src > "$evidence/packages.txt"
colcon --log-base "$root/build/install_audit_20261023_logs" build --base-paths src \
  --build-base "$root/build/install_audit_20261023" \
  --install-base "$root/build/install_audit_20261023_install" \
  --packages-select arena_autonomy arena_vehicle_interface arena_bringup arena_description arena_gazebo \
  --event-handlers console_direct+ > "$evidence/build.log" 2>&1
source "$root/build/install_audit_20261023_install/local_setup.bash"
cd /tmp
python3 -B "$evidence/audit.py" "$root" "$evidence/fresh.json" "$root/build/install_audit_20261023_install" > "$evidence/fresh.log" 2>&1
ctest --test-dir "$root/build/install_audit_20261023/arena_gazebo" --output-on-failure > "$evidence/ctest.log" 2>&1
