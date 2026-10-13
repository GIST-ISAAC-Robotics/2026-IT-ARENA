#!/usr/bin/env bash
set -euo pipefail
cd /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
base=artifacts/validation/2026-10-12/contact_model_audit_v1
for name in probe_build_path_v2.txt configure_v2.log build_v2.log solver_probe.json solver_probe_stderr_v2.log; do
  if [[ -e "$base/$name" ]]; then printf 'Refusing overwrite: %s\n' "$name"; exit 2; fi
done
set +u
source /opt/ros/jazzy/setup.bash
set -u
probe_dir=$(mktemp -d /tmp/arena-contact-audit-XXXXXX)
printf '%s\n' "$probe_dir" > "$base/probe_build_path_v2.txt"
cmake -S "$base" -B "$probe_dir" -DDART_DIR=/opt/ros/jazzy/opt/gz_dartsim_vendor/share/dart/cmake > "$base/configure_v2.log" 2>&1
cmake --build "$probe_dir" -j2 > "$base/build_v2.log" 2>&1
ldd "$probe_dir/solver_probe" > "$base/probe_ldd_v2.log"
"$probe_dir/solver_probe" > "$base/solver_probe.json" 2> "$base/solver_probe_stderr_v2.log"
