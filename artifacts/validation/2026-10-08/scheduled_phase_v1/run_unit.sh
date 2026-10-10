#!/usr/bin/env bash
# 영향 범위 단위 회귀. 로그 이름은 새 이름만 허용한다.
set -e
source "$(dirname "$0")/env.sh"
name="${1:?log name}"
case "$name" in unit_[a-z0-9_]*.log) ;; *) exit 2;; esac
test ! -e "$base/$name"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_scheduled_command.py tests/test_scheduled_phase_analysis.py tests/test_braking_command_profile.py \
  tests/test_physics_step_option.py tests/test_runtime_world_options.py tests/test_zero_speed_modes.py \
  tests/test_r0_safety_contract.py tests/test_multi_vehicle_g0.py tests/test_wheel_contact_probe.py \
  > "$base/$name" 2>&1
rc=$?
tail -5 "$base/$name"
exit $rc
