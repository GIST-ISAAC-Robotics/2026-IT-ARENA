#!/usr/bin/env bash
# 사전 단위 회귀. 로그 이름은 새 이름만 허용한다.
set -e
source "$(dirname "$0")/env.sh"
name="${1:?log name}"
case "$name" in unit_[a-z0-9_]*.log) ;; *) exit 2;; esac
test ! -e "$base/$name"
python3 -B -m pytest -q -p no:cacheprovider tests/test_physics_step_option.py tests/test_runtime_world_options.py \
  tests/test_zero_speed_modes.py tests/test_r0_safety_contract.py tests/test_multi_vehicle_g0.py \
  > "$base/$name" 2>&1
