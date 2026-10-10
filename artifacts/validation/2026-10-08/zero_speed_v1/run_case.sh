#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base='artifacts/validation/2026-10-08/zero_speed_v1'
case "$1" in legacy|reset_integral|no_lag|passive_damping) mode="$1";; *) exit 2;; esac
case "$2" in official|flat) surface="$2";; *) exit 2;; esac
name="${3:-${surface}_${mode}}"
case "$name" in *[!a-zA-Z0-9_]*) exit 2;; esac
test ! -e "$base/$name"
test ! -e "$base/$name.runner.log"
python3 -B scripts/probe_braking_contact.py --surface "$surface" --zero-speed-mode "$mode" \
  --command-profile artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json \
  --output "$base/$name" > "$base/$name.runner.log" 2>&1
