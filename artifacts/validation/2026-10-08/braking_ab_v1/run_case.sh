#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base='artifacts/validation/2026-10-08/braking_ab_v1'
case "$1" in official|flat) surface="$1";; *) exit 2;; esac
name="${2:-$surface}"
case "$name" in *[!a-zA-Z0-9_]*) exit 2;; esac
test ! -e "$base/$name"
test ! -e "$base/$name.runner.log"
python3 -B scripts/probe_braking_contact.py --surface "$surface" --command-profile "$base/command_profile.json" --output "$base/$name" > "$base/$name.runner.log" 2>&1
