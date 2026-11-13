#!/usr/bin/env bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")"
python3 -B replay.py replay_v2 > replay_v2.log 2>&1
rc=$?
printf '%s\n' "$rc" > replay_v2.exit
exit "$rc"
