#!/usr/bin/env bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")"
printf '%s\n' "$$" > wrapper.pid
python3 -B diagnose.py diagnostic_v1 > diagnostic_v1.log 2>&1
rc=$?
printf '%s\n' "$rc" > diagnostic_v1.exit
exit "$rc"
