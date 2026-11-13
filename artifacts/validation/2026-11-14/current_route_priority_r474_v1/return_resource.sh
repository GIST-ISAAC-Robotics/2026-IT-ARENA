#!/usr/bin/env bash
set -u
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")"
python3 -B check_resources.py > resource_return.log 2>&1
rc=$?
printf '%s\n' "$rc" > resource_return.exit
exit "$rc"
