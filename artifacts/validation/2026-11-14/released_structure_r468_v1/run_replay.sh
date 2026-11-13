#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")"
python3 -B replay.py base > base_replay.log 2>&1
python3 -B replay.py stage > stage_replay.log 2>&1
