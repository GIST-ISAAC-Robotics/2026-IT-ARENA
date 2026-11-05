#!/usr/bin/env bash
# 격리 사본으로 replay.py 실행. 작업트리 src/는 PYTHONPATH에 넣지 않는다.
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 2
S="$HOME/arena_scratch/solo_diag_v1_src"
[ -d "$S/arena_autonomy/arena_autonomy" ] || { echo "run isolate.sh first" >&2; exit 2; }
export PYTHONPATH="$S/arena_autonomy:$S/arena_vehicle_interface"
python3 -B -c "import arena_autonomy, arena_vehicle_interface; print(arena_autonomy.__file__); print(arena_vehicle_interface.__file__)"
nice -n 19 python3 -B replay.py
