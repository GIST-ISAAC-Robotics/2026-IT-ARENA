#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run46_ros_preflight
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B - "$O" <<'PY'
import sys,zipfile
from pathlib import Path
paths=set(Path('src').rglob('*.py'))|set(Path('tests').glob('test_m3*.py'))|{Path('scripts/validate_g1_traffic.py')}
with zipfile.ZipFile(Path(sys.argv[1])/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(paths):z.write(p,p.as_posix())
PY
python3 -B -m pytest -q tests/test_m3_departure_integration.py tests/test_m3_execution_integration.py tests/test_m3_deferred_center.py tests/test_m3_rounded_occupancy.py tests/test_m3_parallel_straight.py tests/test_m3_lidar_stress_cli.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
result=$?
printf '%s\n' "$result" > "$O/tests_exit_code.txt"
if [ "$result" -eq 0 ]; then
    python3 -B scripts/compare_m3_deferred_center.py --output "$O/data" > "$O/replay_stdout.txt" 2> "$O/replay_stderr.txt"
    result=$?
    printf '%s\n' "$result" > "$O/replay_exit_code.txt"
fi
if [ "$result" -eq 0 ]; then
    colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$O/build_stdout.txt" 2> "$O/build_stderr.txt"
    result=$?
    printf '%s\n' "$result" > "$O/build_exit_code.txt"
fi
python3 -B - "$O" <<'PY'
import sys,zipfile,json
from pathlib import Path
out=Path(sys.argv[1])
with zipfile.ZipFile(out/'sources.zip') as z:
    changes=[n for n in z.namelist() if Path(n).read_bytes()!=z.read(n)]
(out/'source_check.json').write_text(json.dumps(dict(changed=changes,unchanged=not changes)))
assert not changes
PY
check=$?
if [ "$check" -ne 0 ]; then result=$check; fi
printf '%s\n' "$result" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"
