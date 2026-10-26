#!/usr/bin/env bash
set -uo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$ROOT" || exit 90
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run60_m4_preflight
[ ! -f "$D/test_exit_code.txt" ] || exit 91
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="scripts:src/arena_autonomy:src/arena_vehicle_interface:tests:${PYTHONPATH:-}"
python3 - <<'PY'
from pathlib import Path
import zipfile
paths=['scripts/validate_g1_traffic.py','scripts/m4_operation_trial.py','scripts/m4_operation_eval.py','src/arena_bringup/launch/g1_follow.launch.py','tests/test_m4_operation_trial.py','tests/test_m4_operation_cli.py','tests/test_source_provenance_r0_plus_b3.py','config/tests/m4_operation_r0_sha256.json']
with zipfile.ZipFile('artifacts/validation/2026-10-25/m3_execution_review_v1/run60_m4_preflight/source.zip','w') as z:
    for p in paths:z.write(p,p)
PY
python3 -B -m pytest -q tests/test_m4_operation_trial.py tests/test_m4_operation_cli.py tests/test_race_session.py tests/test_race_session_node.py tests/test_source_provenance_r0_plus_b3.py tests/test_m3_lidar_stress_cli.py tests/test_g1_dense_stop.py > "$D/test_stdout.txt" 2> "$D/test_stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/test_exit_code.txt"
[ "$code" = 0 ] || exit "$code"
colcon build --packages-select arena_bringup --symlink-install > "$D/build_stdout.txt" 2> "$D/build_stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/build_exit_code.txt"
[ "$code" = 0 ] || exit "$code"
set +u
source install/local_setup.bash
set -u
python3 - <<'PY' > "$D/install_check.txt"
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import hashlib
p=Path(get_package_share_directory('arena_bringup'))/'launch/g1_follow.launch.py'
q=Path('src/arena_bringup/launch/g1_follow.launch.py')
assert p.read_bytes()==q.read_bytes()
print(p,hashlib.sha256(p.read_bytes()).hexdigest())
PY
code=$?
printf '%s\n' "$code" > "$D/install_exit_code.txt"
echo "RUN60_DONE test=$(cat "$D/test_exit_code.txt") build=$(cat "$D/build_exit_code.txt") install=$code"
exit "$code"