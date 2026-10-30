#!/usr/bin/env bash
# 사용: check_connection.sh <로그 접미사>
set -eo pipefail
tag="$1"
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_corridor_connection_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
/usr/bin/python3 -B -m pytest -q -rs -p no:cacheprovider tests/test_bypass_corridor_trial.py tests/test_bypass_entry_follow.py \
  tests/test_bypass_entry_guide.py tests/test_bypass_entry_trial.py tests/test_bypass_handoff.py tests/test_bypass_wall_fit.py \
  tests/test_bypass_wall_reanchor.py tests/test_bypass_entry_sequence.py tests/test_g2_handoff_provenance.py \
  tests/test_rear_lane_keep_provenance.py tests/test_fresh_confirmation_provenance.py tests/test_source_provenance_r0_plus_b3.py \
  tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py \
  > "$base/tests_$tag.log" 2>&1
printf '%s\n' "$?" > "$base/tests_$tag.exit"
# 연결 전 소스로 schema5 재생 불일치가 원래 있었는지 확인한다.
scratch="$HOME/arena_scratch/g2_corridor_before"
rm -rf "$scratch"
mkdir -p "$scratch"
cp -r src/arena_autonomy/arena_autonomy "$scratch/arena_autonomy"
/usr/bin/python3 -B -c "import zipfile,sys;z=zipfile.ZipFile('$base/before_connection.zip');[open('$scratch/arena_autonomy/'+n.split('/')[-1],'wb').write(z.read(n)) for n in z.namelist()]"
rm -f "$scratch/arena_autonomy/bypass_corridor_trial.py"
PYTHONPATH="$scratch:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -B "$base/replay_before.py" > "$base/replay_before_$tag.log" 2>&1
printf '%s\n' "$?" > "$base/replay_before_$tag.exit"
