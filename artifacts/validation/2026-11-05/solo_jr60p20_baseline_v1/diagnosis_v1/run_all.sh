#!/usr/bin/env bash
# 진단 재현(WSL). Gazebo를 실행하지 않는다. 각 단계 종료 코드를 run_all.exit에 남긴다.
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 2
step() { n=$1; shift; "$@" > "$n.log" 2>&1; c=$?; echo "exit=$c" >> "$n.log"; printf '%s=%s ' "$n" "$c" >> run_all.exit; }
: > run_all.exit
step isolate bash isolate.sh
step replay bash run_replay.sh
step raycheck nice -n 19 python3 -I -B raycheck.py
step parent_model python3 -I -B parent_model.py
step model_check python3 -I -B model_check.py
step prior_check python3 -B prior_check.py
step make_patch python3 -I -B make_patch.py
C="$HOME/arena_scratch/solo_diag_v1_candA"
step compile_patch python3 -I -B -m py_compile "$C/arena_autonomy/arena_autonomy/local_follow.py" "$C/arena_autonomy/arena_autonomy/local_pursuit.py"
step verify_patch env PYTHONPATH="$C/arena_autonomy:$C/arena_vehicle_interface" nice -n 19 python3 -B verify_patch.py
echo >> run_all.exit
cat run_all.exit
