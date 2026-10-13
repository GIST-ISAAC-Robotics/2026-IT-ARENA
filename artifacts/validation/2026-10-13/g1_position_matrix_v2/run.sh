#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_position_matrix_v2
for seed in 1 2; do
  for case_name in side_left side_right initial_occlusion first_curve; do
    if [[ "$seed" == 1 && "$case_name" == side_left ]]; then continue; fi
    if [[ "$case_name" == side_* ]]; then
      cfg=config/tests/g1_side_occlusion_v1.json
    else
      cfg=config/tests/g1_traffic_extensions_v1.json
    fi
    name="${case_name}_s${seed}"
    test ! -e "$base/$name"
    test ! -e "$base/$name.runner.log"
    set +e
    python3 -B scripts/validate_g1_traffic.py --case "$case_name" --seed "$seed" --mode g1 --config "$cfg" --output "$base/$name" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate > "$base/$name.runner.log" 2>&1
    run_exit=$?
    set -e
    echo "case=$name validator_exit=$run_exit"
    tail -20 "$base/$name.runner.log"
    replay_exit=1
    audit_exit=1
    set +e
    if test -f "$base/$name/follow_decisions.jsonl"; then
      python3 -B scripts/replay_follow_trace.py "$base/$name/follow_decisions.jsonl" --output "$base/$name.replay.json" > "$base/$name.replay.log" 2>&1
      replay_exit=$?
    fi
    if test -f "$base/$name/report.json"; then
      python3 -B scripts/audit_g1_case_evidence.py "$base/$name" --config "$cfg" --output "$base/$name.case_evidence.json" > "$base/$name.case_evidence.log" 2>&1
      audit_exit=$?
    fi
    set -e
    printf 'validator=%s\nreplay=%s\ncase_evidence=%s\n' "$run_exit" "$replay_exit" "$audit_exit" > "$base/$name.exit.txt"
    echo "case=$name replay_exit=$replay_exit audit_exit=$audit_exit"
    if (( run_exit != 0 || replay_exit != 0 || audit_exit != 0 )); then exit 1; fi
  done
done
