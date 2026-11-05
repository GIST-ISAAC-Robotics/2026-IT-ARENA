#!/usr/bin/env bash
# 격리 트리 재생 7종(병렬, nice). Gazebo 미사용. 각 종료 코드를 run_replays.exit에 남긴다.
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 2
V=../..
SOLO=$V/2026-11-05/solo_jr60p20_baseline_v1/run1/follow_decisions.jsonl
R13=$V/2026-11-05/moving_pass_withdrawal_v1/candidate_r13/run1/follow_decisions.jsonl
MPB=$V/2026-11-05/moving_pass_baseline_v1/run1/follow_decisions.jsonl
S=$HOME/arena_scratch/straight_refresh
set +u; source /opt/ros/jazzy/setup.bash; set -u
job() { tree=$1; tag=$2; shift 2
  PYTHONPATH="$S/$tree/src/arena_autonomy:$S/$tree/src/arena_vehicle_interface:/opt/ros/jazzy/lib/python3.12/site-packages" \
    nice -n 19 python3 -B replay_check.py "$tag" "$@" > "replay_$tag.log" 2>&1; echo "$tag=$?" >> run_replays.exit; }
: > run_replays.exit
job base base_solo_off "$SOLO" &
job cand cand_solo_off "$SOLO" &
job cand cand_solo_on "$SOLO" --straight-budget &
job base base_r13 "$R13" &
job cand cand_r13 "$R13" &
job base base_mpb "$MPB" &
job cand cand_mpb "$MPB" &
wait
sort run_replays.exit | tr '\n' ' '; echo
