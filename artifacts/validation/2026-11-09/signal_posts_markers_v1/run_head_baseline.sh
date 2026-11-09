#!/usr/bin/env bash
# HEAD(815932fe) 임시 worktree에서 같은 실패 ID를 실행해 기존 실패와 새 실패를 구분한다.
set -uo pipefail
REPO='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
HERE="$REPO/artifacts/validation/2026-11-09/signal_posts_markers_v1"
WT="$HOME/arena_scratch/head_815932fe_wt"
set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/install/local_setup.bash"; set -u
if [ ! -d "$WT" ]; then git -C "$REPO" worktree add --detach "$WT" 815932fe > "$HERE/checks/head_worktree.log" 2>&1 || exit 3; fi
cd "$WT"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
mapfile -t ids < "$HERE/checks/${IDS:-after1_failed_ids.txt}"
python3 -B -m pytest -q -p no:cacheprovider --tb=line "${ids[@]}" > "$HERE/checks/pytest_head_${LABEL:-same_ids}.log" 2>&1
echo "ACTUAL_EXIT_CODE=$?" > "$HERE/checks/pytest_head_${LABEL:-same_ids}.exit"
