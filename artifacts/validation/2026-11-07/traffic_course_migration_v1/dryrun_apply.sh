#!/usr/bin/env bash
# 작업트리 대신 새 scratch 사본에 apply_patch.py --apply를 수행하고 관련 시험을 돌린다. 사용: dryrun_apply.sh <tests...>
REPO='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A="$REPO/artifacts/validation/2026-11-07/traffic_course_migration_v1"
S="$HOME/arena_scratch/traffic_course_migration_dryrun"
rm -rf "$S"; mkdir -p "$S"
for d in scripts tests config src; do
  rsync -a --exclude '__pycache__' --exclude 'build' --exclude 'install' --exclude 'log' "$REPO/$d" "$S/"
done
{
  python3 -B "$A/apply_patch.py" --check --root "$S"; echo "CHECK_EXIT=$?"
  python3 -B "$A/apply_patch.py" --apply --root "$S"; echo "APPLY_EXIT=$?"
  python3 -B "$A/apply_patch.py" --check --root "$S"; echo "RECHECK_EXIT=$? (1 expected: already applied)"
  (cd "$S" && for f in scripts/validate_g1_traffic.py scripts/official_traffic_course.py scripts/race_ops_eval.py scripts/summarize_motion_lap.py tests/test_traffic_course_track.py; do
     cmp -s "$f" "$A/patched/$f" && echo "SAME_AS_PATCHED $f" || echo "DIFFERENT $f"; done)
  (cd "$S" && git diff --no-index --stat "$REPO/tests/test_source_provenance_r0_plus_b3.py" tests/test_source_provenance_r0_plus_b3.py)
} > "$A/checks/dryrun_apply.log" 2>&1
cd "$S" || exit 2
set +u; source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$S/src/arena_autonomy:$S/src/arena_vehicle_interface:$S/scripts:$S/tests"
nice -n 19 python3 -B -m pytest -q -p no:cacheprovider --tb=line -rfE tests/test_traffic_course_track.py "$@" > "$A/checks/dryrun_related.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "$A/checks/dryrun_related.exit"
exit "$result"
