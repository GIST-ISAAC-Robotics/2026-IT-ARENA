#!/usr/bin/env bash
# r16 등록 인자(기본 track)를 변경 전(base)·후(patched) 검사기로 각각 dry 실행해 launch argv를 비교한다.
# scratch 사본만 사용하고 Gazebo는 띄우지 않는다.
REPO='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A="$REPO/artifacts/validation/2026-11-07/traffic_course_migration_v1"
R16='artifacts/validation/2026-11-05/moving_pass_withdrawal_v1'
set +u; source /opt/ros/jazzy/setup.bash; source "$REPO/install/local_setup.bash"
for variant in base patched; do
  S="$HOME/arena_scratch/traffic_default_argv_$variant"
  rm -rf "$S"; mkdir -p "$S/artifacts/validation/2026-11-07/traffic_course_migration_v1" "$S/$R16/candidate_r16"
  for d in scripts config src; do
    rsync -a --exclude '__pycache__' --exclude 'build' --exclude 'install' --exclude 'log' "$REPO/$d" "$S/"
  done
  cp "$A/$variant"/scripts/*.py "$S/scripts/"
  cp "$REPO/$R16/course.json" "$S/$R16/"
  cp "$REPO/$R16/candidate_r16/registered.json" "$S/$R16/candidate_r16/"
  mkdir -p "$S/artifacts/validation/2026-10-07/g1b_contact_probe_v1"
  cp "$REPO/artifacts/validation/2026-10-07/g1b_contact_probe_v1/report.json" "$S/artifacts/validation/2026-10-07/g1b_contact_probe_v1/" 2>/dev/null
  cp "$A/dryargs_runner.py" "$S/artifacts/validation/2026-11-07/traffic_course_migration_v1/"
  export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$S/src/arena_autonomy:$S/src/arena_vehicle_interface:$S/scripts:$S/tests"
  args=$(cd "$S" && python3 -c "import json;a=json.load(open('$R16/candidate_r16/registered.json'))['args'];print(' '.join(a[1:a.index('--output')]))")
  (cd "$S" && python3 -B artifacts/validation/2026-11-07/traffic_course_migration_v1/dryargs_runner.py \
     "$S/artifacts/dry_r16" $args) > "$A/checks/default_argv_$variant.log" 2>&1
  echo "EXIT=$?" >> "$A/checks/default_argv_$variant.log"
  sed "s#$S#<ROOT>#g" "$S/artifacts/dry_r16/dry_launch_argv.json" > "$A/checks/default_argv_$variant.json"
done
cmp "$A/checks/default_argv_base.json" "$A/checks/default_argv_patched.json" && echo IDENTICAL > "$A/checks/default_argv_compare.txt" \
  || diff "$A/checks/default_argv_base.json" "$A/checks/default_argv_patched.json" > "$A/checks/default_argv_compare.txt"
cat "$A/checks/default_argv_compare.txt"
