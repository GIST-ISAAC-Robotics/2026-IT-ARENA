#!/usr/bin/env bash
# 격리 트리(base/cand)에서 관련 회귀 집합 실행. 인자: base|cand. 작업트리 src/는 PYTHONPATH에 넣지 않는다.
# 사본에 없는 저장소 항목(artifacts, docs, 다른 패키지, config 하위)은 읽기 전용 기호 연결로 보인다.
set -u
NAME=$1
OUT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R="$(cd "$OUT/../../../.." && pwd)"
T="$HOME/arena_scratch/straight_refresh/$NAME"
[ -d "$T/src/arena_autonomy" ] || { echo "missing tree $T" >&2; exit 2; }
for e in "$R"/* "$R"/.[!.]*; do b=$(basename "$e"); case "$b" in src|scripts|tests|config) ;; *) [ -e "$T/$b" ] || ln -s "$e" "$T/$b";; esac; done
for e in "$R"/src/*; do b=$(basename "$e"); [ -e "$T/src/$b" ] || ln -s "$e" "$T/src/$b"; done
for e in "$R"/config/*; do b=$(basename "$e"); [ -e "$T/config/$b" ] || ln -s "$e" "$T/config/$b"; done
set +u; source /opt/ros/jazzy/setup.bash; set -u
export PYTHONPATH="$T/src/arena_autonomy:$T/src/arena_vehicle_interface:$T/scripts:$T/tests:/opt/ros/jazzy/lib/python3.12/site-packages"
cd "$T"
FILES=(test_source_provenance_r0_plus_b3.py test_local_follow.py test_local_follow_validation.py test_local_pursuit.py
  test_boundary_bridge.py test_turn_preview_gate.py test_turn_speed_preview.py test_ray_wall_continuity.py test_solo_route_mode.py
  test_solo_alternative_speed.py test_solo_corridor_arc.py test_structure_route_speed.py test_known_structure_policy.py
  test_basic_autonomy.py test_g1_solo_candidate_validation.py test_follow_trace.py test_jr60_adoption_profile.py
  test_local_sensor_profile.py test_low_speed_arc_reacquire.py test_standing_wall_reacquire.py test_continuous_forward_pace.py
  test_moving_pass.py test_bypass_handoff.py test_signed_forward_drive.py test_corridor_follow.py test_follow_profile.py
  test_race_continuity.py)
for f in tests/*provenance*.py; do FILES+=("$(basename "$f")"); done
[ -f tests/test_solo_straight_budget.py ] && FILES+=(test_solo_straight_budget.py)
mapfile -t UNIQUE < <(printf '%s\n' "${FILES[@]}" | sort -u)
python3 -c "import arena_autonomy,sys;print(arena_autonomy.__file__)"
nice -n 19 python3 -B -m pytest -q -p no:cacheprovider -rfE "${UNIQUE[@]/#/tests/}" > "$OUT/regression_$NAME.log" 2>&1
code=$?
echo "exit=$code files=${#UNIQUE[@]}" >> "$OUT/regression_$NAME.log"
tail -n 5 "$OUT/regression_$NAME.log"
exit $code
