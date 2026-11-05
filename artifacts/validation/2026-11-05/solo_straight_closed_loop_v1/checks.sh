#!/usr/bin/env bash
# 적용 뒤 실제 설치 환경 확인과 영향 범위 회귀(Gazebo 없음). 결과: checks_*.log, checks.exit
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
O=artifacts/validation/2026-11-05/solo_straight_closed_loop_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B -c "
import arena_autonomy.solo_straight_budget as m, arena_autonomy.local_follow as f
from ament_index_python.packages import get_package_share_directory as g
import os
print(m.__file__); print(f.__file__)
p=os.path.join(g('arena_bringup'),'launch/g1_follow.launch.py'); print(p, os.path.realpath(p))
print('flag_in_installed_launch', 'g1_solo_straight_budget_trial' in open(p,encoding='utf-8').read())
print('default_off', f.FollowConfig().solo_straight_budget_trial is False)
" > $O/checks_install.log 2>&1; a=$?
FILES=(test_source_provenance_r0_plus_b3.py test_local_follow.py test_local_follow_validation.py test_local_pursuit.py
  test_boundary_bridge.py test_turn_preview_gate.py test_turn_speed_preview.py test_ray_wall_continuity.py test_solo_route_mode.py
  test_solo_alternative_speed.py test_solo_corridor_arc.py test_structure_route_speed.py test_known_structure_policy.py
  test_basic_autonomy.py test_g1_solo_candidate_validation.py test_follow_trace.py test_jr60_adoption_profile.py
  test_local_sensor_profile.py test_low_speed_arc_reacquire.py test_standing_wall_reacquire.py test_continuous_forward_pace.py
  test_moving_pass.py test_bypass_handoff.py test_signed_forward_drive.py test_corridor_follow.py test_follow_profile.py
  test_race_continuity.py test_solo_straight_budget.py)
for f in tests/*provenance*.py; do FILES+=("$(basename "$f")"); done
mapfile -t U < <(printf '%s\n' "${FILES[@]}" | sort -u)
bash "$ENV" /usr/bin/python3 -B -m pytest -q -p no:cacheprovider -rfE "${U[@]/#/tests/}" > $O/checks_regression.log 2>&1; b=$?
echo "INSTALL=$a REGRESSION=$b FILES=${#U[@]}" > $O/checks.exit
