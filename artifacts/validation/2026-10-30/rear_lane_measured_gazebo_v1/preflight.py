"""Gazebo를 시작하지 않고 새 상대차 배치·반례·명시 후보 설정을 확인한다."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

from shapely.geometry import box

import run_trial as run
from straddle_route import check_straddle, wrap_route_check

for folder in ('scripts', 'src/arena_autonomy', 'src/arena_vehicle_interface'):
    sys.path.insert(0, str(run.ROOT/folder))
import validate_g1_traffic as v


def main():
    checks = {}
    config = {'actor': {'length_m': .2, 'width_m': .15}}
    case = {'route_kind': 'side_straddle_straight', 'actor_lateral_offset_m': .20}
    straight = lambda *a: {'actor_start_s_m': 0., 'actor_end_s_m': 1.}
    pose = lambda cl, s, schedule, t, lateral: (s, lateral, 0.)
    road = box(-1., -.225, 3., .225)
    args = (config, SimpleNamespace(lap=10.), case, 0., road, [], straight, pose, v.rectangle)
    checks['straddle_accepted'] = check_straddle(*args)['sampled_footprints'] == 101
    for name, offset, obstacles in [('fully_road', .13, []), ('fully_grass', .32, []),
                                    ('outside_wall_corridor', .40, []),
                                    ('wall_intersection', .20, [box(.4, .24, .6, .30)])]:
        candidate = {**case, 'actor_lateral_offset_m': offset}
        try:
            check_straddle(config, args[1], candidate, 0., road, obstacles, straight, pose, v.rectangle)
            checks[name+'_rejected'] = False
        except ValueError:
            checks[name+'_rejected'] = True
    # 실제 입력은 원래 공식 v2 고정본, 루트 검사만 어댑터에서 추가한다.
    config = v.load_config(run.CONFIG)
    case = v.overlay_case(v.case_named(config, 'slow_lead'), run.OVERLAY)
    specs = v.actor_specs(config, case)
    centerline, scene = v.load_centerline(run.FIXTURE)
    route = wrap_route_check(v)
    starts, routes = v.check_additional_routes(config, case, specs, scene, 0, centerline,
        v.road_polygon(run.FIXTURE/'world.sdf'), v.obstacle_polygons(run.FIXTURE/'world.sdf'),
        v.actor_start_s, route, v.actor_pose_at)
    checks['actual_route_valid'] = bool(routes['actors'])
    checks['actor_schedule_nonoverlap'] = routes['sampled_actor_pair_min_gap_m'] > 0.
    settings = {}
    for enabled in (False, True):
        run.CANDIDATE = enabled
        _, setting, arguments = run.launch_source(run.BASE/'preflight_unused')
        settings[str(enabled)] = setting
        checks[f'candidate_{enabled}_explicit'] = setting['g1_rear_lane_keep_trial'] is enabled
    changed = [k for k in settings['False'] if settings['False'][k] != settings['True'][k]]
    checks['only_candidate_policy_differs'] = set(changed) == {'g1_rear_lane_keep_trial', 'g1_rear_lane_keep_policy'}
    checks['policy_on_measured'] = settings['True']['g1_rear_lane_keep_policy'] == 'observed_measured_v1'
    checks['policy_off_timed'] = settings['False']['g1_rear_lane_keep_policy'] == 'timed'
    report = dict(passed=all(checks.values()), checks=checks, actual_routes=routes,
                  actor_start_s_m=starts, changed_settings=changed,
                  scope='preflight only; no Gazebo run or autonomous driving proof')
    path = run.BASE/'preflight_result.json'
    with path.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    (run.BASE/'preparation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
