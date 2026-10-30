"""Gazebo를 시작하지 않고 오른쪽 대칭 배치·반례·명시 정책 설정을 확인한다.

10/30 왼쪽 preflight.py를 오른쪽에 맞추고, 왼쪽 실행과의 거울 기하(자차 격자·옆차 횡위치·초기 차체 간격·
상대 종방향 배치·평가 지표 부호)를 추가로 대조한다. 평가 전용 정답 기하이며 제어 입력이 아니다.
"""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from shapely.geometry import box

import run_trial as run
from straddle_route import check_straddle, wrap_route_check

for folder in ('scripts', 'src/arena_autonomy', 'src/arena_vehicle_interface'):
    sys.path.insert(0, str(run.ROOT/folder))
import validate_g1_traffic as v

LEFT = run.ROOT/'artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1'


def set_side(side):
    run.SIDE = side
    run.GRID_SLOT = 3 if side == 'left' else 0
    folder = run.BASE if side == 'right' else LEFT
    run.CONFIG = folder/(side+'_config.json')
    run.OVERLAY = folder/(side+'_overlay.json')


def road_lateral(centerline, x, y):
    along, _ = centerline.nearest_s(x, y)
    cx, cy, heading = centerline.pose(along)
    return along, -(x-cx)*math.sin(heading)+(y-cy)*math.cos(heading)


def initial_geometry(side, centerline, scene):
    """자차 격자와 등록 옆차의 초기 자세·간격. 왼쪽은 10/30 실행 입력 그대로 읽는다."""
    set_side(side)
    config = v.load_config(run.CONFIG)
    case = v.overlay_case(v.case_named(config, 'slow_lead'), run.OVERLAY)
    slot = {int(s['index']): s for s in scene['starting_grid']['slots']}[config['launch']['grid_slot']]
    ego = (slot['x'], slot['y'], slot['yaw_rad'])
    spec = [s for s in v.actor_specs(config, case) if s.get('model_name') == 'eval_e3_side']
    overlay = json.loads(run.OVERLAY.read_text(encoding='utf-8'))['additional_actors'][0]
    s0 = v.actor_start_s(config, scene, 0, {'actor_start_center_gap_m': overlay['start_center_gap_m']})
    actor = v.actor_pose_at(centerline, s0, [], 0., overlay['lateral_offset_m'])
    ego_s, ego_lat = road_lateral(centerline, *ego[:2])
    _, actor_lat = road_lateral(centerline, *actor[:2])
    c, s = math.cos(ego[2]), math.sin(ego[2])
    relative_lateral = -s*(actor[0]-ego[0])+c*(actor[1]-ego[1])
    relative_longitudinal = c*(actor[0]-ego[0])+s*(actor[1]-ego[1])
    gap = v.rectangle(*ego, .20, .15).distance(v.rectangle(*actor, .20, .15))
    return dict(grid_slot=config['launch']['grid_slot'], ego_s_m=ego_s, ego_road_lateral_m=ego_lat,
                actor_road_lateral_m=actor_lat, actor_relative_lateral_m=relative_lateral,
                actor_relative_longitudinal_m=relative_longitudinal, initial_body_gap_m=gap,
                side_actor_spec_found=len(spec) == 1, overlay_schedule=overlay['schedule'])


def main():
    checks = {}
    set_side('right')
    config = {'actor': {'length_m': .2, 'width_m': .15}}
    straight = lambda *a: {'actor_start_s_m': 0., 'actor_end_s_m': 1.}
    pose = lambda cl, s, schedule, t, lateral: (s, lateral, 0.)
    road = box(-1., -.225, 3., .225)
    for sign, label in ((1., 'left'), (-1., 'right')):
        case = {'route_kind': 'side_straddle_straight', 'actor_lateral_offset_m': sign*.20}
        args = (config, SimpleNamespace(lap=10.), case, 0., road, [], straight, pose, v.rectangle)
        checks[f'straddle_accepted_{label}'] = check_straddle(*args)['sampled_footprints'] == 101
        for name, offset, obstacles in [('fully_road', .13, []), ('fully_grass', .32, []),
                                        ('outside_wall_corridor', .40, []),
                                        ('wall_intersection', .20, [box(.4, sign*.24, .6, sign*.30)])]:
            candidate = {**case, 'actor_lateral_offset_m': sign*offset}
            try:
                check_straddle(config, args[1], candidate, 0., road, obstacles, straight, pose, v.rectangle)
                checks[f'{name}_rejected_{label}'] = False
            except ValueError:
                checks[f'{name}_rejected_{label}'] = True
    # 실제 입력: 공식 v2 고정본에서 오른쪽 등록 경로와 상대차 일정 겹침을 검사한다.
    set_side('right')
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
    # 왼쪽 10/30 실행 입력과의 거울 기하.
    right = initial_geometry('right', centerline, scene)
    left = initial_geometry('left', centerline, scene)
    set_side('right')
    checks['side_actor_spec_found'] = right['side_actor_spec_found'] and left['side_actor_spec_found']
    checks['right_actor_on_ego_right'] = right['actor_relative_lateral_m'] < -.25
    checks['left_actor_on_ego_left'] = left['actor_relative_lateral_m'] > .25
    checks['ego_lateral_mirrored_5mm'] = abs(right['ego_road_lateral_m']+left['ego_road_lateral_m']) <= .005
    checks['actor_lateral_mirrored_5mm'] = abs(right['actor_road_lateral_m']+left['actor_road_lateral_m']) <= .005
    checks['relative_lateral_mirrored_5mm'] = abs(right['actor_relative_lateral_m']+left['actor_relative_lateral_m']) <= .005
    checks['relative_longitudinal_equal_5mm'] = abs(right['actor_relative_longitudinal_m']-left['actor_relative_longitudinal_m']) <= .005
    checks['initial_body_gap_equal_5mm'] = abs(right['initial_body_gap_m']-left['initial_body_gap_m']) <= .005
    checks['schedule_equal'] = right['overlay_schedule'] == left['overlay_schedule']
    lc = json.loads((LEFT/'left_config.json').read_text(encoding='utf-8'))
    rc = json.loads(run.CONFIG.read_text(encoding='utf-8'))
    changed = sorted(k for k in set(lc) | set(rc) if lc.get(k) != rc.get(k))
    launch_changed = sorted(k for k in set(lc['launch']) | set(rc['launch']) if lc['launch'].get(k) != rc['launch'].get(k))
    checks['config_only_grid_and_metadata_changed'] = (set(changed) <= {'registered_at', 'scope', 'launch', 'parent_config'}
                                                       and launch_changed == ['grid_slot'])
    lo = json.loads((LEFT/'left_overlay.json').read_text(encoding='utf-8'))
    ro = json.loads(run.OVERLAY.read_text(encoding='utf-8'))
    la, ra = lo['additional_actors'][0], ro['additional_actors'][0]
    checks['overlay_only_lateral_sign_changed'] = (
        sorted(k for k in set(la) | set(ra) if la.get(k) != ra.get(k)) == ['lateral_offset_m']
        and ra['lateral_offset_m'] == -la['lateral_offset_m'] and lo['base_case'] == ro['base_case'])
    # 평가 지표 부호: 기존 maximum_ego_road_lateral_change_m은 절댓값이다. 방향은 signed_lateral.py가 따로 낸다.
    checks['signed_lateral_sign_convention'] = right['actor_road_lateral_m'] < 0 < left['actor_road_lateral_m']
    settings = {}
    for enabled in (False, True):
        run.CANDIDATE = enabled
        _, setting, arguments = run.launch_source(run.BASE/'preflight_unused')
        settings[str(enabled)] = (setting, arguments)
        checks[f'candidate_{enabled}_explicit'] = setting['g1_rear_lane_keep_trial'] is enabled
    run.CANDIDATE = True
    changed_settings = [k for k in settings['False'][0] if settings['False'][0][k] != settings['True'][0][k]]
    checks['only_candidate_policy_differs'] = set(changed_settings) == {'g1_rear_lane_keep_trial', 'g1_rear_lane_keep_policy'}
    checks['policy_on_measured'] = settings['True'][0]['g1_rear_lane_keep_policy'] == 'observed_fresh_v2'
    checks['launch_grid_slot_right'] = settings['True'][1]['grid_slot'] == '0'
    report = dict(passed=all(checks.values()), checks=checks, actual_routes=routes, actor_start_s_m=starts,
                  geometry=dict(right=right, left_1030=left), changed_settings=changed_settings,
                  config_changed_keys=changed, launch_changed_keys=launch_changed,
                  known_asymmetry='공식 격자 0번은 3번보다 0.2 m 앞이다. 상대차는 자차 격자 s 기준이라 상대 배치는 같지만 '
                                  '신호등·첫 코너까지의 절대 거리는 0.2 m 짧다.',
                  scope='preflight only; no Gazebo run or autonomous driving proof')
    path = run.BASE/'preflight_result.json'
    with path.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    (run.BASE/'preparation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], checks=checks, geometry=report['geometry']), ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
