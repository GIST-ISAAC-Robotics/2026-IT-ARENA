"""대표 전방 두 장면을 등록한다. 판정선은 선정 구성 값을 그대로 쓰고 사전 경로 검사는 새 공식 월드로 수행한다."""
import copy
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import validate_g1_traffic as evaluator  # noqa: E402

PARENT = ROOT/'artifacts/validation/2026-11-01/race_distance_v2/course.json'
MERGE_RULES = ROOT/'config/tests/m2_merge_stop_restart_v1.json'
OUT = ROOT/'config/tests/official_forward_v1.json'


def config():
    cfg = json.loads(PARENT.read_bytes())
    acceptance = cfg['acceptance']
    stop = copy.deepcopy(acceptance['stop_restart_first_curve'])
    stop.update(hold_window_s=[3., 8.], basis='선정 구성의 정지·재개·코너 출구 기준. 정지 시작·끝만 이 각본의 3–8초에 맞춘다.')
    cut = copy.deepcopy(acceptance['slow_lead'] if 'slow_lead' in acceptance else {})
    cut.update(minimum_progress_m=2.5, maximum_stationary_while_lead_moves_s=2.5, maximum_final_gap_m=1.2,
               basis='기존 앞차 추종 기준. 상대는 코너 1 m 전에 멈춘다.')
    acceptance['moving_stop_restart'] = stop
    acceptance['cut_in'] = cut
    for name in ('stop_restart_full_course', 'stop_restart_first_curve', 'first_curve', 'initial_occlusion', 'stop_restart'):
        acceptance.pop(name, None)
    rules = copy.deepcopy(json.loads(MERGE_RULES.read_bytes())['merge_acceptance'])
    rules.update(moving_heading_error_max_rad=.085,
                 basis='0.55 m/s·최대 곡률 약 1.26/m의 yaw rate 0.69 rad/s × 기존 0.11 s 지연 + 0.005 rad. 상대 각본 실행 확인 전용.')
    cases = [
        dict(name='moving_stop_restart', behavior_kind='stop_restart', route_kind='first_curve', duration_s=19.,
             schedule=[dict(from_s=0., to_s=3., speed_mps=.825), dict(from_s=3., to_s=8., speed_mps=0.),
                       dict(from_s=8., to_s=19., speed_mps=.825)],
             expected='선정 교통 순항 속도의 앞차를 따라가다 직선에서 급정지하면 멈추거나 근거 있게 대응하고, 재출발 뒤 첫 U자를 함께 지난다.'),
        dict(name='cut_in', behavior_kind='slow_lead', route_kind='merge_straight', duration_s=9.,
             actor_lateral_offset_m=.14,
             schedule=[dict(from_s=0., to_s=2., speed_mps=.35), dict(from_s=2., to_s=9., speed_mps=.55)],
             merge_profile=dict(start_distance_m=.5, transition_m=.8, final_offset_m=0., wheelbase_m=.135, max_steering_rad=.37),
             expected='왼쪽 차로에 보이던 상대가 자차 앞 중앙으로 들어오면 완만히 감속해 뒤따른다.'),
    ]
    cfg.update(registered_at='2026-11-02 공식 v2026.11.01 월드 대표 전방 장면', cases=cases,
               scope='선정 교통 구성(g1g2 paced·양보 전진 근거·후방 스윕·1.5 m/s)의 짧은 대표 장면. 후열 그리드 3 RGB 출발 범위. 전체 본선·실차·독립 상대 아님.',
               merge_acceptance=rules, matrix=dict(g1g2=dict(cases=['moving_stop_restart', 'cut_in'], seeds=[0])),
               parent_config=dict(path=PARENT.relative_to(ROOT).as_posix()))
    cfg['timeouts']['wall_timeout_s'] = 1500
    cfg['launch']['temporary_traffic_light_trigger'] = '공식 2×5 실제 픽셀 출발. 시험 초록은 출발 신호 아님.'
    return cfg


def main():
    OUT.write_bytes((json.dumps(config(), ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
    cfg = evaluator.load_config(OUT)
    line, scene = evaluator.load_centerline()
    world = evaluator.DEMO_WORLD/'world.sdf'
    road, obstacles = evaluator.road_polygon(world), evaluator.obstacle_polygons(world)
    result = {}
    for case in cfg['cases']:
        start = evaluator.actor_start_s(cfg, scene, 0, case)
        route = evaluator.check_route(cfg, line, case, start, road, obstacles)
        result[case['name']] = {k: v for k, v in route.items() if not isinstance(v, (list, dict))}
    (HERE/'geometry.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(result, ensure_ascii=False)[:900])


if __name__ == '__main__':
    main()
