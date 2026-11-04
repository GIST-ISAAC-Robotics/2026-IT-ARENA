"""대표 교통 장면 등록과 새 월드 경로 사전 검사. 평가 기준은 PLAN.md에 고정한다."""
import copy
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import validate_g1_traffic as evaluator  # noqa: E402

PARENT = ROOT/'config/tests/official_forward_v1.json'
OUT = ROOT/'config/tests/official_traffic_v1.json'
LEAD = dict(from_s=0., to_s=3., speed_mps=.825)


def config():
    cfg = json.loads(PARENT.read_bytes())
    acceptance = cfg['acceptance']
    stop = copy.deepcopy(acceptance['moving_stop_restart'])
    stop.update(hold_window_s=[3., 7.5], required_exit_s_m=9.8, maximum_actor_s_m=16.,
                basis='옛 정지·재개·코너 판정을 원본으로 보존한다. 경기 목표는 goal_eval.py의 별도 평가다.')
    acceptance['restart_during_bypass'] = stop
    acceptance['persistent_block'] = dict(minimum_continuous_hold_s=3., maximum_final_gap_m=.7,
                                          basis='옛 HOLD 판정 원본. 사용자 결정의 우회는 goal_eval.py에서 본다.')
    acceptance['close_cut_in'] = copy.deepcopy(acceptance['cut_in'])
    for name in ('moving_stop_restart', 'cut_in'):
        acceptance.pop(name)
    cfg['merge_acceptance'].update(moving_heading_error_max_rad=.146,
        basis='0.6 m/s·최대 곡률 약 2.14/m의 yaw rate 1.28 rad/s × 기존 0.11 s 지연 + 0.005 rad. 상대 각본 실행 확인 전용.')
    cfg['cases'] = [
        dict(name='persistent_block', behavior_kind='stopped_center', route_kind='straight', duration_s=20.,
             schedule=[LEAD, dict(from_s=3., to_s=20., speed_mps=0.)],
             expected='앞차가 계속 막으면 관측 통로로 우회해 지나간 뒤 본선으로 복귀한다.'),
        dict(name='restart_during_bypass', behavior_kind='stop_restart', route_kind='first_curve', duration_s=23.5,
             schedule=[LEAD, dict(from_s=3., to_s=7.5, speed_mps=0.), dict(from_s=7.5, to_s=23.5, speed_mps=.825)],
             expected='우회 중 앞차가 재출발하면 뒤 합류 또는 본선 복귀로 주행을 잇는다.'),
        dict(name='close_cut_in', behavior_kind='slow_lead', route_kind='merge_straight', duration_s=9.5,
             actor_start_center_gap_m=3.25, actor_lateral_offset_m=-.30,
             schedule=[dict(from_s=0., to_s=4.5, speed_mps=0.), dict(from_s=4.5, to_s=9.5, speed_mps=.6)],
             merge_profile=dict(start_distance_m=0., transition_m=.9, final_offset_m=0., wheelbase_m=.135,
                                max_steering_rad=.37, maximum_initial_offset_m=.32),
             expected='오른쪽 잔디의 차가 바로 앞 중앙으로 끼어들면 완만히 감속해 뒤따른다.'),
    ]
    cfg.update(registered_at='2026-11-02 새 공식 월드 대표 교통 장면', parent_config=dict(path=PARENT.relative_to(ROOT).as_posix()),
               scope='우회·우회 중 재출발·근접 끼어들기 대표 장면. 전체 본선·실차·독립 상대 아님. 평가는 PLAN.md.',
               matrix=dict(g1g2=dict(cases=[c['name'] for c in cfg['cases']], seeds=[0])))
    return cfg


def main():
    OUT.write_bytes((json.dumps(config(), ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
    cfg = evaluator.load_config(OUT)
    line, scene = evaluator.load_centerline()
    world = evaluator.DEMO_WORLD/'world.sdf'
    road, obstacles = evaluator.road_polygon(world), evaluator.obstacle_polygons(world)
    result = {}
    for case in cfg['cases']:
        route = evaluator.check_route(cfg, line, case, evaluator.actor_start_s(cfg, scene, 0, case), road, obstacles)
        result[case['name']] = {k: v for k, v in route.items() if not isinstance(v, (list, dict))}
    (HERE/'geometry.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(result, ensure_ascii=False)[:900])


if __name__ == '__main__':
    main()
