"""평가용 5랩 각본. 11/1 대표 코스의 초기 급정지와 옆차는 보존하고, 재출발 뒤 빨라진다."""
import importlib.util
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
MIXED=HERE.parent/'mixed_course_v1/course.json'
OVERLAY=ROOT/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/actor_overlay.json'
CHECK_FIVE=ROOT/'artifacts/validation/2026-10-28/multilap_preparation_v1/course_candidate.py'
DURATION=300.


def expected():
    cfg=json.loads(MIXED.read_bytes());overlay=json.loads(OVERLAY.read_bytes());case=cfg['cases'][0]
    if (len(cfg['cases'])!=1 or case['name']!='stop_restart_full_course' or
            [(x['from_s'],x['to_s'],x['speed_mps']) for x in case['schedule']]!=[(0.,4.,.35),(4.,9.,0.),(9.,155.,.35)]
            or len(overlay['additional_actors'])!=1 or cfg['launch']['speed_bump']):
        raise ValueError('속도·종료를 변경할 11/1 기준 각본이 달라짐')
    case.update(duration_s=DURATION,expected='같은 초기 정지·재출발 뒤 더 빠른 앞차와 센서만으로 순진행 5랩 및 종료.')
    case['schedule'][2:]=[dict(from_s=9.,to_s=20.,speed_mps=.55),dict(from_s=20.,to_s=DURATION,speed_mps=.825)]
    cfg['acceptance'][case['name']].update(minimum_laps=5,maximum_actor_laps=5.2,
        basis='11/1 대표 시험의 HOLD·재출발 2.5초·최종 간격 1.20 m 유지. 재출발 뒤 상대 이동은 이번 각본으로 측정한다.')
    cfg['timeouts']['wall_timeout_s']=10800
    cfg.update(registered_at='2026-11-01 각본 준비, 주행 구성 선택 전',
        scope='2×5 신호, 상대 2대, 300초. 11/1 속도 구성의 다랩 후보. 실제 독립 5대나 자차 랩 판정이 아님.',
        straight_section_check='앞차 반복 경로의 1 cm 간격 평가 전용 기하 검사.')
    overlay['additional_actors'][0]['schedule'][-1]['to_s']=DURATION
    overlay.update(scope='같은 옆차가 15초까지 0.35 m/s로 이동한 뒤 나머지 285초는 잔디 쪽에 정지.')
    return cfg,overlay


def geometry(cfg,overlay):
    from audit_local_pursuit_road import road_polygon
    from build_experimental_track import obstacle_polygons
    from prelap_signal_eval import FIXTURE
    from traffic_actors import actor_specs, overlay_case, check_additional_routes
    from validate_g1_traffic import load_centerline, check_route, actor_start_s, actor_pose_at
    from g1_traffic_eval import validate_schedule,actor_travel_m
    case=overlay_case(cfg['cases'][0],HERE/'actor_overlay.json')
    if case['additional_actors']!=overlay['additional_actors']:raise ValueError('옆차 각본 불일치')
    for x in [case,*case['additional_actors']]:validate_schedule(x['schedule'],DURATION)
    spec=importlib.util.spec_from_file_location('existing_5lap_evaluator',CHECK_FIVE)
    candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
    line,scene=load_centerline(FIXTURE)
    road=road_polygon(FIXTURE/'world.sdf');obstacles=obstacle_polygons(FIXTURE/'world.sdf')
    lead=candidate.check_full_course(cfg,line,case,actor_start_s(cfg,scene,0,case),road,obstacles)
    _,other=check_additional_routes(cfg,case,actor_specs(cfg,case),scene,0,line,road,obstacles,
                                     actor_start_s,check_route,actor_pose_at)
    return dict(primary=lead,additional=other,scheduled_travel_m=actor_travel_m(case['schedule'],DURATION),
        scope='정해 둔 상대 자세의 노면·벽·종점 기하만 확인. 자차 5랩 진행, 추종·우회 성공은 아직 확인하지 않았다.')


def write(p,value):
    with p.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2)


if __name__=='__main__':
    cfg,overlay=expected()
    write(HERE/'course.json',cfg);write(HERE/'actor_overlay.json',overlay)
    write(HERE/'geometry.json',geometry(cfg,overlay))
