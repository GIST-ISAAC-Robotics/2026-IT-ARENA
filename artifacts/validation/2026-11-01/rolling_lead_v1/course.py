"""첫 U자의 기존 코스에서 움직이는 앞차를 따라갈 때 속도를 유지하는지 시험한다."""
from hashlib import sha256
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
PARENT = ROOT/'config/tests/g1_traffic_extensions_v1.json'
CONFIG = BASE/'course.json'
# 35초 × 0.35 m/s와 정확히 같은 12.25 m. 같은 경로에선 정지하지 않고 속도만 바꾼다.
SCHEDULE = [(0.,6.,.85),(6.,13.,.55),(13.,17.,.825)]


def expected():
    old = json.loads(PARENT.read_bytes())
    case = next(c for c in old['cases'] if c['name']=='first_curve')
    return {**old, 'registered_at':'2026-11-01', 'scope':'3번 그리드에서 앞차가 멈추지 않고 첫 U자 코너를 감속·재가속해 통과하는 팀 시험',
        'parent_config':{'path':PARENT.relative_to(ROOT).as_posix(),'sha256':sha256(PARENT.read_bytes()).hexdigest()},
        'launch':{**old['launch'],'speed_profile':'g1_open_1p5','speed_profile_note':'11/1 대표 교통과 같은 센서·통합 제어 및 1.5 m/s 프로필',
                  'temporary_traffic_light_trigger':'2×5 실제 센서 녹색 확인'},
        'cases':[{**case, 'duration_s':17., 'schedule':[{'from_s':a,'to_s':b,'speed_mps':v} for a,b,v in SCHEDULE],
                  'expected':'움직이는 상대를 따라 동일 U자 출구·간격 기준을 만족하고 재가속 구간에서 계속 진행한다.'}]}


def geometry(cfg):
    from audit_local_pursuit_road import road_polygon
    from build_experimental_track import obstacle_polygons
    from prelap_signal_eval import FIXTURE
    from validate_g1_traffic import actor_start_s,check_route,load_centerline
    line,scene=load_centerline(FIXTURE)
    case=cfg['cases'][0];s0=actor_start_s(cfg,scene,0,case)
    route=check_route(cfg,line,case,s0,road_polygon(FIXTURE/'world.sdf'),obstacle_polygons(FIXTURE/'world.sdf'))
    old=json.loads(PARENT.read_bytes())
    prior=next(c for c in old['cases'] if c['name']==case['name'])
    distance=lambda c:sum((r['to_s']-r['from_s'])*r['speed_mps'] for r in c['schedule'])
    if abs(distance(prior)-distance(case))>1e-8 or cfg['acceptance']!=old['acceptance']:
        raise ValueError('기존 출구·간격·경로 길이가 달라짐')
    return dict(scope='실행 전 상대 경로의 기하 검증. 자차 주행 결과가 아님.',
                unchanged_actor_travel_m=distance(case),start_center_gap_m=cfg['actor']['start_center_gap_m'],
                route=route,phase_speeds_mps=[r['speed_mps'] for r in case['schedule']])


if __name__=='__main__':
    cfg=expected();g=geometry(cfg)
    for p,data in ((CONFIG,cfg),(BASE/'geometry.json',g)):
        with p.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    print(json.dumps(g,ensure_ascii=False))
