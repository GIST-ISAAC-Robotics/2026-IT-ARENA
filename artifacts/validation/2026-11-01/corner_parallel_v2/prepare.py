"""평가용 고정 옆차 각본. 제어기에는 상대 자세나 기준 주행을 전달하지 않는다."""
from copy import deepcopy
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
PARENT=HERE.parent/'rolling_lead_v1/course.json'
REFERENCE=HERE.parent/'rolling_lead_v1/curve_paced/trace.json'
DURATION=17.
LATERAL=.18


def expected():
    cfg=json.loads(PARENT.read_bytes())
    cfg['registered_at']='2026-11-01 첫 U자 코너의 왼쪽 동행차'
    cfg['scope']='앞차 추종 중 첫 코너에 옆차를 추가. 실시간 자차 정답 추종이 아닌 과거 주행에서 만든 고정 시간 각본.'
    cfg['vehicle_and_sensor_unchanged']='20x15 cm, 단일 모터축 28카운트·8.970588:1·65 mm, C1 순차 10 Hz. 방지턱 OFF.'
    rows=[r for r in json.loads(REFERENCE.read_bytes()) if r.get('elapsed_since_green_s') is not None]
    import numpy as np
    progress=np.interp(list(range(18)),[r['elapsed_since_green_s'] for r in rows],[r['progress_m'] for r in rows])
    progress=np.maximum.accumulate(progress-progress[0])
    schedule=[dict(from_s=float(i),to_s=float(i+1),speed_mps=float(progress[i+1]-progress[i])) for i in range(17)]
    overlay=dict(schema_version=1,registered_at='2026-11-01 실행 전',base_case='first_curve',
        scope='왼쪽 +0.18 m 고정 오프셋에서 과거 자차 진행 시간표를 따라 첫 U자를 통과. 현재 자차를 따라 움직이지 않음.',
        reference=dict(path=REFERENCE.relative_to(ROOT).as_posix(),sha256=sha256(REFERENCE.read_bytes()).hexdigest()),
        additional_actors=[dict(model_name='eval_corner_side',role='side',start_center_gap_m=0.,
            lateral_offset_m=LATERAL,route_kind='first_curve',minimum_body_gap_m=.08,
            gap_basis='기존 80 mm 측방 진단선. 실제 접촉·정체·두 바퀴 노면을 별도 보고하며 제어기 간격 변경 아님.',
            schedule=schedule)])
    return cfg,overlay


def side_route(config,line,case,start,road,obstacles):
    """기존 first_curve 검사는 중앙선만 검사하므로 옆차에는 실제 오프셋 외형을 검사한다."""
    import numpy as np
    from shapely.strtree import STRtree
    from g1_traffic_eval import actor_travel_m,rectangle
    from g1_course_eval import wheel_center_surface
    from validate_g1_traffic import actor_pose_at
    if case.get('route_kind')!='first_curve' or case.get('actor_lateral_offset_m')!=LATERAL:
        raise ValueError('등록 왼쪽 코너 경로가 아님')
    distance=actor_travel_m(case['schedule'],DURATION)
    samples=np.linspace(start,start+distance,math.ceil(distance/.01)+1)
    poses=[(float(i),*actor_pose_at(line,float(s),[],0.,LATERAL)) for i,s in enumerate(samples)]
    wall_count=0; outside_count=0; tree=STRtree(obstacles)
    for _,x,y,yaw in poses:
        footprint=rectangle(x,y,yaw,config['actor']['length_m'],config['actor']['width_m'])
        wall_count+=any(footprint.intersection(obstacles[int(i)]).area>1e-6 for i in tree.query(footprint))
        outside_count+=footprint.difference(road).area>1e-6
    wheels=wheel_center_surface(road,poses)
    if wall_count or not wheels['valid'] or wheels['max_off_wheels']>2:
        raise ValueError(f'상대 경로 벽/세 바퀴 이탈: {wall_count}, {wheels}')
    return dict(kind='first_curve_side_registered',actor_start_s_m=start,actor_end_s_m=start+distance,
                actual_lateral_offset_m=LATERAL,wall_samples=wall_count,body_outside_samples=outside_count,
                wheels=wheels,scope='1 cm 경로 표본의 상대 외형·노면 검사. 실제 동행 및 연속 충돌 증명 아님.')


def geometry(cfg,overlay):
    import validate_g1_traffic as ev
    from prelap_signal_eval import FIXTURE
    from traffic_actors import actor_specs,check_additional_routes
    from audit_local_pursuit_road import road_polygon
    from build_experimental_track import obstacle_polygons
    from g1_traffic_eval import rectangle
    line,scene=ev.load_centerline(FIXTURE)
    road=road_polygon(FIXTURE/'world.sdf');walls=obstacle_polygons(FIXTURE/'world.sdf')
    case=deepcopy(cfg['cases'][0]);case['additional_actors']=overlay['additional_actors']
    specs=actor_specs(cfg,case)
    lead=ev.check_route(cfg,line,case,ev.actor_start_s(cfg,scene,0,case),road,walls)
    starts,side=check_additional_routes(cfg,case,specs,scene,0,line,road,walls,ev.actor_start_s,side_route,ev.actor_pose_at)
    first=json.loads(REFERENCE.read_bytes())[0]
    own=rectangle(first['x'],first['y'],first['yaw'],.20,.15)
    other=rectangle(*ev.actor_pose_at(line,starts[1],[],0.,LATERAL),.20,.15)
    if own.intersection(other).area>1e-9:raise ValueError('초기 자차와 옆차 겹침')
    return dict(primary=lead,additional=side,initial_ego_side_gap_m=own.distance(other),
                reference_sha256=sha256(REFERENCE.read_bytes()).hexdigest(),
                scope='실행 전 기하 배치. 코너 병렬 주행 성립은 실제 결과로 별도 판정.')


def write(path,data):
    with path.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)


if __name__=='__main__':
    cfg,overlay=expected();result=geometry(cfg,overlay)
    for name,value in [('course.json',cfg),('actor_overlay.json',overlay),('geometry.json',result)]:write(HERE/name,value)
    print(json.dumps(result,ensure_ascii=False))
