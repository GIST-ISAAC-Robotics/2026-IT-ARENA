"""새 차량·신호 월드에서 예전 앞차/옆차 각본이 제자리에서 실제 도로를 지나는지 확인한다."""
import json
from pathlib import Path

from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons
from prelap_signal_eval import FIXTURE
from traffic_actors import actor_specs, overlay_case, check_additional_routes
from validate_g1_traffic import (load_config, case_named, load_centerline, check_route,
                                 actor_start_s, actor_pose_at)

BASE=Path(__file__).resolve().parent
OVERLAY=BASE.parents[3]/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/actor_overlay.json'


def verify():
    cfg=load_config(BASE/'course.json')
    case=overlay_case(case_named(cfg,'stop_restart_full_course'),OVERLAY)
    line,scene=load_centerline(FIXTURE)
    road=road_polygon(FIXTURE/'world.sdf'); obstacles=obstacle_polygons(FIXTURE/'world.sdf')
    specs=actor_specs(cfg,case)
    route=check_route(cfg,line,case,actor_start_s(cfg,scene,0,case),road,obstacles)
    _,extra=check_additional_routes(cfg,case,specs,scene,0,line,road,obstacles,
                                  actor_start_s,check_route,actor_pose_at)
    return dict(scope='평가 측 상대 각본의 기하만 확인. 우리 차량 진행·실제 측방 대응은 미시험.',
                primary=route,additional=extra)


if __name__=='__main__':
    data=verify()
    with (BASE/'geometry.json').open('x',encoding='utf-8') as f:
        json.dump(data,f,indent=2,ensure_ascii=False)
