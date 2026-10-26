"""두 앞차 일정과 공식 직선 경계의 사전 대조(평가 전용)."""
import json
from pathlib import Path
import sys
import validate_g1_traffic as h
import traffic_actors as actors
import g1_traffic_eval as ev


def main():
    cfg=h.load_config(Path('config/tests/g1_traffic_v1.json'))
    case=actors.overlay_case(h.case_named(cfg,'stop_restart'),Path('config/tests/m2_queue_staggered_v1.json'))
    specs=actors.actor_specs(cfg,case)
    line,scene=h.load_centerline()
    road=h.road_polygon(h.DEMO_WORLD/'world.sdf')
    walls=h.obstacle_polygons(h.DEMO_WORLD/'world.sdf')
    primary=h.check_route(cfg,line,case,h.actor_start_s(cfg,scene,0,case),road,walls)
    starts,checked=actors.check_additional_routes(cfg,case,specs,scene,0,line,road,walls,
        start_s=h.actor_start_s,route_check=h.check_route,pose_at=h.actor_pose_at)
    assert checked['sampled_actor_pair_min_gap_m']>=.44
    assert ev.actor_speed_mps(specs[0]['schedule'],8.5)==0.
    assert ev.actor_speed_mps(specs[1]['schedule'],8.5)==.35
    out={'primary_route':primary,'additional_routes':checked,'starts_s_m':starts,
         'near_restart_s':9.,'far_restart_s':8.,'scope':'schedule geometry only, no drive result'}
    path=Path(sys.argv[1])
    assert not path.exists()
    path.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False))


if __name__=='__main__':
    main()

