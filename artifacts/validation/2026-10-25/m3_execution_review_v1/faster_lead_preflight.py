"""앞차 속도 변경 한 항목과 직선 기하 확인. 주행 전 평가 전용."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from shapely.strtree import STRtree
import validate_g1_traffic as h
import g1_traffic_eval as ev


def check_geometry(cfg, case, start, line, road, walls):
    travel=ev.actor_travel_m(case['schedule'],case['duration_s'])
    tree=STRtree(walls)
    steps=np.linspace(start,start+travel,int(np.ceil(travel/.01))+1)
    for s in steps:
        body=ev.rectangle(*line.pose(s),cfg['actor']['length_m'],cfg['actor']['width_m'])
        assert body.difference(road).area <= 1e-6, ('outside_road',s)
        assert not any(body.intersection(walls[int(i)]).area > 1e-6 for i in tree.query(body)), ('wall',s)
    return {'samples':len(steps),'maximum_sample_step_m':.01,'road_outside_samples':0,'wall_overlap_samples':0}


if __name__=='__main__':
    path=Path('config/tests/m2_faster_lead_stop_v1.json')
    cfg=h.load_config(path)
    base=h.load_config(Path('config/tests/g1_traffic_v1.json'))
    case=h.case_named(cfg,'stop_restart')
    original=copy.deepcopy(h.case_named(base,'stop_restart'))
    original['schedule'][0]['speed_mps']=.5
    original['expected']=case['expected']
    assert case==original
    for key in ('launch','vehicle_and_sensor_unchanged','actor','acceptance','timeouts','seeds'):
        assert cfg[key]==base[key],key
    line,scene=h.load_centerline()
    start=h.actor_start_s(cfg,scene,0,case)
    route=h.check_route(cfg,line,case,start)
    road=h.road_polygon(h.DEMO_WORLD/'world.sdf')
    walls=h.obstacle_polygons(h.DEMO_WORLD/'world.sdf')
    assert ev.actor_travel_m(case['schedule'],15.)==4.1
    assert ev.actor_speed_mps(case['schedule'],3.99)==.5
    assert ev.actor_speed_mps(case['schedule'],4.01)==0.
    assert ev.actor_speed_mps(case['schedule'],9.01)==.35
    out={'changed_stimulus_only':True,'config_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
         'route':route,'geometry':check_geometry(cfg,case,start,line,road,walls),
         'scope':'preregistered schedule and sampled geometry, not a drive result'}
    output=Path(sys.argv[1])
    assert not output.exists()
    output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False))
