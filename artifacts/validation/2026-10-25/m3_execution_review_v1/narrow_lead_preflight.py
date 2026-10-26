"""좁은 상대차 형상·시야 측정면·도로 경로 대조. 평가 전용."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import validate_g1_traffic as h
import traffic_actors as actors


if __name__=='__main__':
    path=Path('config/tests/m2_narrow_offset_lead_v1.json')
    cfg=h.load_config(path)
    base=h.load_config(Path('config/tests/m2_faster_lead_stop_v1.json'))
    for key in ('launch','vehicle_and_sensor_unchanged','acceptance','timeouts','seeds'):
        assert cfg[key]==base[key],key
    expected=copy.deepcopy(base['actor'])
    expected.update(width_m=.10,lateral_offset_m=.08,visible_section=cfg['actor']['visible_section'])
    assert cfg['actor']==expected
    assert cfg['cases'][0]['schedule']==base['cases'][0]['schedule']
    case=h.case_named(cfg,'stop_restart')
    specs=actors.actor_specs(cfg,case)
    assert len(specs)==1 and specs[0]['width_m']==.10
    line,scene=h.load_centerline()
    start=h.actor_start_s(cfg,scene,0,case)
    road=h.road_polygon(h.DEMO_WORLD/'world.sdf')
    walls=h.obstacle_polygons(h.DEMO_WORLD/'world.sdf')
    route=h.check_route(cfg,line,case,start,road,walls)
    raw=h.actor_sdf(cfg,h.actor_pose_at(line,start,case['schedule'],0.,.08))
    sdf=ET.fromstring(raw)
    sizes={kind:list(map(float,sdf.findtext(f'model/link/{kind}/geometry/box/size').split()))
           for kind in ('visual','collision')}
    assert sizes['visual']==sizes['collision']==[.20,.10,.12]
    actor=cfg['actor']
    assert actor['bottom_m']<.095<actor['bottom_m']+actor['height_m']
    out={'source_config_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
         'scope':'synthetic smaller opponent only; ego model unchanged; sampled route and declared LiDAR plane',
         'registered_changes_only':True,'actor_sizes_m':sizes,'lidar_plane_intersects':True,'route':route}
    output=Path(sys.argv[1]); assert not output.exists()
    output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False))
