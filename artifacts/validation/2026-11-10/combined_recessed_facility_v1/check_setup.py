"""실행 전 생성기·형상 감사의 일치 확인. 주행 성능 시험은 아니다."""
from copy import deepcopy
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from traffic_actors import actor_specs,compose_sdf,overlay_case
from validate_g1_traffic import actor_sdf
from audit_shapes import canonical


def main():
    origin=ROOT/'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1'
    config=json.loads((origin/'new_track_course.json').read_text())
    case=next(c for c in config['cases'] if c['name']=='two_actor_course_lap')
    case=overlay_case(case,origin/'additional_actors.json')
    before=actor_specs(config,case)
    config['actor']['visual_outline']='recessed_sides'
    for actor in case['additional_actors']:actor['visual_outline']='box'
    after=actor_specs(config,case)
    unchanged=deepcopy(after);unchanged[0].pop('visual_outline')
    unchanged[1].pop('visual_outline')
    assert unchanged==before
    poses=[(0.,0.,0.),(6.,0.,0.)]
    old,new=(compose_sdf(specs,poses,actor_sdf) for specs in (before,after))
    assert canonical(old,True)==canonical(new,True)
    import xml.etree.ElementTree as ET
    counts=[len(m.findall('link/visual')) for m in ET.fromstring(new).findall('model')]
    assert counts==[3,1]
    result=dict(scope='static generation check only; no simulation',passed=True,
        nonvisual_unchanged=True,actor_motion_config_unchanged=True,visual_counts=counts,
        primary_outline=after[0]['visual_outline'],secondary_outline=after[1]['visual_outline'])
    with (A/'setup_check.json').open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':main()
