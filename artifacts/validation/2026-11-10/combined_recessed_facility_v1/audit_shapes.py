"""두 상대 중 주 상대의 오목 시각만 바뀌었는지와 실제 반환을 확인한다."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;ROOT=A.parents[3];D=A/'two_actor_run1';RUN=D/'run1'
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1')]
from traffic_actors import actor_specs,compose_sdf,overlay_case
from validate_g1_traffic import actor_sdf
from outline_moving import evaluate


def canonical(text,no_visual=False):
    root=ET.fromstring(text)
    if no_visual:
        for link in root.findall('model/link'):
            for visual in link.findall('visual'):link.remove(visual)
    return ET.canonicalize(ET.tostring(root,encoding='unicode'),strip_text=True)


def main():
    path=RUN/'evaluation_actor.sdf';actual=path.read_text();root=ET.fromstring(actual)
    config=json.loads((D/'recessed_config.json').read_text())
    case=next(c for c in config['cases'] if c['name']=='two_actor_course_lap')
    case=overlay_case(case,D/'additional_actors.json');specs=actor_specs(config,case)
    poses=[]
    for spec in specs:
        values=list(map(float,root.find(f"model[@name='{spec['model_name']}']/pose").text.split()))
        poses.append((values[0],values[1],values[5]))
    expected=compose_sdf(specs,poses,actor_sdf)
    boxes=deepcopy(specs)
    for spec in boxes:spec['visual_outline']='box'
    original=compose_sdf(boxes,poses,actor_sdf)
    structural=dict(exact_expected=canonical(actual)==canonical(expected),
        unchanged_nonvisual=canonical(actual,True)==canonical(original,True),
        visual_counts=[len(m.findall('link/visual')) for m in root.findall('model')],
        sdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    observed=evaluate(RUN)
    points=observed.pop('sensor_points_actor_frame')
    with (D/'outline_points.jsonl').open('x') as stream:
        for point in points:stream.write(json.dumps(point)+'\n')
    passed=(structural['exact_expected'] and structural['unchanged_nonvisual']
        and structural['visual_counts']==[3,1] and observed['verdict'])
    result=dict(passed=passed,structure=structural,observed=observed,
        scope='primary recessed visual; secondary box; original conservative collision volumes')
    with (D/'outline_assessment.json').open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,structure=structural,returns=observed['recessed_center_returns'],
        scans=observed['recessed_center_unique_scans'],old_side=observed['old_box_center_side_returns'])))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
