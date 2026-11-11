"""후속 비정형 대표의 시각 윤곽·원 충돌 상자 보존·실제 C1 반환을 대조한다."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1')]
import validate_g1_traffic as harness
from outline_moving import evaluate


def without_visuals(text):
    root=ET.fromstring(text)
    for link in root.findall('model/link'):
        for v in link.findall('visual'):link.remove(v)
    return ET.canonicalize(ET.tostring(root,encoding='unicode'),strip_text=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args();run=args.run
    path=run/'evaluation_actor.sdf';text=path.read_text()
    config=json.loads((run.parent/'recessed_config.json').read_text())
    pose=list(map(float,ET.fromstring(text).findtext('model/pose').split()))
    # 높이는 actor_sdf의 기존 설정식을 쓴다. 비교 생성에는 평면 자세만 전달한다.
    planar=(pose[0],pose[1],pose[5])
    config['actor']['visual_outline']='box'
    box=harness.actor_sdf(config,planar)
    config['actor']['visual_outline']='recessed_sides'
    recessed=harness.actor_sdf(config,planar)
    structural=dict(runtime_matches_requested_shape=ET.canonicalize(text,strip_text=True)==ET.canonicalize(recessed,strip_text=True),
                    nonvisual_matches_box=without_visuals(text)==without_visuals(box),
                    visuals=len(ET.fromstring(text).findall('model/link/visual')),
                    sdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    observed=evaluate(run)
    points=observed.pop('sensor_points_actor_frame')
    with (run.parent/'outline_points.jsonl').open('x') as stream:
        for point in points:stream.write(json.dumps(point)+'\n')
    result=dict(scope='visual-only concave shape, conservative original collision box; sensor return audit not all-body proof',
                structure=structural, observed=observed,
                passed=structural['runtime_matches_requested_shape'] and structural['nonvisual_matches_box']
                    and structural['visuals']==3 and observed['verdict'])
    out=run.parent/'outline_assessment.json'
    if out.exists():raise SystemExit('output exists')
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],structure=structural,
        returns=observed['recessed_center_returns'],scans=observed['recessed_center_unique_scans'],
        old_side_returns=observed['old_box_center_side_returns'],stable=observed['stable_under_time_shift'])))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
