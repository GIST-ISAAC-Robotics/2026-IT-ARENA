#!/usr/bin/env python3
"""주행 기록의 평면 차체 외형과 실제 SDF 노면을 사후 대조. 제어기 입력 아님."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from shapely.ops import unary_union
from build_experimental_track import rectangle


def road_polygon(world_path):
    root = ET.parse(world_path)
    model = root.find("./world/model[@name='it_arena_track_static']")
    model_pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
    if any(abs(v) > 1e-8 for v in model_pose):
        raise ValueError('audit requires world-origin track model')
    link = model.find("link[@name='track_surface']")
    lp = list(map(float, link.findtext('pose', '0 0 0 0 0 0').split()))
    patches = []
    for collision in link.findall('collision'):
        cp = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
        size = collision.findtext('geometry/box/size')
        if size is None or any(abs(v) > 1e-8 for v in (lp[3], lp[4], cp[3], cp[4])):
            raise ValueError('unsupported non-box or tilted road patch')
        sx, sy, _ = map(float, size.split())
        c, s = math.cos(lp[5]), math.sin(lp[5])
        patches.append(rectangle(lp[0]+c*cp[0]-s*cp[1], lp[1]+s*cp[0]+c*cp[1], lp[5]+cp[5], sx, sy))
    if not patches:
        raise ValueError('no road collision patches')
    return unary_union(patches)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    world = run/'runtime_world.sdf'
    road = road_polygon(world)
    rows = [json.loads(line) for line in (run/'trajectory.jsonl').read_text().splitlines()]
    stop_path = run/'stop_trace.json'
    if stop_path.exists():
        stop = json.loads(stop_path.read_text())
        # 이전 stop_trace에는 yaw가 없다. 주행 표본의 방향을 정지 방향으로 대신 쓰지 않는다.
    else:
        stop = []
    def measure(records):
        samples = []
        for row in records:
            outside = rectangle(row['x'], row['y'], row['yaw'], .20, .15).difference(road).area
            samples.append({'sim_time_s': row['time'], 'progress_m': row.get('progress_m'),
                            'outside_road_area_m2': outside, 'fraction_of_footprint': outside/.03})
        return samples
    samples = measure([row for row in rows if row['autonomy'].get('started')])
    stop_samples = measure([row for row in stop if 'yaw' in row])
    result = {
        'scope': 'post-run truth-only evaluation: 20x15cm body footprint vs SDF track_surface collision union; '
                 'sampled poses, not continuous wheel contact or official penalty judgement; stopping not assessed without yaw',
        'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
        'audit_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'driving_samples': len(samples), 'stop_samples': len(stop_samples),
        'stop_samples_not_assessed': len(stop)-len(stop_samples),
        'outside_samples_over_1cm2': sum(row['outside_road_area_m2'] > .0001 for row in samples),
        'max_outside_road_area_m2': max((row['outside_road_area_m2'] for row in samples), default=0.),
        'outside_samples': [row for row in samples if row['outside_road_area_m2'] > .0001],
        'stop_outside_samples_over_1cm2': sum(row['outside_road_area_m2'] > .0001 for row in stop_samples),
        'stop_max_outside_road_area_m2': max((row['outside_road_area_m2'] for row in stop_samples), default=0.),
    }
    (run/'road_audit.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({key:value for key,value in result.items() if key != 'outside_samples'}, indent=2))


if __name__ == '__main__':
    main()
