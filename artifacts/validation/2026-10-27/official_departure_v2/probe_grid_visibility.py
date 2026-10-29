"""밀집 그리드의 상대차 높이별 신호 가림. 정지 기하 진단이며 검출/주행 증명은 아니다."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from preflight import BASE, ROOT, boxes, hits, project, rotation


def main():
    output = BASE / 'grid_visibility.json'
    if output.exists():
        raise FileExistsError(output)
    fixture = BASE / 'fixture'
    scene = json.loads((fixture / 'scene.json').read_text(encoding='utf-8'))
    manifest = json.loads((fixture / 'manifest.json').read_text(encoding='utf-8'))
    slots = {s['index']: s for s in scene['starting_grid']['slots']}
    camera = dict(xyz_m=manifest['camera_xyz_m'], rpy_rad=manifest['camera_rpy_rad'],
                  width_px=1280, height_px=720, horizontal_fov_deg=75)
    lamps = [dict(name=l['name'], xyz=l['xyz_m'], radius=l['radius_m'], column=l['column'])
             for l in manifest['lamps']]
    fixed = boxes(ET.parse(fixture / 'world.sdf').find('world'))
    rows = []
    for occupied in ((0, 3), (0, 1, 3), (0, 1, 2, 3, 4)):
        for height in (.25, .40):
            for ego in occupied:
                peer_boxes = [
                    (f'peer_slot_{i}', np.array([slots[i]['x'], slots[i]['y'], height / 2]),
                     rotation(0., 0., slots[i]['yaw_rad']), np.array([.10, .075, height / 2]))
                    for i in occupied if i != ego
                ]
                projection = project(lamps, slots[ego], camera)
                origin = np.asarray(projection['camera_world_xyz_m'])
                cr = rotation(0., 0., slots[ego]['yaw_rad'])
                observations = []
                for lamp in lamps:
                    xyz = np.asarray(lamp['xyz']); radius = lamp['radius']
                    toward = (origin - xyz) / np.linalg.norm(origin - xyz)
                    targets = [xyz + toward * radius]
                    targets += [xyz + toward * radius * .6 + .8 * radius *
                                (math.cos(a) * cr[:, 1] + math.sin(a) * cr[:, 2])
                                for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
                    blocked = [hits(origin, target, fixed + peer_boxes) for target in targets]
                    observations.append(dict(name=lamp['name'], column=lamp['column'],
                        blocked_samples=sum(bool(b) for b in blocked), total_samples=9,
                        blocking_objects=sorted({v for b in blocked for v in b})))
                rows.append(dict(occupied_slots=occupied, ego_slot=ego, peer_height_m=height,
                    all_lamps_in_roi=projection['all_inside_detector_region'], lamps=observations,
                    red_fully_obscured=sum(o['column'] < 4 and o['blocked_samples'] == 9 for o in observations),
                    green_fully_obscured=sum(o['column'] == 4 and o['blocked_samples'] == 9 for o in observations)))
    paths = [Path(__file__), BASE / 'preflight.py', fixture / 'world.sdf',
             fixture / 'scene.json', fixture / 'manifest.json']
    result = dict(scope='static opaque box peer occlusion, nine sampled rays per lamp; no pixel or motion proof',
        assumptions=dict(peer_length_m=.20, peer_width_m=.15, peer_heights_m=[.25, .40],
                         own_camera_hole=True, camera=camera), rows=rows,
        source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps([{k: r[k] for k in ('occupied_slots', 'ego_slot', 'peer_height_m',
                                       'red_fully_obscured', 'green_fully_obscured')} for r in rows]))


if __name__ == '__main__':
    main()
