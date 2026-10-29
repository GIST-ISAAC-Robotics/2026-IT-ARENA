"""run1 정지 영상의 마커판 가림을 원래 SDF 좌표로 확인하는 사후 분석."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parent


def intersection(origin, target, pose, size):
    # 이번 네 공식 마커판은 roll/pitch=0. 원점을 상자 좌표계로 변환한다.
    assert pose[3:5] == [0.0, 0.0]
    c, s = math.cos(pose[5]), math.sin(pose[5])
    def local(point):
        x, y, z = [point[i] - pose[i] for i in range(3)]
        return [c*x+s*y, -s*x+c*y, z]
    a, b = local(origin), local(target)
    enter, leave = 0.0, 1.0
    for start, end, length in zip(a, b, size):
        delta, half = end-start, length/2
        if abs(delta) < 1e-12:
            if abs(start) > half:
                return None
            continue
        near, far = sorted(((-half-start)/delta, (half-start)/delta))
        enter, leave = max(enter, near), min(leave, far)
        if enter > leave:
            return None
    return enter if 0 < enter < 1 else None


def main():
    fixture = BASE/'fixture'
    scene = json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    manifest = json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    slot = next(s for s in scene['starting_grid']['slots'] if s['index'] == 3)
    x, y, z = manifest['camera_xyz_m']
    yaw = slot['yaw_rad']
    origin = [slot['x']+x*math.cos(yaw)-y*math.sin(yaw),
              slot['y']+x*math.sin(yaw)+y*math.cos(yaw), z]
    model = ET.parse(fixture/'world.sdf').find("world/model[@name='it_arena_track_static']")
    markers, missing, rows = [], [], []
    for link in model.findall('link'):
        if not link.get('name').startswith('aruco_'):
            continue
        pose = list(map(float, link.findtext('pose').split()))
        size = list(map(float, link.findtext('visual/geometry/box/size').split()))
        markers.append((link.get('name'), pose, size))
        material = link.findtext('visual/material/pbr/metal/albedo_map')
        if material and not (fixture/material).exists():
            missing.append(material)
    for lamp in manifest['lamps']:
        link = model.find("link[@name='"+lamp['name']+"']")
        target = list(map(float, link.findtext('pose').split()))[:3]
        hits = []
        for name, pose, size in markers:
            fraction = intersection(origin, target, pose, size)
            if fraction is not None:
                hits.append(dict(marker=name, fraction_from_camera=fraction))
        rows.append(dict(lamp=lamp['name'], column=lamp['column'], row=lamp['row'],
                         center=target, marker_intersections=hits))
    files = [Path(__file__), fixture/'world.sdf', fixture/'scene.json', fixture/'manifest.json',
             BASE/'run1/signal_red.png', BASE/'run1/signal_green.png']
    result = dict(scope='post-run static center-ray/box diagnosis; not full visible-area or detector test',
                  camera_world=origin, lamps=rows, missing_texture_assets=missing,
                  source_sha256={str(p.relative_to(BASE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (BASE/'run1_scene_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(missing=missing, lamp_hits=[(r['lamp'],r['marker_intersections']) for r in rows])))


if __name__ == '__main__':
    main()
