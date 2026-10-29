#!/usr/bin/env python3
"""공식 고정 SDF와 B0183 그리드 핀홀 기하. 렌더링·실물 시야 합격 판정 아님."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
REVISION = 'b27f94730b869837a804a3088447103d6b35b87e'
BASE = f'https://raw.githubusercontent.com/MOSW626/istech-it-arena/{REVISION}/track/output_final/'


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def pose(element):
    item = element.find('pose')
    if item is not None and item.attrib:
        raise ValueError('named/degree pose is unsupported; do not silently assume a frame')
    values = list(map(float, '0 0 0 0 0 0'.split() if item is None else item.text.split()))
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid SDF pose')
    return np.array(values[:3]), rotation(*values[3:])


def lamps_from_sdf(raw):
    tree = ET.fromstring(raw)
    model = tree.find("world/model[@name='it_arena_track_static']")
    if model is None:
        raise ValueError('expected official track model missing')
    origin, orientation = pose(model)
    lamps = []
    for name in ('lamp_red_1', 'lamp_red_2', 'lamp_red_3', 'lamp_red_4', 'lamp_green'):
        link = model.find(f"link[@name='{name}']")
        if link is None:
            raise ValueError('lamp missing: '+name)
        visual = link.find('visual')
        local, local_r = pose(link)
        visual_xyz, _ = pose(visual)
        radius = float(visual.findtext('geometry/sphere/radius', 'nan'))
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('expected spherical emitter')
        lamps.append(dict(name=name, xyz=(origin+orientation@(local+local_r@visual_xyz)).tolist(), radius=radius))
    return lamps


def project(lamps, slot, camera, base_z=0.):
    if not math.isfinite(base_z):
        raise ValueError('invalid base height')
    yaw = slot['yaw_rad']
    base_r = rotation(0., 0., yaw)
    camera_xyz = np.array([slot['x'], slot['y'], base_z])+base_r@np.asarray(camera['xyz_m'])
    camera_r = base_r@rotation(*camera['rpy_rad'])
    width, height = camera['width_px'], camera['height_px']
    focal = width/(2*math.tan(math.radians(camera['horizontal_fov_deg'])/2))
    rows = []
    for lamp in lamps:
        point = camera_r.T@(np.asarray(lamp['xyz'])-camera_xyz)
        x, y, z = point
        row = dict(name=lamp['name'], camera_xyz_m=point.tolist(), inside_image=False,
                   inside_detector_region=False, diameter_approx_px=None, center_uv=None, cube_bound_uv=None)
        # 구체를 둘러싼 직육면체의 보수적인 영상 범위다. 가림 판정은 하지 않는다.
        if x > lamp['radius']:
            cube = point+np.array(list(itertools.product((-lamp['radius'], lamp['radius']), repeat=3)))
            pixels = np.column_stack((width/2-focal*cube[:, 1]/cube[:, 0], height/2-focal*cube[:, 2]/cube[:, 0]))
            low, high = pixels.min(axis=0), pixels.max(axis=0)
            row.update(diameter_approx_px=float(focal*2*lamp['radius']/x),
                       center_uv=[float(width/2-focal*y/x), float(height/2-focal*z/x)],
                       cube_bound_uv=[*low.tolist(), *high.tolist()],
                       inside_image=bool(low[0]>=0 and low[1]>=0 and high[0]<width and high[1]<height),
                       inside_detector_region=bool(low[0]>=0 and low[1]>=0 and high[0]<width and high[1]<height*.58))
        rows.append(row)
    centers = [r['center_uv'] for r in rows]
    left_to_right = all(centers[i] is not None and centers[i+1] is not None and
                        centers[i+1][0] > centers[i][0] for i in range(4))
    axis = np.asarray(lamps[-1]['xyz'])-np.asarray(lamps[0]['xyz'])
    norm = float(np.linalg.norm(axis))
    parallel = None if norm <= 1e-12 else float(abs(np.dot(axis/norm, camera_r[:, 0])))
    return dict(grid_index=slot['index'], camera_world_xyz_m=camera_xyz.tolist(), lamps=rows,
                all_inside_image=all(r['inside_image'] for r in rows),
                all_inside_detector_region=all(r['inside_detector_region'] for r in rows),
                red_to_green_left_to_right=left_to_right, lamp_axis_abs_dot_camera_forward=parallel)


def diagnostic_transform(lamps, *, yaw_delta=0., z=None):
    center = np.mean([lamp['xyz'] for lamp in lamps], axis=0)
    r = rotation(0., 0., yaw_delta)
    result = []
    for lamp in lamps:
        point = center+r@(np.asarray(lamp['xyz'])-center)
        if z is not None:
            point[2] += z-center[2]
        result.append(dict(lamp, xyz=point.tolist()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-dir', type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifacts subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    code_paths = ['scripts/audit_start_signal_geometry.py', 'tests/test_start_signal_geometry.py',
                  'src/arena_description/config/b0183_c1.yaml']
    code_hashes = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in code_paths}
    with zipfile.ZipFile(output/'analysis_source.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in code_paths:
            archive.write(ROOT/name, name)
    sources = {}
    payloads = {}
    for name in ('scene.json', 'world.sdf'):
        raw = (args.source_dir/name).read_bytes() if args.source_dir else urllib.request.urlopen(BASE+name, timeout=30).read(16_000_001)
        if len(raw) > 16_000_000:
            raise ValueError('source too large')
        (output/name).write_bytes(raw)
        payloads[name] = raw
        sources[name] = dict(url=BASE+name, sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw))
    camera_path = ROOT/'src/arena_description/config/b0183_c1.yaml'
    camera = yaml.safe_load(camera_path.read_text(encoding='utf-8'))['camera']
    scene = json.loads(payloads['scene.json'])
    lamps = lamps_from_sdf(payloads['world.sdf'])
    scenarios = {
        'official_sdf': lamps,
        'diagnostic_rotate_plus90_only': diagnostic_transform(lamps, yaw_delta=math.pi/2),
        'diagnostic_z030_only': diagnostic_transform(lamps, z=.30),
        'diagnostic_rotate_plus90_z030': diagnostic_transform(lamps, yaw_delta=math.pi/2, z=.30),
        'diagnostic_rotate_minus90_z030': diagnostic_transform(lamps, yaw_delta=-math.pi/2, z=.30),
    }
    result = dict(scope='pinhole_geometry_only_no_occlusion_rendering_or_hardware_claim',
                  official_revision=REVISION, sources=sources, analysis_source_hashes=code_hashes, camera=camera,
                  camera_sha256=hashlib.sha256(camera_path.read_bytes()).hexdigest(),
                  assumptions=['base z=0 nominal; suspension/contact height not modeled',
                    'square pixels, configured horizontal FoV, no distortion',
                    'bounding cube is conservative; no walls/boards/cars occlusion check',
                    'diagnostic variants are virtual coordinates, not edits or installation recommendations'],
                  scenarios={name: [project(values, slot, camera) for slot in scene['starting_grid']['slots']]
                             for name, values in scenarios.items()})
    (output/'geometry.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({name: dict(inside_image=sum(r['all_inside_image'] for r in rows),
                                     left_to_right=sum(r['red_to_green_left_to_right'] for r in rows))
                      for name, rows in result['scenarios'].items()}))


if __name__ == '__main__':
    main()
