#!/usr/bin/env python3
"""M5 고정 공식 램프의 카메라 피치 필요조건. 장착/시설 변경이나 인식 합격이 아니다."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import zipfile

import numpy as np
import yaml

from audit_start_signal_geometry import lamps_from_sdf, project, rotation, REVISION

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'artifacts/validation/2026-10-13/race_start_geometry_v1/result'
CAMERA_PATH = ROOT/'src/arena_description/config/b0183_c1.yaml'


def vertical_interval(points, camera, bottom_fraction=1.):
    """x전방/z위 좌표 점들의 수직 영상 포함 피치 구간. 위쪽 양수(ROS pitch는 음수).

    수평 화면/가림/검출기/광량은 별도다. 구체를 둘러싼 cube 표본이면 보수적 필요조건이
    아니라 그 cube의 수직 포함 조건이다. 중심만 넣으면 구체 포함의 필요조건이다.
    """
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
            or not np.all(np.isfinite(points)) or np.any(points[:, 0] <= 0)
            or not 0.5 <= bottom_fraction <= 1.):
        raise ValueError('finite forward targets and top crop >= 0.5 required')
    focal = camera['width_px']/(2*math.tan(math.radians(camera['horizontal_fov_deg'])/2))
    half = math.atan(camera['height_px']/(2*focal))
    lower = math.atan((bottom_fraction-.5)*camera['height_px']/focal)
    elevation = np.arctan2(points[:, 2], points[:, 0])
    lo, hi = float(np.max(elevation)-half), float(np.min(elevation)+lower)
    return dict(min_up_deg=math.degrees(lo), max_up_deg=math.degrees(hi), nonempty=bool(lo <= hi),
                target_min_elevation_deg=float(np.degrees(elevation.min())),
                target_max_elevation_deg=float(np.degrees(elevation.max())))


def relative_targets(lamps, slot, camera, base_z=0.):
    basis = rotation(0., 0., slot['yaw_rad'])
    origin = np.array([slot['x'], slot['y'], base_z])+basis@camera['xyz_m']
    centers = np.array([basis.T@(np.array(lamp['xyz'])-origin) for lamp in lamps])
    corners = np.vstack([center+np.array(list(itertools.product((-lamp['radius'], lamp['radius']), repeat=3)))
                         for center, lamp in zip(centers, lamps)])
    return centers, corners


def intersect(intervals):
    lo, hi = max(row['min_up_deg'] for row in intervals), min(row['max_up_deg'] for row in intervals)
    return dict(min_up_deg=lo, max_up_deg=hi, nonempty=lo <= hi)


def assess(lamps, slots, camera, *, base_z=0.):
    if camera['rpy_rad'] != [0., 0., 0.]:
        raise ValueError('this pitch-only audit requires horizontal nominal mount')
    rows = []
    for slot in sorted(slots, key=lambda s:s['index']):
        centers, corners = relative_targets(lamps, slot, camera, base_z)
        rows.append(dict(grid_index=slot['index'], full_cube=vertical_interval(corners, camera),
            detector_cube=vertical_interval(corners, camera, .58),
            detector_centers=vertical_interval(centers, camera, .58)))
    full = intersect([r['full_cube'] for r in rows])
    cropped_centers = intersect([r['detector_centers'] for r in rows])
    focal = camera['width_px']/(2*math.tan(math.radians(camera['horizontal_fov_deg'])/2))
    half = math.degrees(math.atan(camera['height_px']/(2*focal)))
    # 전체 영상의 공통 구간 중앙값 한 개만 대조한다. 운영 장착값으로 채택하지 않는다.
    trial = None
    if full['nonempty']:
        up = (full['min_up_deg']+full['max_up_deg'])/2
        cam = dict(camera, rpy_rad=[0., -math.radians(up), 0.])
        views = [project(lamps, slot, cam, base_z) for slot in slots]
        # 실제 마커 배치가 아니라 높이 .35..45 m, 카메라 정면 .5/1/2 m의 수직 판이다.
        markers = []
        for forward in (.5, 1., 2.):
            points = np.array([[forward, y, z-(camera['xyz_m'][2]+base_z)]
                for y, z in itertools.product((-.05, .05), (.35, .45))])
            rotated = (rotation(0., -math.radians(up), 0.).T@points.T).T
            pixels = np.column_stack((camera['width_px']/2-focal*rotated[:, 1]/rotated[:, 0],
                                     camera['height_px']/2-focal*rotated[:, 2]/rotated[:, 0]))
            markers.append(dict(forward_from_camera_m=forward, center_height_m=.4, plate_size_m=.1,
                all_inside_image=bool(np.all(rotated[:, 0]>0) and np.all(pixels[:, 0]>=0)
                    and np.all(pixels[:, 0]<camera['width_px']) and np.all(pixels[:, 1]>=0)
                    and np.all(pixels[:, 1]<camera['height_px'])), vertical_px=[float(pixels[:, 1].min()),float(pixels[:, 1].max())]))
        trial = dict(up_deg=up, ros_pitch_rad=-math.radians(up), full_image_count=sum(v['all_inside_image'] for v in views),
            detector_region_count=sum(v['all_inside_detector_region'] for v in views),
            expected_color_order_count=sum(v['red_to_green_left_to_right'] for v in views),
            flat_ground_in_front_visible=up < half, generic_marker_views=markers, views=views)
    return dict(base_z_m=base_z, vertical_fov_deg=2*half, per_slot=rows,
                all_slots_full_cube_interval=full, all_slots_detector_centers_interval=cropped_centers,
                midpoint_trial=trial)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        p.error('fresh artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    paths = [Path(__file__).resolve(), ROOT/'scripts/audit_start_signal_geometry.py',
             ROOT/'tests/test_grid_camera_pitch.py', CAMERA_PATH, SOURCE/'scene.json', SOURCE/'world.sdf']
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    with zipfile.ZipFile(output/'source.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in paths:
            archive.write(path, str(path.relative_to(ROOT)))
    scene = json.loads((SOURCE/'scene.json').read_text())
    camera = yaml.safe_load(CAMERA_PATH.read_text())['camera']
    lamps = lamps_from_sdf((SOURCE/'world.sdf').read_bytes())
    report = dict(scope='offline geometry only; no mount change or detection/physical proof',
        official_revision=REVISION, source_sha256=hashes, camera=camera,
        limitations=['pinhole and no occlusion/exposure', 'lamp spheres versus final hardware unresolved',
            'generic marker height, not official route visibility/detection', 'cube containment is conservative'],
        results=[assess(lamps, scene['starting_grid']['slots'], camera, base_z=z) for z in (0., .01)])
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    print(json.dumps([dict(base_z_m=r['base_z_m'], full=r['all_slots_full_cube_interval'],
                          detector_centers=r['all_slots_detector_centers_interval'],
                          up_deg=r['midpoint_trial']['up_deg'] if r['midpoint_trial'] else None) for r in report['results']]))


if __name__ == '__main__':
    main()
