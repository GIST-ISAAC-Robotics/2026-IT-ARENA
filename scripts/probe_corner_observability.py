#!/usr/bin/env python3
"""동심 원형 회랑의 안쪽 가시 실루엣. 정답 형상은 평가 전용이며 주행 허가 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from inspect_corner_geometry import components
from profile_follow_trace import SNAPSHOT_MODULES

ROOT = Path(__file__).resolve().parents[1]


def visible_inner_diameter(radius, observer_distance):
    """원 밖의 한 점에서 보이는 가까운 원호의 최대 점간 거리(연속 광선 이상형)."""
    if not (0 < radius < observer_distance):
        raise ValueError('observer must be outside a positive circle')
    return 2*radius*math.sqrt(1-(radius/observer_distance)**2)


def circle_first_hit(origin, directions, center, radius):
    delta = np.asarray(origin)-np.asarray(center)
    direction = np.asarray(directions, dtype=float)
    along = direction@delta
    disc = along*along-float(delta@delta)+radius*radius
    root = np.sqrt(np.maximum(0., disc))
    near, far = -along-root, -along+root
    positive = np.where(near >= 0., near, np.where(far >= 0., far, np.inf))
    return np.where(disc >= 0., positive, np.inf)


def annular_scan(radius, sign=1, samples=500, noise=None):
    cfg = FollowConfig()
    center = np.array([0., sign*(radius+cfg.wall_offset_m)])
    origin = np.array([.06, 0.])
    angles = -math.pi+np.arange(samples)*math.tau/samples
    directions = np.column_stack((np.cos(angles),np.sin(angles)))
    inner = circle_first_hit(origin,directions,center,radius)
    outer = circle_first_hit(origin,directions,center,radius+2*cfg.wall_offset_m)
    labels = inner < outer
    ranges = np.minimum(inner,outer)
    if noise is not None:
        ranges = ranges+noise
    return ranges, labels, directions, origin, center


def fixed_history():
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for t in np.arange(0.,1.01,.01):
        history.add_mean_wheel(float(t),0.)
        history.add_gyro(float(t),0.)
    return history


def evaluate(radius, sign, noise_name):
    noise = None if noise_name=='none' else np.random.default_rng(0).uniform(-.03,.03,500)
    ranges, labels, directions, origin, center = annular_scan(radius,sign,noise=noise)
    cfg = FollowConfig(boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')
    controller, history = LocalFollow(cfg), fixed_history()
    for now in (.4,.5,.6,.7):
        scene=scene_from_scan(ranges.tolist(),angle_min=-math.pi,angle_increment=math.tau/500,
            range_min=.05,range_max=12.,acquired_at_s=(now-.1+np.arange(500)*.0002).tolist(),
            t_ref_s=now,rear_half_angle_deg=30.)
        result=controller.command(scene,history,0.,'left' if sign==1 else 'right')
    # label은 평가에서만 사용. 제어기에는 동일한 전체 scene만 전달했다.
    observed=np.array([r.hit_xy_m for r in scene.rays if r.status=='return' and labels[r.source_index]])
    extent=float(np.max(np.linalg.norm(observed[:,None]-observed[None],axis=2))) if len(observed)>1 else 0.
    parts=components(np.asarray(scene.occupied_endpoints_xy_m),'left' if sign==1 else 'right')
    return dict(inner_radius_m=radius,side='left' if sign==1 else 'right',noise=noise_name,
        observer_distance_m=float(np.linalg.norm(origin-center)),
        ideal_visible_diameter_m=visible_inner_diameter(radius,float(np.linalg.norm(origin-center))),
        actual_inner_returns=len(observed),actual_inner_diameter_m=extent,
        current_component_extent_m=[p['extent_m'] for p in parts],
        length_necessary_condition=extent>=cfg.boundary_min_ahead_m,
        output=result,driving_authorized=False,closed_loop=False)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    sources=[f'src/{pkg}/{pkg}/{mod}.py' for pkg,mods in SNAPSHOT_MODULES.items() for mod in mods]
    sources+=['scripts/probe_corner_observability.py','scripts/inspect_corner_geometry.py',
              'scripts/profile_follow_trace.py','tests/test_corner_observability.py']
    hashes={p:digest(ROOT/p) for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sources:z.write(ROOT/p,p)
    rows=[evaluate(r,s,n) for r in (.10,.15,.20,.30,.40,.60,.80)
          for s in (1,-1) for n in ('none','uniform_seed0')]
    report=dict(scope='static_annular_geometry_not_official_track_or_hardware',rows=rows,
        source_sha256=hashes,source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),
        driving_authorized=False,controller_default_changed=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(radius=r['inner_radius_m'],side=r['side'],noise=r['noise'],
        ideal=r['ideal_visible_diameter_m'],observed=r['actual_inner_diameter_m'],
        speed=r['output'][0],reason=r['output'][2]['reason']) for r in rows]))


if __name__=='__main__':main()
