"""stage 파일을 명시 로드한 Windows 순수 straight-wall 검사. 장치/주행 없음."""
from pathlib import Path
from dataclasses import replace
import importlib.util, json, math, os, sys
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
sys.dont_write_bytecode = True
import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.local_scene import scene_from_measurements
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.follow_trace import freeze
import arena_autonomy.bypass_entry_trial as baseline

STAGE = A/'stage/src/arena_autonomy/arena_autonomy/bypass_entry_trial.py'
spec = importlib.util.spec_from_file_location('arena_autonomy.r554_bypass_entry_stage', STAGE)
candidate = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = candidate
spec.loader.exec_module(candidate)
assert Path(candidate.__file__).resolve() == STAGE.resolve()


def fixture(origin):
    # 두 깊이의 직선 면 및 15×20cm 상대 상부 상자. label은 반환 보존 평가에만 쓴다.
    segments = [(-1., .1, .36, .508), (.4, 2.2, 3., 2.2),
                (.6, -.075, .8, -.075), (.8, -.075, .8, .075),
                (.8, .075, .6, .075), (.6, .075, .6, -.075)]
    angles = -math.pi+np.arange(500)*math.tau/500
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    best = np.full(500, np.inf)
    labels = np.full(500, -1)
    origin = np.asarray(origin)
    for index, (x0,y0,x1,y1) in enumerate(segments):
        edge = np.array([x1-x0, y1-y0])
        offset = np.array([x0,y0])-origin
        denominator = directions[:,0]*edge[1]-directions[:,1]*edge[0]
        parallel = abs(denominator)<1e-12
        safe = np.where(parallel, 1., denominator)
        distance = (offset[0]*edge[1]-offset[1]*edge[0])/safe
        along = (offset[0]*directions[:,1]-offset[1]*directions[:,0])/safe
        valid = (~parallel)&(distance>.05)&(along>=0)&(along<=1)&(distance<best)
        best = np.where(valid, distance, best)
        labels = np.where(valid, index, labels)
    scene = scene_from_measurements(best.tolist(), angles_rad=angles.tolist(),
        acquired_at_s=(.6+np.arange(500)*.0002).tolist(), t_ref_s=.7,
        range_min=.05, range_max=12., source_frame_id='laser_frame', source_epoch=0,
        sensor_xy_m=tuple(origin), rear_half_angle_deg=0., preserve_sampling=True)
    return scene, labels


def main():
    cfg = replace(FollowConfig(), wall_offset_m=.505, max_steering_rad=.28475436568260193,
                  wheelbase_m=.128, length_m=.207)
    rows = []
    for name, origin in [('zero_origin', (0.,0.)), ('sensor_front_66mm', (.066,.002))]:
        scene, labels = fixture(origin)
        rays_before = scene.rays
        occupied_before = scene.occupied_endpoints_xy_m
        old = baseline.observed_straight_wall(scene,cfg,'left')
        new = candidate.observed_straight_wall(scene,cfg,'left')
        assert scene.rays == rays_before and scene.occupied_endpoints_xy_m == occupied_before
        actor_sources = [ray.source_index for ray in scene.rays
                         if ray.status=='return' and labels[ray.source_index]>=2]
        assert actor_sources
        assert all(scene.rays[index].hit_xy_m is not None for index in actor_sources)
        selected = [ray for ray in scene.rays if ray.status=='return' and ray.hit_xy_m[1]>.10]
        points = np.asarray([ray.hit_xy_m for ray in selected])
        bearings = np.array([math.atan2(ray.direction_xy[1],ray.direction_xy[0]) for ray in selected])
        near = points[:,1]<1.
        base_changes = int(np.count_nonzero(np.diff(near[np.argsort(-np.arctan2(points[:,1],points[:,0]))])))
        ray_changes = int(np.count_nonzero(np.diff(near[np.argsort(-bearings)])))
        if name == 'zero_origin':
            assert old[0] is not None and new == old
        else:
            assert base_changes>1 and ray_changes==1
            assert old[0] is None and old[1]['wall']['reason']=='wall_segment_short'
            assert new[0] is not None
        rows.append(dict(case=name,origin=origin,baseline=old,candidate=new,
            endpoint_order_depth_changes=base_changes,ray_order_depth_changes=ray_changes,
            actor_return_count=len(actor_sources),unknown_count=sum(r.status=='unknown' for r in scene.rays),
            occupied_points=len(occupied_before),raw_occupancy_and_unknown_unchanged=True))
    output = dict(candidate_module=str(STAGE.relative_to(ROOT)), baseline_module=str(Path(baseline.__file__).relative_to(ROOT)),
        numpy_version=np.__version__,piecewise_executed=False,closed_loop_executed=False,rows=rows)
    (A/'stage_check_result.json').write_text(json.dumps(freeze(output),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(freeze(output),ensure_ascii=False,indent=2))

if __name__=='__main__': main()
