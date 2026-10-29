"""자차 접근 거리별 G1/G2 센서 판단. 고정 장면이며 자동 전이/주행은 아니다."""
import hashlib
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
import numpy as np
from arena_autonomy.bypass_entry_follow import BypassEntryFollow
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import freeze
from probe_bypass_entry_sequence import render_scan


def inspect(target_y, distance):
    # 정답은 센서 생산자의 위치에만 사용한다. 제어기에는 거리값을 따로 넘기지 않는다.
    x=1.2-distance
    physical=[dict(t=t,rear_pose=[x-.0675,0.,0.],speed=0.,steering=0.) for t in np.arange(0.,1.01,.02)]
    cfg=FollowConfig(partial_observation_policy='bounded_local_v4',parallel_hold_trial=True,
        pass_release_trial=True,guard_curve_trial=True,traffic_recovery_trial=True,
        traffic_path_recovery_trial=True,corridor_fallback_enabled=True,
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')
    normal=LocalFollow(cfg)
    bypass=BypassEntryFollow(allow_simulation_trial=True)
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for t in np.arange(0.,1.01,.01):
        history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
    rows=[]
    for now in (.5,.6,.7,.8):
        scan=render_scan(physical,now,target_y)
        scene=motion_scene(scan,history,now,timing_verified=True,rear_half_angle_deg=30.)
        a=normal.command(scene,history,0.,'left',0.,.02,raw_scan=scan)
        b=bypass.command(scene,history,0.,'left',0.,.02,raw_scan=scan)
        rows.append(dict(time_s=now,g1=dict(speed=a[0],steering=a[1],detail=a[2]),
                         g2=dict(speed=b[0],steering=b[1],detail=b[2])))
    return dict(target_y=target_y,target_center_distance_m=distance,rows=rows)


if __name__=='__main__':
    out=BASE/'approach.json'
    if out.exists():raise RuntimeError('preserve prior result')
    cases=[inspect(y,d) for y in (-.15,.15,0.) for d in (1.2,1.,.8,.6,.4)]
    sources=[Path(__file__)]+list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    result=dict(scope='static sensor-only comparison; not moving approach or handoff permission',cases=cases,
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    out.write_text(json.dumps(freeze(result),indent=2,allow_nan=False)+'\n')
    for case in cases:
        r=case['rows'][-1]
        print(case['target_y'],case['target_center_distance_m'],
              'G1',r['g1']['speed'],r['g1']['detail'].get('reason'),r['g1']['detail'].get('selector_choice'),
              'G2',r['g2']['speed'],r['g2']['detail'].get('reason'),r['g2']['detail'].get('selected'))
