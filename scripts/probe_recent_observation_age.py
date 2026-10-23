#!/usr/bin/env python3
"""같은 측방 결측의 취득 나이별 깊이/횡방향 도달 가능성 분리. 합성 입력 전용."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import sweep_support, sweep_queries
from arena_autonomy.observation_lateral import protected_queries
from arena_autonomy.scene_motion import motion_scene
from audit_observation_history import lateral_support
from audit_corridor_runtime import digest
from test_local_follow import history, scan
from validate_pipeline_faults_ros import corridor_range

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    names=['scripts/probe_recent_observation_age.py']+['src/arena_autonomy/arena_autonomy/'+n+'.py'
        for n in ('observation_support','observation_lateral','linear_observation','scene_motion')]
    hashes={n:digest(ROOT/n) for n in names}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for n in names:z.write(ROOT/n,n)
    cfg=FollowConfig(wheelbase_m=.135);rows=[]
    for age in (.105,.115,.135,.155,.175,.195,.215):
        raw=scan(.5)
        raw.angle_min=float(np.float32(raw.angle_min));raw.angle_increment=float(np.float32(raw.angle_increment))
        raw.ranges=np.asarray([corridor_range(raw.angle_min+i*raw.angle_increment,.06) for i in range(500)],dtype=np.float32).tolist()
        raw.ranges[375]=math.inf
        scene=motion_scene(raw,history(),.5+age,timing_verified=True,rear_half_angle_deg=30.)
        for steer in (0.,-9.6137665e-9):
            out=sweep_support([scene],0.,.2,steer,steer,cfg,protected_sensor_pose=(.06,0.,0.))
            q,_=sweep_queries(scene,steer,out['required_distance_m'],cfg)
            # 본체가 보호 범위 밖이라고 표시한 후방 증거와 섞지 않는다.
            selected=q[protected_queries(q,(.06,0.,0.))]
            supported,reached,witness=lateral_support(scene,selected,cfg)
            rows.append(dict(age_s=age,steering_rad=steer,result=out,independent_witness=witness))
    report=dict(scope='same_scan_age_synthetic_not_physical_object_presence',rows=rows,source_sha256=hashes,
                source_unchanged=all(digest(ROOT/n)==h for n,h in hashes.items()))
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(age=r['age_s'],angle=r['steering_rad'],supported=r['result']['supported'],
        lateral=r['result'].get('lateral_rejected_queries'),witness=r['independent_witness'][:1]) for r in rows]))


if __name__=='__main__':main()
