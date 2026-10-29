"""종료 뒤 정지 자세의 합성 관측 진단. 원래 시간 연속 입력 재생과 구별한다."""
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_entry_trial import straight_reference, compare_entries
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry_sequence import render_scan

BASE=Path(__file__).resolve().parent
rows=[]
for file in sorted((BASE/'sequence3').glob('y*.json')):
    saved=json.loads(file.read_text(encoding='utf-8'))
    last=saved['records'][-1]
    x,y,yaw=last['evaluation']['center_pose'];now=last['t']
    rear=(x-.0675*math.cos(yaw),y-.0675*math.sin(yaw),yaw)
    physical=[dict(t=0.,rear_pose=rear,speed=0.,steering=0.),
              dict(t=now,rear_pose=rear,speed=0.,steering=0.)]
    raw=render_scan(physical,now,saved['target_y'],saved['target_restarts'])
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for t in np.arange(now-.3,now+.0001,.01):
        history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
    scene=motion_scene(raw,history,now,timing_verified=True,rear_half_angle_deg=30.)
    reference,evidence=straight_reference(scene,FollowConfig())
    decision=compare_entries(scene,now,0.,last['safe'][1],.02)
    rows.append(dict(case=file.stem,scope='stationary_terminal_pose_sensor_counterfactual_not_original_replay',
        reference=reference,evidence=evidence,decision={key:decision[key] for key in ('reason','selected')},
        candidates=[{k:r.get(k) for k in ('offset_m','transition_m','reason','command_clearance_m')}
                    for r in decision['candidates']]))
out=BASE/'terminal_static_v1.json'
if out.exists():raise ValueError('preserve existing diagnostic')
out.write_text(json.dumps(rows,indent=2)+'\n',encoding='utf-8')
print(json.dumps(rows))
