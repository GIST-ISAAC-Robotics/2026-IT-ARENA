"""동일 시각 센서 반환과 두 앵커 기준의 보존점을 비교한다."""
import json
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
before=json.loads((BASE/'threat_reconstruction.json').read_text())
after=json.loads((BASE/'anchor_reference_comparison.json').read_text())
frames=json.loads((BASE/'target_frames.json').read_text())
stamp=131.82
one=next(x for x in before['observations'] if abs(x['t_s']-stamp)<1e-8)
two=next(x for x in after['observations'] if abs(x['t_s']-stamp)<1e-8)
row=next(x for x in frames if abs(x['data']['input']['reference_s']-stamp)<1e-8)
with (BASE.parent/'traffic_path_recovery_connection_v1/run1_course/follow_decisions.jsonl').open() as stream:
    cfg=MotionConfig(**json.loads(next(stream))['data']['motion_config'])
inp,scan,h=restore_call(row['data']['input'],cfg)
source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
p,ids,_=h.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,
    source,scan.time_increment,stamp,mode='both',timing_verified=True)
old=np.asarray(one['corrected_points']);new=np.asarray(two['corrected_points'])
fig,ax=plt.subplots(figsize=(8,5),layout='constrained')
ax.scatter(p[:,0],p[:,1],s=12,c='#555555',label='Current measured returns')
ax.scatter(old[:,0],old[:,1],s=72,marker='x',c='#bd2525',label='Old retained points + current-scan anchor')
ax.scatter(new[:,0],new[:,1],s=68,facecolors='none',edgecolors='#007b4d',linewidths=1.8,
    label='Retained points + same original-scan anchor')
ax.set_xlim(-.12,.08);ax.set_ylim(.39,.54);ax.set_aspect('equal');ax.grid(alpha=.25)
ax.set_xlabel('Vehicle forward x (m)');ax.set_ylabel('Vehicle left y (m)')
ax.set_title('131.82 s: all 6 retained points, unchanged observation threshold')
ax.legend(fontsize=8,loc='best')
out=BASE/'anchor_reference_comparison.png'
if out.exists():raise FileExistsError(out)
fig.savefig(out,dpi=160);print(out)
