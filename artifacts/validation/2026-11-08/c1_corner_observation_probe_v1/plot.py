"""같은 저장 스캔의 경로 표현 비교. 실제 주행 궤적이나 자유 공간 그림이 아니다."""
from pathlib import Path
import importlib.util,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
from arena_autonomy.local_path import local_path
import arena_autonomy.piecewise_wall as pw

A=Path(__file__).resolve().parent
d=json.loads((A/'snapshots.json').read_bytes());cfg=FollowConfig(**d['follow_config'])
i,scan,history=restore_call(d['calls'][-1]['input'],MotionConfig(**d['motion_config']))
scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
points=np.asarray(scene.occupied_endpoints_xy_m)
paths={};details={}
for label,name in (('Original RDP + cubic fallback','before.py'),('Two-line candidate (r35)','piecewise_wall.py')):
    p=A.parent/'c1_piecewise_noise_v1'/name
    spec=importlib.util.spec_from_file_location('plot_variant',p);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    pw.corner_path=mod.corner_path
    paths[label],details[label]=local_path(points,'right',cfg.wall_offset_m,horizon=1.8,piecewise_corner=True,minimum_radius=cfg.wheelbase_m/np.tan(cfg.max_steering_rad))
fig,ax=plt.subplots(figsize=(9,6),constrained_layout=True)
ax.scatter(points[:,0],points[:,1],s=10,color='#626a78',label='Same raw C1 occupied returns',zorder=3)
for (label,path),color in zip(paths.items(),('#d1495b','#157a6e')):
    if path is not None:ax.plot(path[:,0],path[:,1],color=color,lw=2.8,label=label)
ax.add_patch(Rectangle((-.10,-.075),.20,.15,facecolor='#2e4057',alpha=.7))
ax.annotate('Ego / forward',xy=(.22,0),xytext=(-.25,-.35),arrowprops=dict(arrowstyle='->',color='#2e4057'))
ax.set(xlim=(-.4,1.8),ylim=(-.75,1.1),xlabel='Forward x [m]',ylabel='Left y [m]',
       title=f'Same delayed noisy scan at t={scene.t_ref_s:.2f} s\nPath proposal only; occupancy checks still use all raw returns')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=9)
fig.savefig(A/'same_scan_corner_path.png',dpi=170)
(A/'plot_paths.json').write_text(json.dumps({k:dict(info=details[k],path=paths[k].tolist() if paths[k] is not None else None) for k in paths},indent=2),encoding='utf8')
print(A/'same_scan_corner_path.png')
