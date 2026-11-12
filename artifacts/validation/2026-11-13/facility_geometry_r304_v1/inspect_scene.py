"""실패 원 입력·시설 반환을 평가 전용으로 대조. 제거 비교는 제어기에 사용하지 않는다."""
from pathlib import Path
import json,math,sys
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(R/'scripts'),str(R/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.fragmented_wall_hypothesis import propose
from shapely.geometry import Point
from shapely.affinity import rotate,translate
import prepare_fixture as fixture
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

with (A/'run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data']
    calls=[json.loads(line)['data'] for line in f if json.loads(line)['kind']=='call']
cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
trace=json.loads((A/'run1/trace.json').read_bytes())
_,_,tri,_=fixture.analysis.audit.load(R/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/support_frame_m.stl')
section=fixture.section(tri,.1346)
places=json.loads((R/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/fixture.json').read_bytes())['locations']
world=[translate(rotate(section,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y']) for p in places]
chosen=[min(calls,key=lambda c:abs(c['input']['reference_s']-t)) for t in (9.,10.4,10.8,17.6)]
fig,axes=plt.subplots(2,2,figsize=(12,10));results=[]
for call,ax in zip(chosen,axes.flat):
    d,scan,history=restore_call(call['input'],motion)
    scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    points=np.asarray(scene.occupied_endpoints_xy_m);rays=np.asarray([r.source_index for r in scene.rays if r.status=='return'])
    pose=min(trace,key=lambda p:abs(p['time']-scene.t_ref_s))
    yaw=pose['yaw'];c,s=math.cos(yaw),math.sin(yaw)
    world_points=points@np.array([[c,s],[-s,c]])+np.array([pose['x'],pose['y']])
    # 평가 전용 정답 분류이며 실행 입력으로 전달하지 않는다.
    labels=[next((places[i]['name'] for i,shape in enumerate(world) if shape.distance(Point(p))<.012),None) for p in world_points]
    is_support=np.array([x is not None for x in labels])
    age=scene.t_ref_s-scene.source_first_s;v=cfg.open_route_speed_mps
    horizon=max(1.8,.06+cfg.standstill_gap_m+v*(age+.10)+v**2/4);radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad)
    out={}
    for label,mask in [('original',np.ones(len(points),dtype=bool)),('diagnostic_without_fixture_hits',~is_support)]:
        out[label]={}
        for side,color in [('left','#ba2435'),('right','#188f42')]:
            cloud=points[mask]
            path,info=local_path(cloud,side,cfg.wall_offset_m,horizon=horizon,rays=rays[mask].tolist() if cfg.ray_wall_continuity else None,
                piecewise_corner=cfg.piecewise_wall_trial,minimum_radius=radius,narrow_occluder_bridge=cfg.narrow_occluder_bridge)
            hypothesis,evidence=propose(cloud,side,cfg.wall_offset_m,radius)
            out[label][side]=dict(info=info,fragmented=evidence,fragment_path=None if hypothesis is None else hypothesis.tolist())
            if path is not None:ax.plot(path[:,0],path[:,1],color=color,ls='-' if label=='original' else '--',label=side+' '+label)
    ax.scatter(points[:,0],points[:,1],s=8,c='#607792',label='Raw occupied returns')
    if is_support.any():ax.scatter(points[is_support,0],points[is_support,1],s=25,c='#b515db',label='Facility section match (evaluation only)')
    ax.add_patch(plt.Rectangle((-.10,-.075),.2,.15,color='black',alpha=.5));ax.arrow(0,0,.2,0,width=.008,color='black')
    ax.set(xlim=(-.5,2),ylim=(-1,1),xlabel='Forward x (m)',ylabel='Left y (m)',title=f'{scene.t_ref_s:.2f}s  speed {d["speed_mps"]:.2f}m/s  {call["output"][2]["reason"]}')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=6)
    results.append(dict(reference_s=scene.t_ref_s,speed_mps=d['speed_mps'],source_first_s=scene.source_first_s,observed=len(points),fixture_returns=int(is_support.sum()),
        recorded=call['output'],sides=out,pose_time_delta_s=pose['time']-scene.t_ref_s))
fig.tight_layout();fig.savefig(A/'sensor_path_diagnosis.png',dpi=150)
(A/'inspection.json').write_text(json.dumps(freeze(results),indent=2)+'\n')
print(json.dumps([dict(time=x['reference_s'],fixture=x['fixture_returns'],sides={label:{side:{k:v for k,v in result['info'].items() if k in ('reason','boundary_observed_ahead_m','path_anchor_error_m','wall_model','wall_distance_m')} for side,result in values.items()} for label,values in x['sides'].items()}) for x in results]))
