"""r182 노면 이탈 직전 경로/반환의 사후 지도 투영. 평가 전용, 제어 변경 없음."""
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from audit_local_pursuit_road import road_polygon

A=Path(__file__).resolve().parent
RUN=A.parent/'guard_approach_runtime_r182_v1/two_actor_run1/run1'


def main():
    trajectory=[json.loads(x) for x in (RUN/'trajectory.jsonl').read_text().splitlines()]
    with (RUN/'follow_decisions.jsonl').open() as stream:
        header=json.loads(next(stream))['data']
        events=[e for e in map(json.loads,stream) if e['kind']=='call']
    motion=MotionConfig(**header['motion_config'])
    road=road_polygon(RUN/'runtime_world.sdf')
    fig,axes=plt.subplots(1,3,figsize=(15,6),constrained_layout=True)
    report=[]
    for ax,seq in zip(axes,[784,790,796]):
        event=next(e for e in events if e['seq']==seq)
        d=event['data']; inp=d['input'];t=inp['reference_s'];normal=d['output'][2]['normal']
        _,scan,history=restore_call(inp,motion)
        scene=motion_scene(scan,history,t,timing_verified=inp['timing_verified'],epoch=inp['epoch'],
                           rear_half_angle_deg=inp['rear_half_angle_deg'])
        ego=min(trajectory,key=lambda x:abs(x['t']-t))
        c,s=math.cos(ego['yaw']),math.sin(ego['yaw']);rot=np.array([[c,-s],[s,c]])
        def world(points):return np.asarray(points)@rot.T+np.array([ego['x'],ego['y']])
        points=world([r.hit_xy_m for r in scene.rays if r.status=='return'])
        path=world(normal['path_points'])
        for poly in getattr(road,'geoms',[road]):
            x,y=poly.exterior.xy;ax.fill(x,y,color='.86',zorder=0)
            for interior in poly.interiors:
                x,y=interior.xy;ax.fill(x,y,color='white',zorder=0)
        ax.scatter(points[:,0],points[:,1],s=8,c='#336699',label='LiDAR returns')
        ax.plot(path[:,0],path[:,1],'-',c='#d43b2a',lw=2,label='selected path')
        recent=[x for x in trajectory if 59<=x['t']<=t]
        ax.plot([x['x'] for x in recent],[x['y'] for x in recent],c='black',lw=2,label='ego truth (evaluation)')
        ax.scatter([ego['x']],[ego['y']],c='black',s=40)
        ax.set(xlim=(1.7,4.4),ylim=(.5,4.4),xlabel='world x [m]',ylabel='world y [m]',
               title=f"seq {seq}, t={t:.2f}s; {normal['path_wall']} wall")
        ax.set_aspect('equal');ax.grid(alpha=.2)
        report.append(dict(seq=seq,reference_s=t,ego_truth_age_s=abs(ego['t']-t),
            speed=d['output'][0],steering=d['output'][1],normal={k:v for k,v in normal.items()
            if k not in ('path_points','parallel_hold_trial','selector_sides','selector_guard_candidate')},
            path_world_xy_m=path.tolist()))
    axes[0].legend(fontsize=8,loc='upper right')
    fig.savefig(A/'departure_paths.png',dpi=150)
    with (A/'selected_frames.json').open('x',encoding='utf8') as out:
        json.dump(report,out,indent=2);out.write('\n')
    print('saved three evaluation-only frames')


if __name__=='__main__':main()
