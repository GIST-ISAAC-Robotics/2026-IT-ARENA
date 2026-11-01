"""직선 최고 목표 근처의 원시 관측 연결과 끝부분을 확인한다. 평가/시각화 전용."""
from hashlib import sha256
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import local_path


if __name__=='__main__':
    source=HERE.parent/'solo_corridor_arc_v1/run1/follow_decisions.jsonl'
    wanted={16.92,28.12,44.12};rows=[]
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            e=json.loads(line);d=e['data']
            if e['kind']=='header':motion=MotionConfig(**d['motion_config']);config=d['follow_config']
            if e['kind']!='call' or round(d['input']['reference_s'],2) not in wanted:continue
            inp,scan,history=restore_call(d['input'],motion)
            scene=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],
                               epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
            points=np.asarray(scene.occupied_endpoints_xy_m);v,a,out=d['output']
            capture={}
            def trace(frame,event,arg):
                if frame.f_code is local_path.__code__ and event=='return':
                    capture.update({k:frame.f_locals.get(k) for k in ('wall','arc','anchor','anchor_s','first','last','breaks','points','gaps','limits')})
                return trace
            sys.settrace(trace)
            try:path,info=local_path(points,inp['side'],config['wall_offset_m'],out['observed_search_horizon_m'])
            finally:sys.settrace(None)
            wall=capture['wall'];arc=capture['arc'];anchor_s=capture['anchor_s']
            gap_indices=capture['breaks'];ordered=capture['points']
            gaps=[dict(before=ordered[i].tolist(),after=ordered[i+1].tolist(),
                       distance=float(capture['gaps'][i]),allowed=float(capture['limits'][i])) for i in gap_indices]
            row=dict(t=inp['reference_s'],target_mps=v,horizon_m=out['observed_search_horizon_m'],
                     source_age_s=inp['reference_s']-out['source_first_s'],wall_ahead_m=info.get('boundary_observed_ahead_m'),
                     actual_segment_ahead_m=float(arc[-1]-anchor_s),path_ahead_m=out['observed_path_ahead_m'],
                     clearance_m=out['swept_clearance_m'],wall_end=wall[-1].tolist(),path_end=path[-1].tolist(),
                     gaps=gaps,source_first_s=scene.source_first_s)
            rows.append(row)
            fig,ax=plt.subplots(figsize=(7,6))
            ax.scatter(points[:,0],points[:,1],s=5,c='0.65',label='current returns')
            ax.plot(wall[:,0],wall[:,1],'b.',ms=3,label='chosen connected wall')
            ax.plot(path[:,0],path[:,1],'r-',lw=2,label='local path')
            ax.plot(wall[-1,0],wall[-1,1],'bo',ms=8);ax.plot(0,0,'ks',ms=8,label='body origin')
            ax.set(xlim=(-.5,5),ylim=(-2.5,2.5),xlabel='Forward x (m)',ylabel='Left y (m)',title=f't={inp["reference_s"]:.2f}, target={v:.3f} m/s')
            ax.set_aspect('equal');ax.grid();ax.legend();fig.tight_layout()
            image=HERE/f'peak_{int(round(inp["reference_s"]*100))}.png'
            if image.exists():raise FileExistsError(image)
            fig.savefig(image,dpi=130);plt.close(fig)
    if len(rows)!=len(wanted):raise ValueError('missing target calls')
    result=dict(scope='실제 반환과 local_path의 연결 구간/끝 비교. 지도나 정답은 사용하지 않음.',rows=rows,
                input_sha256=sha256(source.read_bytes()).hexdigest(),script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
    with (HERE/'peaks.json').open('x',encoding='utf-8') as stream:json.dump(freeze(result),stream,ensure_ascii=False,indent=2)
    print(json.dumps([{k:v for k,v in r.items() if k!='gaps'} for r in rows]))
