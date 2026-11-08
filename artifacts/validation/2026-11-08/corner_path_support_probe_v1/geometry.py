"""기록된 경로 기억과 현재 반환의 형상 비교. 출력 권한을 바꾸지 않는다."""
from pathlib import Path
import hashlib, json, math, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from arena_autonomy.local_path import local_path, _path_ahead, _origin_error
from arena_autonomy.local_follow import path_agrees, path_agrees_arc
from arena_autonomy.fragmented_wall_hypothesis import propose

HERE=Path(__file__).resolve().parent
data=json.loads((HERE/'snapshots.json').read_bytes())
rows=[];plot_rows=[]
for capture in data['captures']:
    cfg=capture['config'];pts=np.asarray(capture['points']);old=capture['carried']
    old=None if old is None else np.asarray(old)
    result=dict(t=capture['t'],old_age=None if capture['old_path_source'] is None else capture['t']-capture['old_path_source'],
                old_wall=capture['old_wall'],baseline=capture['baseline'][:2],walls={})
    paths={}
    for side in ('left','right'):
        path,info=local_path(pts,side,cfg['wall_offset_m'],horizon=1.8,piecewise_corner=True,
            minimum_radius=cfg['wheelbase_m']/math.tan(cfg['max_steering_rad']))
        candidate,evidence=propose(pts,side,cfg['wall_offset_m'],cfg['wheelbase_m']/math.tan(cfg['max_steering_rad']))
        if candidate is not None and (path is None or path_agrees(candidate,path,cfg['boundary_agreement_m'])):
            path=candidate
            info=dict(reason='fragment_candidate',boundary_observed_ahead_m=_path_ahead(path),path_anchor_error_m=_origin_error(path))
        paths[side]=path
        result['walls'][side]={k:v for k,v in info.items() if k not in ('path_points',)}
        result['walls'][side].update(agrees_x=bool(path is not None and old is not None and path_agrees(path,old,.08)),
            agrees_arc=bool(path is not None and old is not None and path_agrees_arc(path,old,.08)))
    rows.append(result)
    window=(34.51,35.23) if '--failure' in sys.argv else (33.09,35.33)
    spacing=.099 if '--failure' in sys.argv else .199
    if window[0]<capture['t']<window[1] and len(plot_rows)<8:
        if not plot_rows or capture['t']-plot_rows[-1][0]['t']>=spacing:
            plot_rows.append((capture,paths))

fig,axes=plt.subplots(2,4,figsize=(17,8))
for ax,(cap,paths) in zip(axes.flat,plot_rows):
    pts=np.asarray(cap['points']);old=None if cap['carried'] is None else np.asarray(cap['carried'])
    ax.scatter(pts[:,0],pts[:,1],s=3,c='#555555',label='measured endpoints')
    if old is not None:ax.plot(old[:,0],old[:,1],c='black',lw=2,label='transported old path')
    for side,color in [('left','#cc3366'),('right','#0066bb')]:
        if paths[side] is not None:ax.plot(*paths[side].T,c=color,label=side+' current path')
    age='none' if cap['old_path_source'] is None else f"{cap['t']-cap['old_path_source']:.3f}s"
    ax.scatter([0],[0],c='green',marker='x');ax.set(xlim=(-.4,1.8),ylim=(-.9,1.3),aspect='equal',title=f"t={cap['t']:.2f}, old age={age}")
    ax.grid(alpha=.25)
axes.flat[0].legend(fontsize=7)
fig.tight_layout();fig.savefig(HERE/('corner_failure_comparison.png' if '--failure' in sys.argv else 'corner_path_comparison.png'),dpi=150);plt.close(fig)
out=dict(scope='current geometry on exact r30 captured states; no drive decision or simulation',rows=rows)
(HERE/'geometry_result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps([r for r in rows if r['old_age'] is not None and r['old_age']<.31 and not r['walls']['right']['agrees_x']],ensure_ascii=False),flush=True)
