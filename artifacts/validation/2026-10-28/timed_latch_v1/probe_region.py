"""정체 전후 이동 영역의 미관측을 센서 광선별로 분류한다. 주행/출력 없음."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import pass_release_trial as sel
from arena_autonomy.observation_support import depth_support
from arena_autonomy.scan_topology import adjacent_indices,cyclic_count


def classify(scene,q,pad):
    rays=scene.rays
    origins=np.array([r.ray_origin_xy_m for r in rays]);dirs=np.array([r.direction_xy for r in rays])
    ids=np.array([r.source_index for r in rays]);valid=np.array([r.status=='return' for r in rays])
    depth=np.array([np.linalg.norm(np.array(r.hit_xy_m)-r.ray_origin_xy_m) if r.status=='return' else -1. for r in rays])
    delta=q[:,None,:]-origins[None,:,:];radius=np.linalg.norm(delta,axis=2)
    angle=np.arctan2(dirs[None,:,0]*delta[:,:,1]-dirs[None,:,1]*delta[:,:,0],np.sum(dirs[None,:,:]*delta,axis=2))
    lo=np.argmin(np.where(angle>=-1e-10,np.maximum(angle,0),np.inf),axis=1)
    hi=np.argmin(np.where(angle<=1e-10,np.maximum(-angle,0),np.inf),axis=1);row=np.arange(len(q))
    reasons=np.full(len(q),'free',dtype=object)
    conditions=[('origin_overlap',np.any(radius<=1e-9,axis=1)),
        ('angular_bracket',(angle[row,lo]<-1e-10)|(angle[row,hi]>1e-10)|(abs(angle[row,lo])+abs(angle[row,hi])>.025)),
        ('nonadjacent',~adjacent_indices(ids[lo],ids[hi],cycle_count=cyclic_count(scene,None))),
        ('nonreturn',~valid[lo]|~valid[hi]),
        ('return_before_query',(depth[lo]<=radius[row,lo])|(depth[hi]<=radius[row,hi])),
        ('return_padding',(depth[lo]-radius[row,lo]-pad[lo]<=0)|(depth[hi]-radius[row,hi]-pad[hi]<=0))]
    for reason,mask in conditions:reasons[(reasons=='free')&mask]=reason
    assert np.array_equal(reasons=='free',depth_support(scene,q,pad))
    masked=sel.mx.masked_only(scene,q,pad);reasons[(reasons!='free')&masked]='body_mask'
    details=[]
    for reason in sorted(set(reasons)-{'free','body_mask'}):
        ix=np.flatnonzero(reasons==reason)
        details.append(dict(reason=reason,count=len(ix),ray_ids=sorted(set(ids[lo[ix]].tolist()+ids[hi[ix]].tolist())),
            samples=[dict(xy=q[i].tolist(),pair=[int(ids[lo[i]]),int(ids[hi[i]])],
                status=[rays[lo[i]].status,rays[hi[i]].status],depth=[float(depth[lo[i]]),float(depth[hi[i]])],
                query_radius=[float(radius[i,lo[i]]),float(radius[i,hi[i]])],
                padding=[float(pad[lo[i]]),float(pad[hi[i]])]) for i in ix[:3]]))
    return reasons,details


def main():
    if (BASE/'region.json').exists():raise FileExistsError('preserve diagnostic')
    trace=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    sampled=ROOT/'artifacts/validation/2026-10-27/obstacle_latch_v1/pass_continuity_v1/samples.json'
    samples=json.loads(sampled.read_text());chosen=[min(samples,key=lambda q:abs(q['t_s']-t)) for t in (129.3,132.)]
    wanted={q['t_s']:q for q in chosen};inputs={}
    with trace.open() as stream:
        header=json.loads(next(stream))['data']
        for line in stream:
            r=json.loads(line)
            if r['kind']!='call':continue
            d=r['data']['input']
            if d['reference_s'] in wanted:inputs[d['reference_s']]=d
            if d['reference_s']>132.:break
    motion=MotionConfig(**header['motion_config']);cfg=FollowConfig(**header['follow_config']);lat=sel.lateral_config(cfg)
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained');results=[]
    for ax,(stamp,sample) in zip(axes,wanted.items()):
        d,scan,history=restore_call(inputs[stamp],motion)
        scene=motion_scene(scan,history,stamp,timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        detail=sample['detail'];guard=detail.get('selector_guard_candidate')
        target=guard['target_speed_mps'] if guard else sample['command'][0]
        steer=guard['steering_rad'] if guard else sample['command'][1]
        expected=guard['region'] if guard else detail['selector_region']
        args=(scene,d['speed_mps'],target,steer,d['previous_steering_rad'],d['dt_s'],cfg,lat)
        region=sel.motion_region(*args);assert region==expected,(stamp,region,expected)
        _,q=sel.motion_queries(*args);pad=sel.mx.ray_padding(scene,lat,.5)
        reasons,details=classify(scene,q,pad);counts=dict(Counter(reasons))
        assert sum(n for k,n in counts.items() if k not in ('free','body_mask'))==region['unobserved']
        results.append(dict(t_s=stamp,region=region,counts=counts,details=details))
        pts=np.array(scene.occupied_endpoints_xy_m);ax.scatter(*pts.T,s=4,c='#555555',label='Returns')
        palette={'free':'#56b4e9','body_mask':'#999999','nonreturn':'#d55e00','nonadjacent':'#cc79a7','return_padding':'#e69f00'}
        for reason in sorted(counts):
            sub=q[reasons==reason];ax.scatter(*sub.T,s=6,c=palette.get(reason,'#009e73'),label=f'{reason}: {len(sub)}')
        body=np.array([[-.1,-.075],[.1,-.075],[.1,.075],[-.1,.075],[-.1,-.075]]);ax.plot(*body.T,c='black')
        ax.set(xlim=(-.2,.4),ylim=(-.23,.3),aspect='equal',title=f't={stamp:.2f}s',xlabel='Forward (m)',ylabel='Left (m)')
        ax.grid(alpha=.2);ax.legend(fontsize=7)
    sources=[Path(__file__),trace,sampled,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    result=dict(scope='saved-input geometry diagnosis; no controller changes, ROS or driving',frames=results,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (BASE/'region.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');fig.savefig(BASE/'region.png',dpi=160)
    print(json.dumps([dict(t_s=r['t_s'],counts=r['counts']) for r in results]))


if __name__=='__main__':main()
