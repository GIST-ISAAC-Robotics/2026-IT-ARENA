"""r308 저장 입력의 창/원호 절단 추적. 시설 분류는 평가 전용이다."""
from pathlib import Path
import hashlib, json, math, sys, inspect
import numpy as np

A=Path(__file__).resolve().parent; R=A.parents[3]
OLD=R/'artifacts/validation/2026-11-13/straight_fragment_drive_r308_v1'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(R/'scripts'),str(R/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1')]
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
import arena_autonomy.local_path as lp
from arena_autonomy.piecewise_wall import corner_path
from arena_autonomy.fragmented_wall_hypothesis import propose
from shapely.geometry import Point
from shapely.affinity import rotate,translate
import prepare_fixture as fixture

def traced(fn, points, side, cfg, horizon, rays):
    events=[]
    def tracer(frame,event,arg):
        if event=='return' and frame.f_code in (fn.__code__,corner_path.__code__):
            values=frame.f_locals
            keys=('anchor','anchor_s','first','last','roughness','start','before','after','lo','hi','t0','t1','angle','center')
            item={'function':frame.f_code.co_name,**{k:values[k] for k in keys if k in values}}
            for name in ('wall','samples','arc'):
                if name in values:
                    arr=np.asarray(values[name]);item[name]={'count':len(arr),'first':arr[0],'last':arr[-1]}
            if 'wall' in values and 'anchor' in values and arg[0] is not None:
                item['anchor_xy']=values['wall'][values['anchor']]
            events.append(freeze(item))
        return tracer
    try:
        sys.settrace(tracer)
        result=fn(points,side,cfg.wall_offset_m,horizon=horizon,rays=rays.tolist(),piecewise_corner=True,
                  minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),narrow_occluder_bridge=True)
    finally:sys.settrace(None)
    path,info=result
    return dict(info={k:v for k,v in info.items() if k!='path_points'},events=events,
                path=None if path is None else path.tolist())

def main():
    with (OLD/'run1/follow_decisions.jsonl').open() as f:
        header=json.loads(next(f))['data']
        calls=[j['data'] for line in f if (j:=json.loads(line))['kind']=='call']
    cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
    trace=json.loads((OLD/'run1/trace.json').read_bytes())
    _,_,tri,_=fixture.analysis.audit.load(R/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/support_frame_m.stl')
    section=fixture.section(tri,.1346)
    places=json.loads((R/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/fixture.json').read_bytes())['locations']
    world=[translate(rotate(section,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y']) for p in places]
    variants={'baseline':lp.local_path}
    for back in (.5,1.):
        source=inspect.getsource(lp.local_path).replace('anchor_s-.25',f'anchor_s-{back}')
        namespace={};exec(compile(source,__file__,'exec'),lp.__dict__,namespace)
        variants[f'backward_window_{back}_diagnostic']=namespace['local_path']
    results=[]
    for target in (19.8,20.4,20.9,27.6):
        call=min(calls,key=lambda c:abs(c['input']['reference_s']-target))
        d,scan,history=restore_call(call['input'],motion)
        scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        points=np.asarray(scene.occupied_endpoints_xy_m);rays=np.asarray([r.source_index for r in scene.rays if r.status=='return'])
        pose=min(trace,key=lambda p:abs(p['time']-scene.t_ref_s));c,s=math.cos(pose['yaw']),math.sin(pose['yaw'])
        wp=points@np.array([[c,s],[-s,c]])+np.array([pose['x'],pose['y']])
        support=np.asarray([any(shape.distance(Point(p))<.012 for shape in world) for p in wp])
        age=scene.t_ref_s-scene.source_first_s;v=cfg.open_route_speed_mps
        horizon=max(1.8,.06+cfg.standstill_gap_m+v*(age+.10)+v**2/4)
        item={'time':scene.t_ref_s,'variants':{}}
        if target==27.6:
            np.savez(A/'stall_sensor.npz',points=points,rays=rays,facility_evaluation_only=support,
                     offset=cfg.wall_offset_m,radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),horizon=horizon)
            order=np.argsort(np.arctan2(points[:,1],points[:,0]),kind='stable');ordered=points[order]
            limits=.10+.035*np.minimum(np.linalg.norm(ordered[:-1],axis=1),np.linalg.norm(ordered[1:],axis=1))
            breaks=np.linalg.norm(np.diff(ordered,axis=0),axis=1)>limits
            removed,joins=lp.break_adjacent_occluders(ordered,breaks)
            comp=np.split(order,np.flatnonzero(breaks)+1)
            item['components']=[dict(n=len(ids),first=points[ids[0]].tolist(),last=points[ids[-1]].tolist(),
                near=int(np.count_nonzero((np.abs(points[ids,0])<=.35)&(points[ids,1]<-.1)&(np.abs(points[ids,1]+cfg.wall_offset_m)<=.12))),
                ids=ids.tolist()) for ids in comp]
            item['bridge']=dict(joins=joins,removed=int(removed.sum()))
        for name,fn in variants.items():
            item['variants'][name]=traced(fn,points,'right',cfg,horizon,rays)
        item['variants']['diagnostic_remove_facility']=traced(lp.local_path,points[~support],'right',cfg,horizon,rays[~support])
        results.append(item)
    (A/'window_trace.json').write_text(json.dumps(freeze(results),indent=2)+'\n')
    print(json.dumps({k:v for k,v in results[-1].items() if k!='variants'}))

if __name__=='__main__':main()
