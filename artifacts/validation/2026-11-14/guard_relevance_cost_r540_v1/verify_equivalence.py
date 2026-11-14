"""동일 입력 전체 helper 출력/전체 기억 상태의 정확 일치. Windows fixture만 실행한다."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
P=A.parent/'guard_memory_relevance_r538_v1'
FROZEN=P/'stage/src/arena_autonomy'
sys.dont_write_bytecode=True
sys.path[:0]=[str(FROZEN),str(ROOT/'src/arena_vehicle_interface'),
              str(A.parent/'field_wall_irregularity_r518_v1/test_deps')]
original_cross=np.cross
def compat_cross(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:
        return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return original_cross(a,b,*args,**kwargs)
np.cross=compat_cross
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scan_input import project_scan


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def canonical(value):
    if isinstance(value,np.ndarray):
        return dict(array_bytes=value.tobytes().hex(),dtype=str(value.dtype),shape=value.shape,
                    writeable=bool(value.flags.writeable))
    if isinstance(value,dict):return {str(k):canonical(v) for k,v in value.items()}
    if isinstance(value,set):return dict(set_values=sorted([canonical(v) for v in value],key=repr))
    if isinstance(value,(tuple,list)):return [canonical(v) for v in value]
    if isinstance(value,np.generic):return value.item()
    return value


def history():
    h=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=5.))
    for t in np.arange(0.,.7001,.005):
        h.add_mean_wheel(float(t),0.);h.add_gyro(float(t),0.)
    return h


def scan(points):
    raw=NS(header=NS(frame_id='laser_frame',stamp=NS(sec=0,nanosec=500000000)),
        ranges=[2.]*500,angle_min=-math.pi,angle_increment=math.tau/500,time_increment=.0002,
        range_min=.05,range_max=12.)
    for x,y in points:
        index=round((math.atan2(y,x-.06)+math.pi)/math.tau*500)%500
        for delta in (-1,0,1):raw.ranges[(index+delta)%500]=float('nan')
    return raw


def policy(module,points,empty=False):
    p=NS(command_observation_priority=True,latched=True,latch_reason='previous_point_in_missing_sweep',
        latch_evidence={'scope':'test_original_points','points':copy.deepcopy(points)})
    p.guard_memory=module.RetainedCommandMemory(points,.4,.3,('laser_frame',0),range(len(points)))
    if empty:
        p.guard_memory.points=np.empty((0,2));p.guard_memory.ray_cleared_total=len(points)
    return p


def main():
    base_path=FROZEN/'arena_autonomy/guard_memory_command.py'
    stage_path=A/'stage/guard_memory_command.py'
    digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    assert digest(base_path)=='dc81494c5162d0e07851e835bee7dfc96fac303a857fe0c922da50959647b26d'
    base=load('r540_frozen_r538',base_path);stage=load('r540_equal_angle',stage_path)
    points113=[(.055+i*.00001,-.38) for i in range(113)]
    points3=[(-.168,-.372),(-.171,-.378),(-.174,-.385)]
    specs=[
        dict(name='retained113_equal',points=points113,steering=.3,previous=.3),
        dict(name='retained113_different',points=points113,steering=.3,previous=0.),
        dict(name='retained113_near_but_not_equal',points=points113,steering=.3,previous=.3+1e-14),
        dict(name='rear3_equal',points=points3,steering=.05688,previous=.05688),
        dict(name='resolved_empty_equal',points=points3,empty=True,steering=.03824,previous=.03824),
        dict(name='forward_occupied_equal',points=[(.35,0.)],steering=0.,previous=0.),
        dict(name='previous_only_occupied',points=[(.13,.13)],steering=0.,previous=.45),
        dict(name='explicit_zero',points=points3,steering=.1,previous=.1,requested=0.),
        dict(name='same_scan_next_changed_angle',points=[(.06,-.38)],steering=0.,previous=0.,next_angle=-.55),
    ]
    results=[]
    for spec in specs:
        h=history();raw=scan(spec['points']);frozen_ranges=np.array(raw.ranges).tobytes()
        p0=policy(base,spec['points'],spec.get('empty',False));p1=policy(stage,spec['points'],spec.get('empty',False))
        original_state=(canonical(h.wheels.times),canonical(h.wheels.values),canonical(h.gyro.times),canonical(h.gyro.values))
        calls=[0,0]
        def instrument(module,index):
            original=module.swept_limit
            def traced(*args,**kwargs):calls[index]+=1;return original(*args,**kwargs)
            module.swept_limit=traced
            return original
        old0,old1=instrument(base,0),instrument(stage,1)
        try:
            for step in range(2 if 'next_angle' in spec else 1):
                now=.6+step*.02
                angle=spec['steering'] if step==0 else spec['next_angle']
                previous=spec['previous'] if step==0 else angle
                points=project_scan(raw,h,now,timing_verified=True)[0]
                outputs=[];counts_before=calls.copy()
                for module,p in [(base,p0),(stage,p1)]:
                    outputs.append(module.limited_memory_command(p,
                        dict(allowed=False,reason='known_obstacle_latched',speed_cap_mps=None),
                        raw,points,h,now,spec.get('requested',1.),0.,angle,previous,.128,.207,.15))
                assert outputs[0]==outputs[1],(spec['name'],outputs)
                states=[canonical(vars(p.guard_memory)) for p in (p0,p1)]
                assert states[0]==states[1],spec['name']
                assert canonical(p0.latch_evidence)==canonical(p1.latch_evidence)
                assert p0.latched is p1.latched is True
                used=[calls[i]-counts_before[i] for i in range(2)]
                assert used[1]==(used[0]//2 if angle==previous else used[0]),(spec['name'],used)
                results.append(dict(case=spec['name'],step=step,base_swept_calls=used[0],stage_swept_calls=used[1],
                    full_output_exact=True,full_memory_state_exact=True,
                    output=outputs[0],memory_state_sha256=hashlib.sha256(
                        json.dumps(states[0],sort_keys=True,allow_nan=False).encode()).hexdigest(),
                    retained_points=len(p0.guard_memory.points)))
        finally:base.swept_limit,stage.swept_limit=old0,old1
        assert np.array(raw.ranges).tobytes()==frozen_ranges
        assert original_state==(canonical(h.wheels.times),canonical(h.wheels.values),canonical(h.gyro.times),canonical(h.gyro.values))
    assert 'rclpy' not in sys.modules
    out=dict(scope='Windows full helper fixtures, not saved670 callback replay or hardware timing',
        fixtures=len(specs),calls=len(results),passed=True,base_sha256=digest(base_path),stage_sha256=digest(stage_path),
        equality='entire returned dictionaries and every memory.__dict__ field; ndarray raw bytes/dtype/shape/writeability; latch evidence; input ranges and motion values',
        numpy_cross_compatibility='process-local 2D cross fallback only',cases=results)
    (A/'equivalence.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in out.items() if k!='cases'}))


if __name__=='__main__':main()
