"""r548 두 비대칭 벽 대표. 현재 공통 소스, 측정 count 조향 상한. 장치/ROS 없음."""
from pathlib import Path
from dataclasses import replace
import copy,hashlib,importlib.util,json,math,sys,time,os
from collections import Counter
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
if sys.platform=='win32':
    import ctypes
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(),0x4000)
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.dont_write_bytecode=True
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(A.parent/'field_wall_irregularity_r518_v1/test_deps')]
old_cross=np.cross
def compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return old_cross(a,b,*args,**kwargs)
np.cross=compat
spec=importlib.util.spec_from_file_location('r521_test_current',ROOT/'tests/test_current_wall_reacquire.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
t.CFG=replace(t.CFG,max_steering_rad=.28475436568260193)
from arena_autonomy.follow_trace import freeze,comparable_decision,equal_output
from arena_autonomy import current_wall_reacquire as reacquire
from arena_autonomy.race_runtime_policy import terminal_plan,released_plan
generator=(A/'sensor_generator.py').read_text(encoding='utf8')
namespace=dict(t.__dict__);exec(compile(generator,str(A/'sensor_generator.py'),'exec'),namespace);sensor=namespace['sensor']
def summarize(out):
    if out is None:return dict(speed_mps=0.,candidate_available=False)
    d=out[2];recovery=d.get('current_wall_reacquire') or {}
    if recovery.get('reason'):recovery=(recovery.get('current_wall_reacquire') or recovery)
    return dict(speed_mps=out[0],steering_rad=out[1],reason=d.get('reason'),
        underlying=d.get('underlying_reason'),original_geometry_reason=d.get('reacquire_original_reason'),
        recovery=recovery,swept_clearance_m=d.get('swept_clearance_m'),
        boundary_reason=d.get('boundary_candidate_reason'),boundary_ahead_m=d.get('boundary_candidate_ahead_m'),
        moving_entry_reason=(d.get('moving_pass') or {}).get('entry',{}).get('reason'),
        deferred=d.get('current_follow_handoff_deferred'))

def baseline_local_follow(scene,legacy=False):
    owner=t.LocalFollow(t.CFG)
    original=reacquire.current_wall_candidate
    if legacy:reacquire.current_wall_candidate=lambda *a,**kw:None
    try:
        for now in (.4,.5,.6):
            old,_=t.sensor(now)
            t.baseline_command(owner,old)
        return t.baseline_command(owner,scene)
    finally:reacquire.current_wall_candidate=original

def moving(scene,raw,legacy=False):
    owner=t.active_owner_with_old_wall()
    original=reacquire.connect_moving_pass
    if legacy:reacquire.connect_moving_pass=lambda adapter,baseline,*a,**kw:baseline
    try:return owner.command(scene,t.history(),0.,'left',0.,.1,raw_scan=raw)
    finally:reacquire.connect_moving_pass=original

def helper_trace(scene):
    calls={};originals={}
    for name in ('local_path','pursue_path','swept_limit','braking_speed'):
        original=getattr(reacquire,name);originals[name]=original
        def wrapper(*args,_name=name,_original=original,**kwargs):
            result=_original(*args,**kwargs)
            if _name=='local_path':
                path,info=result
                value=dict(path_available=path is not None,boundary=info)
            elif _name=='swept_limit':value=dict(angle=args[1],clearance=result)
            else:value=result
            calls.setdefault(_name,[]).append(freeze(value));return result
        setattr(reacquire,name,wrapper)
    try:out=reacquire.current_wall_candidate(scene,0.,'left',0.,.1,t.CFG)
    finally:
        for name,original in originals.items():setattr(reacquire,name,original)
    return out,calls

cases=[dict(name='asym60_gap50_left_in100',amplitude=.06,gap=.05,shift=0.),
       dict(name='asym60_gap50_left_in100_actual_obstacle',amplitude=.06,gap=.05,shift=0.,obstacle=True)]
results=[];terminal_plan.install()
try:
    with released_plan.enabled():
        for case in cases:
            started=time.perf_counter()
            kw={k:v for k,v in case.items() if k!='name'}
            scene,raw=sensor(.7,closed=True,actor=1.2,**kw)
            preserved_scene=copy.deepcopy(scene);preserved_ranges=copy.deepcopy(raw.ranges)
            local_scene,local_raw=sensor(1.1,closed=False,**kw)
            local_before=baseline_local_follow(local_scene,True);local_after=baseline_local_follow(local_scene)
            before=moving(scene,raw,True);after=moving(scene,raw)
            helper,trace=helper_trace(scene)
            assert scene==preserved_scene and raw.ranges==preserved_ranges
            assert len(raw.ranges)==500 and len(scene.rays)==500
            assert [r.source_index for r in scene.rays]==list(range(500))
            assert [r.status for r in scene.rays]==[r.status for r in preserved_scene.rays]
            row=dict(**case,all_geometry_assumed_not_measured=True,wavelength_m=1.,joint_centers_x_m=[.4,1.],
                local_before=summarize(local_before),local_after=summarize(local_after),
                moving_before=summarize(before),moving_after=summarize(after),helper=summarize(helper),helper_trace=trace,
                local_positive_to_zero=local_before[0]>0 and local_after[0]==0,
                moving_positive_to_zero=before[0]>0 and after[0]==0,
                positive_moving_exact_match=before[0]<=0 or equal_output(comparable_decision(before),comparable_decision(after)),
                statuses=dict(Counter(r.status for r in scene.rays)),
                unknown_source_indices=[r.source_index for r in scene.rays if r.status=='unknown'],
                raw_scene_unchanged=True,original_ray_metadata_preserved=True)
            row.update(left_inward_m=.10,left_phase_rad=0.,right_phase_rad=math.pi/2,elapsed_windows_s=time.perf_counter()-started)
            results.append(row)
            (A/(case['name']+'_outputs.json')).write_text(json.dumps(freeze(dict(local_before=local_before,local_after=local_after,moving_before=before,moving_after=after,helper=helper)),indent=2),encoding='utf-8')
            (A/(case['name']+'_ranges.json')).write_text(json.dumps(freeze(dict(ranges=raw.ranges,angle_min=raw.angle_min,angle_increment=raw.angle_increment,time_increment=raw.time_increment,source_first_s=scene.source_first_s,source_last_s=scene.source_last_s))),encoding='utf-8')
            print(json.dumps(dict(case=case['name'],local=[local_before[0],local_after[0]],moving=[before[0],after[0]],moving_reason=after[2].get('reason'),helper=0 if helper is None else helper[0],statuses=row['statuses'])),flush=True)
finally:terminal_plan.uninstall()

source_paths={ROOT/'tests/test_current_wall_reacquire.py',A.parent/'field_large_walls_r525_v1/sensor_generator.py',A.parent/'field_large_walls_r525_v1/probe.py',ROOT/'artifacts/validation/2026-11-14/measured_steering_apply_r541_v1/selected_v2/derived_limits.json'}
for module in list(sys.modules.values()):
    name=getattr(module,'__file__',None)
    if name:
        path=Path(name).resolve()
        if path.is_file() and path.suffix=='.py' and path.is_relative_to(ROOT/'src'):
            source_paths.add(path)
manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source_paths)}
(A/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
report=dict(scope='two assumed asymmetric wall shapes; original r525 local and active-pass fixture protocols, not closed-loop',
    numpy_version=np.__version__,compatibility='process-local 2D determinant; Python numeric default; no installed dependency changes',
    command_limit_rad=t.CFG.max_steering_rad,full_follow_config=vars(t.CFG),cases=results,WSL=False,Jetson=False,Gazebo=False,
    obstacle_preserved_zero=all(results[-1][key]['speed_mps']==0 for key in ('local_after','moving_after','helper')),
    common_file_changes=False,success_search_repeated=False,
    source_hashes=manifest)
(A/'report.json').write_text(json.dumps(freeze(report),indent=2)+'\n',encoding='utf8')
