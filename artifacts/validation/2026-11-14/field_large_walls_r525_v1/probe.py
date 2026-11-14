"""r521 ray 교차 생성기를 확장한 큰 벽 변형 5개 대표. Windows 전용 저장 진단."""
from pathlib import Path
import copy,hashlib,importlib.util,inspect,json,math,shutil,sys
from collections import Counter
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[3];STAGE=A/'source_snapshot'
if not STAGE.exists():
    for name in ('arena_autonomy','arena_vehicle_interface'):
        shutil.copytree(ROOT/'src'/name,STAGE/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copy2(ROOT/'tests/test_current_wall_reacquire.py',STAGE/'test_current_wall_reacquire.py')
    hashes={str(p.relative_to(STAGE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in STAGE.rglob('*') if p.is_file()}
    (A/'source_manifest.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
sys.path[:0]=[str(STAGE/'arena_autonomy'),str(STAGE/'arena_vehicle_interface'),str(A.parent/'field_wall_irregularity_r518_v1/test_deps')]
old_cross=np.cross
def compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return old_cross(a,b,*args,**kwargs)
np.cross=compat
spec=importlib.util.spec_from_file_location('r521_test',STAGE/'test_current_wall_reacquire.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
from arena_autonomy.follow_trace import freeze,comparable_decision,equal_output
from arena_autonomy import current_wall_reacquire as reacquire
from arena_autonomy.race_runtime_policy import terminal_plan,released_plan

# 교차 엔진/원 ray metadata는 r521 코드 그대로. 벽 기하 생성 부분만 명시적으로 확장한다.
generator=inspect.getsource(t.sensor)
generator=generator.replace('def sensor(now=.7, shift=0., end=2., curved=False, obstacle=False, closed=False, actor=None):',
    'def sensor(now=.7, shift=0., end=2., curved=False, obstacle=False, closed=False, actor=None, amplitude=0., gap=0.):')
old='''    xs = np.linspace(-2., end, 160)
    center = shift + (.05*np.sin(6*xs) if curved else xs*0)
    segments = []
    for sign in (-1., 1.):
        points = np.c_[xs, center+sign*CFG.wall_offset_m]
        segments.extend(zip(points[:-1], points[1:]))'''
new='''    joints = [.40, 1.00]
    intervals = [(j-gap/2,j+gap/2) for j in joints] if gap else []
    xs = np.unique(np.r_[np.linspace(-2.,end,801),[x for pair in intervals for x in pair]])
    xs = xs[(xs>=-2.) & (xs<=end)]
    center = shift + amplitude*np.sin(math.tau*xs/1.0)
    segments = []
    for sign in (-1.,1.):
        points = np.c_[xs,center+sign*CFG.wall_offset_m]
        for a,b in zip(points[:-1],points[1:]):
            middle=(a[0]+b[0])/2
            if not any(lo<middle<hi for lo,hi in intervals):
                segments.append((a,b))'''
assert old in generator;generator=generator.replace(old,new,1)
(A/'sensor_generator.py').write_text(generator,encoding='utf-8')
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

cases=[dict(name='straight_baseline',amplitude=0.,gap=0.,shift=0.),
       dict(name='bend30_gap20_shift100',amplitude=.03,gap=.02,shift=.10),
       dict(name='bend60_gap20_shift100',amplitude=.06,gap=.02,shift=.10),
       dict(name='bend60_gap50_shift160',amplitude=.06,gap=.05,shift=.16),
       dict(name='bend60_gap50_shift160_actual_obstacle',amplitude=.06,gap=.05,shift=.16,obstacle=True)]
results=[];terminal_plan.install()
try:
    with released_plan.enabled():
        for case in cases:
            kw={k:v for k,v in case.items() if k!='name'}
            scene,raw=sensor(.7,closed=True,actor=1.2,**kw)
            preserved_scene=copy.deepcopy(scene);preserved_ranges=copy.deepcopy(raw.ranges)
            local_scene,local_raw=sensor(1.1,closed=False,**kw)
            local_before=baseline_local_follow(local_scene,True);local_after=baseline_local_follow(local_scene)
            before=moving(scene,raw,True);after=moving(scene,raw)
            helper,trace=helper_trace(scene)
            assert scene==preserved_scene and raw.ranges==preserved_ranges
            assert len(raw.ranges)==500 and len(scene.rays)==500
            row=dict(**case,all_geometry_assumed_not_measured=True,wavelength_m=1.,joint_centers_x_m=[.4,1.],
                local_before=summarize(local_before),local_after=summarize(local_after),
                moving_before=summarize(before),moving_after=summarize(after),helper=summarize(helper),helper_trace=trace,
                local_positive_to_zero=local_before[0]>0 and local_after[0]==0,
                moving_positive_to_zero=before[0]>0 and after[0]==0,
                positive_moving_exact_match=before[0]<=0 or equal_output(comparable_decision(before),comparable_decision(after)),
                statuses=dict(Counter(r.status for r in scene.rays)),
                unknown_source_indices=[r.source_index for r in scene.rays if r.status=='unknown'],
                raw_scene_unchanged=True,original_ray_metadata_preserved=True)
            results.append(row)
            (A/(case['name']+'_outputs.json')).write_text(json.dumps(freeze(dict(local_before=local_before,local_after=local_after,moving_before=before,moving_after=after,helper=helper)),indent=2),encoding='utf-8')
            (A/(case['name']+'_ranges.json')).write_text(json.dumps(freeze(dict(ranges=raw.ranges,angle_min=raw.angle_min,angle_increment=raw.angle_increment,time_increment=raw.time_increment,source_first_s=scene.source_first_s,source_last_s=scene.source_last_s))),encoding='utf-8')
            print(json.dumps(dict(case=case['name'],local=[local_before[0],local_after[0]],moving=[before[0],after[0]],moving_reason=after[2].get('reason'),helper=0 if helper is None else helper[0],statuses=row['statuses'])),flush=True)
finally:terminal_plan.uninstall()
assert results[-1]['moving_after']['speed_mps']==0 and results[-1]['local_after']['speed_mps']==0
report=dict(scope='five synthetic ray-intersection cases, no ROS/closed-loop/hardware',numpy_version=np.__version__,compatibility='process-local 2D determinant; Python numeric default',cases=results,
    current_sources_unchanged={str(p.relative_to(STAGE)):hashlib.sha256((ROOT/'src'/p.relative_to(STAGE)).read_bytes()).hexdigest()==hashlib.sha256(p.read_bytes()).hexdigest() for p in [STAGE/'arena_autonomy/arena_autonomy/current_wall_reacquire.py',STAGE/'arena_autonomy/arena_autonomy/bypass_wall_fit.py']})
(A/'report.json').write_text(json.dumps(freeze(report),indent=2),encoding='utf-8')
