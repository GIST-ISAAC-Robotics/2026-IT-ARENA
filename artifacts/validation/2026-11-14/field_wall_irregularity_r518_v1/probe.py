"""저장 입력의 벽 한 면에 연속 굴곡/연결 틈을 넣는 Windows 전용 민감도 검사."""
from pathlib import Path
import copy, hashlib, json, math, shutil, sys, time
import numpy as np

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
STAGE=A/'source_snapshot'
MODE=sys.argv[1] if len(sys.argv)>1 else 'current'
if not STAGE.exists():
    for name in ('arena_autonomy','arena_vehicle_interface'):
        shutil.copytree(ROOT/'src'/name,STAGE/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    manifest={str(p.relative_to(STAGE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in STAGE.rglob('*') if p.is_file()}
    (A/'source_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
sys.path[:0]=[str(STAGE/'arena_autonomy'),str(STAGE/'arena_vehicle_interface')]
old_cross=np.cross
def compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return old_cross(a,b,*args,**kwargs)
np.cross=compat

from arena_autonomy.follow_trace import freeze,thaw,restore_call,comparable_decision,equal_output
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import local_path
from arena_autonomy.bypass_wall_fit import tracked_straight_wall
from arena_autonomy.moving_pass import MovingPassGuide
import arena_autonomy.numeric_kernels as numeric
original_configure=numeric.configure
# Windows에는 Linux native 확장 모듈이 없다. 표준 Python 수치 경로만 사용하고
# 원 header는 보존한다. baseline 전체 출력 대조로 차이 유무를 확인한다.
numeric.configure=lambda mode:original_configure('python')

source=A.parent/'continuous_pass_preview_drive_r513_v1/run1/follow_decisions.jsonl'
raw=source.read_bytes();plans=[];expected={}
for line in raw.splitlines():
    row=json.loads(line);d=row['data'];kind=row['kind']
    if kind=='runtime_evaluator_header':header=d['header']
    elif kind=='runtime_plan_submitted':plans.append(d)
    elif kind=='runtime_plan_completed':expected[(d['generation'],d['sequence'])]=d['output']
if MODE=='historic':
    # 원 r513와 공통의 차이를 소급 성공으로 취급하지 않는다.
    import arena_autonomy.moving_pass as moving
    before=A.parent/'current_guide_extent_r514_v1/moving_pass.before.py'
    exec(compile(before.read_text(encoding='utf-8'),str(before),'exec'),moving.__dict__)
    MovingPassGuide=moving.MovingPassGuide
elif MODE=='candidate':
    import arena_autonomy.bypass_wall_fit as fit_module
    import arena_autonomy.bypass_entry_trial as entry_module
    proposal=A/'bypass_wall_fit.candidate.py'
    exec(compile(proposal.read_text(encoding='utf-8'),str(proposal),'exec'),fit_module.__dict__)
    entry_module.tracked_straight_wall=fit_module.tracked_straight_wall
    tracked_straight_wall=fit_module.tracked_straight_wall

def summary(out):
    d=out[2]
    normal=d.get('normal',d)
    entry=d.get('moving_pass',{}).get('entry',{})
    return dict(speed=out[0],steer=out[1],reason=d.get('reason'),behavior=d.get('behavior'),
        entry_reason=entry.get('reason'),normal_reason=normal.get('reason'),
        boundary_reason=normal.get('boundary_candidate_reason'),boundary_ahead=normal.get('boundary_candidate_ahead_m'),
        fresh_boundary=normal.get('fresh_boundary_accepted'),swept_clearance=normal.get('swept_clearance_m'))

captures=[]
original=MovingPassGuide._wall_covers_guide
def capture(self,fit,wall):
    result=original(self,fit,wall)
    captures.append(dict(fit=fit,wall=wall,accepted=result))
    return result
MovingPassGuide._wall_covers_guide=capture
from arena_autonomy.bypass_entry_trial import EntryGuide
original_outer=EntryGuide._outer_wall_fit
fit_calls=[]
def capture_outer(self,scene,remembered):
    fit,detail=original_outer(self,scene,remembered)
    fit_calls.append(dict(remembered_wall=freeze(remembered),fit=fit,detail=detail))
    return fit,detail
EntryGuide._outer_wall_fit=capture_outer

def scene_of(data,e):
    d,scan,history=restore_call(data,e.motion)
    scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    return d,scan,history,scene

def wall_selection(data,e,baseline_captures):
    d,scan,history,scene=scene_of(data,e)
    cfg=e.follow.config
    if baseline_captures:
        wall=baseline_captures[-1]['wall']
        ids=wall.get('source_ray_indices',[])
    else:
        points=np.array(scene.occupied_endpoints_xy_m).reshape(-1,2)
        path,info=local_path(points,'left',cfg.wall_offset_m,horizon=1.8,piecewise_corner=cfg.piecewise_wall_trial,minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),narrow_occluder_bridge=cfg.narrow_occluder_bridge)
        if path is None:raise RuntimeError(('no baseline path',info))
        line=np.asarray([path[0],path[-1]])
        _,wall=tracked_straight_wall(scene,cfg,'left',line)
        ids=wall.get('source_ray_indices',[])
    if len(ids)<12:raise RuntimeError(('no observed straight wall',wall))
    raymap={r.source_index:r for r in scene.rays}
    pts=np.array([raymap[i].hit_xy_m for i in ids])
    center=pts.mean(axis=0);t=np.linalg.svd(pts-center,full_matrices=False)[2][0]
    if t[0]<0:t=-t
    n=np.array([-t[1],t[0]])
    # 왼쪽 벽의 차량 쪽 법선을 양의 방향으로 둔다.
    if n@(-center)<0:n=-n
    error=(pts-center)@n
    if np.max(np.abs(error))>.008:raise RuntimeError(('wall selection not straight',np.max(np.abs(error))))
    return dict(ids=ids,points=pts,center=center,t=t,n=n,scene=scene,raymap=raymap,
        selection_residual_max_m=float(max(abs(error))),wall=wall)

def perturb(data,selected,amplitude=0.,gap=False,wavelength=.8,gap_phase=None,joint_period=None):
    # 곡면 n·(p-c)=A sin(2π t·(p-c)/λ)와 각 원 광선의 교점을 구한다.
    # 원 측정 잔차는 보존한다. 독립 거리 잡음이 아닌 공간 연속 변형이다.
    changed=copy.deepcopy(data);ranges=thaw(changed['scan']['measurements']['ranges_m'])
    ids=selected['ids'];center=selected['center'];t=selected['t'];n=selected['n'];raymap=selected['raymap']
    points=selected['points'];scoords=(points-center)@t
    # 10 mm joint centred on the observed wall point nearest x=.45 m.
    anchor=int(np.argmin(np.abs(points[:,0]-.45)))
    gap_center=float(scoords[anchor]) if gap_phase is None else gap_phase
    removed=[];moved=[];normal_moves=[]
    for j,index in enumerate(ids):
        ray=raymap[index];p=np.array(ray.hit_xy_m);direction=np.array(ray.direction_xy)
        scoord=float(scoords[j]);base=float((p-center)@n)
        r0=ranges[index];r=r0
        for _ in range(8):
            q=p+(r-r0)*direction;u=float((q-center)@t)
            f=float((q-center)@n)-base-amplitude*math.sin(math.tau*u/wavelength)
            derivative=float(direction@n)-amplitude*math.tau/wavelength*math.cos(math.tau*u/wavelength)*float(direction@t)
            if abs(derivative)<.03:break
            r-=f/derivative
        q=p+(r-r0)*direction;u=float((q-center)@t)
        gap_distance=abs(u-gap_center) if joint_period is None else abs((u-gap_center+joint_period/2)%joint_period-joint_period/2)
        if gap and gap_distance<=.005+1e-12:
            ranges[index]=float('nan');removed.append(index)
        else:
            ranges[index]=r;moved.append(index);normal_moves.append(float((q-p)@n))
    changed['scan']['measurements']['ranges_m']=freeze(ranges)
    untouched=[i for i in range(len(ranges)) if i not in set(ids)]
    original_ranges=data['scan']['measurements']['ranges_m']
    assert all(changed['scan']['measurements']['ranges_m'][i]==original_ranges[i] for i in untouched)
    for k,v in changed['scan']['measurements'].items():
        if k!='ranges_m':assert v==data['scan']['measurements'][k]
    return changed,dict(amplitude_assumed_m=amplitude,wavelength_assumed_m=wavelength,joint_width_m=.01 if gap else 0.,
        removed_wall_ray_indices=removed,wall_ray_indices=ids,nonwall_rays_unchanged=len(untouched),
        modified_wall_count=len(moved),normal_displacement_max_m=max(map(abs,normal_moves)),gap_center_wall_s_m=gap_center,
        selected_wall_residual_max_m=selected['selection_residual_max_m'],joint_period_assumed_m=joint_period)

e=Evaluator('plan',header);generation=None;rows=[];probes=[];baseline_outputs=[];started=time.time()
candidate={tuple(r['key']):r for r in json.loads((A.parent/'current_guide_extent_r514_v1/candidate_replay.json').read_text())['rows']}
selected_sequences={60,90,99}
try:
    for d in plans:
        key=(d['generation'],d['sequence'])
        if key[0]!=generation:e.reset(d['reset_reason']);generation=key[0]
        before=copy.deepcopy(e.follow) if d['sequence'] in selected_sequences and MODE in ('current','candidate') else None
        captures.clear();out=e(d['input']);base_caps=copy.deepcopy(captures)
        baseline_outputs.append(dict(key=key,output=out))
        row=dict(key=key,**summary(out),historic_full_match=key in expected and equal_output(comparable_decision(out),comparable_decision(expected[key])))
        if key in candidate:row['r514_control_match']=abs(out[0]-candidate[key]['speed'])<1e-10 and abs(out[1]-candidate[key]['steer'])<1e-10
        rows.append(row)
        if before is not None:
            baseline_follow=e.follow
            selected=wall_selection(d['input'],e,base_caps)
            long_wave=float(np.ptp((selected['points']-selected['center'])@selected['t']))/1.5
            variants=[('identity',0.,False,.8,None),('gap10',0.,True,.8,None)]
            variants += [(f'bend{int(a*1000)}'+('_gap10' if g else ''),a,g,.8,None) for a in (.005,.01,.02,.03) for g in (False,True)]
            variants += [(f's15_bend{int(a*1000)}'+('_gap10' if g else ''),a,g,long_wave,None) for a in (.005,.01,.02,.03) for g in (False,True)]
            variants += [(f's15_bend{int(a*1000)}_repeated_gap10',a,True,long_wave,.6) for a in (.02,.03)]
            for label,amplitude,gap,wavelength,period in variants:
                changed,meta=perturb(d['input'],selected,amplitude,gap,wavelength,joint_period=period)
                e.follow=copy.deepcopy(before);captures.clear();fit_calls.clear();altered=e(changed)
                probe=dict(key=key,variant=label,**meta,baseline=summary(out),output=summary(altered),
                    positive_to_zero=out[0]>0 and altered[0]==0,full_identity=equal_output(comparable_decision(out),comparable_decision(altered)),
                    wall_captures=copy.deepcopy(captures),fit_calls=copy.deepcopy(fit_calls))
                probes.append(probe)
                folder=A/MODE;folder.mkdir(exist_ok=True)
                (folder/f'input_seq{d["sequence"]}_{label}.json').write_text(json.dumps(changed),encoding='utf-8')
                (folder/f'output_seq{d["sequence"]}_{label}.json').write_text(json.dumps(altered,indent=2),encoding='utf-8')
                print(json.dumps(dict(seq=d['sequence'],variant=label,base=out[0],new=altered[0],reason=probe['output']['reason'],entry=probe['output']['entry_reason'],removed=len(meta['removed_wall_ray_indices']))),flush=True)
            e.follow=baseline_follow
finally:
    e.close();MovingPassGuide._wall_covers_guide=original;EntryGuide._outer_wall_fit=original_outer
report=dict(mode=MODE,source_sha256=hashlib.sha256(raw).hexdigest(),source_unchanged=source.read_bytes()==raw,numpy_version=np.__version__,
    numpy_compatibility='2D determinant only, process-local',numeric_backend='python fallback; source header preserved; full output checked',elapsed_seconds=time.time()-started,
    rows=rows,probes=probes,calls=len(rows),historic_full_matches=sum(r['historic_full_match'] for r in rows),
    r514_control_matches=sum(r.get('r514_control_match',False) for r in rows),positive_to_zero=sum(r['positive_to_zero'] for r in probes))
(A/f'{MODE}_report.json').write_text(json.dumps(freeze(report),indent=2),encoding='utf-8')
(A/f'{MODE}_baseline_outputs.json').write_text(json.dumps(baseline_outputs),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('rows','probes')}),flush=True)
