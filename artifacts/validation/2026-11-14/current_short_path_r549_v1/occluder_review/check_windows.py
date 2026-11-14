"""저장 반환·순수 경로 함수만 확인. NumPy/환경 패치, 장치 실행 없음."""
from pathlib import Path
import importlib.util, json, math, os, sys
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'): os.environ[key]='1'
sys.dont_write_bytecode=True
import numpy as np
A=Path(__file__).resolve().parent; ROOT=A.parents[4]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import local_path as baseline
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.corner_corridor import corridor_evidence
STAGE=sys.argv[1] if len(sys.argv)>1 else 'stage2'
spec=importlib.util.spec_from_file_location('candidate_path',A/STAGE/'src/arena_autonomy/arena_autonomy/local_path.py')
candidate=importlib.util.module_from_spec(spec); spec.loader.exec_module(candidate)
P=ROOT/'artifacts/validation/2026-11-14/measured_steering_lap_r544_attempt2_v1/stall_extract'

def restored(case,phase='current'):
    header=json.loads((P/'header.json').read_text())['record']['data']
    data,scan,history=restore_call(json.loads((P/f'{case}_{phase}_input.json').read_text()),MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],epoch=data['epoch'],
        rear_half_angle_deg=data['rear_half_angle_deg'],timing_policy=data.get('timing_policy','verified_only'))
    return header,data,scene

def cast(segments,origins):
    angles=-math.pi+np.arange(500)*math.tau/500
    directions=np.c_[np.cos(angles),np.sin(angles)]
    origins=np.broadcast_to(origins,(500,2))
    best=np.full(500,np.inf)
    for x0,y0,x1,y1 in segments:
        e=np.array([x1-x0,y1-y0]); q=np.array([x0,y0])-origins
        den=directions[:,0]*e[1]-directions[:,1]*e[0]
        safe=np.where(abs(den)<1e-12,1.,den)
        distance=(q[:,0]*e[1]-q[:,1]*e[0])/safe
        along=(q[:,0]*directions[:,1]-q[:,1]*directions[:,0])/safe
        best=np.where((abs(den)>1e-12)&(distance>.05)&(along>=0)&(along<=1)&(distance<best),distance,best)
    valid=np.isfinite(best)&(abs(angles)<math.radians(150))
    return origins[valid]+directions[valid]*best[valid,None],angles[valid]

def main():
    results=[]
    for case in ('last_positive','first_zero_discarded','first_zero_candidate'):
        header,data,scene=restored(case)
        points=np.asarray(scene.occupied_endpoints_xy_m); before=points.copy()
        bearings=np.array([math.atan2(r.direction_xy[1],r.direction_xy[0]) for r in scene.rays if r.status=='return'])
        args=dict(offset=.505,horizon=.65,narrow_occluder_bridge=True)
        old,oi=baseline(points,'left',**args)
        new,ni=candidate.local_path(points,'left',bearings_rad=bearings,**args)
        np.testing.assert_array_equal(points,before)
        assert new is not None and ni['boundary_observed_ahead_m']>=.28
        config=FollowConfig(**header['follow_config'])
        pursuit=pursue_path(new,data['speed_mps'],config,scene.t_ref_s-scene.source_first_s,speed_cap=.1)
        right,ri=baseline(points,'right',offset=.505)
        corridor=corridor_evidence(points,'left',config)
        results.append(dict(case=case,reference_s=data['reference_s'],baseline=oi,candidate=ni,
            polynomial_pursuit=pursuit,occupancy_unchanged=True,
            right_default_path=ri,right_distance_anchor_error_m=abs(ri.get('wall_distance_m',0)-.505),
            corridor_reason=corridor['reason']))
    synth=[]
    walls=[(-1.,.1,.36,.508),(.4,2.2,3.,2.2)]
    for label,origin in [('zero',[0.,0.]),('front_60mm',[.06,.002]),('front_75mm',[.075,.002]),
                         ('smooth_acquisition_motion',np.c_[np.linspace(.06,.075,500),np.full(500,.002)])]:
        p,b=cast(walls,origin); before=p.copy()
        old,oi=baseline(p,'left',.505,horizon=.65,narrow_occluder_bridge=True)
        new,ni=candidate.local_path(p,'left',.505,horizon=.65,narrow_occluder_bridge=True,bearings_rad=b)
        assert new is not None
        np.testing.assert_array_equal(p,before)
        if label=='zero':
            np.testing.assert_array_equal(new,old); assert ni==oi
        synth.append(dict(case=label,baseline=oi,candidate=ni,occupancy_unchanged=True))
    p,b=cast([(-1.,.5,2.,.5),(.28,.35,.29,.35)],[.06,0.])
    before=p.copy()
    path,info=candidate.local_path(p,'left',.5,narrow_occluder_bridge=True,bearings_rad=b)
    assert path is not None and info.get('narrow_occluders_bridged',0)>0
    assert np.any(p[:,1]<.45)
    np.testing.assert_array_equal(p,before)
    synth.append(dict(case='physical_narrow_post',candidate=info,raw_post_still_present=True))
    # 기존 기둥 반례를 그대로 유지: 넓은 차량 앞면을 작은 조각으로 지우지 않는다.
    cluster=np.c_[np.linspace(.375,.525,12),np.full(12,.37)]
    p=np.vstack(([.33,.53],[.36,.51],cluster,[.55,.52],[.58,.53]))
    gaps=np.linalg.norm(np.diff(p,axis=0),axis=1)
    limits=.10+.035*np.minimum(np.linalg.norm(p[:-1],axis=1),np.linalg.norm(p[1:],axis=1))
    mask,joins=candidate.break_adjacent_occluders(p,gaps>limits)
    assert not mask.any() and not joins
    for bad in ([0.],np.full(len(p),np.nan)):
        assert candidate.local_path(p,'left',bearings_rad=bad)[1]['reason']=='wall_bearings_invalid'
    output=dict(scope='Windows polynomial path and raw-chain geometry only; piecewise/helper native replay pending',
        numpy_version=np.__version__,saved=results,synthetic=synth,wide_vehicle_preserved=True,invalid_alignment_rejected=True)
    (A/f'windows_result_{STAGE}.json').write_text(json.dumps(freeze(output),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(saved=[dict(case=r['case'],old=r['baseline'].get('boundary_observed_ahead_m'),
        old_reason=r['baseline']['reason'],new=r['candidate']['boundary_observed_ahead_m'],
        polynomial_pursuit=r['polynomial_pursuit'],right_anchor=r['right_distance_anchor_error_m']) for r in results],
        synthetic=[dict(case=r['case'],old=r.get('baseline',{}).get('boundary_observed_ahead_m'),
            new=r['candidate'].get('boundary_observed_ahead_m'),bridges=r['candidate'].get('narrow_occluders_bridged')) for r in synth]),indent=2))
if __name__=='__main__':main()
