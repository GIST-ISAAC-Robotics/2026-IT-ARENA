from pathlib import Path
from types import SimpleNamespace
import copy,json,math,sys
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.dont_write_bytecode=True
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),str(ROOT/'tests'),
    str(ROOT/'artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps')]
import arena_autonomy
arena_autonomy.__path__.insert(0,str(A/'stage/src/arena_autonomy/arena_autonomy'))
# Windows NumPy 2.5 기본 2D cross만 기존 determinant 수식으로 재현한다.
# 생산 소스/입력/정밀도 변경 없음. 이전 r486 Windows 검사와 같은 호환 범위다.
original_cross=np.cross;cross_calls=0
def compatible_cross(a,b,*args,**kwargs):
    global cross_calls
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.ndim and bb.ndim and aa.shape[-1]==bb.shape[-1]==2:
        cross_calls+=1;return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return original_cross(a,b,*args,**kwargs)
np.cross=compatible_cross
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.moving_pass import MovingPassGuide,MovingTargetTrack
import arena_autonomy.bypass_entry_trial as entry

SOURCE=ROOT/'artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1/failure_inputs/follow_records.jsonl'
ROWS=[json.loads(x) for x in SOURCE.read_text(encoding='utf-8').splitlines()]
HEADER=next(r['data']['header'] for r in ROWS if r['kind']=='runtime_evaluator_header')
CFG=FollowConfig(**HEADER['follow_config']);MOTION=MotionConfig(**HEADER['motion_config'])

def captured(seq):
    row=next(r for r in ROWS if r['kind']=='call' and r['seq']==seq)
    data,raw,history=restore_call(row['data']['input'],MOTION)
    scene=motion_scene(raw,history,data['reference_s'],timing_verified=data['timing_verified'],
        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    return row,data,raw,history,scene

def observed_tracker(guide,row,scene):
    detail=copy.deepcopy(row['data']['output'][2]['moving_pass']['target'])
    guide.tracker.detail=detail
    guide.tracker.lost_advantage_scans=detail.get('lost_advantage_scans',0)
    guide.tracker.speed_upper=detail.get('observed_speed_upper_mps')
    guide.tracker.speed_lower=detail.get('observed_speed_lower_mps')
    bounds=detail.get('observed_bounds_xy')
    if bounds:
        points=np.asarray(scene.occupied_endpoints_xy_m)
        selected=np.all((points>=np.asarray(bounds[0])-1e-10)&(points<=np.asarray(bounds[1])+1e-10),axis=1)
        guide.tracker.current=points[selected].copy()
        guide.tracker.reference=scene.t_ref_s;guide.tracker.source=scene.source_first_s;guide.tracker.epoch=scene.source_epoch
    guide.tracker.rear_count=detail.get('rear_confirmations',0)

def at_entry(seq):
    row,d,raw,history,scene=captured(seq)
    result=copy.deepcopy(row['data']['output'][2]['moving_pass']['entry'])
    guide=MovingPassGuide(CFG,MovingTargetTrack(CFG,'medium_v1'));guide.raw_scan=raw
    candidate=result['candidates'][0]
    guide.path=np.asarray(candidate['path_points']);guide.offset=candidate['offset_m'];guide.transition=candidate['transition_m']
    guide.side='left';guide.phase='entry';guide.reference_s=scene.t_ref_s;guide.last_source=scene.source_first_s;guide.epoch=scene.source_epoch
    wall=result['reference']['walls'][guide.side];x=np.asarray(wall['observed_x_m'])
    guide.wall_line=np.c_[x,wall['slope']*x+wall['intercept_m']]
    guide.wall_confirmations=result['outer_wall_fresh_confirmations'];guide.intent.sign=result['intent_direction']
    observed_tracker(guide,row,scene)
    return guide,row,d,raw,history,scene,result

def compare(guide,scene,data,result,*,center=False):
    reference=result['reference'];wall=reference['walls'][guide.side]
    fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
    path=entry.entry_path(fit,0.,guide.config,guide.entry,guide.entry.transitions_m[-1]) if center else guide.path
    return entry._compare_entries(scene,scene.t_ref_s,data['speed_mps'],data['previous_steering_rad'],data['dt_s'],
        config=guide.config,entry=guide.entry,guide=dict(reference=reference,path=path,
            offset_m=0. if center else guide.offset,transition_m=guide.entry.transitions_m[-1] if center else guide.transition,
            outer_side=guide.side,stationary_outer_boundary=guide.stationary_outer_boundary and guide.wall_confirmations>=2),
        padding_provider=guide.padding_provider,follow_headway=center)

if __name__=='__main__':
    results=[]
    for seq in (525,528,567):
        guide,row,data,raw,history,scene,result=at_entry(seq)
        variants={}
        for center in (False,True):
            calls=[];original=entry.swept_limit
            def observed(points,angle,*args,**kwargs):
                value=original(points,angle,*args,**kwargs);calls.append(dict(angle=angle,clearance=value));return value
            entry.swept_limit=observed
            try:out=compare(guide,scene,data,result,center=center)
            finally:entry.swept_limit=original
            candidate=out['candidates'][0]
            variants['center' if center else 'retained']=dict(selected=out.get('selected'),reason=candidate['reason'],
                target=candidate.get('target_speed_mps'),clearance=candidate.get('command_clearance_m'),
                preview=candidate.get('preview_return_progress_m'),steering=candidate.get('steering_rad'),
                region=candidate.get('motion_region'),sweeps=calls)
        results.append(dict(seq=seq,ahead_clear=guide.tracker.ahead_clear(scene),measured=data['speed_mps'],variants=variants))
    name=sys.argv[1] if len(sys.argv)>1 else 'candidate_geometry.json'
    (A/name).write_text(json.dumps(freeze(results),indent=2),encoding='utf-8')
    print(json.dumps(results))
