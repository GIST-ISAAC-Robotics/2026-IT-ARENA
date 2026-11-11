"""저속 후측방 무반환의 작은 차체 외곽 이동 후보. 기존 관측 수치는 유지한다."""
import math
import numpy as np
from arena_autonomy.observation_support import source_valid
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn
import arena_autonomy.pass_release_trial as ps

ORIGINAL=ps.same_path_risk


def bounded_body_nonreturn(scene,raw_scan,queries,padding,length,width,speed,target):
    result=dict(accepted=False,reason='invalid_input',unknown_filled=False,
        residual_hidden_object_risk=True,maximum_exterior_m=.01,
        maximum_gap_rad=math.radians(6),minimum_neighbor_range_m=.6,
        maximum_local_chord_m=.05,policy='guard_low_speed_body_nonreturn_r136')
    def reject(reason):return {**result,'reason':reason}
    if (raw_scan is None or not source_valid(scene)
            or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x)
                   for x in (length,width,speed,target))
            or length<=0 or width<=0 or not 0<=speed<=.35 or not 0<target<=.35):
        return result
    q=np.asarray(queries,dtype=float).reshape(-1,2);pad=np.asarray(padding,dtype=float)
    if (not len(q) or not np.all(np.isfinite(q)) or pad.shape!=(len(scene.rays),)
            or not np.all(np.isfinite(pad)) or np.any(pad<0)):
        return result
    exterior=np.abs(q[:,1])-width/2
    result.update(queries=len(q),exterior_max_m=float(exterior.max()))
    if np.any(abs(q[:,0])>length/2) or np.any(exterior<=0) or np.any(exterior>.01):
        return reject('outside_small_body_side')
    checked=_bounded_nonreturn(scene,raw_scan,q,pad,result)
    if not checked['accepted']:return checked
    chords=[]
    for gap in checked['gaps']:
        rays=scene.rays[gap['first']:gap['last']+1]
        if any(abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100) for r in rays):
            return {**checked,'accepted':False,'reason':'front_gap_outside_scope'}
        radius=max(float(np.max(np.linalg.norm(q-np.asarray(r.ray_origin_xy_m),axis=1))) for r in rays)
        chord=2*(radius+float(pad.max()))*math.sin(gap['gap_rad']/2)
        chords.append(chord)
        if chord>.05:return {**checked,'accepted':False,'reason':'wide_local_gap','maximum_chord_m':chord}
    return {**checked,'reason':'bounded_low_speed_body_nonreturn_risk','maximum_chord_m':max(chords)}


def same_path_with_body(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline):
    old=ORIGINAL(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline)
    partial=baseline.get('partial_observation') or {}
    if (old['accepted'] or not context or not region['unobserved']
            or not owner.follow_config.traffic_recovery_trial
            or owner.mode!='GUARD' or any(s['present'] for s in owner.sides.values())
            or partial.get('allowed') is not True or partial.get('reason')!='fully_observed'):
        return old
    q=context['queries'][~context['free'] & ~context['masked']]
    proof=bounded_body_nonreturn(scene,raw_scan,q,context['padding'],owner.follow_config.length_m,
                                owner.follow_config.width_m,speed,target)
    return proof if proof['accepted'] else {**old,'body_nonreturn_recheck':proof}


def install():ps.same_path_risk=same_path_with_body
def uninstall():ps.same_path_risk=ORIGINAL


def worker_entry(*args):
    """spawn 자식에서도 같은 명시 후보를 설치한다. 운영 worker 파일은 수정하지 않는다."""
    install()
    from arena_autonomy.decision_process import _worker
    return _worker(*args)
