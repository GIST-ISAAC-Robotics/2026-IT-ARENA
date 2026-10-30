"""저장 입력 전용: 최근 벽 탐색 위치와 현재 연결 반환을 이용한 초기 기준 비교.

세 모듈의 별칭 치환은 단일 스레드 오프라인 비교에서만 쓰며 ROS에 연결하지 않는다.
기억은 탐색 위치만 정한다. 현재 양벽 반환과 원래 공통 범위/합의가 모두 필요하다.
"""
from contextlib import ExitStack
from unittest.mock import patch
import numpy as np
import arena_autonomy.bypass_handoff as handoff
import arena_autonomy.bypass_entry_trial as entry
import arena_autonomy.bypass_corridor_trial as corridor
from arena_autonomy.bypass_wall_fit import tracked_straight_wall
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points

ORIGINAL_REFERENCE = entry.straight_reference


def line_of(fit):
    a,b,lo,hi=fit
    return np.array([[lo,a*lo+b],[hi,a*hi+b]])


class CurrentPairReference:
    def __init__(self):self.clear()

    def clear(self):
        self.anchors={};self.key=None;self.last_ref=None

    def resolve(self,scene,config,history):
        original,detail=ORIGINAL_REFERENCE(scene,config)
        now,source=scene.t_ref_s,scene.source_first_s
        if not source_valid(scene) or source is None or not 0<=now-source<.30:
            self.clear();return original,detail
        key=(scene.source_frame_id,scene.source_epoch,source)
        if self.key is not None and (key[:2]!=self.key[:2] or source<self.key[2]
                or now<self.last_ref or source-self.key[2]>.25):self.clear()
        fresh=key!=self.key;self.key=key;self.last_ref=now
        if original is not None:
            if fresh:
                for side,wall in detail['walls'].items():
                    fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
                    self.anchors[side]=dict(line=line_of(fit),time=now)
            return original,detail
        fits=[];walls={}
        for side in ('left','right'):
            old=self.anchors.get(side);fit=None;evidence=dict(reason='no_recent_seed')
            if old is not None and 0<=now-old['time']<=1.:
                try:
                    line=transport_points(old['line'],history,old['time'],now)
                    fit,evidence=tracked_straight_wall(scene,config,side,line)
                except ValueError:evidence=dict(reason='seed_transport_unavailable')
            fits.append(fit);walls[side]=evidence
            if fit is not None and fresh:self.anchors[side]=dict(line=line_of(fit),time=now)
        evidence=dict(original=detail,walls=walls,all_returns_remain_occupied=True,
            remembered_line_is_current_observation=False)
        if any(f is None for f in fits):return None,dict(reason='current_pair_unavailable',**evidence)
        a,b=fits;lower,upper=max(a[2],b[2]),min(a[3],b[3])
        if lower>-config.wheelbase_m/2 or upper-max(lower,0.)<.25:
            return None,dict(reason='reference_does_not_cover_entry',**evidence)
        disagreement=max(abs((a[0]-b[0])*x+a[1]-b[1]) for x in (lower,upper))
        if disagreement>config.boundary_agreement_m:
            return None,dict(reason='observed_wall_reference_disagrees',**evidence)
        return ((a[0]+b[0])/2,(a[1]+b[1])/2,lower,upper),dict(
            reason='current_tracked_pair_candidate',disagreement_m=float(disagreement),
            wall_offset_assumption_m=config.wall_offset_m,**evidence)


class CurrentPairFollow(handoff.BypassHandoffFollow):
    def __init__(self,*args,**kwargs):
        self.reference_memory=CurrentPairReference();self.last_reference=None
        super().__init__(*args,**kwargs)

    def reset(self):
        super().reset();self.reference_memory.clear();self.last_reference=None

    def command(self,scene,history,speed,*args,**kwargs):
        if speed<-.02:self.reference_memory.clear()
        result=self.reference_memory.resolve(scene,self.config,history)
        self.last_reference=result
        def scoped(current,config):
            assert current is scene and config is self.config,'offline scope mismatch'
            return result
        with ExitStack() as stack:
            for module in (handoff,entry,corridor):stack.enter_context(patch.object(module,'straight_reference',scoped))
            output=super().command(scene,history,speed,*args,**kwargs)
        if output[2].get('reason') in ('handoff_input_fault','handoff_invalid_input',
                'handoff_source_reset','handoff_reverse_motion_hold'):self.reference_memory.clear()
        return output
