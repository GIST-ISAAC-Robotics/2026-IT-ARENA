"""미연결 비교 후보: 이미 시작한 준비 대기만 최대1초 유지한다. 우회 허가는 주지 않는다."""
import math
import numpy as np
from arena_autonomy.bypass_handoff import BypassHandoffFollow,front_patch
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


class PreparationHoldFollow(BypassHandoffFollow):
    max_hold_age_s=1.

    def __init__(self,*args,**kwargs):
        self.preparation_anchor=None;self.current_history=None
        super().__init__(*args,**kwargs)

    def reset(self):
        self.preparation_anchor=None
        super().reset()

    def _follow_entry_space(self,scene,normal,previous,target):
        value,detail=super()._follow_entry_space(scene,normal,previous,target)
        anchor=self.preparation_anchor
        # 현재 조각/원호 검사는 base에서 마쳤다. 벽 직선 기준만 잠시 없어졌을 때의 추가 감속이다.
        if anchor is None or detail.get('reason')!='two_wall_straight_reference_unavailable':return value,detail
        now=scene.t_ref_s;key=(scene.source_frame_id,scene.source_epoch,scene.source_first_s)
        if (key[:2]!=anchor['key'][:2] or key[2]<anchor['key'][2]
            or not 0<=now-anchor['time']<=self.max_hold_age_s
            or not 0<=now-key[2]<.30 or self.monitor.motion_evidence()):
            self.preparation_anchor=None;return value,detail
        patch,_=front_patch(scene,self.execution.front_patch_policy)
        if patch is None:
            self.preparation_anchor=None;return value,detail
        try:
            old=transport_points(anchor['patch'],self.current_history,anchor['time'],now)
            displacement=float(np.linalg.norm(np.median(old,axis=0)-np.median(patch,axis=0)))
        except (ValueError,MotionUnavailable):
            self.preparation_anchor=None;return value,detail
        if not math.isfinite(displacement) or displacement>self.execution.maximum_patch_displacement_m:
            self.preparation_anchor=None;return value,detail
        return 0.,dict(detail,applied=value>0,reason='bounded_preparation_hold_candidate',
            original_reason=detail['reason'],hold_age_s=now-anchor['time'],maximum_age_s=self.max_hold_age_s,
            compensated_patch_displacement_m=displacement,prior_source_first_s=anchor['key'][2],
            current_front_evidence=True,free_space_assumed=False,bypass_permission=False)

    def command(self,scene,history,speed,side,previous,dt,*,raw_scan=None):
        self.current_history=history
        result=super().command(scene,history,speed,side,previous,dt,raw_scan=raw_scan)
        detail=result[2];reason=detail.get('reason')
        if self.mode!='FOLLOW' or reason in ('handoff_reverse_motion_hold','handoff_input_fault','handoff_invalid_input','handoff_source_reset'):
            self.preparation_anchor=None;return result
        prep=detail.get('preparation') or {};entry=detail.get('entry_space') or {}
        if result[0]==0. and prep.get('applied') is True and entry.get('hold') is True:
            key=(scene.source_frame_id,scene.source_epoch,scene.source_first_s)
            patch,_=front_patch(scene,self.execution.front_patch_policy)
            if patch is not None and (self.preparation_anchor is None or key!=self.preparation_anchor['key']):
                self.preparation_anchor=dict(key=key,time=scene.t_ref_s,patch=patch.copy())
        if prep.get('reason') in ('patch_motion_observed','no_current_front_patch','patch_outside_g1_swept_arc'):
            self.preparation_anchor=None
        return result
