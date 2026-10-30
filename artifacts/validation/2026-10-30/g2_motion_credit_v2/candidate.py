"""오프라인 비교: 다중 관측창의 전방 이동 추정만 간격 예약식에 더한다.

반환점 교체를 완벽히 구별하는 속도계가 아니다. 가림 반례는 별도로 보존한다.
보태는 속도는 기존 준비 상한/물체 운동 한도 이하이며, G1 명령도 넘지 않는다.
"""
from collections import deque
import math
import numpy as np
from arena_autonomy.bypass_handoff import BypassHandoffFollow, front_patch
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


class FrontMotionCredit:
    def __init__(self):
        self.reset()

    def reset(self):
        self.rows = deque(); self.key = None; self.last_ref = None
        self.result = dict(credit_mps=0.,reason='no_motion_support',comparisons=[])

    def update(self, scene, history, *, policy, cap):
        now, source = scene.t_ref_s, scene.source_first_s
        if not source_valid(scene) or source is None or not 0 <= now-source < .30:
            self.reset();return self.result
        key = (scene.source_frame_id,scene.source_epoch,source)
        if self.key is not None and (key[:2]!=self.key[:2] or source<self.key[2]
                or source-self.key[2]>.25 or now<self.last_ref):
            self.reset()
        if key == self.key:
            self.last_ref = now
            return self.result
        self.key=key;self.last_ref=now
        points, reason = front_patch(scene,policy)
        if points is None:
            self.rows.clear();self.result=dict(credit_mps=0.,reason=reason,comparisons=[])
            return self.result
        while self.rows and source-self.rows[0]['source']>.81:
            self.rows.popleft()
        comparisons=[]
        try:
            for lag in (.4,.6,.8):
                if not self.rows:continue
                old=min(self.rows,key=lambda r:abs(source-r['source']-lag))
                dt=source-old['source']
                if abs(dt-lag)>.025:continue
                projected=transport_points(old['points'],history,old['time'],now)
                dx=float(np.quantile(points[:,0],.1)-np.quantile(projected[:,0],.1))
                comparisons.append(dict(lag_s=dt,front_delta_m=dx,rate_mps=(dx-.06)/dt))
        except (ValueError,MotionUnavailable):
            self.reset();return self.result
        self.rows.append(dict(source=source,time=now,points=points.copy()))
        credit=(min(cap,max(0.,min(c['rate_mps'] for c in comparisons))) if len(comparisons)>=2 else 0.)
        self.result=dict(credit_mps=credit,reason='multi_window_front_motion' if credit>0 else 'no_positive_multi_window_motion',
                         comparisons=comparisons,identity_or_stationarity_proven=False)
        return self.result


class MotionCreditCandidate(BypassHandoffFollow):
    def __init__(self,*args,**kwargs):
        self.motion_credit=FrontMotionCredit();self._credit_history=None
        super().__init__(*args,**kwargs)

    def reset(self):
        self.motion_credit.reset()
        return super().reset()

    def command(self,scene,history,*args,**kwargs):
        self._credit_history=history
        if self.mode!='FOLLOW':self.motion_credit.reset()
        try:
            result=super().command(scene,history,*args,**kwargs)
            if result[2].get('reason')!='handoff_follow':self.motion_credit.reset()
            return result
        finally:
            self._credit_history=None

    def _follow_entry_space(self,scene,normal,previous,target):
        original,evidence=super()._follow_entry_space(scene,normal,previous,target)
        credit=self.motion_credit.update(scene,self._credit_history,policy=self.execution.front_patch_policy,
            cap=min(self.execution.preparation_cap_mps,self.config.object_motion_bound_mps))
        if evidence['reason']!='observed_entry_distance_reserve':
            return original,dict(evidence,motion_credit=credit)
        # 원래 max(0, ...)를 계산한 뒤 더하면 가까운 거리에서도 계속 진행하게 된다.
        # 부호 있는 원식에 먼저 더하여 공간 부족분을 그대로 차감한다.
        signed=(evidence['rear_axle_to_patch_m']-evidence['reserve_m'])/evidence['approach_window_s']
        cap=max(0.,signed+credit['credit_mps'])
        return min(target,cap),dict(evidence,reason='observed_entry_motion_credit',
            applied=cap<target,cap_mps=cap,original_cap_mps=evidence['cap_mps'],motion_credit=credit)
