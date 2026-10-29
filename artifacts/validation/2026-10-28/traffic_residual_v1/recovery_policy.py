"""저장 재생 전용 정책 사본. ROS 제어/보호층에는 연결하지 않는다."""
import copy
import numpy as np
from probe_surface_anchor import SurfaceAnchorProbe
from probe_timed_policy import policy_class


def recovery_policy_class():
    TimedPolicy,_,_=policy_class()

    class RecoveryPolicy(TimedPolicy):
        def reset(self):
            super().reset()
            self.anchor_probe=SurfaceAnchorProbe()
            self.anchor_latch_time=None

        def evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,
                     wheelbase=.135, length=.20, width=.15):
            arguments=(scan,points,history,now,speed,steering,previous_steering,wheelbase,length,width)
            result=super().evaluate(*arguments)
            if not self.latched:
                self.anchor_probe.reset();self.anchor_latch_time=None
                return result
            if (result.get('reason')!='known_obstacle_latched' or
                    self.latch_reason!='previous_point_in_missing_sweep'):
                return result
            evidence=self.latch_evidence or {}
            witnesses=evidence.get('witnesses',[])
            # 현재 기록은 최대8쌍이다. 잘린 기록을 전체 위협으로 간주하지 않는다.
            if not witnesses or evidence.get('pairs')!=len(witnesses):
                return {**result,'anchor_diagnostic':dict(reason='incomplete_threat_set',released=False)}
            first=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            projected,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,
                scan.range_min,scan.range_max,first,scan.time_increment,now,mode='both',timing_verified=True)
            assert np.allclose(points,projected,rtol=0,atol=1e-8)
            if self.anchor_latch_time!=self.latch_time:
                self.anchor_probe.start([w['previous_point_now_xy'] for w in witnesses],points,ids,now,first)
                self.anchor_latch_time=self.latch_time
                return {**result,'anchor_diagnostic':dict(reason='initialized',released=False)}
            diagnostic=self.anchor_probe.observe(points,ids,history,now,first)
            result={**result,'anchor_diagnostic':{**diagnostic,'released':False}}
            if not diagnostic['matched']:
                return result
            # 옛 증거가 다시 보였어도 현재 스캔의 다른 위협/공백을 통과해야 한다.
            trial=copy.deepcopy(self)
            trial.latched=False
            trial.latch_reason=trial.latch_time=trial.latch_evidence=None
            reevaluated=TimedPolicy.evaluate(trial,*arguments)
            result['anchor_diagnostic']['current_recheck_reason']=reevaluated.get('reason')
            if not reevaluated.get('allowed'):
                return result
            # 격리 재생 안에서만 상태를 채택한다. 같은 스캔을 추가 확인표로 세지 않는다.
            self.__dict__=trial.__dict__
            self.anchor_probe.reset();self.anchor_latch_time=None
            return {**reevaluated,'anchor_diagnostic':{**diagnostic,'released':True,
                'current_recheck_reason':reevaluated.get('reason')}}

    return RecoveryPolicy
