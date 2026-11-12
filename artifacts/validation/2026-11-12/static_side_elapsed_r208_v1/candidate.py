"""저장 입력 전용: 정지 물체 근거의 이동 적분에 실제 판단 시각을 사용한다."""
import math
from arena_autonomy.static_side_return import StaticSideTracker

class TimedTracker(StaticSideTracker):
    def reset(self):
        super().reset()
        self.reference_s=self.reference_speed=None

    def advance_to(self, stamp, speed):
        """양 끝 측정 속도의 사다리꼴 근사. 실제 위치/정지의 증명이 아니다."""
        if (not math.isfinite(stamp) or not math.isfinite(speed) or stamp<0 or speed<0):
            self.reset()
            return
        if self.reference_s is not None:
            elapsed=stamp-self.reference_s
            if elapsed==0:
                return
            if elapsed<0 or elapsed>self.config.maximum_gap_s:
                self.reset()
            else:
                self.travel_m+=(self.reference_speed+speed)*.5*elapsed
        self.reference_s,self.reference_speed=stamp,speed

    def advance(self,speed,dt):
        # 비교용 연결만 바꾼다. dt는 조향/명령 예측에서 계속 원 값이다.
        self.advance_to(self._current_reference_s,speed)

def install():
    from arena_autonomy import pass_release_trial as ps
    original=ps.PassReleaseSelector._select
    def select(owner,scene,*args,**kwargs):
        for tracker in (owner.static_side or {}).values():
            if isinstance(tracker,TimedTracker):tracker._current_reference_s=scene.t_ref_s
        return original(owner,scene,*args,**kwargs)
    ps.StaticSideTracker=TimedTracker
    ps.PassReleaseSelector._select=select
