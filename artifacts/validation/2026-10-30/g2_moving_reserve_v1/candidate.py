"""오프라인 비교 전용: 이동 근거가 남아 있을 때 추가 우회 공간 예약만 생략한다.

G1의 현재 속도·조향·장애물 판단, 진입 관측 창과 독립 보호층은 그대로다.
이동 근거는 정지/차량 ID 증명이 아니며 갑자기 멈추면 우회 공간이 부족할 수 있다.
런타임·ROS 정책이나 기본값으로 등록하지 않는다.
"""
from arena_autonomy.bypass_handoff import BypassHandoffFollow


class MovingReserveCandidate(BypassHandoffFollow):
    def _follow_entry_space(self, scene, normal, previous, target):
        result = self.monitor.result
        if (result.get('observed') and not result.get('ready')
                and self.monitor.motion_evidence()):
            return target, dict(applied=False, reason='observed_motion_no_entry_reserve',
                motion_rate_mps=self.monitor.motion_rate_mps,
                before_mps=target, free_space_assumed=False,
                identity_or_stationarity_proven=False)
        return super()._follow_entry_space(scene, normal, previous, target)
