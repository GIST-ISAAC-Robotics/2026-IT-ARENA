"""오프라인 G2 signed 속도 대기 후보. 역방향 측정은 그대로 보존하고 전진을 내지 않는다.

유효한 동일 시간축에서만 이전 기하를 signed 운동으로 이송한다. 기하 보관은 허가가 아니다.
정상 입력 복귀 뒤에도 현재 외벽/장애물/관측 검사와 서로 다른 2개 스캔의 진행 후보를 요구한다.
출처/세대/범위/운동 이송 실패는 기존처럼 초기화한다. 운영에 연결하지 않는다.
"""
import math
import numpy as np

from arena_autonomy.bypass_corridor_trial import CorridorGuide
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points


class SignedHoldGuide(CorridorGuide):
    def reset(self):
        super().reset()
        self.reverse_wait = False
        self.forward_confirmations = 0
        self.forward_source = None

    def evaluate(self, scene, history, speed, previous, dt):
        valid = (source_valid(scene) and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in (speed, previous, dt))
                 and abs(speed) <= self.config.max_speed_mps+1e-12 and 0 < dt <= .2
                 and abs(previous) <= self.config.max_steering_rad)
        changed = ((self.epoch is not None and self.epoch != scene.source_epoch)
            or (self.reference_s is not None and scene.t_ref_s < self.reference_s)
            or (self.last_source is not None and scene.source_first_s is not None
                and scene.source_first_s < self.last_source))
        if valid and not changed and speed < 0 and self.path is not None:
            try:
                path = transport_points(self.path, history, self.reference_s, scene.t_ref_s)
                wall = transport_points(self.wall_line, history, self.reference_s, scene.t_ref_s)
                if not np.isfinite(path).all() or not np.isfinite(wall).all():
                    raise ValueError('nonfinite carried geometry')
            except (ValueError, MotionUnavailable):
                self.reset()
                return self._hold('entry_reverse_motion_transport_unavailable')
            self.path, self.wall_line, self.reference_s = path, wall, scene.t_ref_s
            self.epoch, self.last_source = scene.source_epoch, scene.source_first_s
            self.wall_confirmations = self.rejoin_confirmations = self.center_confirmations = 0
            for monitor in self.monitors.values():
                monitor.reset()
            self.reverse_wait, self.forward_confirmations = True, 0
            self.forward_source = scene.source_first_s
            self.corridor_use = {}
            return self._hold('entry_reverse_motion_hold', measured_speed_mps=speed,
                geometry_retained=True, geometry_is_permission=False, forward_confirmations=0)
        result = super().evaluate(scene, history, speed, previous, dt)
        if not self.reverse_wait:
            return result
        # 원래 코드가 reset한 경우 위 reset()이 이 대기 상태도 지운다.
        reference = result.get('reference') or {}
        eligible = (speed >= 0 and result.get('selected') is not None
            and reference.get('reason') == 'current_outer_wall_rechecked_for_remembered_guide'
            and result.get('outer_wall_fresh_confirmations', 0) >= 2)
        fresh = self.forward_source is None or scene.source_first_s > self.forward_source
        if fresh:
            self.forward_source = scene.source_first_s
            self.forward_confirmations = self.forward_confirmations+1 if eligible else 0
        # 같은 스캔이 나이를 먹으며 현재 명령이 막혀도 그 틱은 0을 내되,
        # 별개 새 스캔에서 확인했던 횟수까지 지우지 않는다. 현재 허가는 재사용하지 않는다.
        if eligible and self.forward_confirmations >= 2:
            self.reverse_wait = False
            return dict(result, signed_hold_recovered=True, forward_confirmations=self.forward_confirmations)
        return self._hold('entry_reverse_recovery_wait', measured_speed_mps=speed,
            reference=reference, underlying_reason=result['reason'],
            forward_confirmations=self.forward_confirmations, geometry_is_permission=False)
