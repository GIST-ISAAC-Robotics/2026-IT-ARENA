"""저장 입력 비교 전용. 명령 상한 초과 측정 중0명령과 기하 보관을 분리한다."""
import math
import numpy as np
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points


class OverspeedHoldGuide(ForecastGuide):
    def reset(self):
        super().reset()
        self.overspeed_start_s = None

    def evaluate(self, scene, history, speed, previous, dt):
        finite = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                     for v in (speed, previous, dt))
        overspeed = finite and speed > self.config.max_speed_mps+1e-12
        valid = (finite and source_valid(scene) and 0 < dt <= .2
                 and abs(previous) <= self.config.max_steering_rad)
        changed = ((self.epoch is not None and self.epoch != scene.source_epoch)
            or (self.reference_s is not None and scene.t_ref_s < self.reference_s)
            or (self.last_source is not None and scene.source_first_s is not None
                and scene.source_first_s < self.last_source))
        if not (valid and not changed and overspeed and self.path is not None):
            self.overspeed_start_s = None
            return super().evaluate(scene, history, speed, previous, dt)
        if self.overspeed_start_s is None:
            self.overspeed_start_s = scene.t_ref_s
        # 명목 명령 상한은 올리지 않는다. 입력 초과가 계속되거나 이송 시각이
        # 기존 기하 기억 TTL을 넘으면 보관을 끝낸다. 이후 새 기하로 다시 시작한다.
        if (scene.t_ref_s-self.overspeed_start_s >= self.config.memory_ttl_s
                or self.reference_s is None or scene.t_ref_s-self.reference_s > self.config.memory_ttl_s):
            self.reset()
            return self._hold('entry_overspeed_retention_expired', measured_speed_mps=speed)
        try:
            path = transport_points(self.path, history, self.reference_s, scene.t_ref_s)
            wall = transport_points(self.wall_line, history, self.reference_s, scene.t_ref_s)
            if (not np.isfinite(path).all() or not np.isfinite(wall).all()
                    or np.max(path[:, 0]) < -self.config.length_m/2):
                raise ValueError('carried path is no longer in front of the body')
        except (ValueError, MotionUnavailable):
            self.reset()
            return self._hold('entry_overspeed_transport_unavailable', measured_speed_mps=speed)
        self.path, self.wall_line, self.reference_s = path, wall, scene.t_ref_s
        self.epoch, self.last_source = scene.source_epoch, scene.source_first_s
        self.wall_confirmations = self.rejoin_confirmations = self.center_confirmations = 0
        for monitor in self.monitors.values():
            monitor.reset()
        self.reverse_wait, self.forward_confirmations = True, 0
        self.forward_source = scene.source_first_s
        self.corridor_use = {}
        return self._hold('entry_overspeed_motion_hold', measured_speed_mps=speed,
            geometry_retained=True, geometry_is_permission=False, forward_confirmations=0,
            overspeed_elapsed_s=scene.t_ref_s-self.overspeed_start_s)
