"""ROS 독립 평면 운동 보정. 입력은 측정 시각의 후륜 속도·차체 자이로뿐이다.

지역 후륜축 궤적을 구간별 일정 twist로 적분한다. 세계 위치·지도·조향 명령을
받지 않는다. 출력 XY는 재표본화하지 않으며 원래 점 인덱스를 유지한다.
"""
from bisect import bisect_right
from dataclasses import dataclass
import math

import numpy as np


MODES = ("none", "deskew", "shift", "both")


class MotionUnavailable(ValueError):
    """시간 계약이나 이동 이력이 부족할 때 무보정으로 몰래 전환하지 않는다."""


@dataclass(frozen=True)
class MotionConfig:
    wheel_radius_m: float = .025
    rear_axle_x_m: float = -.0725
    lidar_x_m: float = -.03
    lidar_y_m: float = 0.
    lidar_yaw_rad: float = 0.
    gyro_bias_rad_s: float = 0.
    wheel_scale: float = 1.
    history_s: float = 2.
    max_input_gap_s: float = .05
    max_extrapolation_s: float = .03
    scan_timeout_s: float = .45

    def __post_init__(self):
        if not all(math.isfinite(float(v)) for v in vars(self).values()):
            raise ValueError("운동 설정은 유한해야 합니다.")
        for name in ("wheel_radius_m", "wheel_scale", "history_s", "max_input_gap_s",
                     "max_extrapolation_s", "scan_timeout_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.history_s <= self.scan_timeout_s:
            raise ValueError("이력 길이는 스캔 제한보다 길어야 합니다.")


class Series:
    def __init__(self, horizon):
        self.horizon = horizon
        self.times, self.values = [], []
        self.rejected = 0

    def add(self, stamp, value):
        if (not math.isfinite(stamp) or stamp < 0 or not math.isfinite(value)
                or (self.times and stamp <= self.times[-1])):
            self.rejected += 1
            return False
        self.times.append(float(stamp))
        self.values.append(float(value))
        keep = max(0, bisect_right(self.times, stamp - self.horizon) - 1)
        if keep:
            del self.times[:keep]
            del self.values[:keep]
        return True

    def available(self, now):
        end = bisect_right(self.times, now + 1e-10)
        return np.asarray(self.times[:end]), np.asarray(self.values[:end])


def advance_local(v, w, dt):
    """후륜축 기준 일정 twist의 정확한 평면 적분, w=0에서도 연속."""
    theta = w * dt
    distance = v * dt * np.sinc(theta / (2 * np.pi))
    return distance * np.cos(theta / 2), distance * np.sin(theta / 2), theta


class MotionHistory:
    def __init__(self, config=None):
        self.config = config or MotionConfig()
        self.reset()

    def reset(self):
        self.wheels = Series(self.config.history_s)
        self.gyro = Series(self.config.history_s)

    def add_wheels(self, stamp, left_rad_s, right_rad_s):
        return self.wheels.add(stamp, (left_rad_s + right_rad_s) * .5
                               * self.config.wheel_radius_m * self.config.wheel_scale)

    def add_gyro(self, stamp, yaw_rate):
        return self.gyro.add(stamp, yaw_rate - self.config.gyro_bias_rad_s)

    def poses(self, query, now):
        """현재까지 도착하고 취득 시각도 현재 이하인 표본만 사용한다.

        표본 사이 ZOH, 마지막 표본 이후 제한된 ZOH 외삽. 장시간 공백은 거부.
        query 순서가 달라도 동일 궤적에서 계산하며 yaw를 wrap하지 않는다.
        """
        q = np.atleast_1d(np.asarray(query, dtype=float))
        if not len(q) or not np.all(np.isfinite(q)) or not math.isfinite(now):
            raise MotionUnavailable("invalid_query")
        if np.max(q) > now + 1e-9:
            raise MotionUnavailable("future_query")
        vt, vv = self.wheels.available(now)
        wt, wv = self.gyro.available(now)
        if not len(vt) or not len(wt):
            raise MotionUnavailable("motion_waiting")
        start, finish = float(np.min(q)), float(np.max(q))
        if start < max(vt[0], wt[0]) - 1e-9:
            raise MotionUnavailable("history_missing")
        # 시작 시각을 지역 원점으로 잡아 오래전 공백의 영향을 다음 스캔에 전파하지 않는다.
        for stamps in (vt, wt):
            first = max(0, np.searchsorted(stamps, start, side="right") - 1)
            selected = stamps[first:np.searchsorted(stamps, finish, side="right")]
            if len(selected) > 1 and np.max(np.diff(selected)) > self.config.max_input_gap_s + 1e-9:
                raise MotionUnavailable("motion_gap")
            age = finish - selected[-1]
            if age > self.config.max_extrapolation_s + 1e-9:
                raise MotionUnavailable("motion_stale")
            if start - selected[0] > self.config.max_input_gap_s + 1e-9:
                raise MotionUnavailable("motion_gap")
        knots = np.unique(np.r_[start, vt[(vt > start) & (vt < finish)],
                                wt[(wt > start) & (wt < finish)], finish])
        vi = np.searchsorted(vt, knots, side="right") - 1
        wi = np.searchsorted(wt, knots, side="right") - 1
        v, w = vv[vi], wv[wi]
        dx, dy, da = advance_local(v[:-1], w[:-1], np.diff(knots))
        yaw = np.r_[0., np.cumsum(da)]
        c, s = np.cos(yaw[:-1]), np.sin(yaw[:-1])
        x = np.r_[0., np.cumsum(c * dx - s * dy)]
        y = np.r_[0., np.cumsum(s * dx + c * dy)]
        qi = np.maximum(0, np.searchsorted(knots, q, side="right") - 1)
        dx, dy, da = advance_local(v[qi], w[qi], q - knots[qi])
        c, s = np.cos(yaw[qi]), np.sin(yaw[qi])
        angle = yaw[qi] + da
        # 후륜축 위치에서 base_link로 이동. 후륜축의 vy=0을 차체 중앙에 적용하지 않는다.
        offset = -self.config.rear_axle_x_m
        poses = np.column_stack((x[qi] + c * dx - s * dy + offset * np.cos(angle),
                                 y[qi] + s * dx + c * dy + offset * np.sin(angle), angle))
        return poses, {"wheel_age_s": float(now - vt[-1]), "gyro_age_s": float(now - wt[-1]),
                       "extrapolation_s": max(0., finish - min(vt[-1], wt[-1]))}

    def project(self, ranges, angle_min, angle_increment, range_min, range_max,
                stamp, time_increment, now, mode="none", timing_verified=False):
        if mode not in MODES:
            raise ValueError("unknown compensation mode")
        values = (angle_min, angle_increment, range_min, range_max, stamp, time_increment, now)
        if not all(math.isfinite(float(x)) for x in values) or time_increment < 0:
            raise MotionUnavailable("scan_timing_invalid")
        r = np.asarray(ranges, dtype=float)
        if r.ndim != 1 or len(r) < 2 or range_min < 0 or range_max <= range_min:
            raise MotionUnavailable("scan_invalid")
        end = stamp + (len(r) - 1) * time_increment
        if stamp < 0 or end > now + 1e-9:
            raise MotionUnavailable("scan_future")
        if now - stamp >= self.config.scan_timeout_s:
            raise MotionUnavailable("scan_stale")
        indices = np.flatnonzero(np.isfinite(r) & (r >= range_min) & (r <= range_max))
        angles = angle_min + indices * angle_increment + self.config.lidar_yaw_rad
        points = np.column_stack((r[indices] * np.cos(angles) + self.config.lidar_x_m,
                                  r[indices] * np.sin(angles) + self.config.lidar_y_m))
        ref = now if mode in ("shift", "both") else end
        meta = {"mode": mode, "scan_stamp_s": stamp, "last_ray_stamp_s": end,
                "reference_stamp_s": ref, "oldest_observation_age_s": now - stamp,
                "newest_observation_age_s": now - end, "valid_points": len(indices),
                "coordinate_time_assumption": mode in ("none", "shift")}
        if mode == "none":
            return points, indices, meta
        if not timing_verified:
            raise MotionUnavailable("scan_timing_unverified")
        times = stamp + indices * time_increment if mode in ("deskew", "both") else np.full(len(indices), end)
        poses, diagnostic = self.poses(np.r_[times, ref], now)
        meta.update(diagnostic)
        source, target = poses[:-1], poses[-1]
        c, s = np.cos(source[:, 2]), np.sin(source[:, 2])
        wx = c * points[:, 0] - s * points[:, 1] + source[:, 0] - target[0]
        wy = s * points[:, 0] + c * points[:, 1] + source[:, 1] - target[1]
        c, s = math.cos(target[2]), math.sin(target[2])
        return np.column_stack((c * wx + s * wy, -s * wx + c * wy)), indices, meta
