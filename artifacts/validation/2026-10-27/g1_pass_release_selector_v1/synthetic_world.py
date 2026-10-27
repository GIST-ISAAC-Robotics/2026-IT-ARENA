"""이동 자차 직선로 합성 장면. 정답 물체는 광선 생성과 사후 평가에만 쓰고 제어기 입력으로 넘기지 않는다.

각 광선은 자신의 취득 시각의 자차·물체 위치에서 투사하고, `scene_from_scan`에 취득→기준 SE(2)를 넘겨
운동 보정 원점을 만든다(실제 `motion_scene`과 같은 형태). 후방 ±30° 마스크도 같은 함수가 적용한다.
실제 C1 분포·회전 순서·Gazebo 물리·차량 동역학이 아니다.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_scene import scene_from_scan

N_RAYS = 500
SCAN_S = .1
SENSOR_X = .06
WALL_Y = .425
WALL_T = .05


@dataclass
class Body:
    """직선 운동 물체. trajectory(t) -> (x, y) 세계 중심, 자세 0. length=x 방향."""
    length: float
    width: float
    trajectory: object
    name: str = 'object'

    def polygon(self, t):
        x, y = self.trajectory(t)
        hl, hw = self.length/2, self.width/2
        return np.array([[x-hl, y-hw], [x+hl, y-hw], [x+hl, y+hw], [x-hl, y+hw]], dtype=float)


def piecewise(t0, x0, segments):
    """[(끝 시각, 속도)] 구간 등속. t0 이전은 x0에 정지."""
    def at(t):
        x, start = x0, t0
        for end, speed in segments:
            if t <= start:
                break
            x += speed*(min(t, end)-start)
            start = end
        return x
    return at


@dataclass
class World:
    ego_speed: float
    ego_y: float
    bodies: list = field(default_factory=list)
    ego_x0: float = 0.
    dropout: object = None   # (t, angle_deg) -> True면 그 광선을 무응답으로 둔다(합성 결측)

    def ego_x(self, t):
        return self.ego_x0+self.ego_speed*t

    def walls(self):
        # 실내 코스처럼 세계 고정 끝벽을 둔다(12 m 밖 무응답이 전방 영역 판정을 막지 않게). tests/test_lateral_intent와 같은 의도.
        end0, end1 = -5., 8.
        return [np.array([[end0, WALL_Y], [end1, WALL_Y], [end1, WALL_Y+WALL_T], [end0, WALL_Y+WALL_T]]),
                np.array([[end0, -WALL_Y-WALL_T], [end1, -WALL_Y-WALL_T], [end1, -WALL_Y], [end0, -WALL_Y]]),
                np.array([[end1, -.5], [end1+WALL_T, -.5], [end1+WALL_T, .5], [end1, .5]]),
                np.array([[end0-WALL_T, -.5], [end0, -.5], [end0, .5], [end0-WALL_T, .5]])]

    def ranges(self, t_ref, bodies=None):
        bodies = self.bodies if bodies is None else bodies
        stamps = t_ref-SCAN_S+np.arange(N_RAYS)*(SCAN_S/N_RAYS)
        angles = -math.pi+np.arange(N_RAYS)*2*math.pi/N_RAYS
        out = np.full(N_RAYS, np.inf)
        walls = self.walls()
        for i, (t, a) in enumerate(zip(stamps, angles)):
            origin = np.array([self.ego_x(t)+SENSOR_X, self.ego_y])
            d = np.array([math.cos(a), math.sin(a)])
            best = math.inf
            for poly in walls+[b.polygon(t) for b in bodies]:
                p, q = poly, np.roll(poly, -1, axis=0)
                e = q-p
                denom = d[0]*e[:, 1]-d[1]*e[:, 0]
                w = p-origin
                with np.errstate(divide='ignore', invalid='ignore'):
                    s = (w[:, 0]*e[:, 1]-w[:, 1]*e[:, 0])/denom
                    u = (w[:, 0]*d[1]-w[:, 1]*d[0])/denom
                ok = (np.abs(denom) > 1e-12) & (s > 1e-9) & (u >= 0) & (u <= 1)
                if np.any(ok):
                    best = min(best, float(np.min(s[ok])))
            if self.dropout is not None and self.dropout(t, math.degrees(a)):
                best = math.nan
            out[i] = best if best <= 12. else math.inf
        return out, stamps

    def scene(self, t_ref, bodies=None):
        ranges, stamps = self.ranges(t_ref, bodies)
        poses = [(self.ego_x(t)-self.ego_x(t_ref), 0., 0.) for t in stamps]
        return scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=2*math.pi/N_RAYS,
                               range_min=.05, range_max=12., acquired_at_s=stamps.tolist(), t_ref_s=t_ref,
                               acquisition_to_ref_se2=poses)

    def history(self, end, gap=None):
        h = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=SENSOR_X, history_s=5.))
        for time in np.arange(0., end+.0001, .01):
            if gap is not None and gap[0] <= time < gap[1]:
                continue
            h.add_mean_wheel(float(time), self.ego_speed/.0325)
            h.add_gyro(float(time), 0.)
        return h

    def path(self):
        return np.column_stack((np.linspace(-.25, 1.6, 36), np.full(36, -self.ego_y)))

    def relative(self, body, t):
        """사후 평가: 자차 중심 기준 물체 중심(앞+, 왼쪽+)과 차체 사각형 간격."""
        x, y = body.trajectory(t)
        dx, dy = x-self.ego_x(t), y-self.ego_y
        gx = max(0., abs(dx)-(.10+body.length/2))
        gy = max(0., abs(dy)-(.075+body.width/2))
        return {'forward_m': dx, 'left_m': dy, 'body_gap_m': math.hypot(gx, gy)}


def same_scan(a, b, tol=1e-12):
    """두 장면의 광선 상태·끝점·시각이 모두 같은지(구별불가 장면 확인)."""
    if len(a.rays) != len(b.rays):
        return False
    for r, s in zip(a.rays, b.rays):
        if r.status != s.status or r.acquired_at_s != s.acquired_at_s:
            return False
        if r.hit_xy_m is not None and max(abs(r.hit_xy_m[0]-s.hit_xy_m[0]), abs(r.hit_xy_m[1]-s.hit_xy_m[1])) > tol:
            return False
    return True
