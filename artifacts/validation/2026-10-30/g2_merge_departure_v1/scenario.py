"""평가 전용 후륜 기준 bicycle 곡선. 제어기는 합성 센서만 받는다."""
import math
from g0_geometry import TruthBox, Footprint


class Scenario:
    radius = .35
    angle = .6
    rear_offset = .0675

    def __init__(self, kind, trigger_x, blocked=False):
        if kind != 's_merge' or blocked:
            raise ValueError('unsupported registered scene')
        self.trigger_x = trigger_x
        self.start = None

    def observe_ego(self, now, rear_pose):
        center_x = rear_pose[0]+self.rear_offset*math.cos(rear_pose[2])
        if self.start is None and center_x >= self.trigger_x:
            self.start = now

    @staticmethod
    def distance_at(dt):
        # 1m/s²로0.5m/s까지, 4.5초 정속, 같은 감속으로 정지. 총거리2.5m.
        dt = max(0., dt)
        if dt <= .5:
            return .5*dt*dt
        if dt <= 5.:
            return .125+.5*(dt-.5)
        if dt <= 5.5:
            d = dt-5.
            return 2.375+.5*d-.5*d*d
        return 2.5

    def pose_at(self, at):
        s = self.distance_at(0. if self.start is None else at-self.start)
        r, a = self.radius, self.angle
        first = r*a
        if s <= first:
            theta = s/r
            dx, y, yaw = r*math.sin(theta), -r*(1-math.cos(theta)), -theta
        elif s <= 2*first:
            theta = (s-first)/r
            dx = r*math.sin(a)+r*(math.sin(-a+theta)+math.sin(a))
            y = -r*(1-math.cos(a))+r*(math.cos(a)-math.cos(-a+theta))
            yaw = -a+theta
        else:
            dx = 2*r*math.sin(a)+s-2*first
            y, yaw = -2*r*(1-math.cos(a)), 0.
        # 후륜 축의 원호 길이에서 차체 중심을 계산한다. 초기 중심x=1.2m.
        return 1.2-self.rear_offset+dx+self.rear_offset*math.cos(yaw), y+self.rear_offset*math.sin(yaw), yaw

    def boxes(self, at):
        walls = [TruthBox(2., side*.45, .4, Footprint(8., .05), label='wall') for side in (-1,1)]
        walls += [TruthBox(x, 0., .4, Footprint(.05, 2.), label='wall') for x in (-2.025,6.025)]
        x, y, yaw = self.pose_at(at)
        return walls+[TruthBox(x, y, .12, yaw_rad=yaw)]

    def description(self):
        return dict(kind='s_merge', trigger_x_m=self.trigger_x, start_sim_s=self.start,
            trigger_scope='evaluation actor only, never controller input',
            scope='rear-axle kinematic bicycle path; instantaneous curvature switches; no tyre dynamics',
            acceleration_mps2=1., speed_cap_mps=.5, total_rear_axle_distance_m=2.5,
            arc_radius_m=self.radius, each_arc_yaw_rad=self.angle,
            final_lateral_m=-2*self.radius*(1-math.cos(self.angle)))
