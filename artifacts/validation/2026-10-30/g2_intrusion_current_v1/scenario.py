"""평가 전용 상대차 움직임. 정답 자차 위치는 장면 시작에만 쓰며 제어기 입력이 아니다."""
import math
from g0_geometry import TruthBox,Footprint

class Scenario:
    def __init__(self,kind,trigger_x,blocked=False):
        self.kind,self.trigger_x,self.blocked=kind,trigger_x,blocked
        self.start=None
    def observe_ego(self,now,rear_pose):
        x=rear_pose[0]+.0675*math.cos(rear_pose[2])
        if self.start is None and x>=self.trigger_x:self.start=now
    def pose_at(self,at):
        dt=0. if self.start is None else max(0.,at-self.start)
        if self.kind=='slide':
            return 1.2,-min(.20,.15*dt),0.
        # 1m/s²로 .5m/s까지 가속, .2초 유지, 같은 감속으로 정지한다.
        if dt<=.5:s=.5*dt*dt
        elif dt<=.7:s=.125+.5*(dt-.5)
        elif dt<=1.2:
            d=dt-.7;s=.225+.5*d-.5*d*d
        else:s=.35
        radius=.35;arc=min(s,radius*.6);angle=arc/radius;rest=s-arc
        return 1.2+radius*math.sin(angle)+rest*math.cos(angle),-radius*(1-math.cos(angle))-rest*math.sin(angle),-angle
    def boxes(self,at):
        walls=[TruthBox(2.,sign*.45,.4,Footprint(8.,.05),label='wall') for sign in (-1,1)]
        walls += [TruthBox(x,0.,.4,Footprint(.05,2.),label='wall') for x in (-2.025,6.025)]
        if self.blocked:walls += [TruthBox(1.85,-.26,.12)]
        x,y,yaw=self.pose_at(at)
        return walls+[TruthBox(x,y,.12,yaw_rad=yaw)]
    def description(self):
        return dict(kind=self.kind,trigger_x_m=self.trigger_x,start_sim_s=self.start,
          blocked_forward=self.blocked,trigger_scope='evaluation actor only, not controller input',
          shape='20x15x12cm box',physical_path=self.kind=='pullout',
          acceleration_mps2=1. if self.kind=='pullout' else None,
          speed_cap_mps=.5 if self.kind=='pullout' else .15,
          note='scripted assessment assumption, not actual competitor behavior')
