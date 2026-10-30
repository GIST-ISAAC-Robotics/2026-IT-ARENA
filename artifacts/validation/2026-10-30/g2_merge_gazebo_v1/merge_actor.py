"""Gazebo 평가기에만 연결하는 S자 상대차. 제어 노드는 이 모듈을 쓰지 않는다."""
import math
from scenario import Scenario


class MergeActor:
    def __init__(self):
        self.scenario=Scenario('s_merge',1.15)
        self.frame=None
        self.observed_trigger=None

    def bind(self, centerline, s0):
        x,y,yaw=centerline.pose(s0-1.2)
        new=(float(x),float(y),float(yaw))
        if self.frame is not None and max(abs(a-b) for a,b in zip(self.frame,new))>1e-10:
            raise ValueError('evaluation frame changed')
        self.frame=new

    def world(self, local):
        x,y,yaw=local;ox,oy,a=self.frame
        return ox+x*math.cos(a)-y*math.sin(a),oy+x*math.sin(a)+y*math.cos(a),a+yaw

    def pose(self, config, centerline, s0, case, elapsed):
        self.bind(centerline,s0)
        return self.world(self.scenario.pose_at(elapsed))

    def observe(self, elapsed, ego_center):
        if elapsed<0 or self.frame is None:return
        x,y,yaw=ego_center;ox,oy,a=self.frame
        local_x=(x-ox)*math.cos(a)+(y-oy)*math.sin(a)
        local_y=-(x-ox)*math.sin(a)+(y-oy)*math.cos(a)
        relative_yaw=yaw-a
        before=self.scenario.start
        self.scenario.observe_ego(elapsed,(local_x-.0675*math.cos(relative_yaw),
            local_y-.0675*math.sin(relative_yaw),relative_yaw))
        if before is None and self.scenario.start is not None:
            self.observed_trigger=dict(elapsed_s=elapsed,ego_center_local=[local_x,local_y,relative_yaw])

    def speed_at(self, elapsed):
        if self.scenario.start is None:return 0.
        dt=elapsed-self.scenario.start
        if dt<=0 or dt>=5.5:return 0.
        if dt<.5:return dt
        if dt<=5:return .5
        return 5.5-dt

    def validate_route(self, config, centerline, case, s0, road=None, obstacles=()):
        from shapely.geometry import Polygon
        from shapely.strtree import STRtree
        from g0_geometry import Footprint
        self.bind(centerline,s0)
        if road is None:raise ValueError('route has no road evidence')
        original=self.scenario.start
        self.scenario.start=0.
        tree=STRtree(obstacles);corridor=road.buffer(.20)
        maximum_outside=0.
        try:
            for k in range(551):
                pose=self.world(self.scenario.pose_at(k*.01))
                body=Polygon(Footprint().polygon(*pose))
                maximum_outside=max(maximum_outside,float(body.difference(road).area))
                if body.difference(corridor).area>1e-6 or any(body.intersection(obstacles[int(i)]).area>1e-6 for i in tree.query(body)):
                    raise ValueError(f'S-merge actor touches wall/outside corridor at {k*.01}')
            final=self.scenario.pose_at(5.5)
            end_s=s0+final[0]-1.2
            corner=centerline.unwrap(config['first_corner_s_m'],s0)
            if corner<s0:corner+=centerline.lap
            if end_s>corner-1.:raise ValueError('actor leaves registered straight')
        finally:
            self.scenario.start=original
        return dict(kind='evaluation_s_merge',actor_start_s_m=s0,actor_end_s_m=end_s,
            first_corner_s_m=corner,body_samples=551,maximum_actor_body_outside_road_m2=maximum_outside,
            wall_clear=True,scope='evaluation actor geometry within road/grass corridor; not autonomous rival')

    def receipt(self):
        return dict(scenario=self.scenario.description(),frame=self.frame,trigger=self.observed_trigger,
            controller_input=False,scope='evaluation-only trigger and scripted actor; no feedback into driving controller')


TRIAL=MergeActor()
