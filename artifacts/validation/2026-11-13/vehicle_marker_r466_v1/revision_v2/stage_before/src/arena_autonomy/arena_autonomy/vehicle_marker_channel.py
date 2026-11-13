"""선택 packet의 공통 단위/시계 계약과 ROS 입출력. 빈 입력은 기존 결과를 유지한다."""
import copy
import json
import math
from pathlib import Path
import time

import numpy as np


def checked_packet(packet,profile,now,clock_domain):
    """외부 packet 오류는 optional unavailable. LiDAR 입력/STOP 상태는 변경하지 않는다."""
    try:
        if not isinstance(packet,dict) or packet.get('mode')!='approximate_v1':return None
        from arena_autonomy.vehicle_marker_geometry import rectangle
        if (packet['profile_id']!=profile['profile_id'] or packet['frame_id']!='base_link'
                or packet['source_frame_id']!=profile['camera']['optical_frame_id']
                or packet['clock_domain']!=clock_domain
                or packet['time_basis']!='image_header_unverified_exposure'
                or packet.get('timing_verified') is not False
                or packet.get('purpose')!='occupancy_assist'):return None
        stamp=packet['source_stamp_s'];epoch=packet['vision_epoch']
        if type(stamp) not in (int,float) or not math.isfinite(stamp):return None
        if type(epoch) is not int or epoch<0 or packet['reference_s']!=stamp:return None
        age=now-stamp
        if not math.isfinite(age) or not 0<=age<profile.get('max_age_s',.25):return None
        targets={(x['dictionary'],x['id']):x for x in profile['targets']}
        if type(packet['regions']) is not list or len(packet['regions'])>len(targets):return None
        seen=set()
        for region in packet['regions']:
            key=region['dictionary'],region['id']
            if key not in targets or key in seen:return None
            seen.add(key);target=targets[key]
            for field in ('black_code_size_m','printed_image_size_m'):
                if region[field]!=target[field]:return None
            if region['width_m']!=.15 or region['length_m']!=.20:return None
            center=np.asarray(region['rear_center_xy_m'],dtype=float);heading=region['heading_rad']
            poly=np.asarray(region['polygon_xy_m'],dtype=float)
            if center.shape!=(2,) or not np.isfinite(center).all() or not math.isfinite(heading):return None
            if (poly.shape!=(4,2) or not np.isfinite(poly).all()
                    or not np.allclose(poly,rectangle(center,heading),rtol=0,atol=1e-8)):return None
        result=copy.deepcopy(packet)
        result.update(fresh=True,age_s=age,max_age_s=profile.get('max_age_s',.25))
        return result
    except (KeyError,TypeError,ValueError,OverflowError):return None


class MarkerChannel:
    """같은 ROS 시계의 영상 결과 한 개만 보관. OFF면 publisher/subscription 없음."""
    def __init__(self,node,*,producer):
        self.node=node;self.producer=producer;self.profile=None
        self.latest=None;self.received_wall=0.;self.floor=-math.inf;self.last_clock=None
        path=str(node.declare_parameter('vehicle_marker_profile_path','').value)
        self.clock_domain='ros_sim' if node.get_parameter('use_sim_time').value else 'ros_system'
        if not path:return
        from arena_autonomy.vehicle_marker_geometry import validate_profile
        self.profile=validate_profile(json.loads(Path(path).read_text(encoding='utf-8')))
        if self.profile.get('purpose')!='occupancy_assist':raise ValueError('runtime requires occupancy-assist profile')
        from std_msgs.msg import String
        self.message_type=String
        if producer:self.publisher=node.create_publisher(String,'/perception/vehicle_marker_regions',1)
        else:node.create_subscription(String,'/perception/vehicle_marker_regions',self.receive,1)

    def now(self):return self.node.get_clock().now().nanoseconds*1e-9

    def reset(self,boundary=-math.inf):
        self.latest=None;self.floor=boundary

    def receive(self,message):
        now=self.now()
        if self.last_clock is not None and now<self.last_clock:self.reset()
        self.last_clock=now
        try:raw=json.loads(message.data)
        except (ValueError,TypeError):return
        value=checked_packet(raw,self.profile,now,self.clock_domain)
        if value is None or value['source_stamp_s']<self.floor:return
        if self.latest is not None and value['source_stamp_s']<=self.latest['source_stamp_s']:return
        self.latest=value;self.received_wall=time.monotonic()

    def publish(self,pipeline):
        if self.profile is None:return
        packet=pipeline.vehicle_regions.snapshot(self.now(),pipeline.epoch)
        if not packet.get('fresh'):return
        packet.update(clock_domain=self.clock_domain,timing_verified=False)
        self.publisher.publish(self.message_type(data=json.dumps(packet,allow_nan=False)))

    def snapshot(self,now,pipeline=None):
        if self.profile is None:return None
        if self.producer:
            if pipeline is None or pipeline.vehicle_regions is None:return None
            packet=pipeline.vehicle_regions.snapshot(now,pipeline.epoch)
            if not packet.get('fresh'):return None
            packet.update(clock_domain=self.clock_domain,timing_verified=False)
        else:
            if self.last_clock is not None and now<self.last_clock:self.reset()
            self.last_clock=now
            if time.monotonic()-self.received_wall>self.node.wall_timeout:return None
            packet=self.latest
        return checked_packet(packet,self.profile,now,self.clock_domain)


def node_snapshot(node,now):
    channel=getattr(node,'vehicle_marker_channel',None)
    if channel is None:return None
    return channel.snapshot(now,getattr(node,'vision_pipeline',None))


def snapshot_options(node,now):
    packet=node_snapshot(node,now)
    return {} if packet is None else {'vehicle_marker_regions':packet}
