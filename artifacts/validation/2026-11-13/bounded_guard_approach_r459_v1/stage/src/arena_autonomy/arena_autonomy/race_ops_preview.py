"""선택형 ROS 관측 미리보기. 장치/파일/제어 publisher를 열지 않는다."""
import json
import math
import re
import threading
import time


def topic(value):
    return isinstance(value,str) and len(value)<=200 and re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*',value)


def config(value):
    if not isinstance(value,dict) or set(value)!={'lidar','camera','lidar_type','lidar_topic','camera_topic'}:
        raise ValueError('preview config fields invalid')
    if any(type(value[k]) is not bool for k in ('lidar','camera')):
        raise ValueError('preview enable flags must be boolean')
    if value['lidar_type'] not in ('laser_scan','native') or not all(topic(value[k]) for k in ('lidar_topic','camera_topic')):
        raise ValueError('explicit ROS message type and absolute topic required')
    return dict(value)


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def lidar_points(message,kind):
    """점 사이 보간/무반환 채움/벽 분류/운동 보정 없이 원시 유효 반환만 축약한다."""
    if kind=='native':
        text=message.data
        if not isinstance(text,str) or len(text)>2_000_000:raise ValueError('native payload too large')
        raw=json.loads(text)
        if raw.get('schema')!='arena_native_lidar_v1':raise ValueError('unknown native schema')
        ranges,angles=raw['ranges_m'],raw['angles_rad']
        if len(ranges)!=len(angles):raise ValueError('native array lengths differ')
        frame=raw['source_frame_id'];minimum=raw['range_min_m'];maximum=raw['range_max_m']
        source_stamp=None;basis=str(raw.get('time_basis','unverified'))[:120]
    else:
        ranges=message.ranges;angles=[message.angle_min+i*message.angle_increment for i in range(len(ranges))]
        frame=message.header.frame_id;minimum=message.range_min;maximum=message.range_max
        source_stamp=stamp(message);basis='header_stamp_not_verified_acquisition'
    if len(ranges)>10000 or not 0<=minimum<maximum or not math.isfinite(maximum):
        raise ValueError('invalid range envelope or count')
    points=[];invalid=0
    for distance,angle in zip(ranges,angles):
        if (type(distance) not in (int,float) or type(angle) not in (int,float)
            or not math.isfinite(distance) or not math.isfinite(angle) or not minimum<=distance<=maximum or distance<=0):
            invalid+=1;continue
        points.append([distance*math.cos(angle),distance*math.sin(angle),distance])
    valid=len(points)
    if valid>720:points=[points[math.floor(i*valid/720)] for i in range(720)]
    return dict(points=points,frame=str(frame)[:200],source_stamp_s=source_stamp,time_basis=basis,
        raw_count=len(ranges),valid_count=valid,invalid_count=invalid,drawn_count=len(points),subsampled=valid>720,
        coordinates='sensor_frame_mount_unverified',motion_compensated=False,classification=False)


def jpeg_image(message):
    """worker 전용. 원영상은 저장하지 않으며 JPEG 최대 가로640 px를 반환한다."""
    import cv2
    import numpy as np
    channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1}
    count=channels.get(message.encoding)
    if count is None or not 0<message.width<=4096 or not 0<message.height<=2160:
        raise ValueError('unsupported RGB image format/size')
    if message.step<message.width*count or len(message.data)!=message.step*message.height:
        raise ValueError('invalid image step/data length')
    view=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)
    image=view[:,:message.width*count].reshape(message.height,message.width,count)
    if message.width>640:
        image=cv2.resize(image,(640,max(1,round(message.height*640/message.width))),interpolation=cv2.INTER_AREA)
    conversion={'rgb8':cv2.COLOR_RGB2BGR,'rgba8':cv2.COLOR_RGBA2BGR,'bgra8':cv2.COLOR_BGRA2BGR}
    if message.encoding in conversion:image=cv2.cvtColor(image,conversion[message.encoding])
    success,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,70])
    if not success:raise ValueError('JPEG encode failed')
    return data.tobytes(),dict(frame=str(message.header.frame_id)[:200],source_stamp_s=stamp(message),
        width=int(image.shape[1]),height=int(image.shape[0]),encoding='jpeg',acquisition_time_verified=False)


class Preview:
    LEASE_S=3.
    PERIOD={'lidar':.2,'camera':.5}

    def __init__(self,wall=time.monotonic,encode=jpeg_image):
        self.wall,self.encode=wall,encode
        self.lock=threading.RLock();self.condition=threading.Condition(self.lock)
        self.desired=dict(lidar=False,camera=False,lidar_type='native',
            lidar_topic='/scan/native',camera_topic='/camera/color/image_raw')
        self.active=None;self.client=None;self.lease_until=0.;self.generation=0
        self.subscriptions={};self.pending={};self.cache={};self.errors={};self.next_at={}
        self.worker=None;self.closed=False;self.serial=0;self.dirty=False

    def configure(self,value,client):
        selected=config(value)
        if not isinstance(client,str) or not 1<=len(client)<=64:raise ValueError('preview client required')
        with self.condition:
            self.desired=selected;self.client=client;self.lease_until=self.wall()+self.LEASE_S
            self.dirty=True
            self.condition.notify_all()
        return self.snapshot()

    def keepalive(self,client):
        with self.lock:
            if client==self.client and (self.desired['lidar'] or self.desired['camera']):
                self.lease_until=self.wall()+self.LEASE_S

    def offer(self,name,message,generation):
        with self.condition:
            if generation!=self.generation or self.active is None or not self.active[name]:return
            self.pending[name]=(message,self.wall(),generation)
            self.condition.notify_all()

    def sync(self,node):
        """ROS thread 전용: ON/OFF/설정 변경시에만 구독을 만들거나 제거한다."""
        with self.condition:
            desired=dict(self.desired)
            if self.closed or self.wall()>=self.lease_until:
                desired.update(lidar=False,camera=False)
                self.desired=desired
            self.dirty=False
            if desired==self.active:return
            self.generation+=1;generation=self.generation
            self.pending.clear();self.cache.clear();self.errors.clear();self.next_at.clear()
            for handle in self.subscriptions.values():node.destroy_subscription(handle)
            self.subscriptions.clear();self.active=desired
            if desired['lidar'] or desired['camera']:
                from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
                from sensor_msgs.msg import LaserScan,Image
                from std_msgs.msg import String
                qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
                if desired['lidar']:
                    kind=LaserScan if desired['lidar_type']=='laser_scan' else String
                    self.subscriptions['lidar']=node.create_subscription(kind,desired['lidar_topic'],
                        lambda message:self.offer('lidar',message,generation),qos)
                if desired['camera']:
                    self.subscriptions['camera']=node.create_subscription(Image,desired['camera_topic'],
                        lambda message:self.offer('camera',message,generation),qos)
                if self.worker is None or not self.worker.is_alive():
                    self.worker=threading.Thread(target=self.work,name='race_preview_encoder',daemon=True);self.worker.start()
            self.condition.notify_all()

    def needs_sync(self):
        return self.dirty or bool(self.subscriptions and self.wall()>=self.lease_until)

    def work(self):
        while True:
            with self.condition:
                if self.closed or not self.active or not (self.active['lidar'] or self.active['camera']):
                    self.worker=None
                    return
                now=self.wall()
                eligible=[name for name in self.pending if now>=self.next_at.get(name,0.)]
                if not eligible:
                    delay=min([max(.001,self.next_at.get(name,now)-now) for name in self.pending] or [.5])
                    self.condition.wait(min(.5,delay));continue
                name=eligible[0];message,received,generation=self.pending.pop(name)
                kind=self.active['lidar_type'];self.next_at[name]=now+self.PERIOD[name]
            try:
                if name=='camera':data,metadata=self.encode(message)
                else:data,metadata=None,lidar_points(message,kind)
                error=None
            except Exception as exc:
                data=None;metadata=None;error=type(exc).__name__+': '+str(exc)[:160]
            with self.lock:
                if generation!=self.generation or not self.active or not self.active[name]:continue
                if error:self.errors[name]=error;self.cache.pop(name,None)
                else:
                    self.serial+=1;self.errors.pop(name,None)
                    self.cache[name]=dict(metadata,received_wall_s=received,sequence=self.serial,data=data)

    def snapshot(self):
        with self.lock:
            result=dict(config=dict(self.desired),active=bool(self.active and (self.active['lidar'] or self.active['camera'])),
                lease_remaining_s=max(0.,self.lease_until-self.wall()),errors=dict(self.errors),
                lidar=None,camera=None,default_off=True)
            for name,row in self.cache.items():
                age=max(0.,self.wall()-row['received_wall_s'])
                result[name]={k:v for k,v in row.items() if k not in ('data','received_wall_s')}
                result[name].update(receipt_age_s=age,stale=age>1.)
            return result

    def jpeg(self):
        with self.lock:
            row=self.cache.get('camera')
            return None if row is None or self.wall()-row['received_wall_s']>1. else row['data']

    def close(self,node):
        with self.condition:
            worker=self.worker;self.closed=True;self.condition.notify_all()
        self.sync(node)
        if worker is not None:worker.join(timeout=2.)
