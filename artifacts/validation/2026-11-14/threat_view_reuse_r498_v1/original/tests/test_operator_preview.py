"""미리보기 단위/수명 검사. 장치 입력/제어 성능 검증은 별도다."""
import json
import math
import sys
import threading
import time
from types import SimpleNamespace as N

import pytest
import numpy as np

from arena_autonomy.race_ops_preview import Preview, config, lidar_points, jpeg_image


def image(width=848,height=480,encoding='rgb8'):
    data=np.zeros((height,width,3),np.uint8);data[:,:,0]=220
    return N(width=width,height=height,step=width*3,encoding=encoding,data=data.tobytes(),
        header=N(frame_id='b0390_receive_time_unverified',stamp=N(sec=10,nanosec=0)))


def scan():
    return N(ranges=[1.,float('nan'),float('inf'),0.,2.],angle_min=0.,angle_increment=math.pi/2,
        range_min=.05,range_max=12.,header=N(frame_id='c1_sensor',stamp=N(sec=10,nanosec=0)))


def selected(**kw):
    return dict(lidar=True,camera=False,lidar_type='laser_scan',lidar_topic='/scan',camera_topic='/rgb',**kw)


def test_raw_returns_do_not_fill_or_classify_nonreturns():
    row=lidar_points(scan(),'laser_scan')
    assert row['raw_count']==5 and row['valid_count']==2 and row['invalid_count']==3
    assert row['drawn_count']==2 and row['points'][0]==[1.,0.,1.]
    assert row['motion_compensated'] is False and row['classification'] is False
    assert row['coordinates']=='sensor_frame_mount_unverified'


def test_native_explicit_shape_and_point_budget():
    raw=dict(schema='arena_native_lidar_v1',ranges_m=[1.]*1500+[None,'+inf'],angles_rad=[0.]*1502,
        range_min_m=.05,range_max_m=12.,source_frame_id='raw_c1',time_basis='unverified_estimate')
    row=lidar_points(N(data=json.dumps(raw)),'native')
    assert row['drawn_count']<=720 and row['valid_count']==1500 and row['invalid_count']==2
    assert row['time_basis']=='unverified_estimate' and row['subsampled']
    raw['angles_rad']=[]
    with pytest.raises(ValueError):lidar_points(N(data=json.dumps(raw)),'native')


@pytest.mark.parametrize('change',[{'lidar_type':'guess'},{'lidar_topic':'/dev/ttyUSB0;run'},
    {'camera_topic':'http://camera'},{'camera':1},{'lidar_topic':'relative'}])
def test_preview_has_no_device_or_url_input(change):
    with pytest.raises(ValueError):config({**selected(),**change})


def test_rgb_jpeg_downsample_and_metadata_without_file_output():
    import cv2
    data,meta=jpeg_image(image())
    decoded=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
    assert decoded.shape[:2]==(362,640)
    assert meta['width']==640 and meta['frame']=='b0390_receive_time_unverified'
    assert meta['acquisition_time_verified'] is False
    assert decoded[:,:,2].mean()>200 and decoded[:,:,0].mean()<10


@pytest.fixture
def fake_ros(monkeypatch):
    monkeypatch.setitem(sys.modules,'rclpy.qos',N(QoSProfile=lambda **kw:kw,
        ReliabilityPolicy=N(BEST_EFFORT='best'),DurabilityPolicy=N(VOLATILE='volatile')))
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',N(LaserScan=object,Image=object))
    monkeypatch.setitem(sys.modules,'std_msgs.msg',N(String=object))
    class Node:
        def __init__(self):self.subs=[];self.destroyed=[]
        def create_subscription(self,kind,topic,callback,qos):
            handle=N(kind=kind,topic=topic,callback=callback,qos=qos);self.subs.append(handle);return handle
        def destroy_subscription(self,handle):self.subs.remove(handle);self.destroyed.append(handle)
    return Node()


def until(predicate):
    deadline=time.monotonic()+1.
    while not predicate() and time.monotonic()<deadline:time.sleep(.005)
    assert predicate()


def test_off_has_no_worker_or_subscription_and_lease_destroys_everything(fake_ros):
    clock=[10.];preview=Preview(wall=lambda:clock[0])
    assert not preview.needs_sync() and preview.worker is None and fake_ros.subs==[]
    preview.configure(selected(),'viewer');preview.sync(fake_ros)
    assert len(fake_ros.subs)==1 and fake_ros.subs[0].qos['depth']==1
    fake_ros.subs[0].callback(scan());until(lambda:preview.snapshot()['lidar'] is not None)
    clock[0]=13.01;assert preview.needs_sync();preview.sync(fake_ros)
    assert fake_ros.subs==[] and preview.snapshot()['lidar'] is None
    until(lambda:preview.worker is None);assert not preview.needs_sync()
    preview.close(fake_ros)


def test_generation_drops_encoding_completed_after_off_and_only_latest_pending(fake_ros):
    entered=threading.Event();release=threading.Event();calls=[]
    def encode(message):
        calls.append(message);entered.set();release.wait(1.)
        return b'jpeg',dict(frame='fixture',width=1,height=1)
    preview=Preview(encode=encode)
    preview.configure({**selected(), 'lidar':False,'camera':True},'viewer');preview.sync(fake_ros)
    callback=fake_ros.subs[0].callback
    callback('first');assert entered.wait(1.)
    callback('middle');callback('last')
    assert len(preview.pending)==1 and preview.pending['camera'][0]=='last'
    preview.configure({**selected(),'lidar':False,'camera':False},'viewer');preview.sync(fake_ros)
    release.set();until(lambda:preview.worker is None)
    assert calls==['first'] and preview.jpeg() is None and preview.pending=={} and fake_ros.subs==[]
    preview.close(fake_ros)


def test_only_owner_state_poll_renews_preview_and_stale_is_explicit(fake_ros):
    clock=[10.];preview=Preview(wall=lambda:clock[0])
    preview.configure(selected(),'one');preview.sync(fake_ros)
    fake_ros.subs[0].callback(scan());until(lambda:preview.snapshot()['lidar'] is not None)
    clock[0]=11.2;preview.keepalive('other')
    assert preview.lease_until==13. and preview.snapshot()['lidar']['stale']
    preview.keepalive('one');assert preview.lease_until==14.2
    preview.close(fake_ros)
