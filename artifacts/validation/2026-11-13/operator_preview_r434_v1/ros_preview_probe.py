"""실제 ROS/기존 인증 HTTP의 합성 입력 대표. 장치·주행·원영상 저장 없음."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from arena_autonomy.race_ops_node import RaceOps


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    out=parser.parse_args().output.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'token.txt').write_text('fixture-operator-token',encoding='utf-8')
    (out/'view-token.txt').write_text('fixture-viewer-token',encoding='utf-8')
    rclpy.init(args=['--ros-args','-p','port:=0','-p','vehicle_id:=fixture',
        '-p',f'token_path:={out/"token.txt"}','-p',f'view_token_path:={out/"view-token.txt"}'])
    ops=RaceOps();source=Node('preview_fixture');executor=SingleThreadedExecutor()
    executor.add_node(ops);executor.add_node(source)
    native=source.create_publisher(String,'/fixture/native',qos_profile_sensor_data)
    laser=source.create_publisher(LaserScan,'/fixture/scan',qos_profile_sensor_data)
    rgb=source.create_publisher(Image,'/fixture/rgb',qos_profile_sensor_data)
    emitting=[True];errors=[];stopping=threading.Event();result={'passed':False}
    metrics=dict(published_native=0,published_camera=0,received_lidar=0,received_camera=0,
        encoded_camera=0,max_pending_total=0,max_pending_camera=0)
    original_offer=ops.preview.offer;original_encode=ops.preview.encode
    def observed_offer(name,message,generation):
        original_offer(name,message,generation)
        with ops.preview.lock:
            metrics['received_'+name]+=1
            metrics['max_pending_total']=max(metrics['max_pending_total'],len(ops.preview.pending))
            metrics['max_pending_camera']=max(metrics['max_pending_camera'],int('camera' in ops.preview.pending))
    def observed_encode(message):
        answer=original_encode(message)
        with ops.preview.lock:metrics['encoded_camera']+=1
        return answer
    ops.preview.offer=observed_offer;ops.preview.encode=observed_encode
    feed_count=[0]
    pixel=np.zeros((480,848,3),np.uint8);pixel[:,:,0]=220
    def feed():
        if not emitting[0]:return
        now=source.get_clock().now().to_msg()
        feed_count[0]+=1
        if feed_count[0]%2==0:
            raw=dict(schema='arena_native_lidar_v1',ranges_m=[1.]*722+[None,'+inf',0.],
                angles_rad=[i*math.tau/725 for i in range(725)],source_frame_id='fixture_c1_raw',
                range_min_m=.05,range_max_m=12.,time_basis='unverified_estimate')
            native.publish(String(data=json.dumps(raw)));metrics['published_native']+=1
            scan=LaserScan();scan.header.stamp=now;scan.header.frame_id='fixture_laser_raw'
            scan.angle_min=-math.pi;scan.angle_increment=math.tau/500;scan.angle_max=math.pi-scan.angle_increment
            scan.range_min=.05;scan.range_max=12.;scan.ranges=[2.]*498+[float('nan'),float('inf')];laser.publish(scan)
        image=Image();image.header.stamp=now;image.header.frame_id='fixture_rgb'
        image.encoding='rgb8';image.width=848;image.height=480;image.step=848*3;image.data=pixel.tobytes();rgb.publish(image)
        metrics['published_camera']+=1
    source.create_timer(.05,feed)
    def spin():
        try:
            while not stopping.is_set():executor.spin_once(timeout_sec=.02)
        except BaseException as exc:errors.append(repr(exc))
    thread=threading.Thread(target=spin);thread.start()
    port=ops.server.server_address[1]
    def http(path,body=None,token='fixture-viewer-token'):
        request=urllib.request.Request(f'http://127.0.0.1:{port}'+path,
            data=None if body is None else json.dumps(body).encode(),headers={
                'X-Arena-Token':token,'X-Arena-Vehicle':'fixture','Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=2.) as response:
                data=response.read();return response.status,data if response.headers.get_content_type()=='image/jpeg' else json.loads(data)
        except urllib.error.HTTPError as error:return error.code,json.loads(error.read())
    def state(owner=True):return http('/api/state'+('?client=preview-fixture' if owner else ''))[1]
    def wait(predicate,seconds=4.):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            value=predicate()
            if value:return value
            if errors:raise AssertionError(errors)
            time.sleep(.04)
        raise AssertionError('condition timeout')
    selected=dict(lidar=True,camera=True,lidar_type='native',lidar_topic='/fixture/native',camera_topic='/fixture/rgb')
    def configure(cfg):
        status,body=http('/api/preview',dict(client='preview-fixture',config=cfg));assert status==200,(status,body)
    try:
        initial=state()
        assert not initial['preview']['active'] and ops.preview.worker is None and not ops.preview.subscriptions
        assert source.count_subscribers('/fixture/native')==0 and source.count_subscribers('/fixture/rgb')==0
        assert http('/api/preview.jpg',token='wrong-token')[0]==401
        assert http('/api/command',dict(op='stop'))[0]==403
        configure(selected)
        def ready():
            row=state()['preview']
            return row if row['lidar'] and row['camera'] else None
        first=wait(ready)
        assert first['lidar']['drawn_count']==720 and first['lidar']['invalid_count']==3
        assert first['lidar']['frame']=='fixture_c1_raw' and not first['lidar']['motion_compensated']
        code,jpeg=http('/api/preview.jpg');assert code==200
        decoded=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        assert decoded.shape[:2]==(362,640) and decoded[:,:,2].mean()>200
        observed={'lidar':set(),'camera':set()};started=time.monotonic();before=dict(metrics)
        while time.monotonic()-started<1.05:
            row=state()['preview']
            for name in observed:observed[name].add(row[name]['sequence'])
            time.sleep(.05)
        assert len(observed['lidar'])<=7 and len(observed['camera'])<=4
        window={k:metrics[k]-before[k] for k in ('published_native','published_camera','received_lidar','received_camera','encoded_camera')}
        window['seconds']=time.monotonic()-started
        emitting[0]=False
        def stale_camera():
            row=state()['preview']
            return row if row['camera'] and row['camera']['stale'] else None
        stale=wait(stale_camera)
        assert http('/api/preview.jpg')[0]==404
        result.update(default_off=True,unauthorized_jpeg_rejected=True,viewer_command_rejected=True,
            native={k:v for k,v in first['lidar'].items() if k!='points'},jpeg_shape=list(decoded.shape),
            observed_sequences={k:len(v) for k,v in observed.items()},rate_window=window,
            input_rates_hz={'native':10,'rgb':20},stale_visible=stale['camera']['stale'],stale_jpeg_status=404)
        emitting[0]=True
        configure({**selected,'lidar_type':'laser_scan','lidar_topic':'/fixture/scan','camera':False})
        def laser_ready():
            row=state()['preview']['lidar']
            return row if row and row['frame']=='fixture_laser_raw' else None
        scan=wait(laser_ready)
        assert scan['drawn_count']==498 and scan['invalid_count']==2
        assert '/drive' not in [topic for topic,types in ops.get_publisher_names_and_types_by_node('race_ops','/')]
        assert '/drive/safe' not in [topic for topic,types in ops.get_publisher_names_and_types_by_node('race_ops','/')]
        configure({**selected,'lidar':False,'camera':False})
        wait(lambda:not ops.preview.subscriptions and ops.preview.worker is None)
        wait(lambda:source.count_subscribers('/fixture/native')==0 and source.count_subscribers('/fixture/scan')==0 and source.count_subscribers('/fixture/rgb')==0)
        result['off_subscribers']={topic:source.count_subscribers(topic) for topic in ('/fixture/native','/fixture/scan','/fixture/rgb')}
        assert state()['preview']['lidar'] is None and http('/api/preview.jpg')[0]==404
        configure({**selected,'camera':False})
        wait(lambda:state()['preview']['lidar'])
        wait(lambda:not state(False)['preview']['active'],seconds=4.)
        assert not ops.preview.subscriptions and ops.preview.jpeg() is None
        assert metrics['max_pending_total']<=2 and metrics['max_pending_camera']<=1
        result.update(laser_scan_count=scan['drawn_count'],explicit_off_unsubscribed=True,
            lease_expired_unsubscribed=True,no_drive_publishers=True,passed=True)
    except BaseException as exc:
        result['error']=repr(exc)
    finally:
        stopping.set();thread.join(timeout=3.)
        executor.remove_node(ops);executor.remove_node(source)
        ops.destroy_node();source.destroy_node();executor.shutdown();rclpy.shutdown()
        result['executor_errors']=errors;result['spin_thread_alive']=thread.is_alive();result['metrics']=metrics
        result['worker_alive']=bool(ops.preview.worker and ops.preview.worker.is_alive())
        result['passed']=result['passed'] and not errors and not thread.is_alive() and not result['worker_alive']
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
