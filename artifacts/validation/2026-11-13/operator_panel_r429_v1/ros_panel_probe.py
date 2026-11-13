"""실제 ROS/HTTP/독립 guard + 가짜 MCU 서비스. 장치·Gazebo·펌웨어를 사용하지 않는다."""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
import urllib.request
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.race_ops_node import RaceOps


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    token=out/'token.txt';token.write_text('local-fixture-token',encoding='utf-8')
    rclpy.init(args=['--ros-args','-p','operation_mode:=manual','-p','port:=0',
        '-p','vehicle_id:=fixture','-p',f'token_path:={token}',
        '-p',f'manual_output_root:={out/"jogs"}','-p',f'log_path:={out/"ops.jsonl"}'])
    ops=RaceOps();guard=LidarSafety();host=Node('actuation_host')
    executor=SingleThreadedExecutor()
    for node in (ops,guard,host):executor.add_node(node)
    status=host.create_publisher(String,'/actuation/status',10)
    laser=host.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    case=dict(v=1,boot='fixture',generation=1,owner='fake-host',state='ESTOP_LATCHED',
        estop_held=False,feedback_valid=True,output_ready=True,target_speed_mps=0.)
    commands=[];safe=[];services=[];errors=[]
    def on_drive(msg):commands.append([msg.drive.speed,msg.drive.steering_angle])
    def on_safe(msg):
        safe.append([msg.drive.speed,msg.drive.steering_angle])
        if case['state'] in ('ARMED','ACTIVE'):
            case['target_speed_mps']=msg.drive.speed
            if msg.drive.speed>0:case['state']='ACTIVE'
    host.create_subscription(AckermannDriveStamped,'/drive',on_drive,10)
    host.create_subscription(AckermannDriveStamped,'/drive/safe',on_safe,10)
    def service(name):
        def callback(request,response):
            services.append(name);case['generation']+=1
            case.update(state={'stop':'ESTOP_LATCHED','clear_stop':'DISARMED','arm':'ARMED','disarm':'DISARMED'}[name],target_speed_mps=0.)
            response.success=True;response.message='fake MCU '+name+' only';return response
        return callback
    for name in ('stop','clear_stop','arm','disarm'):host.create_service(Trigger,'/actuation/'+name,service(name))
    def feed():
        stamp=host.get_clock().now().to_msg();ns=stamp.sec*1_000_000_000+stamp.nanosec
        status.publish(String(data=json.dumps({**case,'now_us':ns//1000})))
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser_frame'
        scan.angle_min=-math.pi;scan.angle_max=math.pi-math.tau/500;scan.angle_increment=math.tau/500
        scan.range_min=.05;scan.range_max=12.;scan.ranges=[2.]*500;laser.publish(scan)
    host.create_timer(.02,feed)
    stopping=threading.Event()
    def spin():
        try:
            while not stopping.is_set():executor.spin_once(timeout_sec=.02)
        except BaseException as error:errors.append(repr(error))
    thread=threading.Thread(target=spin);thread.start()
    port=ops.server.server_address[1]
    def http(path,body=None):
        request=urllib.request.Request(f'http://127.0.0.1:{port}'+path,
            data=None if body is None else json.dumps(body).encode(),headers={'X-Arena-Token':'local-fixture-token',
            'X-Arena-Vehicle':'fixture','Content-Type':'application/json'})
        with urllib.request.urlopen(request,timeout=2.) as response:return json.load(response)
    def state():return http('/api/state?client=fixture-browser')
    def wait(predicate,label,seconds=8.):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            current=state()
            if predicate(current):return current
            if errors:raise RuntimeError(errors)
            time.sleep(.1)
        raise TimeoutError(label+': '+json.dumps(current['manual']))
    def send(op,value=None):
        current=state();request=uuid.uuid4().hex
        receipt=http('/api/manual',dict(op=op,request=request,client='fixture-browser',
            view=current['manual']['binding'],value=value))
        assert receipt['stage']=='queued',receipt
        return request
    results={}
    try:
        wait(lambda s:s['connection']['mcu_age_s'] is not None and s['connection']['mcu_age_s']<.25,'MCU feed')
        time.sleep(.5)
        assert services==[] and not commands,'no automatic ARM or drive at startup'
        send('set_speed_limit',.15)
        wait(lambda s:s['manual']['limit_mps']==.15,'cap')
        send('prepare');wait(lambda s:s['manual']['prepared'],'prepare')
        assert services==['clear_stop','arm'] and not commands
        send('run',dict(speed_mps=.1,seconds=.4,steering_deg=5.))
        completed=wait(lambda s:any(c['op']=='run' and c['stage'] in ('completed','failed') for c in s['manual']['commands']),'pulse end')
        receipt=next(c for c in completed['manual']['commands'] if c['op']=='run')
        results['pulse']=receipt
        assert receipt['stage']=='completed',receipt
        assert any(v[0]>0 for v in commands) and any(v[0]>0 for v in safe)
        assert all(abs(v[1]-math.radians(5.))<1e-6 for v in commands)
        assert commands[-1][0]==0 and services[-1]=='stop' and ops.manual.process is None
        results['checks']=dict(no_auto_arm=True,actual_http=True,actual_ros=True,actual_independent_guard=True,
            target_speed_preserved=True,positive_drive_and_safe=True,zero_then_stop_observed=True,
            child_reaped=True,physical_motion=False,physical_stop_verified=False)
        results['passed']=True
    except BaseException as error:
        results.update(passed=False,error=repr(error))
        try:send('stop')
        except BaseException:pass
        deadline=time.monotonic()+5.
        while ops.manual.process is not None and time.monotonic()<deadline:time.sleep(.05)
    finally:
        stopping.set();thread.join(3.)
        results.update(commands=commands,safe=safe,services=services,executor_errors=errors)
        executor.shutdown()
        ops.destroy_node();guard.destroy_node();host.destroy_node();rclpy.shutdown()
        results['child_live_after_cleanup']=ops.manual.process is not None
        (out/'result.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in results.items() if k not in ('commands','safe','pulse')},ensure_ascii=False))
    if not results['passed'] or results['child_live_after_cleanup'] or errors:raise SystemExit(1)


if __name__=='__main__':main()
