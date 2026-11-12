"""실제 ROS+LidarSafety, 가짜 MCU 상태/STOP endpoint. 차량/Gazebo/Jetson 없음."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

os.environ.update(ROS_DOMAIN_ID='217',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_STATIC_PEERS='')
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from arena_autonomy.lidar_safety import LidarSafety

A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def main():
    rclpy.init(args=[])
    node=Node('jog_fixture');guard=LidarSafety()
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(guard)
    status=node.create_publisher(String,'/actuation/status',10)
    laser=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    case={};commands=[];safe=[];service=[];competing=None

    def received_command(msg):commands.append((msg.drive.speed,msg.drive.steering_angle))
    def received_safe(msg):
        safe.append((msg.drive.speed,msg.drive.steering_angle))
        if case and case['state']!='ESTOP':
            case['target_speed_mps']=msg.drive.speed
            if msg.drive.speed>0:case['state']='ACTIVE'
    node.create_subscription(AckermannDriveStamped,'/drive',received_command,10)
    node.create_subscription(AckermannDriveStamped,'/drive/safe',received_safe,10)

    def stop_callback(request,response):
        service.append('STOP');case.update(state='ESTOP',target_speed_mps=0.,generation=case['generation']+1)
        response.success=True;response.message='fixture STOP acknowledged';return response
    node.create_service(Trigger,'/actuation/stop',stop_callback)
    results=[]
    try:
        for name,direction in [('forward','forward'),('left','left'),('right','right'),
                               ('close_obstacle','forward'),('external_stop','forward'),('competing_publisher','forward')]:
            commands.clear();safe.clear();service.clear()
            case.clear();case.update(v=1,boot='fixture_'+name,generation=0,owner='host',state='ARMED',
                estop_held=False,feedback_valid=True,output_ready=True,target_speed_mps=0.)
            if name=='competing_publisher':competing=node.create_publisher(AckermannDriveStamped,'/drive',1)
            folder=A/name;assert not folder.exists()
            with (A/(name+'.log')).open('x') as log:
                proc=subprocess.Popen([sys.executable,str(ROOT/'scripts/development_jog.py'),direction,
                    '--seconds','.4','--speed','.15','--speed-limit','.2','--send','--output',str(folder)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                start=time.monotonic();next_feed=start;external=False
                while proc.poll() is None:
                    if time.monotonic()-start>9:
                        proc.terminate();proc.wait(timeout=4);raise TimeoutError(name)
                    if time.monotonic()>=next_feed:
                        stamp=node.get_clock().now().to_msg();now=stamp.sec+stamp.nanosec*1e-9
                        if name=='external_stop' and len(commands)>=2 and not external:
                            case.update(state='ESTOP',generation=1,target_speed_mps=0.);external=True
                        status.publish(String(data=json.dumps({**case,'now_us':round(now*1e6)})))
                        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser_frame'
                        scan.angle_min=-math.pi;scan.angle_max=math.pi-math.tau/500
                        scan.angle_increment=math.tau/500;scan.range_min=.05;scan.range_max=12.
                        scan.ranges=[2.]*500
                        if name=='close_obstacle':
                            for i in range(242,259):scan.ranges[i]=.07
                        laser.publish(scan);next_feed=time.monotonic()+.02
                    executor.spin_once(timeout_sec=.002)
                for _ in range(5):executor.spin_once(timeout_sec=.002)
            record=json.loads((folder/'result.json').read_bytes())
            positive=[v for v in safe if v[0]>0]
            checks=dict(no_auto_arm=record['armed_automatically'] is False,
                only_drive=record['output_topic']=='/drive',bounded_commands=all(0<=v[0]<=.2 for v in commands),
                stopped=(record['stop_ack'] and record['fresh_estop_zero_target_seen']) if name!='competing_publisher' else not record['started'],
                exit_expected=(proc.returncode==1 if name in ('external_stop','competing_publisher') else proc.returncode==0))
            if name in ('forward','left','right'):
                expected={'forward':0.,'left':.2,'right':-.2}[name]
                checks['independent_safe_delivery']=bool(positive) and all(abs(v[1]-expected)<1e-6 for v in positive)
            if name=='close_obstacle':checks['independent_guard_blocks']=not positive and any(v[0]>0 for v in commands)
            if name=='competing_publisher':checks['refusal_has_no_output_or_stop']=not commands and not service
            if name=='external_stop':checks['external_stop_not_rearmed']=record['error'] is not None and case['state']=='ESTOP'
            results.append(dict(case=name,passed=all(checks.values()),checks=checks,
                positive_safe_count=len(positive),command_count=len(commands),stop_calls=len(service)))
            if competing is not None:node.destroy_publisher(competing);competing=None
            # 직전 /drive/scan 상태를 다음 사례의 초기 동작으로 섞지 않는다.
            guard.command=None;guard.scan=None
    finally:
        executor.shutdown();guard.destroy_node();node.destroy_node();rclpy.shutdown()
    summary=dict(passed=all(r['passed'] for r in results),cases=results,
        actual_ros_and_lidar_guard=True,mcu_status_and_stop='fixture only',vehicle_motion=False,
        jetson_access=False,source='scripts/development_jog.py')
    (A/'ros_result.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
    assert summary['passed']


if __name__=='__main__':main()
