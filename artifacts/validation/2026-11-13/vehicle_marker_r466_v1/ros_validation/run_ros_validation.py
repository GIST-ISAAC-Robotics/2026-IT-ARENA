"""무구동 ROS 연결 대표. 합성 scan/encoder/IMU/RGB만 발행하며 포트·MCU·Gazebo 없음."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--import-tests',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='96':
        raise RuntimeError('dedicated localhost domain 96 is required')
    args.output.mkdir(parents=True,exist_ok=False)
    result=dict(scope='synthetic_local_ROS_no_actuator',physical_safety_verified=False,
        domain=96,localhost_only=True,profile=str(args.profile.resolve()),
        child_processes=[],cases={},passed=False)
    (args.output/'pid.json').write_text(json.dumps(dict(pid=os.getpid(),parent=os.getppid()),indent=2))
    nodes=[];executor=None;initialized=False;producer=None
    try:
        import pytest
        result['previously_excluded_tests_exit']=int(pytest.main(['-q','-p','no:cacheprovider',
            str(args.import_tests)+'::test_existing_node_command_is_unchanged_when_opt_in_is_off',
            str(args.import_tests)+'::test_node_g1_input_failure_cannot_fall_back_to_r0',
            '--junitxml='+str(args.output/'ros_import_tests.xml')]))
        if result['previously_excluded_tests_exit']!=0:raise RuntimeError('ROS node import tests failed')

        import cv2
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image,LaserScan,JointState,Imu
        from std_msgs.msg import String
        from ackermann_msgs.msg import AckermannDriveStamped
        from arena_autonomy.vehicle_marker_channel import MarkerChannel
        from arena_autonomy.vision_process import VisionPipeline,VisionProcess
        from arena_autonomy.lidar_safety import LidarSafety
        import arena_autonomy.lidar_safety as guard_module
        import arena_autonomy.vehicle_marker_channel as channel_module
        import arena_autonomy.local_pursuit as pursuit_module
        import arena_autonomy.vision_process as vision_module
        snapshot_root=Path(__file__).resolve().parents[1]
        result['import_paths']={name:module.__file__ for name,module in (
            ('guard',guard_module),('channel',channel_module),('local_pursuit',pursuit_module),('vision',vision_module))}
        if not all(Path(path).resolve().is_relative_to(snapshot_root) for path in result['import_paths'].values()):
            raise RuntimeError('a target module was imported outside the isolated snapshot')

        rclpy.init(args=['--ros-args','-p','use_sim_time:=false',
            '-p','vehicle_marker_profile_path:='+str(args.profile.resolve()),
            '-p','lidar_compensation:=both','-p','motion_scan_timing_verified:=true',
            '-p','motion_feedback_mode:=rear_wheel_pair_legacy'])
        initialized=True

        def stamp(seconds):
            from builtin_interfaces.msg import Time
            sec,ns=divmod(round(seconds*1e9),1_000_000_000)
            return Time(sec=sec,nanosec=ns)

        class Producer(Node):
            def __init__(self):
                super().__init__('r466_marker_producer')
                self.phase='absent';self.want_rgb=False;self.target=.75
                self.channel=MarkerChannel(self,producer=True)
                self.pipeline=VisionPipeline(VisionProcess(),vehicle_region_profile=self.channel.profile)
                self.pipeline.signal_detection_enabled=False
                self.last_image=-math.inf;self.counts=dict(feed=0,images=0,regions_published=0)
                self.wheels=self.create_publisher(JointState,'/wheel_states',20)
                self.imu=self.create_publisher(Imu,'/camera/imu',qos_profile_sensor_data)
                self.scan=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
                self.drive=self.create_publisher(AckermannDriveStamped,'/drive',10)
                self.images={}
                dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
                for name,size,x,y in [('front',80,640,440),('side',48,100,420)]:
                    rgb=np.full((720,1280,3),255,np.uint8)
                    rgb[y-size//2:y+size//2,x-size//2:x+size//2]=draw(dictionary,1,size)[:,:,None]
                    self.images[name]=rgb.tobytes()
                self.timer=self.create_timer(.01,self.feed)

            def feed(self):
                now=self.get_clock().now().nanoseconds*1e-9
                t=stamp(now)
                wheels=JointState();wheels.header.stamp=t
                wheels.name=['rear_left_wheel_joint','rear_right_wheel_joint'];wheels.velocity=[0.,0.]
                self.wheels.publish(wheels)
                imu=Imu();imu.header.stamp=t;imu.header.frame_id='camera_imu';imu.angular_velocity.z=0.
                self.imu.publish(imu)
                scan=LaserScan();scan.header.stamp=stamp(now-.08);scan.header.frame_id='laser_frame'
                scan.angle_min=-math.pi;scan.angle_max=math.pi-math.tau/720;scan.angle_increment=math.tau/720
                scan.time_increment=.0001;scan.scan_time=.1;scan.range_min=.05;scan.range_max=12.
                scan.ranges=[3.]*720;self.scan.publish(scan)
                command=AckermannDriveStamped();command.header.stamp=t;command.header.frame_id='base_link'
                command.drive.speed=self.target;command.drive.steering_angle=0.;self.drive.publish(command)
                if self.want_rgb and now-self.last_image>=.15:
                    image=Image();image.header.stamp=t;image.header.frame_id=self.channel.profile['camera']['optical_frame_id']
                    image.height=720;image.width=1280;image.encoding='rgb8';image.step=1280*3
                    image.data=self.images['side' if self.phase=='side' else 'front']
                    self.pipeline.offer(image);self.last_image=now;self.counts['images']+=1
                accepted=self.pipeline.pump(now)
                if self.pipeline.fault:raise RuntimeError('vision pipeline: '+self.pipeline.fault)
                if accepted is not None:
                    self.channel.publish(self.pipeline);self.counts['regions_published']+=1
                self.counts['feed']+=1

        class Collector(Node):
            def __init__(self):
                super().__init__('r466_marker_collector');self.rows=[];self.safe=[];self.packets=[]
                self.create_subscription(String,'/safety/status',self.status,100)
                self.create_subscription(AckermannDriveStamped,'/drive/safe',self.on_safe,100)
                self.create_subscription(String,'/perception/vehicle_marker_regions',self.on_packet,10)
            def status(self,message):
                self.rows.append(dict(phase=producer.phase,wall=time.monotonic(),data=json.loads(message.data)))
            def on_safe(self,message):
                self.safe.append(dict(phase=producer.phase,wall=time.monotonic(),speed=message.drive.speed,
                    steering=message.drive.steering_angle,frame=message.header.frame_id))
            def on_packet(self,message):
                self.packets.append(dict(phase=producer.phase,wall=time.monotonic(),packet=json.loads(message.data)))

        producer=Producer();nodes.append(producer)
        result['child_processes'].append(producer.pipeline.worker.process.pid)
        guard=LidarSafety();nodes.append(guard)
        collector=Collector();nodes.append(collector)
        executor=SingleThreadedExecutor()
        for node in nodes:executor.add_node(node)
        began=time.monotonic()
        def run_phase(name,duration,images,*,target=.75):
            producer.phase=name;producer.want_rgb=images;producer.target=target
            start=time.monotonic()
            while time.monotonic()-start<duration:
                executor.spin_once(timeout_sec=.01)
                if time.monotonic()-began>12.:raise RuntimeError('representative wall-time bound exceeded')
            return start

        starts={}
        starts['absent']=run_phase('absent',1.0,False)
        graph=producer.get_node_names()
        if sorted(graph)!=sorted(['r466_marker_producer','lidar_safety','r466_marker_collector']):
            raise RuntimeError('unexpected ROS graph on dedicated domain: '+repr(graph))
        starts['front']=run_phase('front',1.2,True)
        starts['side']=run_phase('side',1.2,True)
        starts['front_before_ttl']=run_phase('front_before_ttl',.7,True)
        starts['expired']=run_phase('expired',.8,False)
        starts['final_zero']=run_phase('final_zero',.25,False,target=0.)
        for name in ('absent','front','side','expired'):
            rows=[r['data'] for r in collector.rows if r['phase']==name and r['wall']>=starts[name]+.65]
            safe=[r for r in collector.safe if r['phase']==name and r['wall']>=starts[name]+.65]
            if len(rows)<3 or len(safe)<3:raise RuntimeError('insufficient stable timer output '+name)
            speeds=[r['safe_speed_mps'] for r in rows]
            actual=[r['speed'] for r in safe]
            if name=='front':
                passed=all(0<=v<.74 for v in speeds+actual) and all('vehicle_marker_occupancy' in r for r in rows)
            else:
                passed=all(abs(v-.75)<1e-9 for v in speeds+actual) and all('vehicle_marker_occupancy' not in r for r in rows)
            passed=passed and all(r['frame']=='base_link' for r in safe)
            result['cases'][name]=dict(passed=passed,status_count=len(rows),safe_count=len(safe),
                min_speed=min(speeds),max_speed=max(speeds),reasons=sorted(set(r['reason'] for r in rows)))
            if not passed:raise RuntimeError('unexpected representative result '+name)
        zeros=[r for r in collector.safe if r['phase']=='final_zero' and r['wall']>starts['final_zero']+.1]
        result['final_zero_observed']=bool(zeros) and all(r['speed']==0. for r in zeros)
        if not result['final_zero_observed']:raise RuntimeError('final zero not observed')
        result['guard_motion_counts']=guard.motion.counts
        result['producer_counts']=producer.counts
        result['vision']=producer.pipeline.snapshot()
        result['guard_packet_after_expiry']=guard.vehicle_marker_channel.snapshot(guard.vehicle_marker_channel.now())
        if result['guard_packet_after_expiry'] is not None:raise RuntimeError('expired marker retained')
        result['passed']=True
    except BaseException:
        result['error']=traceback.format_exc()
    finally:
        if 'collector' in locals():
            for name,rows in [('safety_status',collector.rows),('drive_safe',collector.safe),('marker_packets',collector.packets)]:
                with (args.output/(name+'.jsonl')).open('w',encoding='utf-8') as stream:
                    for row in rows:stream.write(json.dumps(row,allow_nan=False)+'\n')
        if producer is not None:
            producer.target=0.;producer.want_rgb=False
            result['vision_cleanup']=producer.pipeline.close()
            result['vision_child_alive_after_close']=producer.pipeline.worker.process.is_alive()
            result['normal_vision_cleanup']=result['vision_cleanup']=={'forced':False,'exitcode':0}
            if not result['normal_vision_cleanup']:result['passed']=False
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        for node in reversed(nodes):node.destroy_node()
        if initialized:
            import rclpy
            rclpy.shutdown()
        result['nodes_destroyed']=len(nodes)
        (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['passed'] and not result.get('vision_child_alive_after_close',True) else 1


if __name__=='__main__':raise SystemExit(main())
