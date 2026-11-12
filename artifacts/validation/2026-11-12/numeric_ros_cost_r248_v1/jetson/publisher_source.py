"""ROS 200Hz 합성 IMU 수신과 같은610판단의 경합 비교. 구동 출력 없음."""
import argparse
import csv
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
TOPIC='/arena_probe_r248/imu'


def publisher(stop, ready, path):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init(args=[])
    node=Node('r248_publisher')
    pub=node.create_publisher(Imu,TOPIC,qos_profile_sensor_data)
    deadline=time.monotonic()+15.
    try:
        while pub.get_subscription_count()==0:
            if stop.is_set() or time.monotonic()>=deadline:raise RuntimeError('no diagnostic subscriber')
            time.sleep(.01)
        sequence=0;next_tick=time.monotonic()
        ready.send({'ready':True})
        with Path(path).open('x',newline='') as stream:
            out=csv.writer(stream);out.writerow(['seq','stamp_ns','monotonic_s'])
            while not stop.is_set():
                remaining=next_tick-time.monotonic()
                if remaining>0:time.sleep(min(remaining,.005));continue
                now=time.monotonic();next_tick=now+.005
                msg=Imu();msg.header.stamp=node.get_clock().now().to_msg()
                msg.header.frame_id='diagnostic_only'
                # 진단 전용 순번. MotionInput/차량 토픽에는 연결하지 않는다.
                msg.angular_velocity.x=float(sequence)
                pub.publish(msg)
                out.writerow([sequence,msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,now])
                sequence+=1
        ready.send({'sent':sequence})
    finally:
        node.destroy_node();rclpy.shutdown();ready.close()


def quantiles(values):
    import numpy as np
    return dict(count=len(values),p50=float(np.percentile(values,50)),
        p95=float(np.percentile(values,95)),maximum=max(values)) if values else dict(count=0)

