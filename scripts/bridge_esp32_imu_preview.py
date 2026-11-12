#!/usr/bin/env python3
"""진단 펌웨어 gyro를 격리 ROS 토픽으로 전달한다. 자율 제어용 취득시각 승인이 아니다."""
import argparse
import collections
import gc
import json
import math
from pathlib import Path
import time

from probe_esp32_imu import Decoder
from arena_vehicle_interface.imu_read_clock import ImuReadClock
from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock

TOPIC = '/hardware_preview/imu'
STATUS_TOPIC = '/hardware_preview/imu_status'
FRAME = 'lsm6dsox_sensor'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--imu-clock', choices=('mcu_read','fifo_tick'), default='mcu_read',
        help='fifo_tick은 과거 FIFO/register 기반 추정이며 ROS 절대 취득시각 검증은 아님')
    args=p.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        p.error('seconds must be within 1..120')
    import serial
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[])
    node=Node('esp32_imu_preview')
    if node.get_parameter('use_sim_time').value:
        raise RuntimeError('hardware preview requires wall clock')
    pub=node.create_publisher(Imu,TOPIC,qos_profile_sensor_data)
    status_pub=node.create_publisher(String,STATUS_TOPIC,10)
    decoder=Decoder(); clock=ImuFifoClock() if args.imu_clock=='fifo_tick' else ImuReadClock(); counts=collections.Counter()
    observed_boots=set(); metadata=None; first_gyro=None; last_status=-math.inf
    last_active=None; stopped=False; complete=False; error=None
    measurement_start=measurement_end=None
    serial_port=serial.Serial(port=None,baudrate=921600,timeout=.02,write_timeout=2)
    serial_port.dtr=False; serial_port.rts=False
    serial_port.port=args.port
    gc_before=gc.isenabled()
    began=time.monotonic()
    with (args.output/'uart.bin').open('xb') as raw, (args.output/'frames.jsonl').open('x') as frames, \
            (args.output/'clock_estimates.jsonl').open('x') as clock_records:
        def receive(phase):
            nonlocal metadata, first_gyro, last_status, last_active
            data=serial_port.read(max(1,min(serial_port.in_waiting,8192)))
            mono_ns=time.monotonic_ns(); ros_ns=node.get_clock().now().nanoseconds
            raw.write(data); decoded=decoder.feed(data)
            for row in decoded:
                frames.write(json.dumps(dict(row,host_monotonic_ns=mono_ns,
                    host_ros_ns=ros_ns,phase=phase),separators=(',',':'))+'\n')
                observed_boots.add(row['boot'])
                if row['type']==3: metadata=row
                if row['type']==2: last_active=row['active']
            if phase in ('warmup','measure'):
                gyros=[row for row in decoded if 'gyro_rad_s' in row]
                counts['gyro_received_'+phase]+=len(gyros)
                if gyros and first_gyro is None:first_gyro=time.monotonic()
                for stamp in clock.convert_batch(decoded if args.imu_clock=='fifo_tick' else gyros,host_monotonic_ns=mono_ns,ros_receipt_ns=ros_ns):
                    msg=Imu(); msg.header.frame_id=FRAME
                    msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp.stamp_ns,1_000_000_000)
                    msg.orientation_covariance[0]=-1.
                    msg.linear_acceleration_covariance[0]=-1.
                    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=stamp.sample['gyro_rad_s']
                    pub.publish(msg);counts['published_'+phase]+=1
                    if args.imu_clock=='fifo_tick':
                        clock_records.write(json.dumps(dict(phase=phase,boot=stamp.sample['boot'],sequence=stamp.sample['sequence'],
                            stamp_ns=stamp.stamp_ns,generation=stamp.generation,sensor_tick=stamp.sensor_tick,
                            mcu_lower_ns=stamp.mcu_lower_ns,mcu_upper_ns=stamp.mcu_upper_ns,
                            register_anchor_sequence=stamp.register_anchor_sequence,read_based_stamp_ns=stamp.read_based_stamp_ns,
                            time_contract_verified=False))+'\n')
            now=time.monotonic()
            if now-last_status>=1.:
                msg=String();msg.data=json.dumps(dict(time_basis=clock.time_basis,
                    time_contract_verified=False,sensor_acquisition_timestamp=False,
                    mounting_calibrated=False,boot=clock.boot,generation=clock.generation,
                    counts=dict(counts),clock=dict(clock.counts),decoder=dict(decoder.counters)))
                status_pub.publish(msg);last_status=now
            rclpy.spin_once(node,timeout_sec=0.)

        try:
            serial_port.open()
            serial_port.write(b'X')
            until=time.monotonic()+2.
            while time.monotonic()<until:receive('setup')
            if (metadata is None or metadata['who_am_i']!=0x6c
                    or metadata['memory_errors'] or metadata['spi_errors']):
                raise RuntimeError('diagnostic firmware identity check failed')
            gc.disable()
            serial_port.write(b'S')
            deadline=time.monotonic()+10.
            while first_gyro is None or time.monotonic()-first_gyro<.5:
                if time.monotonic()>deadline:raise TimeoutError('gyro stream did not start')
                receive('warmup')
            measurement_start=time.monotonic()
            while time.monotonic()-measurement_start<args.seconds:receive('measure')
            measurement_end=time.monotonic();complete=True
        except BaseException as exc:
            error=type(exc).__name__+': '+str(exc)
            raise
        finally:
            try:
                if serial_port.is_open:
                    last_active=None
                    serial_port.write(b'X')
                    until=time.monotonic()+2.
                    while time.monotonic()<until:receive('stop')
                    stopped=last_active is False
            finally:
                serial_port.close()
                if gc_before:gc.enable()
                duration=(measurement_end-measurement_start) if measurement_end is not None else None
                result=dict(complete=complete,error=error,duration_s=duration,counts=dict(counts),
                    decoder=dict(decoder.counters),clock=dict(clock.counts),boot_count=len(observed_boots),
                    inactive_status_observed=stopped,time_contract_verified=False,
                    time_basis=clock.time_basis,frame_id=FRAME,topic=TOPIC,
                    sensor_acquisition_timestamp=False,mounting_calibrated=False,
                    drive_output=False,wall_elapsed_s=time.monotonic()-began)
                (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
                node.destroy_node();rclpy.shutdown()
    print(json.dumps(result))
    if not stopped:raise RuntimeError('sensor stop acknowledgement missing')


if __name__=='__main__':main()
