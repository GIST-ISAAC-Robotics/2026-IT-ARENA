#!/usr/bin/env python3
"""출력 비활성 sensor_link 앱의 단일 UART 소유자. 모터 DRIVE 전송을 허용하지 않는다."""
import argparse
from collections import Counter
from arena_vehicle_interface.mcu_status_clock import McuStatusClock
import gc
import json
from pathlib import Path
import time

from arena_vehicle_interface.actuation_wire import Decoder as ControlDecoder, encode
from arena_vehicle_interface.sensor_link_wire import SensorLinkDecoder
from arena_vehicle_interface.imu_read_clock import ImuReadClock
from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock
from arena_vehicle_interface.serial_ingress import SerialIngress
from probe_esp32_imu import Decoder as ImuDecoder


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=20.)
    p.add_argument('--imu-clock',choices=('mcu_read','fifo_tick'),default='mcu_read',
        help='FIFO tick은 선택형 시각 추정. 취득시각 검증/구동 허가 아님')
    p.add_argument('--mcu-status',action='store_true',help='선택형 관측 상태 발행; 호환 offer 필드 필요, 구동 허가 아님')
    args=p.parse_args()
    if not 1<=args.seconds<=120: p.error('seconds outside 1..120')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import UInt8MultiArray, String
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[]); node=Node('sensor_link_preview')
    if node.get_parameter('use_sim_time').value: raise RuntimeError('wall clock required')
    pub=node.create_publisher(Imu,'/hardware_preview/imu',qos_profile_sensor_data)
    rx=node.create_publisher(UInt8MultiArray,'/actuation/rx',10)
    status=node.create_publisher(String,'/hardware_preview/imu_status',10)
    wire=SensorLinkDecoder(); imu=ImuDecoder(); control=ControlDecoder(); tx=ControlDecoder()
    mcu_clock=McuStatusClock() if args.mcu_status else None
    mcu_status_pub=node.create_publisher(String,'/hardware_preview/actuation_status',10) if args.mcu_status else None
    clock=ImuFifoClock() if args.imu_clock=='fifo_tick' else ImuReadClock(); counts=Counter(); boots=set(); offer_boots=set(); complete=False
    link=SerialIngress(args.port)
    was_gc=gc.isenabled(); began=time.monotonic(); last_status=0.; error=None
    maxima={};slow_stages=[];queue_age_ms=[]
    def measured(stage,begin):
        ms=(time.perf_counter_ns()-begin)/1e6
        maxima[stage]=max(maxima.get(stage,0),ms)
        if ms>=20:slow_stages.append(dict(stage=stage,ms=ms,host_ns=time.monotonic_ns()))
    with (args.output/'uart.bin').open('xb') as raw, (args.output/'imu.jsonl').open('x') as frames, \
            (args.output/'control.jsonl').open('x') as operations, \
            (args.output/'clock_estimates.jsonl').open('x') as clock_records:
        def outgoing(message):
            # 진단 앱에는 출력 코드가 없지만 호스트 송신도 명시 제어 네 종류로 한정한다.
            for payload in tx.feed(bytes(message.data)):
                command=payload.get('command',{})
                if payload.get('type')!='command' or command.get('kind') not in ('ARM','CLEAR','DISARM','STOP'):
                    counts['tx_rejected']+=1; continue
                packet=encode(payload)
                link.send(packet)
                counts['tx_'+command['kind']]+=1
                operations.write(json.dumps(dict(direction='tx_queued',host_ns=time.monotonic_ns(),payload=payload))+'\n')
        node.create_subscription(UInt8MultiArray,'/actuation/tx',outgoing,10)
        def receive(*, spin=True):
            nonlocal last_status
            receipt=link.get(timeout=.002)
            if receipt is None:
                if spin:
                    begin=time.perf_counter_ns();rclpy.spin_once(node,timeout_sec=0.);measured('spin',begin)
                return
            if receipt.kind=='tx':
                operations.write(json.dumps(dict(direction='tx_written',host_ns=receipt.monotonic_ns,
                    wire_hex=receipt.data.hex()))+'\n')
                return
            data=receipt.data;mono=receipt.monotonic_ns;ros=receipt.realtime_ns
            queue_age_ms.append((time.monotonic_ns()-mono)/1e6)
            begin=time.perf_counter_ns()
            raw.write(data); gyros=[];imu_frames=[];mcu_states=[]
            for kind,packet in wire.feed(data):
                if kind=='control':
                    for row in control.feed(packet):
                        counts['rx_'+row.get('type','unknown')]+=1
                        if row.get('type')=='offer':
                            offer_boots.add(row['boot'])
                            if mcu_clock is not None:
                                if not all(k in row for k in ('status_now_us','estop_held','reason')):
                                    raise RuntimeError('MCU status fields missing; use matching output-disabled firmware')
                                mcu_states.append(row)
                            if row.get('output_ready') is not False: raise RuntimeError('output-enabled firmware is outside this preview')
                        operations.write(json.dumps(dict(direction='rx',host_ns=mono,payload=row))+'\n')
                    pub_begin=time.perf_counter_ns()
                    rx.publish(UInt8MultiArray(data=list(packet)))
                    measured('publish_control',pub_begin)
                else:
                    for row in imu.feed(packet):
                        boots.add(row['boot'])
                        imu_frames.append(row)
                        frames.write(json.dumps(dict(row,host_monotonic_ns=mono,host_ros_ns=ros))+'\n')
                        counts['imu_frames']+=1
                        if 'gyro_rad_s' in row:gyros.append(row)
            for item in (mcu_clock.convert_batch(mcu_states,monotonic_ns=mono,realtime_ns=ros) if mcu_clock is not None else []):
                mcu_status_pub.publish(String(data=json.dumps(item)))
                operations.write(json.dumps(dict(direction='status_published',host_ns=mono,payload=item))+'\n')
                counts['mcu_status_published']+=1
            measured('decode_record',begin);begin=time.perf_counter_ns()
            stamps=clock.convert_batch(imu_frames if args.imu_clock=='fifo_tick' else gyros,host_monotonic_ns=mono,ros_receipt_ns=ros)
            measured('clock',begin);begin=time.perf_counter_ns()
            for stamp in stamps:
                msg=Imu();msg.header.frame_id='lsm6dsox_sensor'
                msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp.stamp_ns,1_000_000_000)
                msg.orientation_covariance[0]=-1.;msg.linear_acceleration_covariance[0]=-1.
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=stamp.sample['gyro_rad_s']
                pub.publish(msg);counts['gyro_published']+=1
                if args.imu_clock=='fifo_tick':
                    clock_records.write(json.dumps(dict(boot=stamp.sample['boot'],sequence=stamp.sample['sequence'],
                        stamp_ns=stamp.stamp_ns,generation=stamp.generation,sensor_tick=stamp.sensor_tick,
                        mcu_lower_ns=stamp.mcu_lower_ns,mcu_upper_ns=stamp.mcu_upper_ns,
                        register_anchor_sequence=stamp.register_anchor_sequence,read_based_stamp_ns=stamp.read_based_stamp_ns,
                        time_contract_verified=False))+'\n')
            measured('publish_gyro',begin)
            now=time.monotonic()
            if now-last_status>=1.:
                status.publish(String(data=json.dumps(dict(time_basis=clock.time_basis,time_contract_verified=False,
                    sensor_acquisition_timestamp=False,mounting_calibrated=False,counts=dict(counts)))))
                last_status=now
            if spin:
                begin=time.perf_counter_ns();rclpy.spin_once(node,timeout_sec=0.);measured('spin',begin)
        try:
            link.start();gc.disable()
            (args.output/'ready').write_text('serial owner ready')
            while time.monotonic()-began<args.seconds:receive()
            complete=True
        except BaseException as exc:
            error=type(exc).__name__+': '+str(exc);raise
        finally:
            try:
                if not link.stopping.is_set():
                    outgoing(UInt8MultiArray(data=list(encode(dict(type='command',request='preview-final-stop',command=dict(v=1,kind='STOP'))))))
                    until=time.monotonic()+.5
                    while time.monotonic()<until:receive()
            finally:
                link.close()
                # 포트를 닫은 뒤 큐에 남은 마지막 원문/시각도 보존한다. 새 serial 읽기는 없다.
                while not link.receipts.empty():receive(spin=False)
                if was_gc:gc.enable()
                result=dict(complete=complete,error=error,duration_s=time.monotonic()-began,counts=dict(counts),
                    wire=dict(wire.counters),imu_decoder=dict(imu.counters),control_errors=control.errors,
                    tx_decode_errors=tx.errors,mcu_clock=None if mcu_clock is None else dict(mcu_clock.counts),clock=dict(clock.counts),imu_boots=sorted(boots),offer_boots=sorted(offer_boots),
                    matching_boot=bool(boots) and {format(b,'08x') for b in boots}==offer_boots,
                    time_contract_verified=False,time_basis=clock.time_basis,mounting_calibrated=False,drive_output=False,
                    serial_io=dict(link.stats,error=link.error),stage_max_ms=maxima,slow_stages=slow_stages,
                    queue_age_ms=dict(count=len(queue_age_ms),max=max(queue_age_ms,default=0),
                        p95=sorted(queue_age_ms)[int((len(queue_age_ms)-1)*.95)] if queue_age_ms else None),
                    receipt_clock='I/O thread CLOCK_MONOTONIC and CLOCK_REALTIME; unchanged while queued')
                (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
                node.destroy_node();rclpy.shutdown()
    print(json.dumps(result))


if __name__=='__main__':main()
