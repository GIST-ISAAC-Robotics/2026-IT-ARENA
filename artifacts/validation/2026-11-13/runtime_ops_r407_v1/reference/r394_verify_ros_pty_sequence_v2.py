#!/usr/bin/env python3
"""domain 217의 실제 ROS + 새 PTY/가짜 MCU 통합 1회. 실물 포트/출력 없음."""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time
import tty

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
for path in (ROOT/'scripts',ROOT/'src/arena_vehicle_interface'):
    sys.path.insert(0,str(path))
from arena_vehicle_interface.actuation_wire import Decoder,encode
from arena_vehicle_interface.actuation_contract import ActuationReceiver,ContractConfig
from arena_vehicle_interface.drive_feedback import MODE_MOTOR
from audit_actuation_offer_clock import KEYS as OFFER_KEYS


class FakeMcu:
    """기존 PC 수신기 참조 + 합성 stationary count. 실제 출력 HAL은 없다."""
    def __init__(self,master):
        self.master=master;self.stop=threading.Event();self.arm_seen=threading.Event();self.release_arm=threading.Event()
        self.decoder=Decoder();self.commands=[];self.events=[];self.error=None;self.pending=None
        self.rx=ActuationReceiver(ContractConfig(feedback_mode=MODE_MOTOR,ticks_per_rev=28,
            gear_ratio=2.5*81/17,wheel_radius_m=.0325,max_speed_mps=1.,max_steering_rad=.45),boot_id='r394-pty-only')
        self.sequence=0;self.started=time.monotonic_ns()
        self.thread=threading.Thread(target=self.run,name='fake-mcu-PTY',daemon=True)

    def write(self,row):
        packet=encode(row);sent=0
        while sent<len(packet):sent+=os.write(self.master,packet[sent:])
        self.events.append(dict(direction='to_host',monotonic_ns=time.monotonic_ns(),payload=row))

    def offer(self,now):
        full=self.rx.offer(now)
        feedback=self.rx.feedback
        row={key:full[key] for key in OFFER_KEYS-{'type','output_ready'}}
        row.update(type='offer',output_ready=True,status_now_us=now,estop_held=False,reason=self.rx.reason,
            motor_feedback_v1=dict(valid=full['feedback_valid'],
                sequence=None if feedback is None else feedback[0],
                capture_us=None if feedback is None else feedback[1],counts=None if feedback is None else feedback[2]))
        self.write(row)

    def accept(self,row,now):
        accepted,reason=self.rx.receive(row['command'],now)
        record=dict(monotonic_ns=time.monotonic_ns(),payload=row,accepted=accepted,reason=reason)
        self.commands.append(record)
        if row.get('request'):
            self.write(dict(type='ack',request=row['request'],accepted=accepted,reason=reason))
        self.offer(now)

    def run(self):
        next_offer=0
        try:
            while not self.stop.is_set():
                now=max(1,(time.monotonic_ns()-self.started)//1000)
                self.sequence+=1
                self.rx.update_motor_encoder(now,sequence=self.sequence,capture_us=now,motor_counts=0)
                self.rx.tick(now)
                if self.pending is not None and self.release_arm.is_set():
                    row,self.pending=self.pending,None;self.accept(row,now)
                if now>=next_offer:self.offer(now);next_offer=now+20_000
                if select.select([self.master],[],[],.005)[0]:
                    for row in self.decoder.feed(os.read(self.master,4096)):
                        self.events.append(dict(direction='from_host',monotonic_ns=time.monotonic_ns(),payload=row))
                        if row.get('type')!='command':raise ValueError('unexpected host envelope')
                        if row['command']['kind']=='ARM':self.pending=row;self.arm_seen.set()
                        else:self.accept(row,max(1,(time.monotonic_ns()-self.started)//1000))
        except BaseException as error:self.error=repr(error)


def write_json(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():p.error('new output required')
    if os.environ.get('ROS_DOMAIN_ID')!='217':p.error('explicit ROS_DOMAIN_ID=217 required')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1':p.error('explicit ROS_LOCALHOST_ONLY=1 required')
    args.output.mkdir(parents=True)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    from ackermann_msgs.msg import AckermannDriveStamped
    policy=dict(schema='runtime_host_clock_policy_v1',receiver_lease=dict(source_ttl_ns=100_000_000,
        max_receipt_age_ns=100_000_000,max_rate_error_ppm=1000,policy_note='Declared PTY test assumption; not hardware verified'),
        same_host=dict(source_ttl_ns=100_000_000,max_lookback_ns=100_000_000,max_sample_gap_ns=50_000_000,
        max_read_width_ns=1_000_000,ros_timestamp_error_ns=1000,host_timestamp_error_ns=0,max_rate_error_ppm=1000,
        assume_no_unsampled_clock_step=True,policy_note='Same-host PTY test assumptions; unsampled jumps unverified'),
        max_lease_us=100_000,request_timeout_ns=200_000_000)
    motor=dict(schema='runtime_motor_feedback_v1',counts_per_motor_revolution=28,gear_ratio=2.5*81/17,
        wheel_radius_m=.0325,encoder_sign=1,maximum_interval_ns=200_000_000,maximum_age_ns=200_000_000,maximum_speed_mps=5.)
    write_json(args.output/'clock_policy.json',policy);write_json(args.output/'motor_config.json',motor)
    master,slave=os.openpty();tty.setraw(slave);port=os.ttyname(slave)
    assert port.startswith('/dev/pts/'),'only freshly created pseudo terminal permitted'
    fake=FakeMcu(master);child=None;node=executor=None;ros=False;checks={};failure=None;statuses=[];joints=[]
    started=time.monotonic();deadline=started+11.
    log=(args.output/'runner.log').open('x')
    def pump_until(predicate,limit=2.):
        end=min(deadline,time.monotonic()+limit)
        while not predicate():
            if time.monotonic()>=end:raise TimeoutError('bounded ROS/PTY condition timeout')
            if fake.error:raise RuntimeError(fake.error)
            if child is not None and child.poll() is not None:raise RuntimeError('runner exited before check')
            executor.spin_once(timeout_sec=.005)
    def service(kind):
        future=clients[kind].call_async(Trigger.Request());pump_until(future.done)
        response=future.result();assert response.success,response.message
        checks[kind]=json.loads(response.message)
    try:
        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);ros=True
        node=Node('r394_pty_only_probe');executor=SingleThreadedExecutor();executor.add_node(node)
        assert not any(name.startswith('/actuation/') for name,_ in node.get_service_names_and_types()),'domain 217 not exclusive'
        clients={kind:node.create_client(Trigger,'/actuation/'+kind) for kind in ('clear_stop','arm')}
        node.create_subscription(String,'/actuation/status',lambda m:statuses.append(json.loads(m.data)),10)
        node.create_subscription(JointState,'/drive_motor/encoder',joints.append,10)
        publisher=node.create_publisher(AckermannDriveStamped,'/drive/safe',1)
        env=dict(os.environ)
        env['PYTHONPATH']=os.pathsep.join(str(path) for path in (HERE/'staged/scripts',HERE.parent/'staged/scripts',ROOT/'scripts',ROOT/'src/arena_vehicle_interface'))+os.pathsep+env.get('PYTHONPATH','')
        cmd=[sys.executable,str(HERE/'staged/scripts/run_runtime_actuation_host.py'),'--port',port,
            '--output',str(args.output/'runner'),'--seconds','12','--clock-policy',str(args.output/'clock_policy.json'),
            '--max-speed-mps','1','--max-steering-rad','.45','--motor-feedback-config',str(args.output/'motor_config.json'),
            '--enable-drive-wire']
        write_json(args.output/'registered.json',dict(command=cmd,domain=217,port_kind='new_PTY_fake_MCU',
            real_device_opened=False,physical_actuation=False,maximum_external_seconds=15))
        fake.thread.start();child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,cwd=ROOT)
        pump_until(lambda:all(c.service_is_ready() for c in clients.values()) and publisher.get_subscription_count()>0
            and bool(joints) and statuses[-1]['state']=='ESTOP_LATCHED',limit=5.)
        checks['stationary_observed_zero']=bool(joints) and all(list(m.velocity)==[0.] for m in joints)
        service('clear_stop')
        future=clients['arm'].call_async(Trigger.Request());pump_until(fake.arm_seen.is_set)
        hold_until=time.monotonic()+.025
        while time.monotonic()<hold_until:
            executor.spin_once(timeout_sec=.002)
            assert not future.done(),'ARM service completed before held ACK'
        checks['arm_incomplete_before_ack']=True;fake.release_arm.set();pump_until(future.done)
        response=future.result();assert response.success,response.message;checks['arm']=json.loads(response.message)
        message=AckermannDriveStamped();message.header.stamp=node.get_clock().now().to_msg()
        source=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        message.drive.speed=.05;message.drive.steering_angle=.1;publisher.publish(message)
        pump_until(lambda:any(row['payload']['command']['kind']=='DRIVE' for row in tuple(fake.commands)),limit=.5)
        drive=next(row for row in fake.commands if row['payload']['command']['kind']=='DRIVE')
        assert drive['accepted'],drive
        checks['drive_wire']=drive;checks['original_source_ros_ns']=source
        child.send_signal(signal.SIGINT);child.wait(timeout=1.5)
        summary=json.loads((args.output/'runner/summary.json').read_text())
        checks['runner_exit_code']=child.returncode
        checks['shutdown']=summary['shutdown']
        assert child.returncode==1  # v1 runner는 의도한 SIGINT도 incomplete 종료 code 1로 기록한다.
        assert summary['interruption']=='signal_'+str(signal.SIGINT)
        assert summary['error'] is None and not summary['cleanup_errors']
        assert summary['shutdown']['matching_stop_tx_receipt'] and summary['shutdown']['stop_ack_confirmed']
        assert any(row['payload'].get('request')==summary['shutdown']['request'] and
            row['payload']['command']['kind']=='STOP' and row['accepted'] for row in fake.commands)
        rows=[json.loads(line) for line in (args.output/'runner/events.jsonl').read_text().splitlines()]
        prepared=[row for row in rows if row['event']=='drive_prepared' and row['prepared'].get('wire')]
        assert len(prepared)==1 and prepared[0]['source']['source_ros_ns']==source
        assert not prepared[0]['source']['source_retimed'] and not summary['source_timestamp_rewritten']
        assert prepared[0]['prepared']['envelope']==drive['payload']
        checks['original_source_preserved']=True
        checks['joint_header_and_names']=all(m.name==['drive_motor_shaft'] and m.header.frame_id=='drive_motor_encoder' for m in joints)
        assert checks['stationary_observed_zero'] and checks['joint_header_and_names']
    except BaseException as error:failure=type(error).__name__+': '+str(error)
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=1.)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=.5)
        fake.stop.set()
        if fake.thread.ident is not None:fake.thread.join(timeout=.5)
        os.close(master);os.close(slave);log.close()
        if executor is not None:executor.shutdown(timeout_sec=.2)
        if node is not None:node.destroy_node()
        if ros:rclpy.shutdown()
        with (args.output/'fake_mcu.jsonl').open('x') as stream:
            for row in fake.events:stream.write(json.dumps(row)+'\n')
        if fake.error is not None and failure is None:failure='fake MCU I/O: '+fake.error
        report=dict(passed=failure is None,error=failure,checks=checks,seconds=time.monotonic()-started,
            fake_mcu_error=fake.error,pty_only=True,domain=217,physical_motion_verified=False,
            joint_messages=len(joints),status_messages=len(statuses),child_returncode=None if child is None else child.returncode,
            joint_samples=[dict(name=list(m.name),frame=m.header.frame_id,
                stamp_ns=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec,
                position=list(m.position),velocity=list(m.velocity),velocity_container=type(m.velocity).__name__) for m in joints])
        write_json(args.output/'report.json',report)
    print(json.dumps(report))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
