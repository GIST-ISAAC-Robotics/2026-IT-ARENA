#!/usr/bin/env python3
"""명시 선택형 ROS/단일 UART host. 기본 ARM/DRIVE 차단, 자동 ARM/재연결 없음."""
import argparse
from collections import Counter,deque
from dataclasses import asdict,dataclass,fields,is_dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import time
import uuid

from arena_vehicle_interface.actuation_wire import Decoder
from arena_vehicle_interface.clock_bounds import integer
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec,MOTOR_TOPIC,MOTOR_JOINT,MOTOR_FRAME
from arena_vehicle_interface.motor_feedback_runtime import MotorFeedbackRuntime
from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock
from arena_vehicle_interface.mcu_status_clock import McuStatusClock
from arena_vehicle_interface.sensor_link_wire import SensorLinkDecoder
from actuation_host_session import CONTROL_REASONS
from probe_esp32_imu import Decoder as ImuDecoder
from receiver_owned_actuation_host import ReceiverLeaseTiming
from runtime_actuation_link import ActuationSerialIngress,RuntimeActuationLink
from runtime_offer_adapter import adapt_runtime_payload
from same_host_command_clock import SameHostClockBounds,SameHostCommandClock,CommandClockError

SERVICE_KINDS={'/actuation/arm':'ARM','/actuation/clear_stop':'CLEAR','/actuation/disarm':'DISARM','/actuation/stop':'STOP'}
OBSERVATIONS={'runtime_drive_observation_v1','runtime_observation_v1'}
SHUTDOWN_DRAIN_NS=500_000_000


def jsonable(value):
    if is_dataclass(value):return jsonable(asdict(value))
    if isinstance(value,Path):return str(value)
    if isinstance(value,bytes):return {'wire_hex':value.hex()}
    if isinstance(value,dict):return {str(k):jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [jsonable(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite':repr(value)}
    return value


def strict_json(raw):
    def pairs(items):
        result={}
        for key,value in items:
            if key in result:raise ValueError('duplicate JSON key: '+key)
            result[key]=value
        return result
    def invalid(value):raise ValueError('nonfinite JSON number: '+value)
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=invalid)


@dataclass(frozen=True)
class ClockPolicy:
    receiver_lease: ReceiverLeaseTiming
    same_host: SameHostClockBounds
    max_lease_us: int
    request_timeout_ns: int


def load_policy(path):
    raw=Path(path).read_bytes();data=strict_json(raw)
    if type(data) is not dict or set(data)!={'schema','receiver_lease','same_host','max_lease_us','request_timeout_ns'}:
        raise ValueError('exact runtime host clock policy keys required')
    if data['schema']!='runtime_host_clock_policy_v1':raise ValueError('unknown runtime host clock policy')
    for name,kind in (('receiver_lease',ReceiverLeaseTiming),('same_host',SameHostClockBounds)):
        if type(data[name]) is not dict or set(data[name])!={field.name for field in fields(kind)}:
            raise ValueError('exact explicit '+name+' fields required')
    policy=ClockPolicy(ReceiverLeaseTiming(**data['receiver_lease']),SameHostClockBounds(**data['same_host']),
        integer(data['max_lease_us'],'max lease',10_000_000),integer(data['request_timeout_ns'],'request timeout',60_000_000_000))
    if not policy.max_lease_us or not policy.request_timeout_ns:raise ValueError('positive lease and request timeout required')
    if policy.receiver_lease.source_ttl_ns!=policy.same_host.source_ttl_ns:raise ValueError('receiver and same-host source TTL must match')
    return policy,dict(sha256=hashlib.sha256(raw).hexdigest(),declared=data)


def load_motor_config(path):
    if path is None:return None,None
    raw=Path(path).read_bytes();data=strict_json(raw)
    required={'schema','counts_per_motor_revolution','gear_ratio','wheel_radius_m','encoder_sign',
              'maximum_interval_ns','maximum_age_ns','maximum_speed_mps'}
    if type(data) is not dict or set(data)!=required or data['schema']!='runtime_motor_feedback_v1':
        raise ValueError('exact runtime_motor_feedback_v1 configuration required')
    if type(data['maximum_speed_mps']) not in (int,float):raise ValueError('numeric motor speed bound required')
    spec=MotorEncoderSpec(data['counts_per_motor_revolution'],data['gear_ratio'],data['wheel_radius_m'])
    converter=MotorFeedbackRuntime(spec,**{name:data[name] for name in
        ('encoder_sign','maximum_interval_ns','maximum_age_ns','maximum_speed_mps')})
    return converter,dict(sha256=hashlib.sha256(raw).hexdigest(),declared=data)


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,required=True)
    p.add_argument('--clock-policy',type=Path,required=True)
    p.add_argument('--max-speed-mps',type=float,required=True)
    p.add_argument('--max-steering-rad',type=float,required=True)
    p.add_argument('--enable-drive-wire',action='store_true')
    p.add_argument('--imu-topic',default='/hardware_runtime/imu')
    p.add_argument('--debug-imu-events',action='store_true',help='원 UART 외 상세 IMU JSON도 기록; 기본 비활성')
    p.add_argument('--motor-feedback-config',type=Path,help='명시한 경우에만 실제 count 관측의 JointState 발행')
    return p


def validate_args(args):
    if not math.isfinite(args.seconds) or not 1<=args.seconds<=3600:raise ValueError('seconds outside finite 1..3600')
    if any(not math.isfinite(v) or v<=0 for v in (args.max_speed_mps,args.max_steering_rad)):
        raise ValueError('explicit finite positive drive limits required')
    if not args.port.strip():raise ValueError('explicit serial port required')
    if (re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*',args.imu_topic) is None
            or args.imu_topic in ('/drive','/drive/safe','/actuation/tx','/actuation/status') or args.imu_topic in SERVICE_KINDS):
        raise ValueError('separate absolute IMU observation topic required')
    if args.output.exists():raise ValueError('fresh output directory required')
    return load_policy(args.clock_policy)


def exclusive_serial_factory(**options):
    import serial
    return serial.Serial(**options,exclusive=True)


@dataclass
class ControlTicket:
    request: str
    kind: str
    wire: bytes
    started_ns: int
    deadline_ns: int
    future: object=None
    written_ns: int | None=None
    ack: dict | None=None
    result: dict | None=None


class RuntimeHostOwner:
    """ROS 단일 executor 소유. I/O thread는 receipt/mailbox만 공유한다."""
    def __init__(self,*,engine,ingress,policy,clock,monotonic_ns,ros_now_ns,read_mode,event,
                 raw_rx=None,raw_tx=None,publish_imu=None,publish_status=None,watchdog_ns=time.perf_counter_ns,
                 log_imu_details=False,motor_feedback=None):
        self.engine,self.io,self.policy,self.clock=engine,ingress,policy,clock
        self.mono,self.ros_now,self.read_mode=monotonic_ns,ros_now_ns,read_mode
        self.event_sink=event;self.raw_rx,self.raw_tx=raw_rx,raw_tx
        self.publish_imu=publish_imu or (lambda _:None);self.publish_status=publish_status or (lambda _:None)
        self.watchdog_ns=watchdog_ns
        self.log_imu_details=log_imu_details
        self.mixed,self.control,self.imu=SensorLinkDecoder(),Decoder(),ImuDecoder()
        self.imu_clock,self.status_clock=ImuFifoClock(),McuStatusClock()
        self.counts=Counter();self.tickets={};self.completed=deque(maxlen=64)
        self.terminating=False;self.exit_reason=None;self.shutdown_result=None
        self.last_capture_after_ns=None
        self.motor_feedback=motor_feedback
        self.motor_first=self.motor_last=None
        self.motor_first_publish_ns=self.motor_last_publish_ns=None
        self.motor_last_log_ns=None

    def motor_snapshot(self):
        n=self.counts['motor_published']
        span=None if n<2 else self.motor_last_publish_ns-self.motor_first_publish_ns
        return dict(enabled=self.motor_feedback is not None,published=n,
            conversion_counts={} if self.motor_feedback is None else dict(self.motor_feedback.counts),
            first=self.motor_first,last=self.motor_last,
            publication_observed_rate_hz=None if not span or span<0 else (n-1)*1e9/span,
            rate_basis='owner_processing_monotonic_not_sensor_capture',
            time_contract_verified=False,physical_motion_verified=False)

    def note_motor_published(self,motion):
        now=self.mono();self.counts['motor_published']+=1
        self.motor_last=asdict(motion);self.motor_last_publish_ns=now
        if self.motor_first is None:self.motor_first=dict(self.motor_last);self.motor_first_publish_ns=now
        if self.motor_last_log_ns is None or now-self.motor_last_log_ns>=1_000_000_000:
            self.event('motor_feedback_summary',**self.motor_snapshot());self.motor_last_log_ns=now

    def event(self,event_type,**details):self.event_sink(dict(event=event_type,**jsonable(details)))

    def fatal(self,reason):
        if not self.terminating:
            self.terminating=True;self.exit_reason=reason
            self.event('termination_requested',reason=reason,stop_transmission_pending=True,physical_stop_verified=False)

    def session_closed(self):return self.engine.closed or (self.engine.session is not None and self.engine.session.closed)

    def finish(self,ticket,success,reason):
        if ticket.result is not None:return
        ticket.result=dict(success=bool(success),reason=reason,request=ticket.request,kind=ticket.kind,
            tx_written=ticket.written_ns is not None,ack=ticket.ack,physical_stop_verified=False,physical_output_verified=False)
        self.event('control_completed',**ticket.result)
        self.completed.append(ticket.result)
        if ticket.future is not None and not ticket.future.done():ticket.future.set_result(ticket.result)

    def request_control(self,kind,*,future=None,shutdown=False):
        now=self.mono()
        if kind not in CONTROL_REASONS:raise ValueError('known explicit control required')
        if self.terminating and kind!='STOP':result=dict(reason='runner_terminating',envelope=None)
        elif kind!='STOP' and any(t.kind=='STOP' and t.result is None for t in self.tickets.values()):
            result=dict(reason='stop_confirmation_pending',envelope=None)
        elif len([t for t in self.tickets.values() if t.result is None])>=16 and kind!='STOP':
            result=dict(reason='control_capacity',envelope=None)
        else:result=self.engine.control(kind,now)
        if kind=='STOP':
            for ticket in self.tickets.values():
                if ticket.result is None:self.finish(ticket,False,'superseded_by_stop_unconfirmed')
        envelope=result.get('envelope');wire=result.get('wire',b'')
        ticket=ControlTicket(envelope['request'] if envelope else 'not-prepared-'+uuid.uuid4().hex,
            kind,wire,now,now+self.policy.request_timeout_ns,future)
        self.tickets={key:value for key,value in self.tickets.items() if value.result is None}
        self.tickets[ticket.request]=ticket
        self.event('control_prepared',kind=kind,prepared=result,deadline_ns=ticket.deadline_ns,shutdown=shutdown)
        if not wire:self.finish(ticket,False,result['reason'])
        else:
            try:self.io.send(wire);self.counts['queued_'+kind]+=1
            except Exception as error:
                self.finish(ticket,False,'enqueue_failed:'+repr(error));self.fatal('serial_send_failed')
        if self.session_closed() and not shutdown:self.fatal('session_closed')
        return ticket

    def _acks(self,row,receipt_ns,processed_ns):
        if row['type']!='ack':return
        ticket=self.tickets.get(row['request'])
        if ticket is None or ticket.result is not None:return
        if not ticket.started_ns<=receipt_ns<=processed_ns<ticket.deadline_ns:return
        ticket.ack=dict(row,receipt_monotonic_ns=receipt_ns,processed_monotonic_ns=processed_ns)
        if not row['accepted'] or row['reason']!=CONTROL_REASONS[ticket.kind]:
            self.finish(ticket,False,'ack_rejected_or_reason_mismatch')

    def poll_controls(self,now):
        if not self.session_closed():self.engine.poll(now)
        session=self.engine.session
        outcomes={} if session is None else {r['request']:r['outcome'] for r in session.outcomes}
        for ticket in self.tickets.values():
            if ticket.result is not None:continue
            outcome=outcomes.get(ticket.request)
            if now>=ticket.deadline_ns:self.finish(ticket,False,'timeout_unconfirmed')
            elif ticket.ack is not None and (not ticket.ack['accepted'] or ticket.ack['reason']!=CONTROL_REASONS[ticket.kind]):
                self.finish(ticket,False,'ack_rejected_or_reason_mismatch')
            elif ticket.kind=='STOP' and ticket.ack is not None and ticket.written_ns is not None:
                self.finish(ticket,True,'stop_ack_not_physical_stop')
            elif outcome=='transition_evidenced_not_output_permission' and ticket.written_ns is not None:
                self.finish(ticket,True,outcome)
            elif outcome is not None and outcome not in ('transition_evidenced_not_output_permission','stop_ack_not_physical_stop'):
                self.finish(ticket,False,outcome)
        if self.session_closed():self.fatal('session_closed')

    def _control_row(self,row,receipt,processed_ns):
        self.event('control_received',payload=row,receipt_monotonic_ns=receipt.monotonic_ns,
            receipt_realtime_ns=receipt.realtime_ns,processed_monotonic_ns=processed_ns)
        if row.get('type') in OBSERVATIONS:
            self.counts['runtime_observations']+=1
            self.event('runtime_observation',payload=row,session_authority=False)
            return
        try:parts=adapt_runtime_payload(row,max_lease_us=self.policy.max_lease_us)
        except ValueError as error:
            self.event('control_schema_rejected',reason=str(error),payload=row,session_authority=False)
            return
        self._acks(parts.compact,receipt.monotonic_ns,processed_ns)
        # closed 상태에서도 decoder/tx receipt/STOP ACK 회수는 계속한다.
        if not self.session_closed():
            try:
                result=self.engine.observe(row,receipt_ns=receipt.monotonic_ns,processed_ns=processed_ns)
                self.event('session_observed',result=result)
            except Exception as error:self.event('session_observe_error',reason=repr(error));self.fatal('session_observe_error')
        if parts.status is not None:
            for status in self.status_clock.convert_batch([row],monotonic_ns=receipt.monotonic_ns,realtime_ns=receipt.realtime_ns):
                self.publish_status(status);self.event('status_published',payload=status,time_contract_verified=False)
        if self.session_closed():self.fatal('session_closed')

    def receive(self,*,timeout=0.):
        receipt=self.io.get(timeout=timeout)
        if receipt is None:return False
        if receipt.kind=='tx':
            offset=self.raw_tx.tell() if self.raw_tx is not None else None
            if self.raw_tx is not None:self.raw_tx.write(receipt.data)
            matched=None
            for ticket in self.tickets.values():
                if ticket.wire==receipt.data and ticket.written_ns is None:
                    ticket.written_ns=receipt.monotonic_ns;matched=ticket
            self.event('wire_written',receipt_monotonic_ns=receipt.monotonic_ns,receipt_realtime_ns=receipt.realtime_ns,
                raw_offset=offset,wire_hex=receipt.data.hex(),sha256=hashlib.sha256(receipt.data).hexdigest(),
                control_kind=None if matched is None else matched.kind,
                control_request=None if matched is None else matched.request)
            self.counts['tx_receipts']+=1
            return True
        if receipt.kind!='rx':raise ValueError('unknown serial receipt kind')
        offset=self.raw_rx.tell() if self.raw_rx is not None else None
        if self.raw_rx is not None:self.raw_rx.write(receipt.data)
        self.event('uart_received',bytes=len(receipt.data),raw_offset=offset,receipt_monotonic_ns=receipt.monotonic_ns,
            receipt_realtime_ns=receipt.realtime_ns,processed_monotonic_ns=self.mono())
        imu_rows=[]
        for kind,packet in self.mixed.feed(receipt.data):
            if kind=='control':
                for row in self.control.feed(packet):self._control_row(row,receipt,self.mono())
            else:
                for row in self.imu.feed(packet):
                    imu_rows.append(row);self.counts['imu_received']+=1
                    if self.log_imu_details:
                        self.event('imu_received',payload=row,receipt_monotonic_ns=receipt.monotonic_ns,
                            receipt_realtime_ns=receipt.realtime_ns)
        if imu_rows:
            for stamp in self.imu_clock.convert_batch(imu_rows,host_monotonic_ns=receipt.monotonic_ns,ros_receipt_ns=receipt.realtime_ns):
                self.publish_imu(stamp);self.counts['imu_published']+=1
                if self.log_imu_details:
                    self.event('imu_published',estimate=stamp,time_basis=self.imu_clock.time_basis,time_contract_verified=False)
        self.counts['rx_receipts']+=1
        return True

    def capture_clock(self):
        try:
            pair=self.clock.capture(monotonic_ns=self.mono,ros_now_ns=self.ros_now,read_mode=self.read_mode)
            self.last_capture_after_ns=pair.host_after_ns
            self.event('same_host_clock_pair',pair=pair,ready=self.clock.ready,reanchors=self.clock.reanchors)
            return True
        except CommandClockError as error:
            self.event('same_host_clock_rejected',reason=error.reason,clock_fault=error.clock_fault,transient=error.transient)
            if error.clock_fault:self.fatal('same_host_clock:'+error.reason)
            return False

    def drive(self,*,source_ros_ns,speed_mps,steering_rad):
        self.event('drive_received',source_ros_ns=source_ros_ns,speed_mps=speed_mps,steering_rad=steering_rad,
            receipt_monotonic_ns=self.mono())
        if self.terminating:return
        if not self.capture_clock():return
        now=self.mono()
        if self.last_capture_after_ns<=now<self.last_capture_after_ns+self.policy.same_host.host_timestamp_error_ns:
            # 방금 읽은 구간과 now 순서가 오차 범위 안이면 이 명령만 생략한다.
            # 관측된 역행이 아니며 기다리거나 header/now를 미래로 옮기지 않는다.
            self.event('drive_source_rejected',source_ros_ns=source_ros_ns,
                reason='processing_now_within_capture_uncertainty',clock_fault=False,transient=True)
            return
        try:source=self.clock.map_source(source_ros_ns,source_ros_epoch=self.clock.ros_epoch,
            now=HostTimeWindow(self.clock.host_epoch,now,now))
        except CommandClockError as error:
            self.event('drive_source_rejected',source_ros_ns=source_ros_ns,reason=error.reason,
                clock_fault=error.clock_fault,transient=error.transient)
            if error.clock_fault:self.fatal('same_host_clock:'+error.reason)
            return
        result=self.engine.drive(speed_mps=speed_mps,steering_rad=steering_rad,source=source.source,now_ns=now)
        self.event('drive_prepared',source=source,prepared=result)
        if result.get('wire') is not None:
            try:self.io.send(result['wire']);self.counts['queued_DRIVE']+=1
            except Exception as error:self.event('drive_enqueue_failed',reason=repr(error));self.fatal('serial_send_failed')
        if self.session_closed():self.fatal('session_closed')

    def tick(self):
        if not self.terminating:self.capture_clock()
        try:
            for _ in range(64):
                if not self.receive():break
            self.poll_controls(self.mono())
        except Exception as error:self.event('owner_error',reason=repr(error));self.fatal('serial_or_decode_error')

    def shutdown(self,reason):
        self.terminating=True
        if self.exit_reason is None:self.exit_reason=reason
        began=self.watchdog_ns();deadline=began+SHUTDOWN_DRAIN_NS
        ticket=None;drain_error=None
        try:
            ticket=self.request_control('STOP',shutdown=True)
            if ticket.wire and ticket.result is None:
                while self.watchdog_ns()<deadline:
                    try:self.receive(timeout=.002);self.poll_controls(self.mono())
                    except Exception as error:drain_error=repr(error);break
                    if ticket.written_ns is not None and ticket.result is not None:break
        except Exception as error:drain_error=repr(error)
        finally:
            # send()는 enqueue다. matching tx receipt/제한시간 종료 뒤에만 close한다.
            try:self.io.close()
            except Exception as error:drain_error=drain_error or repr(error)
            self.engine.close('runner_shutdown')
            for pending in self.tickets.values():
                if pending.result is None:self.finish(pending,False,'shutdown_unconfirmed')
        self.shutdown_result=dict(reason=self.exit_reason,request=None if ticket is None else ticket.request,
            matching_stop_tx_receipt=ticket is not None and ticket.written_ns is not None,
            stop_ack_confirmed=ticket is not None and ticket.result is not None and ticket.result['success'],
            drain_limit_ns=SHUTDOWN_DRAIN_NS,elapsed_watchdog_ns=self.watchdog_ns()-began,error=drain_error,
            physical_stop_verified=False,already_writing_frame_cancelled=False)
        self.event('shutdown_completed',**self.shutdown_result)
        return self.shutdown_result


def bind_ros(node,owner,*,types,callback_group,future_factory,imu_topic):
    imu_pub=node.create_publisher(types['Imu'],imu_topic,types['sensor_qos'])
    status_pub=node.create_publisher(types['String'],'/actuation/status',10)
    def imu(stamp):
        message=types['Imu']();message.header.frame_id='lsm6dsox_sensor'
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(stamp.stamp_ns,1_000_000_000)
        message.orientation_covariance[0]=-1.;message.linear_acceleration_covariance[0]=-1.
        message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z=stamp.sample['gyro_rad_s']
        imu_pub.publish(message)
    owner.publish_imu=imu
    motor_pub=None
    if owner.motor_feedback is not None:motor_pub=node.create_publisher(types['JointState'],MOTOR_TOPIC,10)
    def status(data):
        status_pub.publish(types['String'](data=json.dumps(jsonable(data),allow_nan=False)))
        if owner.motor_feedback is None:return
        motion=owner.motor_feedback.convert(data)
        if motion is None:return
        message=types['JointState']();message.header.frame_id=MOTOR_FRAME
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(motion.stamp_ns,1_000_000_000)
        message.name=[MOTOR_JOINT];message.position=[motion.position_rad];message.velocity=[motion.velocity_rad_s]
        motor_pub.publish(message);owner.note_motor_published(motion)
    owner.publish_status=status
    def drive(message):
        sec,nano=message.header.stamp.sec,message.header.stamp.nanosec
        if type(sec) is not int or type(nano) is not int or sec<0 or not 0<=nano<1_000_000_000:
            owner.event('drive_source_rejected',reason='invalid_ROS_header_fields');return
        owner.drive(source_ros_ns=sec*1_000_000_000+nano,speed_mps=message.drive.speed,steering_rad=message.drive.steering_angle)
    subscription=node.create_subscription(types['Drive'],'/drive/safe',drive,1,callback_group=callback_group)
    services=[]
    for name,kind in SERVICE_KINDS.items():
        async def control(request,response,kind=kind):
            future=future_factory();owner.request_control(kind,future=future)
            result=await future
            response.success=result['success'];response.message=json.dumps(jsonable(result),allow_nan=False)
            return response
        services.append(node.create_service(types['Trigger'],name,control,callback_group=callback_group))
    timer=node.create_timer(.01,owner.tick,callback_group=callback_group)
    return dict(subscription=subscription,services=services,timer=timer,
        publishers=(imu_pub,status_pub) if motor_pub is None else (imu_pub,status_pub,motor_pub))


def main():
    p=parser();args=p.parse_args()
    try:
        policy,declared=validate_args(args)
        motor_feedback,motor_declared=load_motor_config(args.motor_feedback_config)
    except (ValueError,TypeError,KeyError,OSError,OverflowError) as error:p.error(str(error))
    host_epoch='host-'+uuid.uuid4().hex;ros_epoch='ros-'+uuid.uuid4().hex
    engine=RuntimeActuationLink(host_epoch=host_epoch,client_id='runtime-'+uuid.uuid4().hex,
        timing=policy.receiver_lease,max_speed_mps=args.max_speed_mps,max_steering_rad=args.max_steering_rad,
        max_lease_us=policy.max_lease_us,request_timeout_ns=policy.request_timeout_ns,drive_enabled=args.enable_drive_wire)
    clock=SameHostCommandClock(host_epoch=host_epoch,ros_epoch=ros_epoch,bounds=policy.same_host)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.signals import SignalHandlerOptions
    from rclpy.task import Future
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu,JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from ackermann_msgs.msg import AckermannDriveStamped
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'registered.json').write_text(json.dumps(dict(arguments=jsonable(vars(args)),policy=declared,
        host_epoch=host_epoch,ros_epoch=ros_epoch,drive_wire_enabled=args.enable_drive_wire,motor_feedback=motor_declared,
        physical_mounting_verified=False,clock_bounds_hardware_verified=False),indent=2)+'\n')
    interrupted={'reason':None};old_handlers={}
    def stop_signal(number,_):interrupted['reason']='signal_'+str(number)
    node=executor=owner=None;started=False;failure=None;complete=False;ros_initialized=False
    cleanup_errors=[]
    io=ActuationSerialIngress(args.port,factory=exclusive_serial_factory)
    with (args.output/'uart_rx.bin').open('xb') as rx,(args.output/'uart_tx.bin').open('xb') as tx,(args.output/'events.jsonl').open('x') as events:
        last_flush=time.perf_counter_ns()
        def event(row):
            nonlocal last_flush
            events.write(json.dumps(jsonable(row),allow_nan=False,separators=(',',':'))+'\n')
            now=time.perf_counter_ns()
            critical=(row['event'] in ('termination_requested','shutdown_completed')
                or (row['event']=='same_host_clock_rejected' and row.get('clock_fault'))
                or (row['event']=='wire_written' and row.get('control_kind')=='STOP')
                or (row['event'] in ('control_prepared','control_completed') and row.get('kind')=='STOP'))
            if critical or now-last_flush>=100_000_000:
                rx.flush();tx.flush();events.flush();last_flush=now
        try:
            rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);ros_initialized=True
            for number in (signal.SIGINT,signal.SIGTERM):old_handlers[number]=signal.signal(number,stop_signal)
            node=Node('runtime_actuation_host');ros_clock=node.get_clock()
            def mode():return ros_clock.clock_type.name,bool(getattr(ros_clock,'ros_time_is_active',False))
            if node.get_parameter('use_sim_time').value or mode() not in (('ROS_TIME',False),('SYSTEM_TIME',False)):
                raise RuntimeError('same-host physical ROS system time required before opening serial')
            owner=RuntimeHostOwner(engine=engine,ingress=io,policy=policy,clock=clock,monotonic_ns=time.monotonic_ns,
                ros_now_ns=lambda:ros_clock.now().nanoseconds,read_mode=mode,event=event,raw_rx=rx,raw_tx=tx,
                log_imu_details=args.debug_imu_events,motor_feedback=motor_feedback)
            executor=SingleThreadedExecutor();executor.add_node(node)
            bind_ros(node,owner,types=dict(Imu=Imu,JointState=JointState,String=String,Trigger=Trigger,Drive=AckermannDriveStamped,sensor_qos=qos_profile_sensor_data),
                callback_group=ReentrantCallbackGroup(),future_factory=Future,imu_topic=args.imu_topic)
            io.start();started=True;owner.request_control('STOP')
            began=time.monotonic()
            while not owner.terminating and interrupted['reason'] is None and time.monotonic()-began<args.seconds:
                executor.spin_once(timeout_sec=.01)
            complete=not owner.terminating and interrupted['reason'] is None
        except BaseException as error:failure=type(error).__name__+': '+str(error)
        finally:
            try:
                if owner is not None and started:owner.shutdown(failure or interrupted['reason'] or 'duration_complete')
                else:io.close()
            except Exception as error:
                cleanup_errors.append('STOP drain: '+repr(error))
                try:io.close()
                except Exception as error:cleanup_errors.append('serial close: '+repr(error))
            for label,cleanup in (
                ('executor',lambda:executor.shutdown() if executor is not None else None),
                ('node',lambda:node.destroy_node() if node is not None else None),
                ('ROS',lambda:rclpy.shutdown() if ros_initialized else None)):
                try:cleanup()
                except Exception as error:cleanup_errors.append(label+': '+repr(error))
            for number,previous in old_handlers.items():signal.signal(number,previous)
            result=dict(complete=complete,error=failure,interruption=interrupted['reason'],
                shutdown=None if owner is None else owner.shutdown_result,serial_io=dict(io.stats,error=io.error),
                cleanup_errors=cleanup_errors,exit_reason=None if owner is None else owner.exit_reason,
                counts={} if owner is None else dict(owner.counts),control_outcomes=[] if owner is None else list(owner.completed),
                sensor_wire={} if owner is None else dict(owner.mixed.counters),imu_decoder={} if owner is None else dict(owner.imu.counters),
                control_decode_errors=0 if owner is None else owner.control.errors,
                imu_clock={} if owner is None else dict(owner.imu_clock.counts),status_clock={} if owner is None else dict(owner.status_clock.counts),
                drive_wire_enabled=args.enable_drive_wire,auto_arm=False,reconnected=False,physical_stop_verified=False,
                encoder_motion_published=owner is not None and owner.counts['motor_published']>0,
                motor_feedback=None if owner is None else owner.motor_snapshot(),
                time_contract_verified=False,source_timestamp_rewritten=False)
            (args.output/'summary.json').write_text(json.dumps(jsonable(result),indent=2)+'\n')
    print(json.dumps(jsonable(result)))
    if failure or cleanup_errors or (started and (not complete or result['shutdown'] is None
            or not result['shutdown']['matching_stop_tx_receipt'])):raise SystemExit(1)


if __name__=='__main__':main()
