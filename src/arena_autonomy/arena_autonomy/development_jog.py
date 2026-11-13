#!/usr/bin/env python3
"""개발 전용 짧은 전진/좌회전/우회전. --send 없이는 계획만 출력한다.

자율 제어기를 끈 별도 시험에서 이미 ARM된 공통 MCU 인터페이스를 사용한다.
ARM/CLEAR/RESET은 수행하지 않으며 /drive/safe를 직접 발행하지 않는다.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

from arena_autonomy.vehicle_geometry import float32_limit_not_above
import uuid


MAX_STEERING_RAD = 0.4578436017036438


def plan(direction, speed, seconds, speed_limit, *, steering_rad=None, max_steering_rad=MAX_STEERING_RAD):
    if direction not in ('forward', 'left', 'right'):
        raise ValueError('direction must be forward/left/right; reverse is unsupported')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (speed,seconds,speed_limit)):
        raise ValueError('finite numeric values required')
    if not .05<=speed_limit<=.2 or not 0<speed<=speed_limit or not .05<=seconds<=2.:
        raise ValueError('speed <= explicit limit (0.05..0.2 m/s), duration 0.05..2 s required')
    angle={'forward':0.,'left':.2,'right':-.2}[direction] if steering_rad is None else steering_rad
    if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (angle,max_steering_rad))
        or not 0<max_steering_rad<=MAX_STEERING_RAD or abs(angle)>max_steering_rad):
        raise ValueError('central equivalent wheel angle exceeds configured limit')
    return dict(direction=direction,speed_mps=float(speed),duration_wall_s=float(seconds),
        speed_limit_mps=float(speed_limit),steering_rad=float(angle),steering_deg=math.degrees(angle),
        steering_reference='central_equivalent_wheel_angle_left_positive',
        estimated_distance_m=float(speed*seconds),distance_estimate='v*t only, not measured or guaranteed',
        output_topic='/drive',completion='zero speed holding requested steering, then MCU STOP',automatic_arm=False,
        scope='development only; trial bounds, not calibrated physical limits')


class ParentHeartbeat:
    """패널 자식의 stdin EOF/STOP/연결 공백은 정상 finally 회수로 전달한다."""
    def __init__(self, stream, timeout_s=.75, wall=time.monotonic):
        self.wall,self.timeout_s=wall,timeout_s
        self.last=wall();self.stopped=False
        def read():
            try:
                pending=b''
                while True:
                    chunk=os.read(stream.fileno(),256)
                    if not chunk:break
                    pending+=chunk
                    while b'\n' in pending:
                        line,pending=pending.split(b'\n',1)
                        if line.strip()!=b'HEARTBEAT':return
                        self.last=wall()
                    if len(pending)>256:break
            finally:self.stopped=True
        self.thread=threading.Thread(target=read,name='panel_heartbeat',daemon=True)
        self.thread.start()

    def cancelled(self):return self.stopped or self.wall()-self.last>=self.timeout_s


RUNTIME_TIME_BASIS='minimum_receipt_offset_mcu_state_estimate'


def runtime_status(status):
    return isinstance(status,dict) and any(k in status for k in ('time_basis','motor_feedback','scope'))


def fresh_status(status, now_s, receipt_wall_s, wall_s):
    if (type(status) is not dict or type(status.get('v')) is not int or status['v']!=1
        or type(status.get('boot')) is not str or not status['boot']
        or type(status.get('generation')) is not int or status['generation']<0
        or type(status.get('now_us')) is not int or status['now_us']<0):return False
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (now_s,receipt_wall_s,wall_s)):return False
    if not 0<=now_s-status['now_us']*1e-6<=.25 or not 0<=wall_s-receipt_wall_s<=.25:return False
    if runtime_status(status):
        if (status.get('time_basis')!=RUNTIME_TIME_BASIS or status.get('time_contract_verified') is not False
            or status.get('scope')!='observation_only_no_drive'
            or any(type(status.get(k)) is not int or status[k]<0 for k in
                   ('clock_epoch','mcu_now_us','host_receipt_ns','receipt_age_ns'))):return False
        # McuStatusClock의 mapped ns -> now_us 내림과 원 receipt/age를 그대로 확인한다.
        if not 0<=status['host_receipt_ns']-status['receipt_age_ns']-status['now_us']*1000<1000:return False
    return True


def mcu_identity(status):
    owner=('runtime_observation',status['clock_epoch']) if runtime_status(status) else status['owner']
    return status['boot'],status['generation'],owner


def runtime_feedback_observed(status, now_s):
    raw=status.get('motor_feedback')
    if (type(raw) is not dict or raw.get('schema')!='motor_feedback_observation_v1'
        or raw.get('source_valid') is not True or raw.get('time_contract_verified') is not False
        or raw.get('time_basis')!=RUNTIME_TIME_BASIS
        or any(raw.get(k)!=status[k] for k in ('boot','generation','clock_epoch','host_receipt_ns'))
        or any(type(raw.get(k)) is not int or raw[k]<0 for k in ('sequence','mcu_capture_us','estimated_capture_ns'))
        or type(raw.get('motor_counts')) is not int):return False
    return (raw['mcu_capture_us']<=status['mcu_now_us']
        and raw['estimated_capture_ns']==status['now_us']*1000-(status['mcu_now_us']-raw['mcu_capture_us'])*1000
        and 0<=now_s-raw['estimated_capture_ns']*1e-9<=.25)


def valid_mcu(status, now_s, receipt_wall_s, wall_s, *, identity=None):
    if (not fresh_status(status,now_s,receipt_wall_s,wall_s)
        or status.get('estop_held') is not False or status.get('output_ready') is not True):
        return False
    runtime=runtime_status(status)
    if runtime:
        if not runtime_feedback_observed(status,now_s):return False
    elif (status.get('feedback_valid') is not True or type(status.get('owner')) is not str
          or not status['owner']):return False
    if identity is None:
        # 현재 receiver의 ARMED는 0목표이다. 관측에 없는 owner/target을 합성하지 않는다.
        return status.get('state')=='ARMED' and (runtime or status.get('target_speed_mps')==0)
    return status.get('state') in ('ARMED','ACTIVE') and mcu_identity(status)==identity


def stop_observed(status, now_s, receipt_wall_s, wall_s, identity):
    if (not fresh_status(status,now_s,receipt_wall_s,wall_s) or status['boot']!=identity[0]
        or status['generation']<=identity[1] or status.get('state')!='ESTOP_LATCHED'):return False
    if isinstance(identity[2],tuple):
        return runtime_status(status) and identity[2]==('runtime_observation',status['clock_epoch'])
    return not runtime_status(status) and status.get('target_speed_mps')==0


def graph_ready(node):
    def peer(info):return info.node_namespace,info.node_name
    own=node.get_namespace(),node.get_name()
    drive=node.get_publishers_info_by_topic('/drive')
    if any(peer(info)!=own for info in drive):
        raise RuntimeError('another /drive publisher exists; cancel this jog without terminating that node')
    safe=node.get_publishers_info_by_topic('/drive/safe')
    readers=node.get_subscriptions_info_by_topic('/drive')
    hosts=node.get_subscriptions_info_by_topic('/drive/safe')
    status=node.get_publishers_info_by_topic('/actuation/status')
    return (len(drive)==1 and len(safe)==1 and safe[0].node_name=='lidar_safety'
        and any(peer(r)==peer(safe[0]) for r in readers)
        and any(r.node_name in ('actuation_host','runtime_actuation_host') for r in hosts) and len(status)==1)


def pulse(request, *, wall, pump, check, publish, cancelled):
    """고정 목표의 제한 실행. 합성 호출 검사는 실제 scheduling 검증이 아니다."""
    began=next_emit=wall()
    while wall()-began<request['duration_wall_s']:
        pump(.005)
        if cancelled():raise InterruptedError('jog interrupted')
        current=wall()
        if current-began>=request['duration_wall_s']:break
        check()
        if current>=next_emit:
            publish(request['speed_mps'],request['steering_rad']);next_emit=current+.05


def finish(*, publish_zero, call_stop, wall, pump, observed):
    """0 발행 오류와 STOP 요청을 독립 시도. ACK/상태/실제 정지를 분리한다."""
    result=dict(zero_published=False,stop_requested=False,stop_ack=False,stop_response=None,
        fresh_estop_state_seen=False,physical_stop_verified=False,errors=[])
    try:publish_zero();result['zero_published']=True
    except BaseException as error:result['errors'].append('zero_publish: '+repr(error))
    future=None
    try:future=call_stop();result['stop_requested']=True
    except BaseException as error:result['errors'].append('stop_request: '+repr(error))
    try:
        deadline=wall()+1.  # 기존 r169 회수 예산; 전달/물리 정지 보증 시간이 아니다.
        while wall()<deadline:
            pump(.01)
            if future is not None and future.done():
                response=future.result();result['stop_ack']=bool(response and response.success is True)
                result['stop_response']=None if response is None else response.message
            result['fresh_estop_state_seen']=bool(observed())
            if result['stop_ack'] and result['fresh_estop_state_seen']:break
    except BaseException as error:result['errors'].append('stop_confirmation: '+repr(error))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction',choices=('forward','left','right'))
    parser.add_argument('--speed',type=float,default=.15)
    parser.add_argument('--seconds',type=float,default=.5)
    parser.add_argument('--speed-limit',type=float,default=.2)
    parser.add_argument('--steering-deg',type=float,help='central equivalent wheel angle; +left/-right, not servo shaft')
    parser.add_argument('--max-steering-rad',type=float,default=MAX_STEERING_RAD)
    parser.add_argument('--parent-heartbeat-stdin',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--node-name',default=None,help=argparse.SUPPRESS)
    parser.add_argument('--send',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    try:request=plan(args.direction,args.speed,args.seconds,args.speed_limit,
        steering_rad=None if args.steering_deg is None else math.radians(args.steering_deg),
        max_steering_rad=args.max_steering_rad)
    except ValueError as error:parser.error(str(error))
    if not args.send:
        print(json.dumps(request,ensure_ascii=False,indent=2));return
    heartbeat=ParentHeartbeat(sys.stdin) if args.parent_heartbeat_stdin else None
    if args.node_name is not None and (not args.node_name.startswith('panel_jog_')
        or not args.node_name.replace('_','').isalnum()):parser.error('invalid panel node name')
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    output=args.output or Path('build/development_jog')/str(time.time_ns())
    output.mkdir(parents=True,exist_ok=False)
    (output/'request.json').write_text(json.dumps(request,indent=2)+'\n',encoding='utf8')
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    interrupted={'value':False};old_handlers={}
    for number in (signal.SIGINT,signal.SIGTERM):
        old_handlers[number]=signal.signal(number,lambda *_:interrupted.update(value=True))
    node=Node(args.node_name or 'development_jog_'+uuid.uuid4().hex[:8])
    publisher=node.create_publisher(AckermannDriveStamped,'/drive',1)
    stop=node.create_client(Trigger,'/actuation/stop')
    latest=None;receipt=-math.inf
    events=[];started=False;failure=None
    ending=dict(zero_published=False,stop_requested=False,stop_ack=False,stop_response=None,
        fresh_estop_state_seen=False,physical_stop_verified=False,errors=[])
    identity=None

    def on_status(message):
        nonlocal latest,receipt
        try:latest=json.loads(message.data)
        except (ValueError,TypeError):latest=None
        receipt=time.monotonic()

    node.create_subscription(String,'/actuation/status',on_status,10)

    def now():return node.get_clock().now().nanoseconds*1e-9

    def pump(seconds):rclpy.spin_once(node,timeout_sec=seconds)

    def cancelled():return interrupted['value'] or (heartbeat is not None and heartbeat.cancelled())

    def publish(speed,steering):
        msg=AckermannDriveStamped();msg.header.stamp=node.get_clock().now().to_msg()
        msg.header.frame_id='base_link';msg.drive.speed=(float32_limit_not_above(speed) if speed > 0 else float(speed))
        msg.drive.steering_angle=float(steering)
        publisher.publish(msg)
        events.append(dict(wall_s=time.monotonic(),stamp_s=now(),speed_mps=speed,
            steering_rad=msg.drive.steering_angle))

    try:
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            pump(.02)
            if cancelled():raise InterruptedError('jog cancelled before start')
            if graph_ready(node) and stop.service_is_ready() and valid_mcu(latest,now(),receipt,time.monotonic()):
                break
        else:raise RuntimeError('exclusive /drive, independent safety, STOP service and fresh ARMED MCU required')
        identity=mcu_identity(latest)

        def check():
            if not graph_ready(node) or not stop.service_is_ready():
                raise RuntimeError('drive ownership, independent safety or host connection lost')
            if not valid_mcu(latest,now(),receipt,time.monotonic(),identity=identity):
                raise RuntimeError('MCU stopped, changed or stale; jog cancelled')

        started=True  # publish가 예외로 끝나도 부분 전달 가능성이 있으므로 STOP을 시도한다.
        pulse(request,wall=time.monotonic,pump=pump,check=check,publish=publish,
            cancelled=cancelled)
        if not events:raise RuntimeError('duration expired before any jog publication')
    except BaseException as error:
        failure=type(error).__name__+': '+str(error)
    finally:
        if started:
            # DRIVE0도 runtime_servo의 drive_seen 조향 경로로 전달되므로 자동 중앙을 요청하지 않는다.
            ending=finish(publish_zero=lambda:publish(0.,request['steering_rad']),
                call_stop=lambda:stop.call_async(Trigger.Request()),wall=time.monotonic,pump=pump,
                observed=lambda:stop_observed(latest,now(),receipt,time.monotonic(),identity))
        result=dict(started=started,error=failure,commands=events,**ending,
            armed_automatically=False,output_topic='/drive',status_identity=identity,
            state_observation_is_drive_permission=False)
        try:(output/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
        finally:
            try:node.destroy_node()
            finally:
                try:rclpy.shutdown()
                finally:
                    for number,handler in old_handlers.items():signal.signal(number,handler)
    print(json.dumps({k:v for k,v in result.items() if k!='commands'},ensure_ascii=False))
    if failure or ending['errors'] or not (started and ending['stop_ack'] and ending['fresh_estop_state_seen']):
        raise SystemExit(1)


if __name__=='__main__':main()
