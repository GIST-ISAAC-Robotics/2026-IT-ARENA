#!/usr/bin/env python3
"""개발 전용 짧은 전진/좌회전/우회전. --send 없이는 계획만 출력한다.

자율 제어기를 끈 별도 시험에서 이미 ARM된 공통 MCU 인터페이스를 사용한다.
ARM/CLEAR/RESET은 수행하지 않으며 /drive/safe를 직접 발행하지 않는다.
"""
import argparse
import json
import math
from pathlib import Path
import time
import uuid


def plan(direction, speed, seconds, speed_limit):
    if direction not in ('forward', 'left', 'right'):
        raise ValueError('direction must be forward/left/right; reverse is unsupported')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (speed,seconds,speed_limit)):
        raise ValueError('finite numeric values required')
    if not .05<=speed_limit<=.3 or not 0<speed<=speed_limit or not .05<=seconds<=2.:
        raise ValueError('speed <= explicit limit (0.05..0.3 m/s), duration 0.05..2 s required')
    return dict(direction=direction,speed_mps=float(speed),duration_wall_s=float(seconds),
        speed_limit_mps=float(speed_limit),steering_rad={'forward':0.,'left':.2,'right':-.2}[direction],
        output_topic='/drive',completion='zero command and explicit MCU STOP',automatic_arm=False,
        scope='development only; not race remote driving or proof of physical stopping')


def valid_mcu(status, now_s, receipt_wall_s, wall_s, *, identity=None):
    if not isinstance(status,dict) or type(status.get('v')) is not int or status['v']!=1:
        return False
    if (not isinstance(status.get('boot'),str) or not status['boot']
            or type(status.get('generation')) is not int or status['generation']<0
            or not isinstance(status.get('owner'),str) or not status['owner']
            or type(status.get('now_us')) is not int or status.get('estop_held') is not False
            or status.get('feedback_valid') is not True or status.get('output_ready') is not True):
        return False
    if not 0<=now_s-status['now_us']*1e-6<=.25 or not 0<=wall_s-receipt_wall_s<=.25:
        return False
    if identity is None:
        return status.get('state')=='ARMED' and status.get('target_speed_mps')==0
    return (status.get('state') in ('ARMED','ACTIVE')
        and (status['boot'],status['generation'],status.get('owner'))==identity)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction',choices=('forward','left','right'))
    parser.add_argument('--speed',type=float,default=.15)
    parser.add_argument('--seconds',type=float,default=.5)
    parser.add_argument('--speed-limit',type=float,default=.3)
    parser.add_argument('--send',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    try:request=plan(args.direction,args.speed,args.seconds,args.speed_limit)
    except ValueError as error:parser.error(str(error))
    if not args.send:
        print(json.dumps(request,ensure_ascii=False,indent=2));return
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init(args=[])
    node=Node('development_jog_'+uuid.uuid4().hex[:8])
    output=args.output or Path('build/development_jog')/str(time.time_ns())
    output.mkdir(parents=True,exist_ok=False)
    (output/'request.json').write_text(json.dumps(request,indent=2)+'\n')
    publisher=node.create_publisher(AckermannDriveStamped,'/drive',1)
    stop=node.create_client(Trigger,'/actuation/stop')
    latest=None;receipt=-math.inf
    events=[];started=False;failure=None;stop_ack=False;target_zero=False
    identity=None

    def on_status(message):
        nonlocal latest,receipt
        try:latest=json.loads(message.data)
        except (ValueError,TypeError):latest=None
        receipt=time.monotonic()

    node.create_subscription(String,'/actuation/status',on_status,10)

    def now():return node.get_clock().now().nanoseconds*1e-9

    def graph_ready():
        drive=node.get_publishers_info_by_topic('/drive')
        safe=node.get_publishers_info_by_topic('/drive/safe')
        readers=node.get_subscriptions_info_by_topic('/drive')
        return (len(drive)==1 and drive[0].node_name==node.get_name()
            and len(safe)==1 and safe[0].node_name=='lidar_safety'
            and any(r.node_name=='lidar_safety' for r in readers))

    def publish(speed):
        msg=AckermannDriveStamped();msg.header.stamp=node.get_clock().now().to_msg()
        msg.header.frame_id='base_link';msg.drive.speed=float(speed)
        msg.drive.steering_angle=request['steering_rad'] if speed else 0.
        publisher.publish(msg)
        events.append(dict(wall_s=time.monotonic(),stamp_s=now(),speed_mps=speed,
            steering_rad=msg.drive.steering_angle))

    try:
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if graph_ready() and stop.service_is_ready() and valid_mcu(latest,now(),receipt,time.monotonic()):
                break
        else:raise RuntimeError('exclusive /drive, independent safety, STOP service and fresh ARMED MCU required')
        identity=(latest['boot'],latest['generation'],latest.get('owner'))
        began=time.monotonic();next_emit=began;started=True
        while time.monotonic()-began<request['duration_wall_s']:
            rclpy.spin_once(node,timeout_sec=.005)
            wall=time.monotonic()
            if not valid_mcu(latest,now(),receipt,wall,identity=identity):
                raise RuntimeError('MCU stopped, changed or stale; jog cancelled')
            if wall>=next_emit:
                if not graph_ready():raise RuntimeError('drive ownership or independent safety changed')
                publish(request['speed_mps']);next_emit=wall+.05
    except BaseException as error:
        failure=type(error).__name__+': '+str(error)
    finally:
        if started:
            try:
                publish(0.)
                future=stop.call_async(Trigger.Request())
                deadline=time.monotonic()+1.
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.01)
                    if future.done():
                        response=future.result();stop_ack=bool(response and response.success)
                    target_zero=bool(isinstance(latest,dict) and latest.get('boot')==identity[0]
                        and type(latest.get('generation')) is int and latest['generation']>identity[1]
                        and latest.get('state')=='ESTOP' and latest.get('target_speed_mps')==0
                        and type(latest.get('now_us')) is int and 0<=now()-latest['now_us']*1e-6<=.25
                        and 0<=time.monotonic()-receipt<=.25)
                    if stop_ack and target_zero:break
            except BaseException as error:
                failure=(failure+'; ' if failure else '')+'cleanup: '+str(error)
        result=dict(started=started,error=failure,commands=events,stop_ack=stop_ack,
            fresh_estop_zero_target_seen=target_zero,physical_stop_verified=False,
            armed_automatically=False,output_topic='/drive')
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps({k:v for k,v in result.items() if k!='commands'},ensure_ascii=False))
    if failure or not (started and stop_ack and target_zero):raise SystemExit(1)


if __name__=='__main__':main()
