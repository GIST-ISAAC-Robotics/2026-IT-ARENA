#!/usr/bin/env python3
"""공통 G1/G2 노드의 실물 입력 관측. 구동 토픽/MCU 없이 시계·입력·종료를 확인한다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time


def vehicle_geometry_override(args):
    path = getattr(args, 'vehicle_config', None)
    if path is None:
        return None
    import yaml
    from arena_autonomy.vehicle_geometry import preview_parameters
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    vehicle = yaml.safe_load(raw)['vehicle']
    return dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(), parameters=preview_parameters(vehicle))


def ros_arguments(args,root,*,geometry=None):
    result=['--ros-args','--params-file',str(root/'config/hardware/g1_input_shadow_v1.yaml'),
        '-p','g1_trace_path:='+str(args.output/'decisions.jsonl'),
        '-r','/scan/native:=/hardware_preview/c1/native']
    if geometry is None:
        geometry = vehicle_geometry_override(args)
    if geometry is not None:
        for name, value in geometry['parameters'].items():
            result += ['-p', name+':='+str(value)]
    if args.estimated_time:result += ['-p','motion_scan_time_policy:=host_estimated_v1']
    if args.camera:result += ['-r','/camera/color/image_raw:=/hardware_preview/camera/image_raw',
        '-p','vision_execution:=process','-p','start_signal_mode:=official_4red_local_candidate']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--vehicle-config',type=Path,default=None,help='명시 차량 YAML의 기하만 전달. 생략 시 기존 preview 설정 유지')
    p.add_argument('--seconds',type=float,default=40.)
    p.add_argument('--estimated-time',action='store_true',
        help='추정 시각을 명시 허용하는 비구동 계산. 시각 검증/엔코더 대체가 아님')
    p.add_argument('--camera',action='store_true',help='관측 전용 RGB와 별도 영상 계산 프로세스 연결')
    p.add_argument('--mcu-status',action='store_true',help='출력 비활성 sensor-link의 상태 수신 기록')
    args=p.parse_args()
    if not 1<=args.seconds<=120:p.error('seconds outside 1..120')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='216':
        p.error('isolated localhost domain216 required')
    root=Path(__file__).resolve().parents[1]
    geometry=vehicle_geometry_override(args)
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from arena_autonomy.g1_g2_node import G1G2Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String
    stop=False
    def request_stop(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    rclpy.init(args=ros_arguments(args,root,geometry=geometry),
        signal_handler_options=SignalHandlerOptions.NO)
    stamps=[];states=[];commands=[];accepted_scans=0;images=[];vision_results=[];motion_samples=[];mcu_states=[]
    class ObservedNode(G1G2Node):
        def on_image(self,message):
            images.append(dict(stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                received_ns=self.get_clock().now().nanoseconds,frame=message.header.frame_id,
                width=message.width,height=message.height,step=message.step,encoding=message.encoding,
                bytes=len(message.data)))
            super().on_image(message)
        def control(self):
            result=super().control()
            vision=self.vision_pipeline
            if args.camera and vision is not None and vision.last_result is not None:
                if not vision_results or vision_results[-1]['stamp']!=vision.last_result['stamp']:
                    vision_results.append(dict(vision.last_result,accepted_ros_s=self.now_s(),
                        accepted_wall_s=time.monotonic(),ids=list(vision.ids)))
            return result
        def on_scan(self,message):
            nonlocal accepted_scans
            super().on_scan(message)
            accepted_scans+=1
            stamps.append(dict(epoch=message.source_epoch,frame=message.source_frame_id,
                time_basis=message.time_basis,points=len(message.ranges_m),
                first_s=message.acquired_at_s[0],last_s=message.acquired_at_s[-1],received_s=self.now_s()))
    node=None;fault=None;began=time.monotonic()
    try:
        node=ObservedNode()
        assert node.hardware_shadow and not node.motion.verified
        # 원 콜백의 입력 시각/수락을 그대로 기록한다. 별도 센서 구독으로 수신을 대신 세지 않는다.
        node.motion.record=motion_samples.append
        if args.mcu_status:
            node.create_subscription(String,'/hardware_preview/actuation_status',
                lambda m:mcu_states.append(dict(received_ns=node.get_clock().now().nanoseconds,
                                               payload=json.loads(m.data))),10)
        node.create_subscription(AckermannDriveStamped,'/hardware_preview/drive',
            lambda m:commands.append(dict(speed=m.drive.speed,steering=m.drive.steering_angle,
                stamp_s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)),10)
        node.create_subscription(String,'/hardware_preview/autonomy/status',
            lambda m:states.append(json.loads(m.data)),10)
        for topic in ('/drive','/drive/safe','/actuation/tx','/drive_motor/encoder'):
            if node.count_publishers(topic):raise RuntimeError('unexpected publisher: '+topic)
        (args.output/'ready').write_text('common node ready; no actuator output\n')
        while not stop and time.monotonic()-began<args.seconds:rclpy.spin_once(node,timeout_sec=.02)
    except BaseException as exc:
        fault=type(exc).__name__+': '+str(exc)
        raise
    finally:
        summary=dict(fault=fault,elapsed_s=time.monotonic()-began,scans=accepted_scans,
            source_frames=sorted({r['frame'] for r in stamps}),source_epochs=sorted({r['epoch'] for r in stamps}),
            source_time_bases=sorted({r['time_basis'] for r in stamps}),
            states=sorted({s['state'] for s in states}),commands=len(commands),
            nonzero_private_commands=sum(c['speed']!=0 for c in commands),
            motion=dict(node.motion.counts) if node else None,drive_output=False,
            actuator_node_started=False,synthetic_encoder_published=False,time_contract_verified=False,
            selected_vehicle_geometry=geometry,
            motion_time_policy=node.motion.timing_policy if node else None,
            camera_requested=args.camera,camera_connected=bool(images),mounting_calibrated=False,
            motion_samples_recorded=len(motion_samples),mcu_status_requested=args.mcu_status,
            mcu_status_received=len(mcu_states),
            images=len(images),vision=node.vision_pipeline.snapshot() if node and args.camera else None)
        if node is not None:
            child=node.decision_delivery.worker.process
            vision_child=node.vision_pipeline.worker.process if args.camera else None
            node.destroy_node()
            summary['worker_exitcode']=child.exitcode;summary['worker_alive']=child.is_alive()
            if vision_child is not None:
                summary['vision_exitcode']=vision_child.exitcode;summary['vision_alive']=vision_child.is_alive()
        rclpy.shutdown()
        for name,rows in [('scans',stamps),('states',states),('commands',commands),('images',images),
                          ('vision',vision_results),('motion',motion_samples),('mcu_states',mcu_states)]:
            (args.output/(name+'.json')).write_text(json.dumps(rows,separators=(',',':'))+'\n')
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))
    if not accepted_scans or not summary['motion']['gyro'] or summary['nonzero_private_commands']:
        raise RuntimeError('hardware shadow input/zero-output evidence incomplete')
    if args.camera and (not images or not vision_results or summary['vision']['fault']):
        raise RuntimeError('camera/vision observation evidence incomplete')


if __name__=='__main__':main()
