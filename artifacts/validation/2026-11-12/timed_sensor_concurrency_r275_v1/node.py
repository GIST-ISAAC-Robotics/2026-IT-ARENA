#!/usr/bin/env python3
"""공통 G1/G2 노드의 실물 입력 관측. 구동 토픽/MCU 없이 시계·입력·종료를 확인한다."""
import argparse
import json
import os
from pathlib import Path
import signal
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=40.)
    args=p.parse_args()
    if not 1<=args.seconds<=120:p.error('seconds outside 1..120')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='216':
        p.error('isolated localhost domain216 required')
    root=Path(__file__).resolve().parents[4]
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
    rclpy.init(args=['--ros-args','--params-file',str(root/'config/hardware/g1_input_shadow_v1.yaml'),
        '-p','g1_trace_path:='+str(args.output/'decisions.jsonl'),
        '-r','/scan/native:=/hardware_preview/c1/native',
        '-r','/camera/color/image_raw:=/hardware_preview/camera/image_raw',
        '-p','motion_scan_time_policy:=host_estimated_v1','-p','vision_execution:=process','-p','start_signal_mode:=official_4red_local_candidate'],signal_handler_options=SignalHandlerOptions.NO)
    stamps=[];states=[];commands=[];accepted_scans=0;images=[];vision_results=[]
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
            if vision is not None and vision.last_result is not None:
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
            camera_connected=bool(images),mounting_calibrated=False,
            motion_time_policy=node.motion.timing_policy if node else None,
            images=len(images),vision=node.vision_pipeline.snapshot() if node else None)
        if node is not None:
            child=node.decision_delivery.worker.process;vision_child=node.vision_pipeline.worker.process
            node.destroy_node()
            summary['worker_exitcode']=child.exitcode;summary['worker_alive']=child.is_alive()
            summary['vision_exitcode']=vision_child.exitcode;summary['vision_alive']=vision_child.is_alive()
        rclpy.shutdown()
        for name,rows in [('scans',stamps),('states',states),('commands',commands),('images',images),('vision',vision_results)]:
            (args.output/(name+'.json')).write_text(json.dumps(rows,separators=(',',':'))+'\n')
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))
    if not accepted_scans or not summary['motion']['gyro'] or summary['nonzero_private_commands'] or not images or not vision_results or summary['vision']['fault']:
        raise RuntimeError('hardware shadow input/zero-output evidence incomplete')


if __name__=='__main__':main()
