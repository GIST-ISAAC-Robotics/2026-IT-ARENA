"""실물 관측을 선택 운영 노드/독립 보호층으로 전달한다. 구동 호스트는 실행하지 않는다."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--bindings',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=45.)
    args=p.parse_args()
    if not 1 <= args.seconds <= 90:p.error('finite seconds within 1..90 required')
    if os.environ.get('ROS_DOMAIN_ID')!='220' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('isolated localhost domain220 required')
    bindings=json.loads(args.bindings.read_bytes())
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from ackermann_msgs.msg import AckermannDriveStamped
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.lidar_safety import LidarSafety
    stop=False
    def requested(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,requested);signal.signal(signal.SIGTERM,requested)
    ros=['--ros-args','--params-file',str(args.profile),'-p','g1_trace_path:='+str(args.output/'trace.jsonl')]
    for source,target in bindings['ros_remappings']['local_pursuit']:
        ros+=['-r',source+':='+target]
    # 현재 장치는 기존 비구동 IMU 진단 앱이다. 명시 remap만 사용하며 원 시각/frame은 유지한다.
    ros+=['-r','/hardware_runtime/imu:=/hardware_preview/imu']
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO)
    counts=Counter();scans=[];images=[];motion=[];commands=[];states=[];guard_states=[];vision=[]
    class Observed(G1G2Node):
        def on_scan(self,message):
            super().on_scan(message)
            counts['scan']+=1
            scans.append(dict(epoch=message.source_epoch,frame=message.source_frame_id,
                time_basis=message.time_basis,verified=self.motion.verified,
                points=len(message.ranges_m),first_s=message.acquired_at_s[0],last_s=message.acquired_at_s[-1],
                received_s=self.now_s()))
        def on_image(self,message):
            super().on_image(message);counts['image']+=1
            images.append(dict(stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                received_ns=self.get_clock().now().nanoseconds,frame=message.header.frame_id))
        def control(self):
            result=super().control()
            item=self.vision_pipeline.last_result if self.vision_pipeline else None
            if item is not None and (not vision or vision[-1]['stamp']!=item['stamp']):
                vision.append(dict(item,ids=list(self.vision_pipeline.ids)))
            return result
    node=guard=monitor=executor=worker=vision_process=None;error=None;began=time.monotonic()
    summary={}
    try:
        node=Observed();worker=node.decision_delivery.worker
        vision_process=node.vision_pipeline.worker.process
        node.motion.record=motion.append
        guard=LidarSafety();monitor=Node('runtime_inputs_receipt')
        assert node.runtime_dual_selected and not node.hardware_shadow
        assert node.publisher.topic_name=='/drive' and guard.publisher.topic_name=='/drive/safe'
        for topic in ('/drive_motor/encoder','/actuation/tx'):
            assert monitor.count_publishers(topic)==0, ('unexpected publisher',topic)
        assert node.motion.history.config.lidar_frame_id=='c1_unmounted_preview'
        def drive(message,stage):
            commands.append(dict(stage=stage,speed=message.drive.speed,steering=message.drive.steering_angle,
                stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                received_ns=monitor.get_clock().now().nanoseconds))
        monitor.create_subscription(AckermannDriveStamped,'/drive',lambda m:drive(m,'before_guard'),10)
        monitor.create_subscription(AckermannDriveStamped,'/drive/safe',lambda m:drive(m,'after_guard'),10)
        monitor.create_subscription(String,'/autonomy/status',lambda m:states.append(json.loads(m.data)),10)
        monitor.create_subscription(String,'/safety/status',lambda m:guard_states.append(json.loads(m.data)),10)
        executor=SingleThreadedExecutor()
        for owner in (node,guard,monitor):executor.add_node(owner)
        (args.output/'ready').write_text('runtime controller/guard; no actuation host or encoder producer\n')
        while not stop and time.monotonic()-began < args.seconds:
            executor.spin_once(timeout_sec=.02)
        before=worker.generation
        response=node.on_enable(SetBool.Request(data=False),SetBool.Response())
        assert response.success and worker.generation>before
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        summary.update(motion=dict(node.motion.counts),guard_motion=dict(guard.motion.counts),
            motor_samples=len(node.motion.history.wheels.values),
            guard_scan_present=guard.scan is not None,
            source_frame=node.motion.history.config.lidar_frame_id,
            time_policy=node.motion.timing_policy,vision=node.vision_pipeline.snapshot(),
            policy=node.get_parameter('g1_runtime_policy').value,worker_fault=worker.fault,
            encoder_publishers=monitor.count_publishers('/drive_motor/encoder'),
            actuation_tx_publishers=monitor.count_publishers('/actuation/tx'))
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc)
        raise
    finally:
        if executor is not None:executor.shutdown(timeout_sec=1.)
        for owner in (node,guard,monitor):
            if owner is not None:owner.destroy_node()
        rclpy.shutdown()
        summary.update(error=error,elapsed_s=time.monotonic()-began,counts=dict(counts),
            commands=len(commands),command_stages=dict(Counter(c['stage'] for c in commands)),
            nonzero_commands=sum(c['speed']!=0 for c in commands),
            source_frames=sorted({s['frame'] for s in scans}),
            source_time_verified=any(s['verified'] for s in scans),
            state_names=sorted({s.get('state','') for s in states}),
            cleanup=worker.cleanup if worker else None,
            vision_exitcode=vision_process.exitcode if vision_process else None,
            vision_alive=vision_process.is_alive() if vision_process else None,
            physical_actuation=False,synthetic_encoder=False,firmware_written=False,
            scope='runtime controller receives real sensor inputs; absent encoder is not replaced; no actuation host')
        for name,rows in [('scans',scans),('images',images),('motion',motion),('commands',commands),
                          ('states',states),('guard_states',guard_states),('vision',vision)]:
            (args.output/(name+'.json')).write_text(json.dumps(rows,separators=(',',':'))+'\n')
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    assert counts['scan'] and counts['image'] and summary['motion']['gyro']
    assert summary['guard_motion']['gyro'] and summary['guard_scan_present'] and guard_states
    assert all(summary['command_stages'].get(stage,0)>0 for stage in ('before_guard','after_guard'))
    assert not summary['nonzero_commands'] and not summary['motor_samples']
    assert not summary['source_time_verified'] and not summary['encoder_publishers'] and not summary['actuation_tx_publishers']
    assert vision and not summary['vision']['fault'] and not summary['worker_fault']
    assert not summary['cleanup']['forced'] and not summary['cleanup']['alive']
    assert summary['cleanup']['exitcode']==0 and summary['vision_exitcode']==0 and not summary['vision_alive']
    print(json.dumps(summary))


if __name__=='__main__':main()
