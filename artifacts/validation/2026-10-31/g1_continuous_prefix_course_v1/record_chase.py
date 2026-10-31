"""평가 전용3인칭 영상과 실제 이미지 시각을 보존한다. 제어 토픽 발행 없음."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args()
    root=Path(__file__).resolve().parents[4];run=args.run.resolve()
    if not run.is_relative_to(root/'artifacts'):raise ValueError('artifacts output required')
    folder=run/'video';folder.mkdir(exist_ok=False)
    path=folder/'chase_raw.mp4';stamps=[];writer=None;failure=None;quit_requested=False
    rclpy.init();node=rclpy.create_node('evaluation_chase_recorder')
    def stop(*_):
        nonlocal quit_requested
        quit_requested=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def receive(message):
        nonlocal writer,failure,quit_requested
        try:
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if stamps and stamp<=stamps[-1]:
                if stamp<stamps[-1]-1e-9:raise ValueError('camera time went backwards')
                return
            if message.encoding not in ('rgb8','bgr8'):raise ValueError('unexpected image encoding')
            raw=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)
            frame=raw[:,:message.width*3].reshape(message.height,message.width,3)
            if message.encoding=='rgb8':frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
            if writer is None:
                writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'avc1'),20.,(message.width,message.height))
                if not writer.isOpened():raise RuntimeError('H.264 encoder unavailable')
                if not cv2.imwrite(str(folder/'first_frame.png'),frame):raise RuntimeError('frame capture failed')
            writer.write(frame);stamps.append(stamp)
        except Exception as error:
            failure=f'{type(error).__name__}: {error}';quit_requested=True
    subscription=node.create_subscription(Image,'/sim/chase/image',receive,qos_profile_sensor_data)
    started=time.monotonic()
    try:
        while rclpy.ok() and not quit_requested:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-started>2400:
                failure='recording wall deadline';break
    finally:
        if writer is not None:writer.release()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (folder/'video_timestamps.json').write_text(json.dumps(stamps),encoding='utf-8')
        report=dict(scope='observer video only; driving verdict in parent report.json',failure=failure,
            video=dict(path=str(path.relative_to(root)),frames_written=len(stamps),
                first_frame_sim_time_s=stamps[0] if stamps else None,
                last_frame_sim_time_s=stamps[-1] if stamps else None,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None))
        (folder/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return 1 if failure or len(stamps)<2 else 0


if __name__=='__main__':raise SystemExit(main())
