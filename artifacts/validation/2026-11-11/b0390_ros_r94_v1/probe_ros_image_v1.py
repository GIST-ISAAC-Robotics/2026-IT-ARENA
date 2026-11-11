"""격리된 진단 토픽으로 Argus BGR → ROS 별도 수신 프로세스를 대조한다.

header는 appsink 수신 system 시각이다. 촬영 시각으로 승인하지 않으며 대회
제어기·실내 영상 저장·구동 명령·장치 설정 변경을 수행하지 않는다.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import traceback
import zlib

TOPIC = '/diagnostics/b0390/image_raw'
FRAME = 'b0390_receive_time_unverified'
FIELDS = ('stamp_ns', 'received_ns', 'pts_ns', 'bytes', 'crc32', 'elapsed_ms')


def stats(values):
    a = sorted(values)
    if not a:
        return None
    def p(frac):
        i = (len(a)-1)*frac
        lo = int(i)
        return a[lo]+(a[min(lo+1,len(a)-1)]-a[lo])*(i-lo)
    return dict(count=len(a), median=statistics.median(a), p95=p(.95), maximum=max(a))


def save_csv(path, rows):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def receive(output, timeout):
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('b0390_diagnostic_receiver')
    rows, errors = [], []
    def callback(msg):
        received = time.time_ns()
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if (msg.width, msg.height, msg.encoding, msg.step, msg.header.frame_id) != (1280,720,'bgr8',3840,FRAME):
            errors.append('invalid image contract')
            return
        if len(msg.data) != msg.height*msg.step:
            errors.append('invalid image length')
            return
        rows.append(dict(stamp_ns=stamp,received_ns=received,pts_ns='',bytes=len(msg.data),
                         crc32=zlib.crc32(msg.data),elapsed_ms=(received-stamp)/1e6))
    node.create_subscription(Image, TOPIC, callback, QoSProfile(depth=2))
    end = time.monotonic()+timeout
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if (output/'producer_done').exists():
                # Allow the bounded DDS queue to drain before stopping.
                deadline=time.monotonic()+1.
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.02)
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        save_csv(output/'received.csv',rows)
        (output/'receiver.json').write_text(json.dumps(dict(frames=len(rows),errors=errors,
            receive_age_ms=stats([r['elapsed_ms'] for r in rows])),indent=2)+'\n')


def produce(output, duration):
    import gi
    gi.require_version('Gst','1.0')
    gi.require_version('GstVideo','1.0')
    from gi.repository import Gst, GstVideo
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    Gst.init(None)
    rclpy.init()
    node=rclpy.create_node('b0390_diagnostic_source')
    publisher=node.create_publisher(Image,TOPIC,QoSProfile(depth=2))
    pipeline_text=('nvarguscamerasrc name=camera sensor-id=0 sensor-mode=4 do-timestamp=true ! '
        'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=60/1 ! '
        'nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! '
        'appsink name=receiver sync=false max-buffers=2 drop=true wait-on-eos=false')
    pipeline=Gst.parse_launch(pipeline_text)
    sink=pipeline.get_by_name('receiver')
    bus=pipeline.get_bus()
    rows, errors, warnings=[],[],[]
    state_null=False
    receiver=subprocess.Popen([sys.executable,__file__,'--receive',str(output),'--duration',str(duration+30)],
        stdout=(output/'receiver.log').open('x'),stderr=subprocess.STDOUT)
    cpu_begin=time.process_time()
    try:
        deadline=time.monotonic()+8
        while publisher.get_subscription_count()!=1:
            if time.monotonic()>deadline or receiver.poll() is not None:
                raise RuntimeError('diagnostic subscriber not ready')
            rclpy.spin_once(node,timeout_sec=.05)
        if pipeline.set_state(Gst.State.PLAYING)==Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('camera failed to play')
        first=None
        start=None
        while True:
            sample=sink.emit('try-pull-sample',5*Gst.SECOND)
            received_ns=time.time_ns()
            monotonic=time.monotonic()
            if sample is None:
                raise RuntimeError('no frame within 5 seconds')
            while True:
                event=bus.pop_filtered(Gst.MessageType.ERROR|Gst.MessageType.WARNING)
                if event is None:
                    break
                if event.type==Gst.MessageType.ERROR:
                    raise RuntimeError(str(event.parse_error()))
                warnings.append(str(event.parse_warning()))
            if first is None:
                first=monotonic
            if monotonic-first<3:
                continue
            if start is None:
                start=monotonic
            if monotonic-start>=duration:
                break
            before=time.perf_counter_ns()
            video=GstVideo.VideoInfo.new_from_caps(sample.get_caps())
            if video.width!=1280 or video.height!=720 or video.stride[0]!=3840 or video.offset[0]!=0:
                raise RuntimeError('unexpected BGR stride/size')
            buffer=sample.get_buffer()
            ok,mapped=buffer.map(Gst.MapFlags.READ)
            if not ok:
                raise RuntimeError('cannot map BGR buffer')
            try:
                payload=bytes(mapped.data)
            finally:
                buffer.unmap(mapped)
            if len(payload)!=720*3840:
                raise RuntimeError('unexpected BGR bytes')
            msg=Image()
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(received_ns,10**9)
            msg.header.frame_id=FRAME
            msg.height,msg.width,msg.encoding,msg.step=720,1280,'bgr8',3840
            msg.is_bigendian=False
            msg.data=payload
            publisher.publish(msg)
            elapsed=(time.perf_counter_ns()-before)/1e6
            rows.append(dict(stamp_ns=received_ns,received_ns=received_ns,pts_ns=int(buffer.pts),
                             bytes=len(payload),crc32=zlib.crc32(payload),elapsed_ms=elapsed))
            if receiver.poll() is not None:
                raise RuntimeError('receiver exited during capture')
    except Exception:
        errors.append(traceback.format_exc())
    finally:
        pipeline.set_state(Gst.State.NULL)
        result,state,_=pipeline.get_state(5*Gst.SECOND)
        state_null=result!=Gst.StateChangeReturn.FAILURE and state==Gst.State.NULL
        (output/'producer_done').write_text('done\n')
        try:
            receiver.wait(timeout=8)
        except subprocess.TimeoutExpired:
            errors.append('receiver shutdown timeout')
            receiver.terminate()
            try:
                receiver.wait(timeout=3)
            except subprocess.TimeoutExpired:
                receiver.kill();receiver.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()
    save_csv(output/'sent.csv',rows)
    with (output/'received.csv').open() as stream:
        received=list(csv.DictReader(stream))
    indexed={int(r['stamp_ns']):r for r in rows}
    mismatches=sum(int(r['stamp_ns']) not in indexed or
                   int(r['crc32'])!=int(indexed[int(r['stamp_ns'])]['crc32']) for r in received)
    receiver_report=json.loads((output/'receiver.json').read_bytes())
    intervals=[(b['received_ns']-a['received_ns'])/1e6 for a,b in zip(rows,rows[1:])]
    pts_gaps=[(b['pts_ns']-a['pts_ns'])/1e6 for a,b in zip(rows,rows[1:])]
    summary=dict(scope='isolated 720p60 BGR ROS plumbing; receive timestamp only, not exposure timestamp',
        topic=TOPIC,domain=os.environ['ROS_DOMAIN_ID'],pipeline=pipeline_text,
        duration_requested_s=duration,sent=len(rows),received=len(received),payload_mismatches=mismatches,
        format_errors=receiver_report['errors'],errors=errors,warnings=warnings,
        receive_age_ms=receiver_report['receive_age_ms'],copy_publish_ms=stats([r['elapsed_ms'] for r in rows]),
        sink_interval_ms=stats(intervals),pts_interval_ms=stats(pts_gaps),
        producer_fps=(len(rows)-1)*1e9/(rows[-1]['received_ns']-rows[0]['received_ns']) if len(rows)>1 else None,
        cpu_s=time.process_time()-cpu_begin,receiver_exit=receiver.returncode,pipeline_null=state_null,
        exposure_time_verified=False,controller_connected=False,raw_pixels_saved=False,
        qos=dict(depth=2,reliability='reliable'),environment=dict(localhost_only=True))
    summary['plumbing_passed']=bool(rows) and not errors and not receiver_report['errors'] and not mismatches and len(rows)==len(received) and receiver.returncode==0 and state_null
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)
    return 0 if summary['plumbing_passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('--duration',type=float,default=20.)
    p.add_argument('--receive',action='store_true')
    a=p.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='237':
        p.error('requires isolated ROS_DOMAIN_ID=237 and ROS_LOCALHOST_ONLY=1')
    if not 1<=a.duration<=90:
        p.error('duration must be 1..90 seconds')
    if a.receive:
        receive(a.output,a.duration)
    else:
        a.output.mkdir(parents=True,exist_ok=False)
        raise SystemExit(produce(a.output,a.duration))
