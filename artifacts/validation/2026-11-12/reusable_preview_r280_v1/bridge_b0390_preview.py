"""r94의 검증된 B0390 수신 경로를 관측 노드에 연결. 픽셀은 저장하지 않는다."""
from array import array
import argparse,json,os,time
from pathlib import Path

TOPIC='/hardware_preview/camera/image_raw'
FRAME='b0390_receive_time_unverified'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=20.)
    a=p.parse_args()
    if not 1<=a.seconds<=60:p.error('seconds outside 1..60')
    if os.environ.get('ROS_DOMAIN_ID')!='216' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('isolated localhost216 required')
    import gi
    gi.require_version('Gst','1.0');gi.require_version('GstVideo','1.0')
    from gi.repository import Gst,GstVideo
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    a.output.mkdir(parents=True,exist_ok=False)
    Gst.init(None);rclpy.init(args=[]);node=rclpy.create_node('b0390_shadow_source')
    pub=node.create_publisher(Image,TOPIC,QoSProfile(depth=2))
    pipeline_text=('nvarguscamerasrc name=camera sensor-id=0 sensor-mode=4 do-timestamp=true ! '
        'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=60/1 ! '
        'nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! '
        'appsink name=receiver sync=false max-buffers=2 drop=true wait-on-eos=false')
    pipeline=Gst.parse_launch(pipeline_text);sink=pipeline.get_by_name('receiver');bus=pipeline.get_bus()
    rows=[];warnings=[];fault=None;null=False
    try:
        deadline=time.monotonic()+10
        while not pub.get_subscription_count():
            if time.monotonic()>deadline:raise RuntimeError('common camera subscriber absent')
            rclpy.spin_once(node,timeout_sec=.02)
        if pipeline.set_state(Gst.State.PLAYING)==Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('camera failed to play')
        warmup=None;began=None
        while True:
            sample=sink.emit('try-pull-sample',5*Gst.SECOND)
            received=time.time_ns();now=time.monotonic()
            if sample is None:raise RuntimeError('camera frame timeout')
            while True:
                event=bus.pop_filtered(Gst.MessageType.ERROR|Gst.MessageType.WARNING)
                if event is None:break
                if event.type==Gst.MessageType.ERROR:raise RuntimeError(str(event.parse_error()))
                warnings.append(str(event.parse_warning()))
            if warmup is None:warmup=now
            if now-warmup<3:continue
            if began is None:began=now
            if now-began>=a.seconds:break
            video=GstVideo.VideoInfo.new_from_caps(sample.get_caps())
            if (video.width,video.height,video.stride[0],video.offset[0])!=(1280,720,3840,0):
                raise RuntimeError('camera BGR layout differs')
            buffer=sample.get_buffer();ok,mapped=buffer.map(Gst.MapFlags.READ)
            if not ok:raise RuntimeError('camera map failed')
            try:payload=bytes(mapped.data)
            finally:buffer.unmap(mapped)
            if len(payload)!=720*3840:raise RuntimeError('camera BGR size differs')
            msg=Image();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(received,10**9)
            msg.header.frame_id=FRAME
            msg.width,msg.height,msg.encoding,msg.step=1280,720,'bgr8',3840
            packed=array('B');packed.frombytes(payload);msg.data=packed
            pub.publish(msg)
            rows.append(dict(stamp_ns=received,pts_ns=int(buffer.pts),bytes=len(payload)))
    except BaseException as exc:
        fault=type(exc).__name__+': '+str(exc);raise
    finally:
        pipeline.set_state(Gst.State.NULL)
        state_result,state,_=pipeline.get_state(5*Gst.SECOND)
        null=state_result!=Gst.StateChangeReturn.FAILURE and state==Gst.State.NULL
        summary=dict(fault=fault,warnings=warnings,frames=len(rows),pipeline_null=null,
            pipeline=pipeline_text,topic=TOPIC,frame_id=FRAME,exposure_time_verified=False,
            header_basis='appsink host realtime receipt; not exposure',raw_pixels_saved=False,
            producer_fps=(len(rows)-1)*1e9/(rows[-1]['stamp_ns']-rows[0]['stamp_ns']) if len(rows)>1 else None)
        (a.output/'frames.json').write_text(json.dumps(rows,separators=(',',':'))+'\n')
        (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(summary))
    assert not fault and rows and null


if __name__=='__main__':main()
