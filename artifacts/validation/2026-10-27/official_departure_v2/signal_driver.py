"""평가 전용 신호 발생기. RGB만 차량에 보이고 상태 토픽은 평가기에만 전달한다."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

BASE=Path(__file__).resolve().parent
STAGES=[('green_probe',1.),('off',1.),('red1',1.),('red2',1.),('red3',1.),('red4',1.6),('green',None)]


def apply_stage(stage,lamps,world):
    red=int(stage[-1]) if stage.startswith('red') else 0
    def on(lamp):return lamp['column']<red or (lamp['column']==4 and stage in ('green','green_probe'))
    records=[]
    for lamp in sorted(lamps,key=on):
        active=on(lamp);color=('0 1 0' if lamp['column']==4 else '1 0 0') if active else '.004 .004 .004'
        r,g,b=color.split();rgba=f'r:{r} g:{g} b:{b} a:1'
        glow=rgba if active else 'r:0 g:0 b:0 a:1';name=lamp['name']
        req=(f'name:"{name}_visual" parent_name:"{name}" material {{ambient {{{rgba}}} diffuse {{{rgba}}} '
             f'emissive {{{glow}}} specular {{r:0 g:0 b:0 a:1}}}}')
        result=subprocess.run(['gz','service','-s',f'/world/{world}/visual_config/blocking',
            '--reqtype','gz.msgs.Visual','--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],
            capture_output=True,text=True,timeout=5)
        records.append(dict(lamp=name,lit=active,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr))
        if result.returncode or 'data: true' not in result.stdout:raise RuntimeError(f'visual service failed: {records[-1]}')
    return records


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    out=Path(sys.argv[1]);manifest=json.loads((BASE/'fixture/manifest.json').read_text())
    audit=(out/'signal_sequence.jsonl').open('x',encoding='utf-8')
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('official_signal_trial_driver')
    publisher=node.create_publisher(String,'/sim/traffic_light/state',10)
    state=dict(t=None,rgb=None,images=0,mcu=None);stopped=[False]
    signal.signal(signal.SIGINT,lambda *_:stopped.__setitem__(0,True));signal.signal(signal.SIGTERM,lambda *_:stopped.__setitem__(0,True))
    def rgb(msg):
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if state['rgb'] is None or t>state['rgb']:state['images']+=1;state['rgb']=t
    subscriptions=[node.create_subscription(Clock,'/clock',lambda m:state.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data),
        node.create_subscription(Image,'/camera/color/image_raw',rgb,qos_profile_sensor_data),
        node.create_subscription(String,'/actuation/status',lambda m:state.update(mcu=json.loads(m.data)),10)]
    executor=ThreadPoolExecutor(max_workers=1);future=None;stage=-1;start=None;applied='off';last_publish=-1.;last_wall=time.monotonic()
    try:
        while not stopped[0]:
            rclpy.spin_once(node,timeout_sec=.01);now=state['t']
            if now is None:continue
            if future is not None and future.done():
                receipt=future.result();future=None;applied=STAGES[stage][0];start=now
                audit.write(json.dumps(dict(stage=applied,applied_sim_s=now,receipts=receipt))+'\n');audit.flush()
            ready=(state['images']>=20 and state['mcu'] is not None and state['mcu']['state']=='ARMED')
            due=stage<0 and ready
            if stage>=0 and future is None and start is not None:
                duration=STAGES[stage][1];due=duration is not None and now-start>=duration
            if due and future is None:
                stage+=1;target=STAGES[stage][0]
                audit.write(json.dumps(dict(requested_stage=target,requested_sim_s=now))+'\n');audit.flush()
                future=executor.submit(apply_stage,target,manifest['lamps'],manifest['world_name'])
            if now-last_publish>=.1 or time.monotonic()-last_wall>=1.:
                label='red' if applied.startswith('red') else applied
                publisher.publish(String(data=json.dumps(dict(applied=label,stage=applied,sim_time_s=now,
                    scope='evaluation-only material receipt; not controller input'))))
                last_publish=now;last_wall=time.monotonic()
    finally:
        executor.shutdown(wait=True,cancel_futures=True);audit.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
