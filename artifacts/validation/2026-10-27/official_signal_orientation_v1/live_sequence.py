"""가로 신호의 실제 Gazebo 재질 전환/RGB 시각 시험. 차량 구동/실물 시험 아님."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import cv2

ROOT=Path(__file__).resolve().parents[4]
for part in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'):
    sys.path.insert(0,str(ROOT/part))
from arena_autonomy.core import image_rgb
from arena_autonomy.race_start import OfficialStartSignal
from lidar_shutdown import audit_shutdown,group_processes,stop_gazebo_service,stop_launch
from validate_start_signal_render import stop_bridge_first,cleanup_owned_residue

STAGES=[('green_probe',1.),('off',1.),('red1',1.),('red2',1.),('red3',1.4),('red4',1.6),('green',5.),('end_off',.5)]
LAMPS=[f'lamp_red_{i}' for i in range(1,5)]+['lamp_green']


def set_stage(stage):
    red=int(stage[-1]) if stage.startswith('red') else 0
    lit={f'lamp_red_{i}' for i in range(1,red+1)}
    if stage in ('green','green_probe'):lit={'lamp_green'}
    records=[]
    # 소등을 먼저 적용해 전환 도중 빨강과 초록이 동시에 켜지는 것을 피한다.
    for name in sorted(LAMPS,key=lambda n:n in lit):
        on=name in lit
        color=('0 1 0' if name=='lamp_green' else '1 0 0') if on else '.004 .004 .004'
        r,g,b=color.split();rgba=f'r:{r} g:{g} b:{b} a:1'
        emissive=rgba if on else 'r:0 g:0 b:0 a:1'
        request=(f'name:"{name}_vis" parent_name:"{name}" material {{'
            f'ambient {{{rgba}}} diffuse {{{rgba}}} emissive {{{emissive}}}'
            'specular {r:0 g:0 b:0 a:1}}')
        result=subprocess.run(['gz','service','-s','/world/m5_grid_fixture/visual_config/blocking',
            '--reqtype','gz.msgs.Visual','--reptype','gz.msgs.Boolean','--timeout','3000','--req',request],
            capture_output=True,text=True,timeout=5)
        records.append(dict(lamp=name,on=on,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr))
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError(f'visual update failed: {records[-1]}')
    return records


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();out=args.output.resolve()
    source=Path(__file__).resolve().parent/'horizontal_red4'
    if not out.is_relative_to(ROOT/'artifacts/validation'):parser.error('fresh validation output required')
    out.mkdir(parents=True,exist_ok=False)
    root=ET.parse(source/'fixture.sdf');world=root.find('world')
    spec=json.loads((source/'report.json').read_text());views={k:v for k,v in spec['render_views'].items() if k!='overview'}
    for model in list(world.findall('model')):
        if model.get('name')=='m5_camera_overview':world.remove(model)
    for sensor in world.findall('.//sensor'):
        sensor.find('update_rate').text='10'
    for link in world.findall("model[@name='it_arena_track_static']/link"):
        if link.get('name') in LAMPS:
            mat=link.find('visual/material')
            for tag in ('ambient','diffuse','emissive'):
                e=mat.find(tag)
                if e is None:e=ET.SubElement(mat,tag)
                e.text='.004 .004 .004 1' if tag!='emissive' else '0 0 0 1'
    fixture=out/'fixture.sdf';ET.indent(root);root.write(fixture,encoding='utf-8',xml_declaration=True)
    sources=[Path(__file__),source/'fixture.sdf',source/'report.json',
        ROOT/'src/arena_autonomy/arena_autonomy/race_start.py',ROOT/'src/arena_autonomy/arena_autonomy/core.py',
        ROOT/'scripts/lidar_shutdown.py',ROOT/'scripts/validate_start_signal_render.py']
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    report=dict(passed=False,scope='live Gazebo material sequence and RGB-only detector; no drive command or hardware proof',
        source_sha256=hashes,views=views,sequence=STAGES,
        retained_assumptions='beam1.2m/lamp1.08m, single spheres, 25cm peer boxes; not actual installation',
        camera_rate_hz=10,rate_scope='optical test rate only, not final B0183 operating rate')
    os.environ.update(GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}',ROS_DOMAIN_ID=str(170+os.getpid()%20),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_STATIC_PEERS='',LIBGL_ALWAYS_SOFTWARE='true',GALLIUM_DRIVER='llvmpipe')
    server=['gz','sim','-s','-r','-v','3',str(fixture)]
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']+[v['topic']+'@sensor_msgs/msg/Image[gz.msgs.Image' for v in views.values()]
    launch=('from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n'
        'from launch_ros.actions import Node\n'
        'def generate_launch_description():\n    return LaunchDescription(['+
        f'ExecuteProcess(cmd={server!r},output="screen"),'+
        f'Node(package="ros_gz_bridge",executable="parameter_bridge",arguments={arguments!r},output="screen")])\n')
    launchpath=out/'fixture.launch.py';launchpath.write_text(launch)
    import rclpy
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile
    rclpy.init();node=rclpy.create_node('horizontal_signal_sequence_probe')
    now=[None];detectors={k:OfficialStartSignal() for k in views};records={k:[] for k in views};saved=set()
    def clock(msg):now[0]=msg.clock.sec+msg.clock.nanosec*1e-9
    def image(msg,name):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        rgb=image_rgb(msg).copy();detector=detectors[name]
        before=detector.started;detector.update(rgb,stamp)
        # 검출기에는 영상과 촬영 시각만 전달한다. 평가용 단계는 입력하지 않는다.
        records[name].append(dict(stamp=stamp,observed=detector.observed,started=detector.started,new_start=detector.started and not before))
        key=(name,detector.observed)
        if key not in saved and detector.observed in ('red4_confirmed','green_confirmed'):
            path=out/f'{name}_{detector.observed}.png'
            if not cv2.imwrite(str(path),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):raise RuntimeError('image write failed')
            saved.add(key)
    subscriptions=[node.create_subscription(Clock,'/clock',clock,10)]
    subscriptions += [node.create_subscription(Image,v['topic'],lambda msg,n=n:image(msg,n),QoSProfile(depth=2)) for n,v in views.items()]
    process=None;pool=ThreadPoolExecutor(max_workers=1);future=None;logpath=out/'launch.log';log=logpath.open('wb')
    changes=[];index=-1;applied_at=None;deadline=time.monotonic()+300
    try:
        process=subprocess.Popen(['ros2','launch',str(launchpath)],cwd=out,stdin=subprocess.DEVNULL,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('fixture exited early')
            rclpy.spin_once(node,timeout_sec=.02)
            if future is not None and future.done():
                changes[-1].update(response=future.result(),acknowledged_clock_s=now[0])
                applied_at=now[0];future=None
            ready=now[0] is not None and all(records.values())
            if ready and future is None and (index<0 or now[0]-applied_at>=STAGES[index][1]):
                index+=1
                if index==len(STAGES):break
                changes.append(dict(stage=STAGES[index][0],requested_clock_s=now[0]))
                future=pool.submit(set_stage,STAGES[index][0])
        if index!=len(STAGES):raise RuntimeError('live sequence timed out')
        green_request=next(c['requested_clock_s'] for c in changes if c['stage']=='green')
        per_view={}
        for name,rows in records.items():
            stamps=[r['stamp'] for r in rows];starts=[r['stamp'] for r in rows if r['new_start']]
            confirmations=[r['stamp'] for r in rows if r['observed']=='red4_confirmed']
            per_view[name]=dict(frames=len(rows),ordered=all(b>a for a,b in zip(stamps,stamps[1:])),
                max_frame_gap_s=max(b-a for a,b in zip(stamps,stamps[1:])),start_stamps=starts,
                red4_seen_before_green=any(t<green_request for t in confirmations),
                no_start_before_green=not any(t<green_request for t in starts),
                one_start_during_green=len(starts)==1 and green_request<=starts[0]<=green_request+5.)
        report.update(sequence_complete=True,green_requested_s=green_request,per_view=per_view)
    except Exception as error:report['error']=repr(error)
    finally:
        pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node();rclpy.shutdown()
        before=group_processes(process.pid) if process else {}
        if process:
            report['bridge_shutdown']=stop_bridge_first(process,logpath.read_text(errors='replace'))
            report['server_stop']=stop_gazebo_service(process,logpath.read_text(errors='replace'),os.environ['GZ_PARTITION'])
            cleanup=stop_launch(process);log.close()
            report['shutdown']=audit_shutdown(logpath.read_text(errors='replace'),process.poll(),cleanup or None)
            report['residue_cleanup']=cleanup_owned_residue(process.pid,before)
        else:log.close()
        report['source_unchanged']=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
        report['passed']=bool(report.get('sequence_complete') and report['source_unchanged'] and 'error' not in report and
            report.get('bridge_shutdown',{}).get('verified') and report.get('server_stop',{}).get('verified') and
            report.get('shutdown',{}).get('clean') and not report.get('residue_cleanup',{}).get('remaining') and
            not report.get('residue_cleanup',{}).get('actions') and all(
                d['ordered'] and d['max_frame_gap_s']<=.35 and d['red4_seen_before_green'] and
                d['no_start_before_green'] and d['one_start_during_green'] for d in report.get('per_view',{}).values()))
        report['changes']=changes
        (out/'frames.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
        (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report.get(k) for k in ('passed','error','per_view','source_unchanged')}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
