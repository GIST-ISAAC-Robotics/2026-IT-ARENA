#!/usr/bin/env python3
"""C1·IMU·선택 RGB의 관측 전용 공통 노드 실행. 펌웨어/구동/전력은 바꾸지 않는다."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
A=Path(__file__).resolve().parent


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--vehicle-config',type=Path,default=None,help='선택 차량 YAML. 생략 시 기존 비구동 preview 기하 유지')
    p.add_argument('--imu-port',required=True)
    p.add_argument('--c1-port',required=True)
    p.add_argument('--c1-executable',type=Path,required=True)
    p.add_argument('--c1-sha256',required=True)
    p.add_argument('--seconds',type=float,default=20.)
    p.add_argument('--imu-source',choices=('probe','sensor-link'),default='probe')
    p.add_argument('--camera',action='store_true')
    p.add_argument('--camera-publish-hz',type=float,default=None,
        help='선택 카메라 ROS 발행 상한. 취득60Hz와 기본 전체 발행은 유지')
    p.add_argument('--plan-only',action='store_true',help='장치/ROS를 열지 않고 실행 명령만 출력')
    return p


def commands(args,root=ROOT,node_script=A/'probe_g1_hardware_shadow.py',camera_script=A/'bridge_b0390_preview.py'):
    base=[sys.executable,'-B']
    node=base+[str(node_script),'--output',str(args.output/'node'),
        '--seconds',str(args.seconds+20.),'--estimated-time']
    if getattr(args, 'vehicle_config', None) is not None:
        # 선택 YAML은 장치 점유 검사/자식 실행 전에 검증한다. 생략 시 기존 경로 그대로다.
        from probe_g1_hardware_shadow import vehicle_geometry_override
        vehicle_geometry_override(args)
        node += ['--vehicle-config', str(args.vehicle_config.expanduser().resolve(strict=True))]
    if args.camera:node += ['--camera']
    imu_name='bridge_esp32_imu_preview.py' if args.imu_source=='probe' else 'bridge_sensor_link_preview.py'
    imu=base+[str(root/'scripts'/imu_name),'--port',args.imu_port,'--imu-clock','fifo_tick',
              '--seconds',str(args.seconds),'--output',str(args.output/'imu')]
    if args.imu_source=='sensor-link':
        imu += ['--mcu-status'];node += ['--mcu-status']
    result={'node':node,'imu':imu,'c1':base+[str(root/'scripts/bridge_c1_native_preview.py'),
        '--port',args.c1_port,'--executable',str(args.c1_executable),'--scan-clock','tracking',
        '--seconds',str(args.seconds),'--output',str(args.output/'c1')]}
    if args.camera:result['camera']=base+[str(camera_script),'--seconds',str(args.seconds),
                                        '--output',str(args.output/'camera')]
    if args.camera and getattr(args,'camera_publish_hz',None) is not None:
        result['camera']+=['--publish-hz',str(args.camera_publish_hz)]
    return result


def close_child(process,name,forced):
    if process.poll() is not None:return
    process.send_signal(signal.SIGINT)
    try:process.wait(timeout=10.)
    except subprocess.TimeoutExpired:
        forced.append(name);process.kill();process.wait(timeout=3.)


def ports_free(ports):
    owners=subprocess.run(['fuser',*ports],capture_output=True)
    return owners.returncode==1 and not owners.stdout


def main():
    p=parser();args=p.parse_args()
    if not 1<=args.seconds<=60:p.error('seconds outside 1..60')
    if args.camera_publish_hz is not None:
        if not args.camera:p.error('camera-publish-hz requires camera')
        if not math.isfinite(args.camera_publish_hz) or not 1<=args.camera_publish_hz<=60:
            p.error('camera-publish-hz outside finite 1..60')
    if len(args.c1_sha256)!=64 or any(c not in '0123456789abcdef' for c in args.c1_sha256):
        p.error('lowercase SHA256 required')
    args.output=args.output.resolve();cmd=commands(args)
    if args.plan_only:
        print(json.dumps(dict(commands=cmd,drive_output=False,firmware_written=False),indent=2));return
    if os.environ.get('ROS_DOMAIN_ID')!='216' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('isolated localhost domain216 required')
    ports=[str(Path(x).resolve(strict=True)) for x in (args.imu_port,args.c1_port)]
    if ports[0]==ports[1]:p.error('IMU and C1 require different devices')
    if not ports_free(ports) or not all(os.access(x,os.R_OK|os.W_OK) for x in ports):
        p.error('devices occupied or not accessible')
    digest=hashlib.sha256(args.c1_executable.read_bytes()).hexdigest()
    if digest!=args.c1_sha256:p.error('C1 executable digest mismatch')
    if args.camera:
        clients=subprocess.run(['pgrep','-af','[n]vargus_nvraw|[g]st-launch|[p]robe_jetson_camera|[p]robe_ros_image|[b]ridge_b0390_preview.py|[c]amera.py'],capture_output=True)
        if clients.returncode!=1:p.error('camera client present or query failed')
    args.output.mkdir(parents=True,exist_ok=False)
    sources={command[2]:hashlib.sha256(Path(command[2]).read_bytes()).hexdigest() for command in cmd.values()}
    (args.output/'registered.json').write_text(json.dumps(dict(commands=cmd,script_sha256=sources,
        c1_sha256=digest,ports=ports,scope='unmounted input/compute observation; no actuator or encoder substitute'),indent=2)+'\n')
    children={};logs=[];forced=[];fault=None;interrupted=False
    def requested(*_):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,requested);signal.signal(signal.SIGTERM,requested)
    def start(name):
        stream=(args.output/(name+'.log')).open('x');logs.append(stream)
        children[name]=subprocess.Popen(cmd[name],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
        return children[name]
    def checked_wait():
        if interrupted:raise InterruptedError('capture interrupted')
        time.sleep(.1)
    try:
        node=start('node');deadline=time.monotonic()+15.
        while not (args.output/'node/ready').exists():
            if node.poll() is not None or time.monotonic()>deadline:raise RuntimeError('common node not ready')
            checked_wait()
        sensors=[start(name) for name in cmd if name!='node']
        deadline=time.monotonic()+args.seconds+25.
        while any(x.poll() is None for x in sensors):
            if any(x.poll() not in (None,0) for x in sensors):raise RuntimeError('sensor process failed')
            if node.poll() is not None:raise RuntimeError('common node exited before sensors')
            if time.monotonic()>deadline:raise RuntimeError('bounded capture deadline')
            checked_wait()
        if any(x.returncode!=0 for x in sensors):raise RuntimeError('sensor process failed')
        time.sleep(.3)
    except BaseException as exc:
        fault=type(exc).__name__+': '+str(exc)
        raise
    finally:
        for name,process in reversed(list(children.items())):close_child(process,name,forced)
        for stream in logs:stream.close()
        summary=dict(fault=fault,forced=forced,children={name:p.returncode for name,p in children.items()},
            ports_free=ports_free(ports),source_unchanged=all(hashlib.sha256(Path(n).read_bytes()).hexdigest()==h for n,h in sources.items()),
            firmware_written=False,actuator_started=False,power_settings_changed=False)
        (args.output/'supervisor.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary))
    if forced or not summary['ports_free'] or not summary['source_unchanged'] or any(x!=0 for x in summary['children'].values()):
        raise RuntimeError('capture cleanup/integrity incomplete')


if __name__=='__main__':main()
