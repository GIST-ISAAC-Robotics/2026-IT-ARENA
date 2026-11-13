"""20초 실제 센서 결합. 기존 IMU 진단 앱 사용, 출력 host/firmware 변경 없음."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
IMU='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00'
C1='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_362acd64b231f11190a78f5f30d20014-if00-port0'
SDK=Path('/home/q/it_arena_c1_preview_r204_20261112/stream_c1_native_preview')
SDK_SHA='d8f7d2a02ef47786022206e2a0278c6e7edc2d3ddb40bed2254706b590748e2d'
PROFILE=ROOT/'artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2'


def commands(output):
    base=[sys.executable,'-B']
    return dict(node=base+[str(A/'probe_runtime_inputs.py'),'--profile',str(PROFILE/'runtime_profile.yaml'),
        '--bindings',str(PROFILE/'bindings.json'),'--output',str(output/'node'),'--seconds','55'],
        imu=base+[str(ROOT/'scripts/bridge_esp32_imu_preview.py'),'--port',IMU,'--imu-clock','fifo_tick',
                  '--seconds','20','--output',str(output/'imu')],
        c1=base+[str(ROOT/'scripts/bridge_c1_native_preview.py'),'--port',C1,'--executable',str(SDK),
                 '--scan-clock','tracking','--seconds','20','--output',str(output/'c1'),
                 '--ros-domain','220','--topic','/hardware_runtime/c1/native'],
        camera=base+[str(ROOT/'scripts/bridge_b0390_preview.py'),'--seconds','20','--publish-hz','20',
                    '--output',str(output/'camera'),'--ros-domain','220','--topic','/hardware_runtime/camera/image_raw'])


def passive(out,name):
    import serial
    from probe_esp32_imu import Decoder
    decoder=Decoder();rows=[]
    port=serial.Serial(port=None,baudrate=921600,timeout=.05,exclusive=True)
    port.dtr=False;port.rts=False;port.port=IMU
    with (out/(name+'-uart-private.bin')).open('xb') as raw:
        try:
            port.open();end=time.monotonic()+3.
            while time.monotonic()<end:
                data=port.read(max(1,min(8192,port.in_waiting)));raw.write(data);rows.extend(decoder.feed(data))
        finally:port.close()
    metadata=[r for r in rows if r['type']==3];status=[r for r in rows if r['type']==2]
    result=dict(metadata_last=metadata[-1:],status_last=status[-1:],decoder=dict(decoder.counters),
                uart_commands_written=0,closed=not port.is_open)
    (out/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    assert metadata and metadata[-1]['who_am_i']==0x6c
    assert not metadata[-1]['memory_errors'] and not metadata[-1]['spi_errors']
    assert status and status[-1]['active'] is False


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan-only',action='store_true')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    cmd=commands(args.output)
    if args.plan_only:
        print(json.dumps(dict(commands=cmd,physical_actuation=False,firmware_written=False,synthetic_encoder=False),indent=2));return
    assert os.environ.get('ROS_DOMAIN_ID')=='220' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert hashlib.sha256(SDK.read_bytes()).hexdigest()==SDK_SHA
    owners=subprocess.run(['fuser',IMU,C1],capture_output=True)
    assert owners.returncode==1 and not owners.stdout, 'serial device is occupied'
    clients=subprocess.run(['pgrep','-af','[n]vargus_nvraw|[g]st-launch|[p]robe_jetson_camera|[p]robe_ros_image|[b]ridge_b0390_preview.py|[c]amera.py'],capture_output=True)
    assert clients.returncode==1, 'camera client is present or query failed'
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'registered.json').write_text(json.dumps(dict(commands=cmd,sdk_sha256=SDK_SHA,
        ros_domain=220,physical_actuation=False,firmware_written=False,synthetic_encoder=False),indent=2)+'\n')
    children={};logs=[];forced=[];error=None;interrupted=False
    def request(*_):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,request);signal.signal(signal.SIGTERM,request)
    def start(name):
        log=(args.output/(name+'.log')).open('x');logs.append(log)
        child=subprocess.Popen(cmd[name],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children[name]=child
        return child
    def pause():
        if interrupted:raise InterruptedError('requested stop')
        time.sleep(.1)
    def close(name):
        child=children[name]
        if child.poll() is not None:return
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            forced.append(name);os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3.)
    try:
        passive(args.output,'before')
        node=start('node');deadline=time.monotonic()+15.
        while not (args.output/'node/ready').exists():
            if node.poll() is not None:raise RuntimeError('runtime node startup failed')
            if time.monotonic()>deadline:raise TimeoutError('runtime ready')
            pause()
        sensors=[start(name) for name in ('imu','c1','camera')]
        deadline=time.monotonic()+36.
        while any(child.poll() is None for child in sensors):
            if node.poll() is not None:raise RuntimeError('runtime node exited before sensors')
            if any(child.poll() not in (None,0) for child in sensors):raise RuntimeError('sensor child failed')
            if time.monotonic()>deadline:raise TimeoutError('sensor capture deadline')
            pause()
        if any(child.returncode!=0 for child in sensors):raise RuntimeError('sensor child failed')
        time.sleep(.3);close('node')
        if node.returncode!=0:raise RuntimeError('runtime input/cleanup evidence failed')
        passive(args.output,'after')
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc)
        raise
    finally:
        for name in reversed(children):close(name)
        for log in logs:log.close()
        owners=subprocess.run(['fuser',IMU,C1],capture_output=True)
        result=dict(error=error,children={n:c.returncode for n,c in children.items()},forced=forced,
            ports_free=owners.returncode==1 and not owners.stdout,
            physical_actuation=False,synthetic_encoder=False,firmware_written=False)
        (args.output/'supervisor.json').write_text(json.dumps(result,indent=2)+'\n')
    assert not forced and result['ports_free']
    print(json.dumps(result))


if __name__=='__main__':main()
