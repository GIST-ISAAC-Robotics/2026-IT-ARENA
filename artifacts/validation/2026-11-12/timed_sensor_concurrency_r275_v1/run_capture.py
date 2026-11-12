"""원 IMU/C1와 B0390을 실제 시계 공통 노드·영상 자식에 연결. 구동 없음."""
from pathlib import Path
import hashlib,json,os,signal,subprocess,time
ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
OUT=A/'run';OUT.mkdir(exist_ok=False)
assert os.environ.get('ROS_DOMAIN_ID')=='216' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
manifest=json.loads((A/'source_manifest.json').read_bytes())
unchanged=lambda:all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==v for n,v in manifest.items())
assert unchanged()
C1=Path('/home/q/it_arena_c1_preview_r204_20261112/stream_c1_native_preview')
assert hashlib.sha256(C1.read_bytes()).hexdigest()=='d8f7d2a02ef47786022206e2a0278c6e7edc2d3ddb40bed2254706b590748e2d'
ports=['/dev/ttyACM0','/dev/ttyUSB0']
# Argus daemon owns the camera service; reject another client without restarting it.
clients=subprocess.run(['pgrep','-af','[n]vargus_nvraw|[g]st-launch|[p]robe_jetson_camera|[p]robe_ros_image|[c]amera.py'],capture_output=True,text=True)
assert clients.returncode==1, 'camera client present or query failed'
owners=subprocess.run(['fuser',*ports],capture_output=True)
assert owners.returncode==1 and not owners.stdout, 'device already owned or owner query failed'
assert all(os.access(p,os.R_OK|os.W_OK) for p in ports), 'device access permission missing'
children={};logs=[];fault=None;forced=[]
def start(name,command):
    log=(OUT/(name+'.log')).open('x');logs.append(log)
    children[name]=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    return children[name]
def stop(process,name):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            forced.append(name);process.kill();process.wait(timeout=3)
try:
    node=start('node',['/usr/bin/python3','-B',str(A/'node.py'),
        '--seconds','55','--output',str(OUT/'node')])
    until=time.monotonic()+15
    while not (OUT/'node/ready').exists():
        if node.poll() is not None or time.monotonic()>until:raise RuntimeError('common node not ready')
        time.sleep(.1)
    imu=start('imu',['/usr/bin/python3','-B',str(ROOT/'scripts/bridge_esp32_imu_preview.py'),
        '--port','/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00',
        '--imu-clock','fifo_tick','--seconds','20','--output',str(OUT/'imu')])
    lidar=start('c1',['/usr/bin/python3','-B',str(ROOT/'scripts/bridge_c1_native_preview.py'),
        '--scan-clock','tracking','--port','/dev/ttyUSB0','--executable',str(C1),'--seconds','20','--output',str(OUT/'c1')])
    camera=start('camera',['/usr/bin/python3','-B',str(A/'camera.py'),
        '--seconds','20','--output',str(OUT/'camera')])
    until=time.monotonic()+45
    while any(p.poll() is None for p in (imu,lidar,camera)):
        if any(p.poll() is not None and p.returncode!=0 for p in (imu,lidar,camera)):
            raise RuntimeError('sensor process failed')
        if node.poll() is not None:raise RuntimeError('common node exited before sensors')
        if time.monotonic()>until:raise RuntimeError('bounded sensor capture deadline')
        time.sleep(.1)
    assert imu.returncode==lidar.returncode==camera.returncode==0
    time.sleep(.3)
except BaseException as exc:
    fault=type(exc).__name__+': '+str(exc)
    raise
finally:
    for name,p in reversed(list(children.items())):stop(p,name)
    for log in logs:log.close()
    owners=subprocess.run(['fuser',*ports],capture_output=True)
    summary=dict(fault=fault,forced=forced,children={n:p.returncode for n,p in children.items()},
        ports_free=owners.returncode==1 and not owners.stdout,source_unchanged=unchanged(),
        firmware_written=False,actuator_started=False,power_settings_changed=False)
    (OUT/'supervisor.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))
assert not fault and not forced and summary['ports_free'] and summary['source_unchanged']
assert all(v==0 for v in summary['children'].values())
