import json,os,signal,subprocess,sys,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='190' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
A=Path(__file__).resolve().parent;root=A.parents[3];out=A/'run';out.mkdir(exist_ok=True)
assert not (out/'supervisor.json').exists()
env=os.environ.copy(); children=[];results={};logs=[]
def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(8)
        except subprocess.TimeoutExpired:p.kill();p.wait();results['forced_kill']=True
try:
    log=(out/'receiver.log').open('x');logs.append(log)
    reader=subprocess.Popen(['/usr/bin/python3','-B',str(A/'receiver.py')],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT);children.append(reader)
    deadline=time.monotonic()+12
    while not (out/'receiver.ready').exists():
        if reader.poll() is not None or time.monotonic()>deadline:raise RuntimeError('receiver not ready')
        time.sleep(.1)
    log=(out/'bridge.log').open('x');logs.append(log)
    bridge=subprocess.Popen(['/usr/bin/python3','-B',str(root/'scripts/bridge_esp32_imu_preview.py'),'--port','/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00','--seconds','20','--output',str(out/'capture')],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT);children.append(bridge)
    results['bridge_exit']=bridge.wait(60)
    time.sleep(.5)
finally:
    for p in reversed(children):stop(p)
    for log in logs:log.close()
    results['children_exit']=[p.returncode for p in children]
    (out/'supervisor.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results))
assert results.get('bridge_exit')==0 and not results.get('forced_kill')
