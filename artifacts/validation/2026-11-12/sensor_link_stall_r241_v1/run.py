"""격리 ROS의 실제 UART/공통 host/IMU 수신. finally에서 원 IMU 앱으로 복구한다."""
import json,os,signal,subprocess,sys,time,hashlib
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='187' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
A=Path(__file__).resolve().parent;root=A.parents[3];out=A/'run';out.mkdir(exist_ok=False)
remote=Path('/home/q/it_arena_sensor_link_stall_r241_20261112')
env=os.environ.copy();children=[];logs=[];result={}
python='/home/q/it_arena_imu_20261111/tools_venv/bin/python'
original='/home/q/it_arena_imu_20261111/build/arena_imu_probe.bin'
old_sha='8c9fb78850dcdb56d6c9e14e11e91e8e852dcdf78718038cc85dd6690f81ff27'
def launch(name,cmd):
    log=(out/(name+'.log')).open('x');logs.append(log)
    p=subprocess.Popen(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT);children.append(p)
    return p
def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(8)
        except subprocess.TimeoutExpired:p.kill();p.wait();result['forced_kill']=True
def wait_file(path,process,seconds=12):
    deadline=time.monotonic()+seconds
    while not path.exists():
        if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError(str(path)+' not ready')
        time.sleep(.05)
try:
    target=A/'build/arena_sensor_link.bin'
    expected=json.loads((A/'bundle_receipt.json').read_text())['firmware_sha256']
    assert hashlib.sha256(target.read_bytes()).hexdigest()==expected
    with (out/'flash.log').open('x') as log:
        subprocess.run([python,str(A/'flash.py'),str(remote/'flash'),str(target),expected],check=True,timeout=240,stdout=log,stderr=subprocess.STDOUT)
    result['flash_verified']=True
    receiver=launch('receiver',['/usr/bin/python3','-B',str(A/'receiver.py')])
    wait_file(out/'receiver.ready',receiver)
    host=launch('host',['/usr/bin/python3','-B','-c','from arena_vehicle_interface.actuation_ros import host_main; host_main()',
        '--ros-args','-p','offer_time_policy:=receiver_owned_lease'])
    bridge=launch('bridge',['/usr/bin/python3','-B',str(A/'stall_bridge.py'),
        '--port','/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00','--seconds','20','--output',str(out/'capture')])
    wait_file(out/'capture/ready',bridge)
    import rclpy
    from std_srvs.srv import Trigger
    from std_msgs.msg import UInt8MultiArray
    from arena_vehicle_interface.actuation_wire import Decoder
    rclpy.init(args=[]);node=rclpy.create_node('sensor_link_control_probe');decoder=Decoder();offers=[]
    def observe(msg):
        offers.extend(p for p in decoder.feed(bytes(msg.data)) if p.get('type')=='offer')
    node.create_subscription(UInt8MultiArray,'/actuation/rx',observe,10)
    until=time.monotonic()+5
    while not offers and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    assert offers and offers[-1]['output_ready'] is False
    result['host_now_us']=node.get_clock().now().nanoseconds//1000
    result['mcu_offer_issued_us']=offers[-1]['lease_issued_us']
    result['controls']=[]
    for operation in ('arm','stop','clear_stop','stop'):
        client=node.create_client(Trigger,'/actuation/'+operation)
        assert client.wait_for_service(timeout_sec=3)
        start=time.monotonic();future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=4)
        assert future.done(), operation+' timed out'
        answer=future.result()
        result['controls'].append(dict(operation=operation,success=answer.success,message=answer.message,round_trip_s=time.monotonic()-start))
        assert answer.success is (operation=='stop')
        node.destroy_client(client)
        until=time.monotonic()+.1
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
    node.destroy_node();rclpy.shutdown()
    result['bridge_exit']=bridge.wait(40)
    assert result['bridge_exit']==0
    time.sleep(.5)
finally:
    for p in reversed(children):stop(p)
    for log in logs:log.close()
    result['children_exit']=[p.returncode for p in children]
    try:
        with (out/'restore.log').open('x') as log:
            subprocess.run([python,str(A/'flash.py'),str(remote/'restore'),original,old_sha],check=True,timeout=240,stdout=log,stderr=subprocess.STDOUT)
        result['original_imu_restored']=True
    finally:(out/'supervisor.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
assert not result.get('forced_kill')
