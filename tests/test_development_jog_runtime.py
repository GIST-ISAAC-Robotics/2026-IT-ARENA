"""r408 순수 함수/가짜 ROS 호출 검사. 실제 executor·DDS·시간·장치 검증이 아니다."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as N

import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('_r408_jog',ROOT/'scripts/development_jog.py')
jog=importlib.util.module_from_spec(spec);spec.loader.exec_module(jog)


def legacy(now=10.):
    return dict(v=1,boot='fixture',generation=4,owner='host',state='ARMED',now_us=round(now*1e6),
        estop_held=False,feedback_valid=True,output_ready=True,target_speed_mps=0.)


def runtime(now=10.):
    stamp=round(now*1e6)
    row=dict(v=1,boot='fixture',generation=4,state='ARMED',reason='explicit_arm',
        now_us=stamp,mcu_now_us=stamp,host_receipt_ns=stamp*1000,receipt_age_ns=0,clock_epoch=2,
        estop_held=False,output_ready=True,time_basis=jog.RUNTIME_TIME_BASIS,
        time_contract_verified=False,scope='observation_only_no_drive')
    row['motor_feedback']=dict(schema='motor_feedback_observation_v1',boot='fixture',generation=4,
        clock_epoch=2,source_valid=True,sequence=8,motor_counts=-3,mcu_capture_us=stamp,
        estimated_capture_ns=stamp*1000,host_receipt_ns=stamp*1000,time_basis=jog.RUNTIME_TIME_BASIS,
        time_contract_verified=False,scope='observation_only_no_drive')
    return row


@pytest.mark.parametrize('direction,steering',[('forward',0.),('left',.2),('right',-.2)])
def test_fixed_plan_keeps_forward_only_low_cap(direction,steering):
    row=jog.plan(direction,.15,.5,.2)
    assert row['steering_rad']==steering and row['output_topic']=='/drive'
    assert not row['automatic_arm'] and row['speed_mps']<=row['speed_limit_mps']==.2


@pytest.mark.parametrize('values',[
    ('reverse',.1,.5,.2),('forward',-.1,.5,.2),('left',.21,.5,.2),
    ('right',.1,2.01,.2),('forward',float('nan'),.5,.2),('forward',.1,.5,.3),
    ('forward',True,.5,.2),('forward',.1,float('inf'),.2)])
def test_invalid_plan_rejected(values):
    with pytest.raises(ValueError):jog.plan(*values)


def test_both_existing_status_contracts_preserve_observation_meaning():
    for status in (legacy(),runtime()):
        before=copy.deepcopy(status)
        assert jog.valid_mcu(status,10.1,20.,20.1)
        assert jog.valid_mcu({**status,'state':'ACTIVE'},10.1,20.,20.1,identity=jog.mcu_identity(status))
        assert not jog.valid_mcu({**status,'state':'ACTIVE'},10.1,20.,20.1)
        assert status==before
    assert 'owner' not in runtime() and 'target_speed_mps' not in runtime()


def test_actual_mcu_status_clock_output_is_accepted_without_added_fields():
    sys.path.insert(0,str(ROOT/'src/arena_vehicle_interface'))
    try:
        from arena_vehicle_interface.mcu_status_clock import McuStatusClock
    finally:sys.path.pop(0)
    source=dict(type='offer',v=1,boot='fixture',generation=4,state='ARMED',reason='explicit_arm',
        estop_held=False,output_ready=True,status_now_us=1_000_000,lease_issued_us=1_000_000,
        motor_feedback_v1=dict(valid=True,sequence=8,capture_us=990_000,counts=-3))
    status=McuStatusClock(warmup_ns=0).convert_batch([source],monotonic_ns=20_000_000_000,
        realtime_ns=10_000_000_000)[0]
    assert jog.valid_mcu(status,10.05,20.,20.05)
    assert status['time_contract_verified'] is False
    assert 'owner' not in status and 'target_speed_mps' not in status


@pytest.mark.parametrize('change',[
    {'output_ready':False},{'state':'FAULT_LATCHED'},{'time_contract_verified':True},
    {'time_basis':'unknown'},{'receipt_age_ns':1},{'motor_feedback':None}])
def test_runtime_fault_or_invalid_contract_never_enables_jog(change):
    assert not jog.valid_mcu({**runtime(),**change},10.1,20.,20.1)


def test_unavailable_feedback_identity_clock_and_staleness_stop_running_jog():
    status=runtime();identity=jog.mcu_identity(status)
    bad=copy.deepcopy(status);bad['motor_feedback']['source_valid']=False
    assert not jog.valid_mcu(bad,10.1,20.,20.1,identity=identity)
    for key,value in [('boot','other'),('generation',5),('clock_epoch',3),('now_us',9_000_000)]:
        assert not jog.valid_mcu({**status,key:value},10.1,20.,20.1,identity=identity)
    assert not jog.valid_mcu(status,10.1,20.,20.3,identity=identity)
    assert not jog.valid_mcu(status,9.9,20.,20.1,identity=identity)


def test_stop_state_is_existing_latched_name_new_generation_and_same_epoch():
    for status in (legacy(),runtime()):
        identity=jog.mcu_identity(status)
        stopped={**status,'state':'ESTOP_LATCHED','generation':5,'output_ready':False}
        assert jog.stop_observed(stopped,10.1,20.,20.1,identity)
        assert not jog.stop_observed({**stopped,'state':'ESTOP'},10.1,20.,20.1,identity)
        assert not jog.stop_observed({**stopped,'generation':4},10.1,20.,20.1,identity)
    assert not jog.stop_observed({**stopped,'clock_epoch':3},10.1,20.,20.1,identity)


class FakeROS:
    """main의 실제 발행/서비스 호출 순서를 메모리에 기록하며 ROS는 import하지 않는다."""
    def __init__(self,monkeypatch,tmp_path,mode):
        self.wall=100.;self.mode=mode;self.calls=[];self.callback=None;self.stopped=False;self.positive=0
        self.handlers={};self.node_name='';self.output=tmp_path/'jog';self.spin_count=0
        outer=self
        class Drive:
            def __init__(self):self.header=N(stamp=None,frame_id='');self.drive=N(speed=0.,steering_angle=0.)
        class Node:
            def __init__(self,name):outer.node_name=name
            def get_name(self):return outer.node_name
            def get_namespace(self):return '/'
            def get_clock(self):
                return N(now=lambda:N(nanoseconds=round(outer.ros_time()*1e9),to_msg=lambda:outer.ros_time()))
            def create_publisher(self,kind,topic,depth):
                assert topic=='/drive';outer.calls.append(('publisher',topic))
                def publish(msg):
                    outer.calls.append(('publish',msg.drive.speed,msg.drive.steering_angle,outer.wall))
                    if msg.drive.speed>0:
                        outer.positive+=1
                        if mode=='positive_error':raise OSError('publication outcome unknown')
                    elif mode=='zero_error':raise OSError('zero publish failed')
                return N(publish=publish)
            def create_subscription(self,kind,topic,callback,depth):
                assert topic=='/actuation/status';outer.callback=callback
            def create_client(self,kind,service):
                assert service=='/actuation/stop'
                def call_async(request):
                    outer.calls.append(('stop_request',service));outer.stopped=True
                    if mode=='stop_error':raise OSError('service vanished')
                    return N(done=lambda:mode!='ack_timeout',result=lambda:N(success=mode!='ack_rejected',message='STOP ACK, not physical stop'))
                return N(call_async=call_async,service_is_ready=lambda:not(mode=='host_lost' and outer.positive>0))
            def get_publishers_info_by_topic(self,topic):
                peer=lambda name:N(node_name=name,node_namespace='/')
                if topic=='/drive':
                    others=[peer('g1_g2_node')] if mode=='competing_before' or mode=='competing_during' and outer.positive else []
                    return [peer(outer.node_name),*others]
                if topic=='/drive/safe':return [peer('lidar_safety')]
                if topic=='/actuation/status':return [peer('virtual_mcu' if mode=='legacy' else 'runtime_actuation_host')]
                raise AssertionError(topic)
            def get_subscriptions_info_by_topic(self,topic):
                name='lidar_safety' if topic=='/drive' else ('actuation_host' if mode=='legacy' else 'runtime_actuation_host')
                return [N(node_name=name,node_namespace='/')]
            def destroy_node(self):outer.calls.append(('destroy',))
        def spin_once(node,timeout_sec):
            outer.wall+=.6 if mode=='oversleep' and timeout_sec==.005 else timeout_sec
            outer.spin_count+=1
            # 원 표본을 1 ms 앞에 둔다. 즉시 같은 부동소수값을 미래로 반올림하는 fixture는 피한다.
            status=legacy(outer.ros_time()-.001) if mode=='legacy' else runtime(outer.ros_time()-.001)
            if outer.stopped:
                status.update(state='ESTOP_LATCHED',generation=5)
                if 'motor_feedback' in status:status['motor_feedback']['generation']=5
            elif outer.positive:
                status['state']='ACTIVE'
                if mode=='fault':status['output_ready']=False
                if mode=='interrupt':outer.handlers[2](2,None)
            if mode!='no_status':outer.callback(N(data=json.dumps(status)))
        modules={'rclpy':N(init=lambda **kw:outer.calls.append(('init',kw)),spin_once=spin_once,
                           shutdown=lambda:outer.calls.append(('shutdown',))),
            'rclpy.node':N(Node=Node),'rclpy.signals':N(SignalHandlerOptions=N(NO='NO')),
            'ackermann_msgs.msg':N(AckermannDriveStamped=Drive),'std_msgs.msg':N(String=object),
            'std_srvs.srv':N(Trigger=N(Request=type('Request',(),{})))}
        for name,module in modules.items():monkeypatch.setitem(sys.modules,name,module)
        monkeypatch.setattr(jog,'time',N(monotonic=lambda:outer.wall,time_ns=lambda:1))
        def handler(number,callback):previous=outer.handlers.get(number);outer.handlers[number]=callback;return previous
        monkeypatch.setattr(jog,'signal',N(SIGINT=2,SIGTERM=15,signal=handler))
        monkeypatch.setattr(sys,'argv',['development_jog.py','left','--speed','.15','--seconds','.12',
            '--speed-limit','.2','--send','--output',str(self.output)])

    def ros_time(self):return 10.+self.wall-100.

    def run(self):
        try:jog.main();code=0
        except SystemExit as error:code=error.code
        return code,json.loads((self.output/'result.json').read_bytes())


@pytest.mark.parametrize('mode',['normal','legacy'])
def test_main_sequence_preserves_steering_then_stops_and_never_arms(monkeypatch,tmp_path,mode):
    io=FakeROS(monkeypatch,tmp_path,mode);code,result=io.run()
    assert code==0 and result['stop_ack'] and result['fresh_estop_state_seen']
    assert not result['physical_stop_verified'] and not result['state_observation_is_drive_permission']
    commands=[row for row in io.calls if row[0]=='publish']
    assert commands and commands[-1][1:3]==(0.,.2) and all(row[2]==.2 for row in commands)
    assert all(row[1]==.15 for row in commands[:-1])
    assert io.calls.index(commands[-1])<next(i for i,row in enumerate(io.calls) if row[0]=='stop_request')
    assert sum(row[0]=='stop_request' for row in io.calls)==1


@pytest.mark.parametrize('mode',['competing_before','no_status'])
def test_rejected_preflight_does_not_interfere_with_existing_control(monkeypatch,tmp_path,mode):
    io=FakeROS(monkeypatch,tmp_path,mode);code,result=io.run()
    assert code==1 and not result['started']
    assert not any(row[0] in ('publish','stop_request') for row in io.calls)
    assert io.wall<=103.1


@pytest.mark.parametrize('mode',['zero_error','positive_error','fault','host_lost','interrupt','competing_during','oversleep'])
def test_faults_always_attempt_stop_once_without_rearm(monkeypatch,tmp_path,mode):
    io=FakeROS(monkeypatch,tmp_path,mode);code,result=io.run()
    assert code==1 and result['started'] and result['stop_requested']
    assert sum(row[0]=='stop_request' for row in io.calls)==1
    assert [row for row in io.calls if row[0]=='publish'][-1][1:3]==(0.,.2)
    assert not result['physical_stop_verified']


@pytest.mark.parametrize('mode',['stop_error','ack_timeout','ack_rejected'])
def test_stop_failure_is_not_success_or_physical_stop(monkeypatch,tmp_path,mode):
    io=FakeROS(monkeypatch,tmp_path,mode);code,result=io.run()
    assert code==1 and result['zero_published'] and not result['stop_ack']
    assert not result['physical_stop_verified'] and io.wall<101.3


def test_oversleep_cannot_emit_positive_command_after_requested_duration():
    clock=[0.];sent=[]
    def pump(_):clock[0]+=.6
    jog.pulse(jog.plan('right',.15,.5,.2),wall=lambda:clock[0],pump=pump,check=lambda:None,
        publish=lambda *row:sent.append(row),cancelled=lambda:False)
    assert sent==[]


def test_preview_uses_no_ros_or_output(monkeypatch,tmp_path,capsys):
    output=tmp_path/'unused'
    monkeypatch.setattr(sys,'argv',['development_jog.py','right','--output',str(output)])
    jog.main();request=json.loads(capsys.readouterr().out)
    assert request['steering_rad']==-.2 and request['speed_limit_mps']==.2 and not output.exists()
