"""패널의 요청/모드/준비/자식 수명 경계. 합성 ROS API이며 실물/스케줄링 증명이 아니다."""
import ast
import io
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as N
import time

import pytest

from arena_autonomy import race_ops_manual as mod
from arena_autonomy.development_jog import plan, fresh_status, valid_mcu, mcu_identity, ParentHeartbeat
from arena_autonomy.race_ops_core import OpsCore, ARMED_MCU, LATCHED_MCU
from arena_autonomy.race_session import RaceSupervisor


def test_operator_cap_is_default_and_preserves_explicit_legacy_initial():
    vision=N(epoch=0, signal=N(started=False), fault=None)
    default=RaceSupervisor(vision,now_s=10.)
    default.configure_operator_speed_limit(1.5)
    assert default.development_speed_limit['enabled'] is True
    assert default.limit_speed(1.) == 1. and default.limit_speed(2.) == 1.5
    default.configure_operator_speed_limit(1.5,True,.3)
    assert default.limit_speed(1.) == .3


@pytest.mark.parametrize('angle', [-.2, 0., .2])
def test_angle_and_vt_are_explicit_not_servo_or_actual_distance(angle):
    value=plan('forward',.15,.5,.2,steering_rad=angle,max_steering_rad=.2)
    assert value['steering_rad']==angle and value['estimated_distance_m']==.075
    assert value['steering_reference']=='central_equivalent_wheel_angle_left_positive'
    with pytest.raises(ValueError):plan('forward',.15,.5,.2,steering_rad=.21,max_steering_rad=.2)


def submit(panel,op,number='req1',value=None):
    return panel.submit(op,number,'desk',panel.binding(),value,100.)


def test_autonomous_configuration_has_no_manual_actions():
    panel=mod.ManualPanel()
    for op in ('prepare','run','stop','set_speed_limit'):
        assert submit(panel,op,op)['reason']=='explicit_manual_configuration_required'
    assert panel.next_action() is None and panel.process is None


def test_request_binding_and_stop_invalidate_queued_run(tmp_path):
    panel=mod.ManualPanel('manual',output_root=tmp_path)
    panel.prepared_identity=('boot',4,'host')
    value=dict(speed_mps=.1,seconds=.5,steering_deg=0.)
    first=submit(panel,'run',value=value)
    assert submit(panel,'run',value=value)==first  # 같은 request는 재실행하지 않는다.
    assert submit(panel,'stop','stop1')['stage']=='queued'
    stop=panel.next_action();panel.stop('explicit_stop',100.)
    assert stop['op']=='stop' and panel.next_action() is None
    assert panel.entries['req1']['stage']=='cancelled' and panel.process is None


@pytest.mark.parametrize('value', [dict(speed_mps=.21,seconds=.5,steering_deg=0.),
    dict(speed_mps=.1,seconds=2.1,steering_deg=0.),dict(speed_mps=.1,seconds=.5,steering_deg=12.),
    dict(speed_mps=True,seconds=.5,steering_deg=0.),dict(speed_mps=.1,seconds=.5,steering_deg=float('nan'))])
def test_outside_manual_bounds_is_rejected_without_clamping(value):
    panel=mod.ManualPanel('manual');panel.prepared_identity=('boot',4,'host')
    assert submit(panel,'run',value=value)['reason']=='manual_value_out_of_range'
    assert panel.next_action() is None


class Pipe(io.StringIO):
    def close(self):self.recorded=self.getvalue();super().close()


class Process:
    def __init__(self,command,**kwargs):
        self.command,self.kwargs=command,kwargs
        self.stdin=Pipe();self.code=None;self.killed=False
        self.output=Path(command[command.index('--output')+1]);self.output.mkdir()
    def poll(self):return self.code
    def kill(self):self.killed=True;self.code=-9
    def complete(self):
        (self.output/'result.json').write_text(json.dumps(dict(stop_ack=True,fresh_estop_state_seen=True,
            error=None,errors=[],physical_stop_verified=False)),encoding='utf-8')
        self.code=0


def test_child_reuses_jog_and_eof_stop_is_bounded(tmp_path):
    panel=mod.ManualPanel('manual',output_root=tmp_path,spawn=Process)
    panel.prepared_identity=('boot',4,'host')
    submit(panel,'run',value=dict(speed_mps=.15,seconds=.5,steering_deg=-5.))
    entry=panel.next_action();panel.launch(entry,100.)
    process=panel.process
    assert process.command[1:3]==['-m','arena_autonomy.development_jog']
    assert '--parent-heartbeat-stdin' in process.command and '--send' in process.command
    assert process.kwargs.get('shell') is None
    panel.tick(100.,True);assert process.stdin.getvalue()=='HEARTBEAT\n'
    panel.tick(100.1,False)
    assert process.stdin.closed and process.stdin.recorded.endswith('STOP\n')
    process.complete();ended=panel.tick(100.2,False)
    assert ended['result']['physical_stop_verified'] is False and panel.process is None


def test_successful_child_does_not_allow_second_pulse_without_prepare(tmp_path):
    panel=mod.ManualPanel('manual',output_root=tmp_path,spawn=Process)
    panel.prepared_identity=('boot',4,'host')
    submit(panel,'run',value=dict(speed_mps=.1,seconds=.5,steering_deg=0.))
    panel.launch(panel.next_action(),100.);panel.process.complete()
    assert panel.tick(100.1,True)['stage']=='completed'
    assert submit(panel,'run','again',dict(speed_mps=.1,seconds=.5,steering_deg=0.))['reason']=='prepare_required'


def test_parent_pipe_eof_cancels_child_without_signal_or_buffered_stdin_lock():
    read_fd,write_fd=os.pipe()
    with os.fdopen(read_fd,'rb',buffering=0) as stream:
        heartbeat=ParentHeartbeat(stream)
        os.write(write_fd,b'HEARTBEAT\n');os.close(write_fd)
        heartbeat.thread.join(1.)
        assert heartbeat.cancelled() and not heartbeat.thread.is_alive()


@pytest.fixture
def fake_node(tmp_path):
    from arena_autonomy import race_ops_page
    path=Path(race_ops_page.__file__).with_name('race_ops_node.py')
    tree=ast.parse(path.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RaceOps')
    clock=[100.]
    namespace=dict(Node=object,graph_reason=mod.graph_reason,fresh_status=fresh_status,valid_mcu=valid_mcu,
        mcu_identity=mcu_identity,ARMED_MCU=ARMED_MCU,LATCHED_MCU=LATCHED_MCU,
        Trigger=N(Request=lambda:object()),MCU_CALL_TIMEOUT_S=2.,time=N(monotonic=lambda:clock[0]))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),namespace)
    node=object.__new__(namespace['RaceOps'])
    node.core=OpsCore('car');node.manual=mod.ManualPanel('manual',output_root=tmp_path,spawn=Process)
    node.manual_prepare=None;node.calls=[];node.rearm=None;node.result_timeout_s=3.
    node.get_clock=lambda:N(now=lambda:N(nanoseconds=int(10e9)))
    node.core.client_seen('desk',100.)
    node.core.update_mcu(dict(v=1,boot='fixture',generation=4,owner='host',state='ESTOP_LATCHED',
        now_us=10_000_000,estop_held=False,feedback_valid=True,output_ready=True,target_speed_mps=0.),100.)
    node.get_namespace=lambda:'/'
    peer=lambda name:N(node_name=name,node_namespace='/')
    node.extra_drive=[]
    node.get_publishers_info_by_topic=lambda topic:{'/drive':node.extra_drive,
        '/drive/safe':[peer('lidar_safety')],'/actuation/status':[peer('actuation_host')]}[topic]
    node.get_subscriptions_info_by_topic=lambda topic:[peer('lidar_safety' if topic=='/drive' else 'actuation_host')]
    node.service_calls=[]
    def service(name):
        def call(_):
            node.service_calls.append(name)
            node.core.mcu['state']={'clear_stop':'DISARMED','arm':'ARMED','stop':'ESTOP_LATCHED'}[name]
            node.core.mcu['generation']+=1
            return N(done=lambda:True,result=lambda:N(success=True,message='synthetic ACK'))
        return N(service_is_ready=lambda:True,call_async=call)
    node.mcu_clients={name:service(name) for name in ('clear_stop','arm','stop')}
    return node


def prepare(node):
    submit(node.manual,'prepare')
    for _ in range(3):
        node.tick_manual(100.);node.poll_calls(100.)


def test_explicit_prepare_then_run_uses_clear_arm_without_auto_pulse(fake_node):
    node=fake_node
    node.tick_manual(100.);assert node.service_calls==[]
    prepare(node)
    assert node.service_calls==['clear_stop','arm']
    assert node.manual.prepared_identity is not None and node.manual.process is None
    submit(node.manual,'run','run1',dict(speed_mps=.1,seconds=.5,steering_deg=5.))
    node.tick_manual(100.)
    assert node.manual.process is not None and node.manual.state=='running'
    node.manual.process.complete();node.tick_manual(100.)
    assert node.service_calls[-1]=='stop' and node.manual.state=='idle'


def test_autonomous_publisher_blocks_preparation_even_if_command_zero(fake_node):
    node=fake_node;node.extra_drive=[N(node_name='local_pursuit',node_namespace='/')]
    prepare(node)
    assert node.service_calls==[] and node.manual.prepared_identity is None
    assert node.manual.entries['req1']['reason']=='autonomy_or_other_drive_publisher_present'


def test_prepared_connection_loss_requests_stop(fake_node):
    node=fake_node;prepare(node)
    node.core.clients.clear();node.tick_manual(100.)
    assert node.service_calls[-1]=='stop' and node.manual.prepared_identity is None


def test_manual_cap_requires_stopped_mcu_and_rejects_above_config(fake_node):
    node=fake_node
    submit(node.manual,'set_speed_limit','limit1',.1);node.tick_manual(100.)
    assert node.manual.limit_mps==.1 and node.manual.snapshot()['pulse_limit_mps']==.1
    assert submit(node.manual,'set_speed_limit','bad',2.)['reason']=='speed_limit_out_of_range'
    prepare(node)
    submit(node.manual,'set_speed_limit','armed',.15);node.tick_manual(100.)
    assert node.manual.limit_mps==.1 and node.manual.entries['armed']['reason']=='mcu_stop_required'


def test_clear_arm_service_exception_does_not_escape_tick_and_requests_stop(fake_node):
    node=fake_node
    def failed(_):
        def result():raise RuntimeError('synthetic transport error')
        return N(done=lambda:True,result=result)
    node.mcu_clients['clear_stop']=N(service_is_ready=lambda:True,call_async=failed)
    submit(node.manual,'prepare');node.tick_manual(100.)
    node.poll_calls(100.)  # future.result 예외가 executor를 빠져나가지 않는다.
    node.tick_manual(100.)
    assert node.manual.entries['req1']['stage']=='failed'
    assert node.manual.entries['req1']['reason']=='manual_prepare_service_rejected'
    assert node.service_calls==['stop'] and node.manual.process is None
