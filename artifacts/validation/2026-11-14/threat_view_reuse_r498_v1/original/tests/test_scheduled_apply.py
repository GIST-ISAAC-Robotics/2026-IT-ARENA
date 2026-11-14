import copy
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.evaluation_schedule import EvaluationSchedule,audit_scheduled

HARNESS=r'''
#include <iostream>
#include "arena_gazebo/scheduled_apply.hpp"
using namespace arena;
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  std::string mode=argv[1];ScheduledApply q;
  q.Receive("A1 1 arm boot:1 1000 0 91000 0 0",1000,1000000,1);
  q.Step(1000,1000,1000000);
  std::string set="A1 2 set boot:1 2000 22000 92000 1 0.1";
  if(mode=="late")q.Receive(set,22000,22000000,22);
  else if(mode=="malformed")q.Receive(set+" extra",2000,2000000,2);
  else if(mode=="nonfinite")q.Receive("A1 2 set boot:1 2000 22000 92000 nan 0",2000,2000000,2);
  else if(mode=="epoch")q.Receive("A1 2 set other:1 2000 22000 92000 1 0",2000,2000000,2);
  else if(mode=="sequence")q.Receive("A1 3 set boot:1 2000 22000 92000 1 0",2000,2000000,2);
  else if(mode=="future_stamp")q.Receive(set,1000,1000000,1);
  else if(mode=="expired")q.Receive("A1 2 set boot:1 2000 22000 21000 1 0",2000,2000000,2);
  else {q.Receive(set,2000,2000000,2);
    if(mode=="duplicate")q.Receive(set,2000,2000000,2);
    if(mode=="stop_before")q.Receive("A1 3 stop boot:1 21000 0 0 0 0",21000,21000000,21);
    for(int64_t t=2000;t<=22000;t+=1000)q.Step(t,1000,t*1000);
    if(mode=="stop_after"){
      q.Receive("A1 3 set boot:1 23000 43000 99000 2 0",23000,23000000,23);
      q.Receive("A1 4 stop boot:1 24000 0 0 0 0",24000,24000000,24);
      for(int64_t t=23000;t<=44000;t+=1000)q.Step(t,1000,t*1000);
    }
    if(mode=="wrong_step")q.Step(23000,500,23000000);
    if(mode=="sim_expiry")q.Step(92000,1000,92000000);
    if(mode=="wall_expiry")q.Step(23000,1000,31000000000LL);
    if(mode=="regression")q.Step(1000,1000,23000000);
    if(mode=="rearm_same"){
      q.Receive("A1 3 stop boot:1 23000 0 0 0 0",23000,23000000,23);
      q.Receive("A1 4 arm boot:1 24000 0 99000 0 0",24000,24000000,24);
    }
    if(mode=="missed"){
      q.Receive("A1 3 set boot:1 23000 43000 99000 2 0",23000,23000000,23);
      q.Step(44000,1000,44000000);
    }
  }
  for(const auto &event:q.TakeEvents())std::cout<<event<<"\n";
  std::cout<<"{\"speed\":"<<q.speed<<",\"failed\":"<<(q.failed?"true":"false")<<"}\n";
}
'''

@pytest.fixture(scope='module')
def harness(tmp_path_factory):
    base=tmp_path_factory.mktemp('scheduled_cpp');source=base/'test.cpp';source.write_text(HARNESS)
    exe=base/'test'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT/'src/arena_gazebo/include'),
                    str(source),'-o',str(exe)],check=True,capture_output=True,text=True)
    return lambda case:[json.loads(x) for x in subprocess.check_output([str(exe),case],text=True).splitlines()]

def test_cpp_exact_apply_and_atomic_pair(harness):
    rows=harness('normal');applied=[r for r in rows if r.get('type')=='applied']
    assert len(applied)==1 and applied[0]['sim_us']==22000
    assert applied[0]['speed']==1 and applied[0]['steering']==.1
    assert rows[-1]=={'speed':1,'failed':False}

@pytest.mark.parametrize('case',['late','malformed','nonfinite','epoch','sequence','future_stamp','expired',
    'duplicate','wrong_step','sim_expiry','wall_expiry','regression','rearm_same','missed'])
def test_cpp_faults_latch_zero(harness,case):
    rows=harness(case)
    assert rows[-1]=={'speed':0,'failed':True}
    assert any(r.get('type')=='fault' for r in rows)

@pytest.mark.parametrize('case',['stop_before','stop_after'])
def test_cpp_stop_cancels_future_positive_without_wait(harness,case):
    rows=harness(case)
    assert rows[-1]=={'speed':0,'failed':False}
    assert any(r.get('type')=='cancelled' for r in rows)
    stop=next(r['sim_us'] for r in rows if r.get('type')=='stopped')
    assert not any(r.get('type')=='applied' and r['sim_us']>=stop for r in rows)

def status(stamp=1000,state='ARMED',speed=0):
    return {'boot':'boot','generation':1,'state':state,'now_us':stamp,'deadline_us':91000,'speed':speed}
def guard(speed=0):return SimpleNamespace(blocked=False,speed=speed,steering=.1)

def test_python_only_guard_accepted_and_stamp_based_time():
    q=EvaluationSchedule();s=status();g=guard()
    assert not q.status(s,g,1000,False)
    assert q.status(s,g,1000,True)==['A1 1 arm boot:1 1000 0 91000 0 0']
    s=status(2000,'ACTIVE');g=guard(1)
    p=q.status(s,g,2500,True)[0].split()
    assert p[5]=='22000' # 수신 시각 2500이 아니라 원래 stamp 기준
    assert not q.status(s,g,3000,True)
    g.blocked=True
    assert ' stop ' in q.status(s,g,4000,False)[0]
    assert not q.status(s,guard(1),5000,True)

def test_ack_missing_timeout_stops():
    q=EvaluationSchedule();q.status(status(),guard(),1000,True)
    assert not q.watch(20999)
    assert ' stop ' in q.watch(21000)[0] and q.failed=='ack_timeout'


def test_safety_intervention_preempts_pending_queue():
    pytest.importorskip('rclpy')
    from arena_vehicle_interface.scheduled_actuator import ScheduledSimActuator
    sent=[];q=EvaluationSchedule()
    obj=SimpleNamespace(schedule=q,safety_reason=None,safety_time=None,safety_observed=False,
        now_us=lambda:4000,scheduled_pub=SimpleNamespace(publish=lambda msg:sent.append(msg.data)))
    ScheduledSimActuator.on_safety(obj,SimpleNamespace(data='{"reason":"missing_input"}'))
    assert not sent and q.failed is None  # ARM 전 잠금 상태
    q.status(status(),guard(),1000,True)
    q.status(status(2000,'ACTIVE'),guard(1),2000,True)
    ScheduledSimActuator.on_safety(obj,SimpleNamespace(data='{"reason":"clear"}'))
    ScheduledSimActuator.on_safety(obj,SimpleNamespace(data='{"reason":"braking"}'))
    assert len(sent)==1 and ' stop ' in sent[0] and q.failed=='safety_intervention'
    ScheduledSimActuator.on_safety(obj,SimpleNamespace(data='{"reason":"clear"}'))
    assert not q.status(status(5000,'ACTIVE'),guard(1),5000,True)  # clear만으로 재개하지 않음


def test_sender_stale_safety_stops_without_new_mcu_status():
    pytest.importorskip('rclpy')
    from collections import deque
    from arena_vehicle_interface.applied_guard import AppliedGuard
    from arena_vehicle_interface.scheduled_actuator import ScheduledSimActuator
    sent=[];q=EvaluationSchedule();q.epoch='boot:1'
    g=AppliedGuard(wall_timeout_s=30)
    g.blocked=False;g.epoch=('boot',1)
    obj=SimpleNamespace(schedule=q,guard=g,pending=deque(),now_us=lambda:200000,
        safety_observed=True,safety_time=(100000,0),safety_reason='clear',
        scheduled_pub=SimpleNamespace(publish=lambda msg:sent.append(msg.data)),
        audit=SimpleNamespace(publish=lambda msg:None),schedule_state=SimpleNamespace(publish=lambda msg:None))
    ScheduledSimActuator.output(obj)
    assert g.blocked and q.failed=='safety_status_timeout' and len(sent)==1 and ' stop ' in sent[0]

def test_cross_language_audit(harness):
    q=EvaluationSchedule()
    sent=[q.emit('arm','boot:1',1000,0,91000),q.emit('set','boot:1',2000,22000,92000,1,.1),
          q.emit('set','boot:1',23000,43000,99000,2,0),q.emit('stop','boot:1',24000,0,0)]
    events=harness('stop_after')[:-1]
    assert audit_scheduled([{'packet':p} for p in sent],events,[{'failure':None}])['passed']
    for change in ('lost','wrong_step','wrong_value','duplicate','sender_failure'):
        e=copy.deepcopy(events);states=[{'failure':None}]
        if change=='lost':del e[1]
        if change=='duplicate':e.append(e[-1])
        if change=='wrong_step':next(x for x in e if x['type']=='applied')['sim_us']+=1000
        if change=='wrong_value':next(x for x in e if x['type']=='received')['speed']=1
        if change=='sender_failure':states=[{'failure':'ack_timeout'}]
        assert not audit_scheduled([{'packet':p} for p in sent],e,states)['passed']

def test_launch_opt_in_only():
    spec=importlib.util.spec_from_file_location('scheduled_launch',ROOT/'src/arena_bringup/launch/simulation.launch.py')
    launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    xml='<sdf><model><plugin name="arena::SingleMotorDrive"><evaluation_command_trace>true</evaluation_command_trace></plugin></model></sdf>'
    call=lambda **kw:launch._evaluation_scheduled_apply(**({'model_xml':xml,'enabled':True,'evidence':'test',
        'startup':'embedded_paused','headless':True,'local':True,'guarded':True,'autonomy':False,'wall_timeout':30}|kw))
    assert call(enabled=False)==xml
    assert '<evaluation_scheduled_apply>true' in call()
    for kw in ({'evidence':''},{'startup':'service'},{'headless':False},{'local':False},{'guarded':False},{'autonomy':True}):
        with pytest.raises(ValueError):call(**kw)


def test_registered_r7_chain_and_current_source():
    import hashlib
    from source_contract_helpers import hash_before_observation_extension
    sha=lambda p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
    path='config/tests/scheduled_command_r7_sha256.json'
    c=json.loads((ROOT/path).read_text())
    parent=json.loads((ROOT/c['parent_contract']).read_text())
    assert sha(c['parent_contract'])==c['parent_contract_sha256']
    before=json.loads((ROOT/'artifacts/validation/2026-10-11/scheduled_apply_v1/before_manifest.json').read_text())
    expected=dict(parent['sha256']);impl=dict(parent['implementation_sha256'])
    for group,dest in [('approved_changes',expected),('approved_evaluation_scripts',expected),
                       ('approved_implementation_changes',impl)]:
        for p,change in c[group].items():
            assert change['before']==before.get(p)
            assert change['after']==hash_before_observation_extension(ROOT,p)
            dest[p]=change['after']
    assert c['sha256']==expected and c['implementation_sha256']==impl
    from source_contract_helpers import hash_before_observation_extension
    for p,value in {**expected,**impl,**c['evaluation_tools_sha256']}.items():
        assert hash_before_observation_extension(ROOT,p)==value,p
    for p in ('src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py',
              'src/arena_gazebo/include/arena_gazebo/drivetrain.hpp'):
        assert sha(p)==before[p]
