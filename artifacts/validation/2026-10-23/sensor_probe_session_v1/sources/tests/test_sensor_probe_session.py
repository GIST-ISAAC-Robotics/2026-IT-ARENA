"""센서/ROS 없는 실제 자식으로 선택형 기록 감독자의 종료를 검사한다."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

import sensor_probe_session as tool

CHILD = r'''
import json,os,pathlib,signal,sys,time
mode,path=sys.argv[1],pathlib.Path(sys.argv[2])
if mode == 'early':
    sys.exit(0)
if mode == 'fail':
    sys.exit(7)
if mode == 'hang':
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
else:
    signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
if mode != 'unready':
    path.write_text(json.dumps(dict(ready=True,pid=os.getpid()+(1 if mode=='wrong_pid' else 0))))
while True:
    time.sleep(.01)
'''


def run(tmp_path, mode, cancelled=lambda:False):
    with (tmp_path/'child.log').open('w') as log:
        result = tool.supervise([sys.executable,'-c',CHILD,mode,str(tmp_path/'ready.json')],
            tmp_path/'ready.json',log,duration=.1,startup_timeout=1.,stop_timeout=.1,
            term_timeout=.1,kill_timeout=1.,cancelled=cancelled)
    assert result['process_exited']
    with pytest.raises(ProcessLookupError):
        os.kill(result['pid'],0)
    return result


def test_planned_stop_is_not_forced(tmp_path):
    result = run(tmp_path,'normal')
    assert result['lifecycle_complete'] and result['stop_reason']=='planned_stop'
    assert not result['forced_termination'] and result['exit_code']==0
    assert [e['signal'] for e in result['events'] if e['event']=='signal'] == ['SIGINT']


@pytest.mark.parametrize('mode,code',[('early',0),('fail',7)])
def test_early_exit_including_zero_is_not_success(tmp_path,mode,code):
    result = run(tmp_path,mode)
    assert not result['lifecycle_complete'] and result['stop_reason']=='premature_exit'
    assert result['exit_code']==code and not result['ready_observed']


def test_missing_ready_stops_child_but_never_succeeds(tmp_path):
    result = run(tmp_path,'unready')
    assert result['stop_reason']=='startup_timeout' and not result['lifecycle_complete']
    assert not result['ready_observed']


def test_wrong_pid_ready_is_not_trusted(tmp_path):
    result = run(tmp_path,'wrong_pid')
    assert result['stop_reason']=='supervisor_error' and result['errors']
    assert not result['lifecycle_complete']


def test_ignored_stop_escalates_only_owned_child_and_never_succeeds(tmp_path):
    result = run(tmp_path,'hang')
    assert result['forced_termination'] and not result['lifecycle_complete']
    assert result['exit_code']==-signal.SIGKILL
    assert [e['signal'] for e in result['events'] if e['event']=='signal'] == ['SIGINT','SIGTERM','SIGKILL']


def test_cancel_does_not_masquerade_as_planned_duration(tmp_path):
    start = time.monotonic()
    result = run(tmp_path,'normal',cancelled=lambda:time.monotonic()-start>.05)
    assert result['stop_reason']=='cancelled' and not result['lifecycle_complete']


@pytest.mark.parametrize('name,value', [('duration',0.),('duration',True),('duration',float('nan')),
    ('startup_timeout',0.),('stop_timeout',121.),('term_timeout',31.),('kill_timeout',float('inf'))])
def test_bad_limits_fail_before_child(monkeypatch,tmp_path,name,value):
    monkeypatch.setattr(tool.subprocess,'Popen',lambda *a,**kw:pytest.fail('must not start'))
    args = dict(duration=1.)
    args[name] = value
    with pytest.raises(ValueError):
        tool.supervise([],tmp_path/'ready.json',None,**args)


def test_existing_ready_not_reused(monkeypatch,tmp_path):
    path=tmp_path/'ready.json'
    path.write_text('{}')
    monkeypatch.setattr(tool.subprocess,'Popen',lambda *a,**kw:pytest.fail('must not start'))
    with pytest.raises(FileExistsError):
        tool.supervise([],path,None,duration=1.)


def test_exited_child_never_signalled(monkeypatch):
    class Done:
        def poll(self): return 0
    monkeypatch.setattr(tool.os,'killpg',lambda *a:pytest.fail('must not signal'))
    assert not tool.signal_owned_group(Done(),signal.SIGINT)


def test_changed_group_is_not_signalled(monkeypatch):
    class Child:
        pid=12345
        def poll(self): return None
    monkeypatch.setattr(tool.os,'getpgid',lambda pid:99999)
    monkeypatch.setattr(tool.os,'killpg',lambda *a:pytest.fail('must not signal another group'))
    with pytest.raises(RuntimeError):
        tool.signal_owned_group(Child(),signal.SIGINT)


def test_uninterruptible_child_is_reported_not_claimed_dead(monkeypatch,tmp_path):
    class Stuck:
        pid=12345
        def poll(self): return None
        def wait(self,timeout): raise subprocess.TimeoutExpired('fixture',timeout)
    monkeypatch.setattr(tool.subprocess,'Popen',lambda *a,**kw:Stuck())
    monkeypatch.setattr(tool,'signal_owned_group',lambda *a:True)
    result=tool.supervise([],tmp_path/'ready.json',None,duration=.1,cancelled=lambda:True)
    assert not result['process_exited'] and result['exit_code'] is None
    assert result['forced_termination'] and not result['lifecycle_complete']


def test_invalid_topics_rejected_before_output(tmp_path,monkeypatch):
    monkeypatch.setattr(tool,'ROOT',tmp_path)
    output=tmp_path/'artifacts/session'
    with pytest.raises(ValueError):
        tool.main(['--output',str(output),'--clock-mode','system','--sensor-topic','/drive','--duration','1'])
    assert not output.exists()
