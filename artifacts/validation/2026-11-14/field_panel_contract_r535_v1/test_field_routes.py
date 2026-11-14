"""실제 Handler/운영 코드를 메모리 HTTP 요청과 가짜 MCU에 연결. socket/ROS/장치 없음."""
import ast
import hmac
import io
import json
from pathlib import Path
from types import SimpleNamespace as N
from urllib.parse import parse_qs, urlsplit

import pytest

from arena_autonomy.race_ops_core import OpsCore, token
from arena_autonomy.race_ops_manual import ManualPanel
from arena_autonomy.race_ops_preview import Preview
from arena_autonomy.race_ops_page import PAGE
from test_race_ops import status, VIEW
from test_operator_panel_manual import fake_node, prepare, submit


ROOT = Path(__file__).resolve().parents[4]


def handler():
    source=ROOT/'src/arena_autonomy/arena_autonomy/race_ops_node.py'
    tree=ast.parse(source.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Handler')
    scope=dict(BaseHTTPRequestHandler=object,hmac=hmac,json=json,PAGE=PAGE,token=token,
        MAX_BODY=4096,urlsplit=urlsplit,parse_qs=parse_qs,time=N(monotonic=lambda:100.,time=lambda:100.))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),scope)
    instance=scope['Handler']()
    core=OpsCore('car')
    core.update_status(status(resume_start_ready=True,development=dict(enabled=True,limit_mps=.5,maximum_mps=1.5)),100.)
    instance.server=N(ops=N(core=core,manual=ManualPanel('manual'),preview=Preview(wall=lambda:100.),
        vehicle_id='car',token='operator-fixture',view_token='viewer-fixture'))
    instance.headers={'X-Arena-Token':'operator-fixture','X-Arena-Vehicle':'car'}
    instance.respond=lambda code,body,kind=None,**headers:(code,json.loads(body) if isinstance(body,str) else body)
    return instance


def post(h,path,body):
    raw=json.dumps(body).encode();h.rfile=io.BytesIO(raw);h.path=path
    h.headers['Content-Length']=str(len(raw))
    return h.do_POST()


def test_actual_routes_keep_speed_and_manual_requests_distinct():
    h=handler();ops=h.server.ops
    code,result=post(h,'/api/command',dict(op='set_speed_limit',request='cap',client='desk',view=VIEW,value=.1))
    assert code==200 and result['stage']=='queued' and ops.core.next_action()['value']==.1
    assert ops.manual.next_action() is None
    code,result=post(h,'/api/manual',dict(op='set_speed_limit',request='manualcap',client='desk',view=ops.manual.binding(),value=.1))
    assert code==200 and result['stage']=='queued' and ops.manual.next_action()['value']==.1
    assert ops.core.next_action() is None and ops.manual.process is None


def test_preview_viewer_configuration_uses_no_command_queue_or_device():
    h=handler();ops=h.server.ops;h.headers['X-Arena-Token']='viewer-fixture'
    selected=dict(lidar=True,camera=True,lidar_type='native',lidar_topic='/hardware_runtime/c1/native',
        camera_topic='/hardware_runtime/camera/image_raw')
    code,result=post(h,'/api/preview',dict(client='viewer',config=selected))
    assert code==200 and result['config']==selected
    assert ops.preview.subscriptions=={} and ops.preview.worker is None
    assert ops.core.next_action() is None and ops.manual.next_action() is None
    assert post(h,'/api/command',dict(op='stop',request='denied',client='viewer'))[0]==403
    h.path='/api/preview.jpg';assert h.do_GET()[0]==404


@pytest.mark.parametrize('state,started,latch,allowed',[
    ('WAIT_GREEN',False,None,True),('RUNNING',True,None,False),
    ('WAIT_GREEN',True,None,False),('WAIT_GREEN',False,'explicit_stop',False),
])
def test_signal_button_accepts_only_initial_wait_green(state,started,latch,allowed):
    h=handler();ops=h.server.ops
    ops.core.update_status({**status(started=started,latch=latch),'state':state},100.)
    body=dict(op='manual_visual_start',request='signal',client='desk',view=VIEW,confirm='SIGNAL_CONFIRMED')
    code,result=post(h,'/api/command',body)
    assert code==200 and (result['stage']=='queued') is allowed
    if allowed:
        assert post(h,'/api/command',{**body,'request':'unconfirmed','confirm':None})[1]['adapter_reason']=='signal_confirmation_required'


def test_global_stop_without_snapshot_cancels_active_manual_fake_child(fake_node):
    node=fake_node;prepare(node)
    submit(node.manual,'run','pulse',dict(speed_mps=.05,seconds=.5,steering_deg=0.))
    node.tick_manual(100.);assert node.manual.process is not None
    node.published=[];node.publish=node.published.append
    receipt=node.core.submit('stop','globalstop',None,'desk',None,100.)
    assert receipt['stage']=='queued'
    node.execute(node.core.next_action())
    assert node.manual.prepared_identity is None and node.manual.reason=='explicit_stop'
    assert node.manual.process.stdin.closed and node.service_calls[-1]=='stop'
    assert node.published==[dict(v=1,request='globalstop',op='stop')]
