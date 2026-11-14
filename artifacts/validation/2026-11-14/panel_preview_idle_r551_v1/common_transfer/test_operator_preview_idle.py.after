"""실제 Handler GET을 메모리 요청으로 실행. ROS는 기존 구독 대역이며 socket/장치 없음."""
import ast
import copy
import hmac
import json
from pathlib import Path
from types import SimpleNamespace as N
from urllib.parse import parse_qs,urlsplit

from arena_autonomy.race_ops_core import OpsCore,token
from arena_autonomy.race_ops_manual import ManualPanel
from arena_autonomy.race_ops_preview import Preview
from test_operator_preview import fake_ros,until

ROOT=Path(__file__).resolve().parents[1]


def handler(clock,ops):
    source=ROOT/'src/arena_autonomy/arena_autonomy/race_ops_node.py'
    cls=next(n for n in ast.parse(source.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef) and n.name=='Handler')
    scope=dict(BaseHTTPRequestHandler=object,hmac=hmac,json=json,token=token,PAGE='fixture',
        parse_qs=parse_qs,urlsplit=urlsplit,time=N(monotonic=lambda:clock[0],time=lambda:clock[0]))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),scope)
    h=scope['Handler']();h.server=N(ops=ops)
    h.headers={'X-Arena-Token':'operator-fixture','X-Arena-Vehicle':'car'}
    h.respond=lambda code,body,kind=None,**headers:(code,json.loads(body),headers)
    return h


def ops(clock,fake_ros,client='owner'):
    preview=Preview(wall=lambda:clock[0])
    preview.configure(dict(lidar=True,camera=True,lidar_type='native',lidar_topic='/scan/native',camera_topic='/rgb'),client)
    preview.sync(fake_ros)
    preview.cache['lidar']=dict(points=[[1.,0.,1.]],sequence=1,received_wall_s=clock[0],data=None)
    preview.cache['camera']=dict(frame='camera',sequence=2,width=640,height=480,received_wall_s=clock[0],data=b'fixture')
    return N(core=OpsCore('car'),manual=ManualPanel(),preview=preview,vehicle_id='car',token='operator-fixture',view_token='viewer-fixture')


def get(h,client='owner',hidden=False):
    h.path='/api/state?client='+client+('&preview=0' if hidden else '')
    return h.do_GET()


def test_hidden_owner_keeps_operator_heartbeat_but_preview_expires(fake_ros):
    clock=[10.];state=ops(clock,fake_ros);h=handler(clock,state)
    try:
        assert len(fake_ros.subs)==2 and state.preview.lease_until==13.
        for now in [12.,12.8]:
            clock[0]=now
            code,row,_=get(h,hidden=True)
            assert code==200 and row['preview']['lidar'] is None and row['preview']['camera'] is None
            assert state.core.clients['owner']==now
            assert state.preview.desired['lidar'] and state.preview.desired['camera']
            assert state.preview.lease_until==13.  # GET이 전역 OFF를 보내지는 않는다.
        clock[0]=13.01;state.preview.sync(fake_ros)
        until(lambda:state.preview.worker is None)
        assert fake_ros.subs==[] and state.preview.cache=={}
        assert state.core.client_fresh('owner',clock[0])
    finally:state.preview.close(fake_ros)


def test_hidden_nonowner_cannot_disable_visible_owner_or_clear_shared_cache(fake_ros):
    clock=[10.];state=ops(clock,fake_ros,client='visible');h=handler(clock,state)
    cache=copy.deepcopy(state.preview.cache)
    try:
        clock[0]=12.;get(h,'hidden',hidden=True)
        assert state.preview.lease_until==13.
        get(h,'visible');assert state.preview.lease_until==15.
        clock[0]=14.;_,row,_=get(h,'hidden',hidden=True)
        assert state.preview.lease_until==15. and state.preview.client=='visible'
        assert len(fake_ros.subs)==2 and state.preview.cache==cache
        assert state.preview.desired['lidar'] and state.preview.desired['camera']
        assert row['preview']['lidar'] is None and row['preview']['camera'] is None
        assert state.core.client_fresh('hidden',14.)
    finally:state.preview.close(fake_ros)


def test_query_omission_retains_exact_legacy_response_and_lease(fake_ros):
    clock=[10.];state=ops(clock,fake_ros)
    h=handler(clock,state)
    try:
        clock[0]=12.
        code,row,headers=get(h)
        assert code==200 and headers==dict(X_Arena_Role='operator')
        assert state.preview.lease_until==15. and state.core.clients['owner']==12.
        expected=state.core.view(12.,12.)
        expected.update(manual=state.manual.snapshot(),preview=state.preview.snapshot())
        assert row==expected  # query 생략은 전체 상태와 미리보기 payload를 보존한다.
        assert row['preview']['lidar']['points']==[[1.,0.,1.]]
        assert row['preview']['camera']['sequence']==2
        h.path='/api/state?client=owner&preview=1'
        assert h.do_GET()==(code,row,headers)
    finally:state.preview.close(fake_ros)
