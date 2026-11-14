"""실제 ROS 노드/메시지의 개발 상한 연결. 경로 출력은 고정하며 차량을 구동하지 않는다."""
import json,time
from types import SimpleNamespace
import pytest
rclpy=pytest.importorskip('rclpy')
from std_msgs.msg import String
from arena_autonomy.wall_follow import WallFollow
from test_race_session_node import make_node,close
from test_race_ops_node import ops,call


def test_real_node_command_clamps_emission_and_preserves_pending_stop():
    n=make_node(race_control_mode='b3_candidate',max_speed_mps='2.8',motion_gear_ratio=str(2.5*81/17),
                development_speed_limit_enabled='true',development_speed_limit_mps='0.3')
    original=n.publisher;messages=[];results=[]
    try:
        assert original.topic_name=='/drive'
        assert '/drive/safe' not in {p.topic_name for p in n.publishers}
        n.race_result_publisher=SimpleNamespace(publish=lambda m:results.append(json.loads(m.data)))
        stamp=n.get_clock().now().to_msg();now=n.now_s()
        n.on_mcu_status(String(data=json.dumps(dict(v=1,boot='test-mcu',generation=0,state='DISARMED',
            now_us=round(now*1e6),estop_held=False))))
        payload=dict(v=1,request='limit1',op='set_speed_limit',session=n.race.session,
            race_epoch=0,intervention=n.race.permission.intervention,speed_limit_mps=.2)
        n.on_race_command(String(data=json.dumps(payload)))
        assert results[-1]['accepted'] and n.race.permission.latch=='process_start'
        n.publisher=SimpleNamespace(publish=messages.append)
        # 출발/경로 판정은 별도 기존 회귀 대상. 이 검사는 발행 직전 상한과 메시지 의미만 고정 입력으로 확인.
        n.motion_gate=lambda status:True
        n.motion.project=lambda scan:([],100)
        n.scan=SimpleNamespace(header=SimpleNamespace(stamp=stamp))
        n.rgb_stamp=now;n.scan_wall_time=n.image_wall_time=time.monotonic()
        n.command=lambda points:(2.,.1,{'reason':'fixed_test_path'})
        n.last_speed=1.
        WallFollow.control(n)
        assert messages[-1].drive.speed==.2 and n.last_speed==.2
        assert messages[-1].header.frame_id=='base_link'
        count=len(messages);n.command=lambda points:None
        WallFollow.control(n)
        assert len(messages)==count and not n.command_was_published
        n.on_race_command(String(data=json.dumps(dict(v=1,request='stop2',op='stop'))))
        assert messages[-1].drive.speed==0. and n.race.development_speed_limit['limit_mps']==.2
    finally:
        n.publisher=original;close(n)


def test_http_development_limit_keeps_readonly_and_numeric_payload(ops):
    race=dict(session='s',race_id='r',race_epoch=0,intervention=1,latch='explicit_stop',started=True,
              resume=None,development=dict(enabled=True,limit_mps=.3,maximum_mps=2.8))
    ops.core.update_status(dict(race=race,state='RACE_LATCHED'),time.monotonic())
    view={k:race[k] for k in ('session','race_id','race_epoch','intervention')}
    body=dict(op='set_speed_limit',request='cap1',client='desk',view=view,value=.2)
    assert call(ops,'/api/command',body=body,token='viewer-token')[0]==403
    code,_,data=call(ops,'/api/command',body=body)
    assert code==200 and json.loads(data)['stage']=='queued'
    sent=[];ops.publish=sent.append
    ops.execute(ops.core.next_action())
    assert sent==[dict(v=1,request='cap1',op='set_speed_limit',session='s',race_epoch=0,intervention=1,speed_limit_mps=.2)]
    assert ops.start is None and ops.rearm is None and ops.calls==[]
