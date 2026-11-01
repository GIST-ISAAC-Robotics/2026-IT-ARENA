"""물체 이동 가정 하한 후보의 회귀 시험을 기존 시험 파일에 덧붙인다."""
from pathlib import Path

path = Path(__file__).resolve().parents[4] / 'tests/test_low_motion_reserve.py'
text = path.read_bytes().decode('utf-8')
old = """def control(speeds,enabled=True):
    ctrl=follow_space()
    ctrl.execution=replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True,
                           low_motion_reserve=enabled)
"""
new = """def control(speeds,enabled=True,floor=False):
    ctrl=follow_space()
    ctrl.execution=replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True,
                           low_motion_reserve=enabled,moving_pace_floor=floor)
"""
assert text.count(old) == 1
text = text.replace(old, new)
text += """

def test_pace_floor_is_off_by_default_and_requires_low_motion_reserve():
    assert HandoffExecutionConfig().moving_pace_floor is False
    with pytest.raises(ValueError,match='pace floor'):
        replace(follow_space().execution,moving_pace_floor=True)


@pytest.mark.parametrize('extra,request_mps,expected',[
    (.3,1.4,.875),  # 0.5 + (0.30 m / 0.8 s)
    (-.1,1.4,.5),   # 여유 안쪽이어도 0이 아니라 가정 속도까지
    (.3,.4,.4),     # G1 상한 준수
    (.3,0.,0.),     # 정상 HOLD를 전진으로 바꾸지 않는다
])
def test_pace_floor_limits_fast_patch_without_zero_command(extra,request_mps,expected):
    speed,detail=limited(control([.85]*12,floor=True),extra,request_mps)
    assert speed==pytest.approx(expected)
    assert detail['reason']=='reserve_waits_low_motion' and detail['free_space_assumed'] is False


def test_pace_floor_is_never_below_the_old_zero_pace_reserve():
    for extra in (-.3,-.1,0.,.1,.3):
        old=limited(control([.85]*12,False),extra,1.4)[0]
        assert limited(control([.85]*12,floor=True),extra,1.4)[0]>=old


def test_pace_floor_keeps_blocking_patch_rules():
    speed,detail=limited(control([0.]*6,floor=True),-.1,1.2)
    assert speed==0. and detail['reason']=='observed_entry_distance_reserve'
    assert limited(control([.25]*12,floor=True),.16,.9)[1]['reason']=='advancing_patch_close'


def test_pace_floor_must_be_present_in_actual_trace(tmp_path):
    path=tmp_path/'header.jsonl'
    ex=dict(moving_follow_reserve=True,closing_follow_reserve=True,low_motion_reserve=True,moving_pace_floor=True)
    path.write_text(json.dumps(dict(kind='header',data=dict(follow_config={},execution_config=ex))))
    assert handoff.inspect_header(path,'paced')['checks']['pace_floor'] is True
    assert handoff.inspect_header(path,'low_motion')['checks']['pace_floor'] is False
"""
path.write_bytes(text.encode('utf-8'))
