"""등록 지연 실행의 프로세스 선택과 원 명령 보존 검사."""
import json
import types
from launch_prefix import instrument_start
from node_entry import read_profile


def test_only_original_controller_launch_is_wrapped(tmp_path):
    calls=[]
    fake=types.SimpleNamespace(Popen=lambda *a,**k:calls.append((a,k)))
    scope={'subprocess':fake}
    exec('def start():\n return subprocess.Popen(["ros2","launch","--noninteractive","arena_bringup","g1_follow.launch.py","headless:=true"],start_new_session=True)\n',scope)
    original=scope['start'];profile=tmp_path/'profile.json';evidence=tmp_path/'command.json'
    wrapped=instrument_start(original,profile,evidence)
    wrapped()
    command=calls[0][0][0]
    assert command[:3]==['ros2','launch','--launch-prefix']
    assert command[4:7]==['--launch-prefix-filter','.*g1_g2_handoff_trial.*','--noninteractive']
    assert command[-3:]==['arena_bringup','g1_follow.launch.py','headless:=true']
    assert calls[0][1]=={'start_new_session':True}
    assert json.loads(evidence.read_text())['command']==command
    original()
    assert '--launch-prefix' not in calls[1][0][0]


def test_measured_profile_keeps_sample_order_and_units(tmp_path):
    path=tmp_path/'profile.json'
    record=dict(schema='r106_measured_delivery_v1',scope='test',samples=[
        dict(offset_s=.12,delay_s=.18),dict(offset_s=.22,delay_s=.07)])
    path.write_text(json.dumps(record))
    loaded,timeline=read_profile(path)
    assert loaded==record and timeline==[(.12,.18),(.22,.07)]
