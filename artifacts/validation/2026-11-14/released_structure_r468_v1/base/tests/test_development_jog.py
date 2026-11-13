import json
from pathlib import Path
import subprocess
import sys
import pytest
from development_jog import plan, valid_mcu


def status():
    return dict(v=1,boot='fixture',generation=4,owner='host',state='ARMED',now_us=10_000_000,
        estop_held=False,feedback_valid=True,output_ready=True,target_speed_mps=0.)


@pytest.mark.parametrize('direction,steer',[('forward',0.),('left',.2),('right',-.2)])
def test_development_pulse_has_bounded_speed_and_direction(direction,steer):
    p=plan(direction,.15,.5,.2)
    assert p['speed_mps']==.15 and p['steering_rad']==steer and p['output_topic']=='/drive'
    assert not p['automatic_arm']


@pytest.mark.parametrize('values',[
    ('reverse',.1,.5,.3),('forward',-.1,.5,.3),('left',.4,.5,.3),
    ('right',.1,3.,.3),('forward',float('nan'),.5,.3),('forward',.1,.5,2.),
    ('forward',True,.5,.3)])
def test_outside_development_request_is_rejected(values):
    with pytest.raises(ValueError):plan(*values)


def test_start_requires_explicit_arm_and_no_existing_target():
    s=status();assert valid_mcu(s,10.1,20.,20.1)
    for k,v in [('state','ACTIVE'),('state','ESTOP'),('target_speed_mps',.1),
                ('estop_held',True),('feedback_valid',False),('output_ready',False),('owner',None)]:
        assert not valid_mcu({**s,k:v},10.1,20.,20.1)


def test_active_jog_cannot_survive_epoch_stop_or_old_status():
    s=status();identity=('fixture',4,'host')
    assert valid_mcu({**s,'state':'ACTIVE','target_speed_mps':.15},10.1,20.,20.1,identity=identity)
    for k,v in [('state','ESTOP'),('boot','new'),('generation',5),('owner','other'),('now_us',9_000_000)]:
        assert not valid_mcu({**s,k:v},10.1,20.,20.1,identity=identity)
    assert not valid_mcu(s,10.1,20.,21.,identity=identity)


def test_preview_without_send_never_requires_ros_or_creates_output(tmp_path):
    script=Path(__file__).resolve().parents[1]/'scripts/development_jog.py'
    output=tmp_path/'unused'
    p=subprocess.run([sys.executable,str(script),'left','--output',str(output)],capture_output=True,text=True)
    assert p.returncode==0 and json.loads(p.stdout)['steering_rad']==.2
    assert not output.exists()
