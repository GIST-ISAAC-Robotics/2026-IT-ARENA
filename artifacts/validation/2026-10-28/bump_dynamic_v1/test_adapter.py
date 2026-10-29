import importlib.util
import math
from pathlib import Path
import pytest

BASE=Path(__file__).resolve().parent
def load(name):
    spec=importlib.util.spec_from_file_location('bump_'+name,BASE/(name+'.py'))
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module
runner=load('run'); analyzer=load('analyze')

def test_patch_is_bounded_and_keeps_stop():
    source=(runner.ROOT/'scripts/validate_basic_autonomy.py').read_text(encoding='utf-8')
    result=runner.patch_evaluator(source)
    assert 'or not args.disable_speed_bump' not in result
    assert '"stop_stable"' in result and 'dense_pose_stop_evidence' in result
    assert 'pose_rejected_times == 0' in result
    assert 'message.linear_acceleration' in result
    with pytest.raises(ValueError): runner.patch_evaluator(result)

def test_launch_exposes_actual_speed_and_bump():
    launch,settings,args=runner.build_launch(BASE/'not_run')
    assert args['speed_profile']=='cautious' and args['speed_bump']=='true'
    assert "'speed_profile': 'cautious'" in launch and "'speed_bump': 'true'" in launch
    assert 'signal_driver_v2.py' in launch
    assert settings['g1_follow_enabled'] and settings['partial_observation_policy']=='bounded_local_v4'

def test_brisk_is_explicit_and_limited():
    launch, _, args=runner.build_launch(BASE/'not_run','brisk')
    assert args['speed_profile']=='brisk' and "'speed_profile': 'brisk'" in launch
    with pytest.raises(ValueError):runner.build_launch(BASE/'not_run','hardware_target')

def poses(lateral=0,start=-.3,end=.3):
    return [dict(time=i*.01,position=dict(x=start+(end-start)*i/100,y=lateral,z=0),
                 orientation=dict(x=0,y=0,z=0,w=1)) for i in range(101)]
bump=dict(x=0,y=0,yaw_rad=0)

def test_crossing_requires_all_wheels():
    assert analyzer.pose_metrics(poses(),bump)['all_wheels_passed']
    assert not analyzer.pose_metrics(poses(end=.04),bump)['all_wheels_passed']
    assert not analyzer.pose_metrics(poses(start=.1),bump)['all_wheels_passed']

def test_going_around_is_not_crossing():
    assert not analyzer.pose_metrics(poses(lateral=.22),bump)['all_wheels_passed']

def test_rotated_course_same_result():
    rows=poses()
    for row in rows:
        row['position']['y']=row['position']['x'];row['position']['x']=0
        row['orientation']=dict(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4))
    assert analyzer.pose_metrics(rows,dict(x=0,y=0,yaw_rad=math.pi/2))['all_wheels_passed']

def test_invalid_pose_rejected():
    with pytest.raises(ValueError): analyzer.rotation(dict(w=0))
