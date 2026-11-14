import importlib.util
import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from arena_autonomy.follow_trace import FollowTrace, controller_mode, replay_events
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.p22_trial import P22Entry, entry_command
from arena_autonomy.p22_path import turn
from arena_autonomy.p22_entry_trial_node import P22EntryNode
from arena_autonomy.local_pursuit import LocalPursuit

ROOT = Path(__file__).resolve().parents[1]


class Baseline:
    config = FollowConfig()
    def reset(self): pass
    def command(self, *args, **kwargs): return .7, .02, {'reason': 'baseline'}


def scene(t=.2, epoch=0):
    n=500
    return scene_from_scan([4.]*n, angle_min=-math.pi, angle_increment=2*math.pi/n,
        range_min=.05,range_max=12., acquired_at_s=np.linspace(t-.1,t-.001,n).tolist(),
        t_ref_s=t, source_epoch=epoch, rear_half_angle_deg=0.)


def history():
    h=MotionHistory(MotionConfig())
    for i in range(61):
        h.add_mean_wheel(i*.01,.5/h.config.wheel_radius_m)
        h.add_gyro(i*.01,0.)
    return h


def active():
    f=P22Entry(Baseline())
    f.first_scan=.1;f.intent.reset(.1,0)
    f.path=np.column_stack((np.linspace(0.,1.5,151),np.zeros(151)))
    f.path_t=f.start_t=.2
    return f


def test_absent_marker_preserves_normal_command():
    f=P22Entry(Baseline())
    for t in (.2,.22,.3):
        out=f.command(scene(t),history(),.5,'left')
        assert out[:2]==(.7,.02)
        assert f.path is None


def test_current_obstacle_rejects_positive_command():
    s=scene();p=np.column_stack((np.linspace(0,1.5,151),np.zeros(151)))
    output=entry_command(s,p,.5,0.,.02,FollowConfig())
    assert output[0]>.0
    rays=list(s.rays);rays[250]=replace(rays[250],hit_xy_m=(.13,0.))
    obstacle=replace(s,rays=tuple(rays))
    assert entry_command(obstacle,p,.5,0.,.02,FollowConfig())[0]==0


def test_active_path_uses_measured_motion_without_running_baseline():
    f=active();old=f.path.copy()
    with patch.object(f.baseline,'command',side_effect=AssertionError('baseline not active')):
        f.command(scene(.3),history(),.5,'left')
    assert np.allclose(f.path,old-[.05,0.],atol=1e-12)
    assert f.path_t==.3


def test_active_automatic_reset_requires_explicit_reset():
    f=active();f.reset()
    assert f.command(scene(.3),history(),.5,'left')[0]==0
    assert f.latched_fault=='active_input_reset'
    f.reset(force=True)
    assert f.command(scene(.3),history(),.5,'left')[:2]==(.7,.02)


def test_active_source_epoch_change_latches():
    f=active()
    assert f.command(scene(.3,1),history(),.5,'left')[0]==0
    assert f.latched_fault=='epoch_reset'
    f.reset()
    assert f.command(scene(.4,1),history(),.5,'left')[0]==0
    assert f.latched_fault=='epoch_reset'


def test_trial_trace_is_not_silently_replayed_as_baseline(tmp_path):
    f=active();assert controller_mode(f)=='p22_entry_trial'
    p=tmp_path/'decision.jsonl'
    trace=FollowTrace(p,MotionConfig(),f.config,mode=controller_mode(f))
    trace.emit('p22_frame',dict(ids=[20],stamp=1.,now=1.1,epoch=0))
    trace.close()
    events=[json.loads(v) for v in p.read_text().splitlines()]
    assert events[0]['data']['schema']==6
    with pytest.raises(ValueError,match='unsupported trace header'):
        replay_events(events)


def test_node_records_acquisition_timestamp_and_epoch():
    frames=[];events=[]
    n=P22EntryNode.__new__(P22EntryNode)
    n.vision_pipeline=SimpleNamespace(last_result={'stamp':2.0},ids=[20],epoch=4,fault=False)
    n.now_s=lambda:2.2
    n.follow=SimpleNamespace(frame=lambda **kw:frames.append(kw))
    n.follow_trace=SimpleNamespace(emit=lambda k,v:events.append((k,v)))
    n.p22_recorded_frame=None
    with patch.object(LocalPursuit,'command',return_value=(.65,0.,{'p22_trial':{'path_points':[[0,0]],'state':'active'}})):
        out=n.command(None);n.command(None)
    assert frames[0]==dict(ids=[20],stamp=2.,now=2.2,epoch=4)
    assert len(events)==1 and events[0][0]=='p22_frame'
    assert 'path_points' not in out[2]['p22_trial']


def test_explicit_launch_preserves_default_and_rejects_competitors():
    spec=importlib.util.spec_from_file_location('p22_launch',ROOT/'src/arena_bringup/launch/simulation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    kwargs=dict(guarded=True,actor='',evidence='/tmp/evidence',track='official_v2026_11_06')
    assert module._autonomy_executable('local_pursuit','off',**kwargs)=='local_pursuit'
    assert module._autonomy_executable('local_pursuit','off',**kwargs,p22_entry_trial=True)=='p22_entry_trial'
    for key,value in [('guarded',False),('actor','some_actor'),('evidence',''),('track','official')]:
        with pytest.raises(ValueError):module._autonomy_executable('local_pursuit','off',**(kwargs|{key:value}),p22_entry_trial=True)


def test_empty_scene_cannot_generate_entry():
    s=replace(scene(),rays=(),source_first_s=None,source_last_s=None)
    with pytest.raises(MotionUnavailable,match='empty_scan'):
        entry_command(s,np.zeros((3,2)),.5,0.,.02,FollowConfig())


def test_tail_leaves_whole_vehicle_inside_after_pursuit_horizon_ends():
    hit=dict(e1=(.7,-.5),e2=(1.3,-.5),a_heading_deg=0.)
    p=turn(hit,0.,.36,.65,final_depth_m=.55)
    x,y,yaw,steer,distance=p['poses'][-1]
    # 후륜 목표→차체 중심, 추종 종료 전방 .25 m를 빼도 차체 뒤끝이 입구 안이다.
    center_y=y-.135/2+.25
    assert center_y+.20/2 < -.5
    assert abs(yaw+math.pi/2)<1e-8
