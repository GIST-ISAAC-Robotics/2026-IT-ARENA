from unittest.mock import patch
import json
import numpy as np
import pytest

from arena_autonomy.p22_route_trial import P22Route
from arena_autonomy.follow_trace import FollowTrace, controller_mode, replay_events
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.p22_entry_trial_node import P22EntryNode
from types import SimpleNamespace
from test_p22_trial import Baseline, scene, history


def active():
    f=P22Route(Baseline())
    f.first_scan=.1;f.intent.reset(.1,0)
    f.path=np.column_stack((np.linspace(0,1.5,151),np.zeros(151)))
    f.path_t=f.start_t=f.phase_t=.2
    f.phase='entry'
    return f


def candidate(x=1.5):
    return dict(poses=[(x-.01*i,0.,0.,0.,0.) for i in (2,1,0)])


def test_two_distinct_transported_exit_scans_required():
    f=active()
    with patch('arena_autonomy.p22_route_trial.propose',return_value=[candidate()]):
        assert f._exit_candidate(scene(.2),history(),0.) is None
        assert f._exit_candidate(scene(.2),history(),0.) is None
    with patch('arena_autonomy.p22_route_trial.propose',return_value=[candidate(1.45)]):
        assert f._exit_candidate(scene(.3),history(),0.) is not None


def test_disagreeing_exit_or_remaining_entry_turn_cannot_replace_path():
    f=active()
    with patch('arena_autonomy.p22_route_trial.propose',return_value=[candidate()]):
        assert f._exit_candidate(scene(.2),history(),.18) is None
        assert f.exit_seen is None
        assert f._exit_candidate(scene(.3),history(),0.) is None
    with patch('arena_autonomy.p22_route_trial.propose',return_value=[candidate(1.)]):
        assert f._exit_candidate(scene(.4),history(),0.) is None


def test_obstacle_blocks_route_replacement():
    f=active();original=f.path.copy()
    with patch.object(f,'_exit_candidate',return_value=np.zeros((5,2))),patch(
        'arena_autonomy.p22_route_trial.entry_command',return_value=(0.,0.,{'reason':'p22_occupied','path_remaining_m':1.})):
        out=f.command(scene(.2),history(),.5,'left')
    assert out[0]==0 and f.phase=='entry' and np.array_equal(f.path,original)


def test_main_handoff_requires_two_current_positive_scans():
    f=active();f.phase='exit'
    out=(.65,0.,{'reason':'p22_candidate','path_remaining_m':.4})
    with patch('arena_autonomy.p22_route_trial.entry_command',return_value=out):
        f.command(scene(.2),history(),.5,'left')
        f.command(scene(.2),history(),.5,'left')
        assert f.phase=='exit'
        f.command(scene(.3),history(),.5,'left')
    assert f.phase=='main' and f.merged_at==.3
    f.command(scene(.4),history(),.5,'left')
    assert f.main_progress==pytest.approx(.05)


def test_faulted_route_never_implicitly_resumes_main():
    f=active();f.phase='exit';f.reset()
    assert f.phase=='exit' and f.command(scene(.3),history(),.5,'left')[0]==0
    f.reset(force=True)
    assert f.phase is None and f.command(scene(.3),history(),.5,'left')[0]==.7


def test_route_schema_does_not_claim_entry_only_replay(tmp_path):
    f=active();p=tmp_path/'trace.jsonl'
    assert controller_mode(f)=='p22_route_trial'
    t=FollowTrace(p,MotionConfig(),f.config,mode=controller_mode(f));t.close()
    events=[json.loads(v) for v in p.read_text().splitlines()]
    assert events[0]['data']['schema']==7
    with pytest.raises(ValueError,match='unsupported trace header'):replay_events(events)


def test_node_records_forced_reset_and_refreshes_sparse_frame_key():
    n=P22EntryNode.__new__(P22EntryNode);n.follow=active();events=[]
    n.follow_trace=SimpleNamespace(emit=lambda k,d:events.append((k,d)))
    n.p22_recorded_frame=(0,.2,(20,))
    n.reset_follow('follow_input_stop')
    assert n.p22_recorded_frame is None and n.follow.latched_fault=='active_input_reset'
    assert events[-1]==('reset',{'reason':'follow_input_stop','force':False})
    n.reset_follow('legacy_reset')
    assert n.follow.path is None and n.follow.latched_fault is None
    assert events[-1]==('reset',{'reason':'legacy_reset','force':True})
