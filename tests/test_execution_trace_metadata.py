"""실행 설정 이벤트는 보존하되 계산 재생의 명령/시간을 바꾸지 않는다."""
import copy
import pytest
from arena_autonomy.follow_trace import replay_events, equal_output
from profile_follow_trace import profile
from test_follow_profile import make_handoff_trace
from test_follow_trace import make_trace


def attach(events):
    out=copy.deepcopy(events)
    extra=[dict(kind=kind,data=data) for kind,data in (
        ('decision_submission_policy',dict(policy='fresh_native_v1',completion_window_s=.025)),
        ('current_path_recheck_policy',dict(policy='selected_v1',independent_guard_required=True)),
        ('delivery_delay_profile',dict(profile_sha256='a'*64,samples=598,delay_s=99999.)),
        ('expiry_reference_policy',dict(policy='selected_wall_recent_v3',command_retained=False)))]
    out[1:1]=extra
    for index,row in enumerate(out):row['seq']=index
    out[-1]['data']['written']+=len(extra);out[-1]['data']['accepted']+=len(extra)
    return out


def test_known_metadata_is_preserved_without_affecting_outputs_or_input(tmp_path):
    original=make_handoff_trace(tmp_path)
    events=attach(original);before=copy.deepcopy(events)
    expected=replay_events(original);actual=replay_events(events)
    assert actual['passed'] and actual['matched']==expected['matched']
    assert equal_output([row['details'] for row in expected['outcomes']],
                        [row['details'] for row in actual['outcomes']])
    assert actual['audit_metadata']==events[1:5]
    assert actual['execution_metadata_replayed'] is False and events==before
    measured=profile(events,repeats=1,warmups=0)
    assert measured['passed'] and measured['timed_calls']==expected['calls']
    assert measured['recorded_execution_metadata']==events[1:5]
    assert measured['execution_metadata_replayed'] is False
    assert 'recorded_execution_delay' in measured['timing_excludes']


@pytest.mark.parametrize('fault',['unknown','non_dict','missing_event','footer_count','changed_output'])
def test_metadata_support_does_not_hide_capture_or_output_failures(tmp_path,fault):
    events=attach(make_handoff_trace(tmp_path))
    if fault=='unknown':events[1]['kind']='unregistered_control_event'
    elif fault=='non_dict':events[1]['data']=[]
    elif fault=='missing_event':del events[2]
    elif fault=='footer_count':events[-1]['data']['written']-=1
    else:
        next(row for row in events if row['kind']=='call')['data']['output'][0]+=.1
        assert replay_events(events)['passed'] is False
        with pytest.raises(ValueError,match='outputs'):profile(events,repeats=1,warmups=0)
        return
    with pytest.raises(ValueError):replay_events(events)


def test_execution_metadata_is_not_silently_adopted_by_older_trace_scope(tmp_path):
    _,events=make_trace(tmp_path)
    with pytest.raises(ValueError,match='execution audit metadata'):
        replay_events(attach(events))
