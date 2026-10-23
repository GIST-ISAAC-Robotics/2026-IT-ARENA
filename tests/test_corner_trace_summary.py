import copy

import pytest

from summarize_corner_trace import summarize
from test_follow_trace import make_trace


def test_recorded_summary_never_claims_replay_or_driving_success(tmp_path):
    _,events=make_trace(tmp_path)
    original=copy.deepcopy(events)
    result=summarize(events)
    assert result['calls']==7 and result['mode']=='connected'
    assert not result['driving_authorized'] and result['selected_count']==0
    assert events==original
    assert result['reason_counts']['follow_input_stop']==1


def test_distance_mode_summary_reads_its_own_contract_and_retained_path(tmp_path):
    _,events=make_trace(tmp_path)
    events[0]['data']['follow_config']['boundary_evidence_mode']='corner_distance_candidate'
    selected=next(e for e in events if e['kind']=='call')
    detail=selected['data']['output'][2]
    detail.update(boundary_selection='bilateral_observed_corner_distance',
        corner_distance_evidence=dict(reason='observed_corner_distance_support',supported=True,
                                      supported_prefix_m=.7,path_end_exclusive=25),
        corner_distance_agrees_memory=True,path_points=[[0.,0.]]*25)
    result=summarize(events)
    assert result['selected_count']==1
    row=result['selected'][0]
    assert row['corner_supported'] is True and row['memory_agrees'] is True
    assert row['supported_prefix_m']==.7 and row['path_points_count']==25


@pytest.mark.parametrize('error',['missing_footer','gap','trailing','dropped'])
def test_incomplete_summary_is_not_valid_capture(tmp_path,error):
    _,events=make_trace(tmp_path)
    if error=='missing_footer':events=events[:-1]
    elif error=='gap':del events[2]
    elif error=='trailing':events.append({'seq':len(events),'kind':'reset','data':{}})
    elif error=='dropped':events[-1]['data']['dropped']=1
    with pytest.raises(ValueError):summarize(events)
