import copy
import pytest
from analyze_laps import visits,sensor_window
from test_adapter import poses,bump,analyzer


def test_two_passes_are_not_one_long_observation_window():
    first=poses(start=-.44,end=.44);second=copy.deepcopy(first)
    for p in second:p['time']+=80
    groups=visits(first+second,bump)
    assert len(groups)==2
    assert all(analyzer.pose_metrics(g,bump)['all_wheels_passed'] for g in groups)


def test_far_parallel_road_is_not_a_bump_visit():
    assert not visits(poses(lateral=2.),bump)


def test_missing_passage_evidence_does_not_join_before_and_after():
    rows=poses(start=-.44,end=-.2)+poses(start=.2,end=.44)
    for r in rows[101:]:r['time']+=3.
    groups=visits(rows,bump)
    assert len(groups)==2
    assert not any(analyzer.pose_metrics(g,bump)['all_wheels_passed'] for g in groups)


def test_reverse_crossing_is_not_forward_completion():
    groups=visits(poses(start=.44,end=-.44),bump)
    assert len(groups)==1 and not analyzer.pose_metrics(groups[0],bump)['all_wheels_passed']


def test_time_reversal_is_rejected():
    rows=poses();rows[-1]['time']=0.
    with pytest.raises(ValueError):visits(rows,bump)
