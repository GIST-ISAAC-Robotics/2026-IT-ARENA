import pytest
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionHistory
from arena_autonomy.stopped_entry_centering import stopped_shaft_window


def samples(start=0,end=22,missing=(),pulse=None):
    h=MotionHistory()
    for k in range(start,end+1):
        if k not in missing:h.wheels.add(k*.02, pulse[1] if pulse and k==pulse[0] else 0.)
    return h


def test_fresh_quiet_counts_cover_whole_window_without_future_samples():
    assert stopped_shaft_window(samples(pulse=(21,-.0813)),.40)
    assert not stopped_shaft_window(samples(pulse=(21,-.0813)),.42)


@pytest.mark.parametrize('h,now',[(samples(start=12),.40),(samples(end=17),.40),
    (samples(missing=(13,14,15)),.40),(samples(pulse=(13,.0813)),.40),
    (samples(pulse=(19,-.0813)),.40),(samples(),float('nan'))])
def test_sparse_stale_reverse_or_nonzero_counts_cannot_request_alignment(h,now):
    assert not stopped_shaft_window(h,now)


def test_legacy_execution_does_not_enable_trial():
    assert not HandoffExecutionConfig().stopped_entry_centering
    with pytest.raises(ValueError):HandoffExecutionConfig(stopped_entry_centering=True)
