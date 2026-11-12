from types import SimpleNamespace
from unittest.mock import patch
import pytest
from scan_phase import CurrentScanProcess,CurrentPriorityProcess


def worker():
    w=CurrentScanProcess.__new__(CurrentScanProcess)
    w.closed=False;w.fault=None;w.current=SimpleNamespace(inflight=None)
    w.ready=None;w.last_checked_source=None
    return w


def snapshot():
    return dict(reference_s=10.02,scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=3,acquired_at_s=[9.9,10.])))


def test_new_scan_after_receipt_uses_same_current_checks():
    w=worker();s=snapshot()
    with patch.object(CurrentPriorityProcess,'offer_next_current',return_value=True) as parent:
        assert w.offer_next_current(s,now_s=10.021)
        parent.assert_called_once_with(s)


@pytest.mark.parametrize('fault',['old','duplicate','busy','ready'])
def test_no_stale_or_duplicate_check_started(fault):
    w=worker();s=snapshot();now=10.021
    if fault=='old':now=10.026
    elif fault=='duplicate':w.last_checked_source=(0,1,(3,9.9))
    elif fault=='busy':w.current.inflight={'owned':True}
    elif fault=='ready':w.ready={'owned':True}
    with patch.object(CurrentPriorityProcess,'offer_next_current') as parent:
        assert not w.offer_next_current(s,now_s=now)
        parent.assert_not_called()


def test_failed_submission_does_not_consume_sensor_identity():
    w=worker();s=snapshot()
    with patch.object(CurrentPriorityProcess,'offer_next_current',return_value=False):
        assert not w.offer_next_current(s,now_s=10.021)
        assert w.last_checked_source is None
