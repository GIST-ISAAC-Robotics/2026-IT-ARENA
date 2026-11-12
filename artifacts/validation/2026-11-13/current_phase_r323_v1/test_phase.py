import math
from unittest.mock import patch
import pytest
from phase import CurrentPhaseProcess,CurrentPriorityProcess


def worker():
    w=CurrentPhaseProcess.__new__(CurrentPhaseProcess)
    w.last_application=10.;w.saved_plan=None;w.retained=None;w.geometry_age_limit=.5
    return w


@pytest.mark.parametrize('ref',[9.99,10.])
def test_pre_application_input_is_not_submitted(ref):
    w=worker()
    with patch.object(CurrentPriorityProcess,'offer_next_current') as parent:
        assert not w.offer_next_current({'reference_s':ref})
        parent.assert_not_called()


def test_post_application_input_keeps_original_submit_checks():
    w=worker();snapshot={'reference_s':10.02}
    with patch.object(CurrentPriorityProcess,'offer_next_current',return_value=False) as parent:
        assert not w.offer_next_current(snapshot)
        parent.assert_called_once_with(snapshot)


@pytest.mark.parametrize('source,expected',[(9.60,False),(9.75,True)])
def test_only_retained_scheduling_is_bounded(source,expected):
    w=worker();w.retained={'output':[1.,0.,{'normal':{'path_source_s':source}}]}
    with patch.object(CurrentPriorityProcess,'offer_next_current',return_value=True) as parent:
        assert w.offer_next_current({'reference_s':10.02}) is expected
        assert parent.called is expected
    w.saved_plan={'fresh_plan':True}
    with patch.object(CurrentPriorityProcess,'offer_next_current',return_value=True) as parent:
        assert w.offer_next_current({'reference_s':10.02})
        parent.assert_called_once()


@pytest.mark.parametrize('ref',[math.nan,math.inf])
def test_nonfinite_reference_not_a_new_tick(ref):
    with pytest.raises(ValueError):worker().offer_next_current({'reference_s':ref})
