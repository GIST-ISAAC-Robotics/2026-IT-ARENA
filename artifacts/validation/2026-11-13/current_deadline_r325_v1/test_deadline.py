from unittest.mock import patch
import pytest
from deadline import CurrentDeadlineProcess,CurrentPhaseProcess


@pytest.mark.parametrize('ref,first,now,allowed',[
    (10.02,9.9,10.025,True),
    (10.02,9.82,10.025,False),
    (10.02,9.9,10.13,False),
    (10.02,9.9,10.01,False)])
def test_scheduling_check_preserves_input_and_parent_gate(ref,first,now,allowed):
    w=CurrentDeadlineProcess.__new__(CurrentDeadlineProcess)
    data=dict(reference_s=ref,scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=0,acquired_at_s=[first,first+.1])))
    with patch.object(CurrentPhaseProcess,'offer_next_current',return_value=True) as parent:
        assert w.offer_next_current(data,now_s=now) is allowed
        assert parent.called is allowed
        assert data['reference_s']==ref
