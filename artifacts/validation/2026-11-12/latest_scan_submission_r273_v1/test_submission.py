import copy
import pytest
from submission import eligible


def snapshot():
    return dict(reference_s=1.16,epoch=0,scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=7,acquired_at_s=[1.001,1.101])))


def options(policy='latest'):
    return dict(now_s=1.17,last_reference_s=1.02,last_source=None,policy=policy)


def test_latest_can_use_a_still_valid_scan_after_the_fresh_start_window():
    data=snapshot();before=copy.deepcopy(data)
    assert eligible(data,**options())
    assert not eligible(data,**options('fresh'))
    assert data==before


@pytest.mark.parametrize('field,value',[('now_s',1.302),('now_s',1.37),('last_reference_s',1.10)])
def test_existing_source_reference_and_minimum_plan_interval_are_preserved(field,value):
    args=options();args[field]=value
    assert not eligible(snapshot(),**args)


def test_already_used_scan_and_future_ray_cannot_be_submitted():
    args=options();args['last_source']=(7,1.001)
    assert not eligible(snapshot(),**args)
    data=snapshot();data['scan']['measurements']['acquired_at_s'][-1]=1.18
    assert not eligible(data,**options())


def test_uninitialized_interval_is_allowed_without_rewriting_first_scan():
    args=options();args['last_reference_s']=-float('inf')
    assert eligible(snapshot(),**args)


def test_bad_timestamp_or_unregistered_policy_is_rejected():
    data=snapshot();data['scan']['measurements']['acquired_at_s'][-1]=float('nan')
    with pytest.raises(ValueError):eligible(data,**options())
    with pytest.raises(ValueError):eligible(snapshot(),**options('always_drive'))
