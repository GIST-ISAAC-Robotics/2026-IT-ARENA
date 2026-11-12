import copy
import pytest
from timeline import SensorTimeline


def events():
    def snapshot(t,stamps,values):
        return dict(reference_s=t,epoch=0,speed_mps=99.,previous_steering_rad=0.,
            scan=dict(representation='native_measurements_v1',measurements=dict(
                source_epoch=1,acquired_at_s=[t-.10,t-.02],ranges_m=[1.,2.],angles_rad=[0.,1.])),
            history={k:dict(times=stamps[:],values=values[:]) for k in ('wheels','gyro')})
    return [dict(kind='header',data=dict(motion_config=dict(history_s=2.))),
            dict(kind='call',data=dict(input=snapshot(10.,[9.98,10.],[1.,2.]))),
            dict(kind='call',data=dict(input=snapshot(10.1,[9.98,10.,10.02,10.04,10.06,10.08],[1.,2.,3.,4.,5.,6.])))]


def test_observed_never_imports_a_sample_from_later_received_snapshot():
    data=events();before=copy.deepcopy(data);timeline=SensorTimeline(data)
    current=timeline.current(10.06,mode='observed')
    assert current['reference_s']==pytest.approx(10.06)
    assert current['history']['gyro']['times']==[9.98,10.]
    assert current['scan']==data[1]['data']['input']['scan'] and data==before
    assert current['speed_mps']==2.


def test_delay_assumption_preserves_values_and_rejects_future_capture():
    timeline=SensorTimeline(events());current=timeline.current(10.06,mode='delay20')
    assert current['history']['gyro']==dict(times=[9.98,10.,10.02,10.04],values=[1.,2.,3.,4.])
    assert current['speed_mps']==4.
    assert current['scan']==events()[1]['data']['input']['scan']


def test_held_reference_is_not_refreshed_and_cached_result_is_owned():
    timeline=SensorTimeline(events());current=timeline.current(10.06,mode='held')
    assert current['reference_s']==10.
    current['scan']['measurements']['ranges_m'][0]=999
    assert timeline.current(10.06,mode='held')['scan']['measurements']['ranges_m'][0]==1.


def test_duplicate_sample_values_must_match():
    data=events();data[2]['data']['input']['history']['gyro']['values'][0]=99.
    with pytest.raises(ValueError,match='changed its value'):SensorTimeline(data)


def test_epoch_change_is_not_silently_merged():
    data=events();data[2]['data']['input']['epoch']=1
    with pytest.raises(ValueError,match='one motion epoch'):SensorTimeline(data)


def test_earlier_query_and_unknown_mode_are_rejected():
    timeline=SensorTimeline(events())
    with pytest.raises(ValueError):timeline.current(9.,mode='observed')
    with pytest.raises(ValueError):timeline.current(10.,mode='instant_true_pose')
