import copy
import math
import pytest
from sample_clock import C1SampleClock

def sample(i,*,extra_receipt=0):
    first=10_000_000_000+i*500*196_000
    batch=dict(schema='c1_sdk_native_preview_v1',scan=i,begin_monotonic_ns=first,
        end_monotonic_ns=first+98_000_000,sdk_estimated_first_us=first//1000,us_per_sample=200.,
        nodes=[[j*128,4000,47,1 if j==0 else 0] for j in range(500)])
    receipt=first+99_000_000+extra_receipt
    return batch,dict(receipt_monotonic_ns=receipt,receipt_ros_ns=receipt+1_700_000_000_000_000_000)

def ready():
    clock=C1SampleClock(source_epoch=4)
    for i in range(20):
        b,r=sample(i);p,_,_=clock.convert(b,**r);assert p is None
    return clock

def test_causal_prefix_keeps_values_order_and_unverified_basis():
    c=ready();b,r=sample(20);before=copy.deepcopy(b)
    p,d,due=c.convert(b,**r)
    assert b==before and p['time_basis']=='unverified_estimate' and d['time_contract_verified'] is False
    assert p['ranges_m']==[1.]*500 and len(p['angles_rad'])==500 and p['source_frame_id']=='c1_unmounted_preview'
    assert d['sample_period_ns']==196_000 and due==r['receipt_monotonic_ns']
    assert all(y>x for x,y in zip(p['acquired_at_s'],p['acquired_at_s'][1:]))

def test_receiver_stall_does_not_refresh_source_time():
    a,b=ready(),ready();data,r=sample(20);normal,_,_=a.convert(data,**r)
    data,late=sample(20,extra_receipt=350_000_000);delayed,_,_=b.convert(data,**late)
    assert normal==delayed

@pytest.mark.parametrize('fault',['gap','restart','clock_step','decode_delay','future'])
def test_invalid_time_starts_a_new_clock_epoch(fault):
    c=ready();data,r=sample(20)
    if fault=='gap':data['scan']+=1
    elif fault=='restart':data,r=sample(0)
    elif fault=='clock_step':r['receipt_ros_ns']+=200_000_000
    elif fault=='decode_delay':
        data['sdk_estimated_first_us']+=20_000
        data['end_monotonic_ns']+=20_000_000;r['receipt_monotonic_ns']+=20_000_000;r['receipt_ros_ns']+=20_000_000
    elif fault=='future':
        data['end_monotonic_ns']-=10_000_000;r['receipt_monotonic_ns']-=10_000_000;r['receipt_ros_ns']-=10_000_000
    p,d,_=c.convert(data,**r)
    assert p is None and c.epoch==5 and c.counts['candidate_batches']==0
    assert d['time_contract_verified'] is False

def test_small_future_is_delayed_without_clipping():
    c=ready();data,r=sample(20)
    data['end_monotonic_ns']-=2_000_000;r['receipt_monotonic_ns']-=2_000_000;r['receipt_ros_ns']-=2_000_000
    p,d,due=c.convert(data,**r)
    assert p is not None and due>r['receipt_monotonic_ns'] and d['publication_wait_ns']==804_000
    assert d['timestamps_clipped'] is False

def test_no_cross_batch_overlap():
    c=ready();previous=None
    for i in range(20,60):
        data,r=sample(i);p,_,_=c.convert(data,**r)
        assert p is not None
        if previous is not None:assert p['acquired_at_s'][0]>previous
        previous=p['acquired_at_s'][-1]
