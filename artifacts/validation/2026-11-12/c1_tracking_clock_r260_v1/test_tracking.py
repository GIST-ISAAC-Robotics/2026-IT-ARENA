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


def test_tracking_refits_without_rewriting_published_timestamps():
    c=C1SampleClock(source_epoch=4)
    first=10_000_000_000
    held=[]
    refits=0
    errors=[]
    for i in range(600):
        # 약 1분의 느린 표본 주기 변화와 호스트 복호 지터. 절대 정확도 실측이 아니다.
        period=196_000+i*0.4
        data,r=sample(i)
        decode_jitter=(i%7)*50_000
        data.update(begin_monotonic_ns=round(first),
                    end_monotonic_ns=round(first+500*period+1_000_000),
                    sdk_estimated_first_us=round(first+decode_jitter)//1000)
        r.update(receipt_monotonic_ns=data['end_monotonic_ns']+1_000_000,
                 receipt_ros_ns=data['end_monotonic_ns']+1_000_000+1_700_000_000_000_000_000)
        p,d,due=c.convert(data,**r)
        if i>=20:
            assert p is not None and c.epoch==4
            assert d['effective_sample_period_ns']>=180_000
            assert d['effective_sample_period_ns']<=220_000
            assert due>=r['receipt_monotonic_ns']
            refits+=d['model_refitted']
            errors.append(abs(c.last_published_end-(first+499*period)))
            held.append((p,copy.deepcopy(p)))
        first+=500*period
    assert refits>=50
    assert max(errors)<2_000_000
    assert all(original==snapshot for original,snapshot in held)
    assert c.counts['candidate_batches']==580


def test_small_ros_clock_backstep_cannot_overlap_published_batch():
    c=ready()
    data,r=sample(20)
    p,_,_=c.convert(data,**r)
    assert p is not None
    data,r=sample(21)
    r['receipt_ros_ns']-=20_000_000
    p,d,_=c.convert(data,**r)
    assert p is None and d['rejected']=='mapped_clock_overlap'
    assert c.epoch==5 and d['source_epoch']==5


def test_excessive_refit_phase_step_restarts_instead_of_rewriting_history():
    c=ready()
    for i in range(20,29):
        data,r=sample(i)
        assert c.convert(data,**r)[0] is not None
    # 이미 보낸 끝시각에 비해 이번 위상 보정이 지나치게 큰 후보를 주입한다.
    c.last_published_end+=15_000_000
    data,r=sample(29)
    p,d,_=c.convert(data,**r)
    assert p is None and d['rejected']=='phase_step_outside_candidate'
    assert c.epoch==5
