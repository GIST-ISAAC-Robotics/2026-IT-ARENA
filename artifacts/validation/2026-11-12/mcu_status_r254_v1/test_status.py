import copy
from pathlib import Path
import subprocess
import sys

import pytest

A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'tests'),str(R/'scripts')]
from status_clock import McuStatusClock
from test_actuation_tx_policy import Mcu
from arena_autonomy.race_session import McuObserver


def offer(t, **kw):
    return dict(type='offer',v=1,boot='1234abcd',generation=0,state='DISARMED',reason='startup',
                output_ready=False,estop_held=False,status_now_us=t,lease_issued_us=t,**kw)


def convert(clock,rows,t,delay=0):
    return clock.convert_batch(rows,monotonic_ns=(t+delay+10_000_000)*1000,
                               realtime_ns=(t+delay+1_790_000_000_000_000)*1000)


def test_steady_state_and_delay_keep_original_receipt():
    clock=McuStatusClock()
    rows=[]
    for t in range(0,500_001,20_000):rows+=convert(clock,[offer(t)],t)
    assert len(rows)==13 and all(x['output_ready'] is False for x in rows)
    old=convert(clock,[offer(520_000)],520_000,delay=350_000)[0]
    assert old['receipt_age_ns']==350_000_000
    assert old['now_us']-old['mcu_now_us']==1_790_000_000_000_000
    assert not old['time_contract_verified']
    observer=McuObserver()
    assert observer.update(old,old['host_receipt_ns']*1e-9,1.)=='clock_mismatch'


def test_batch_old_samples_do_not_become_latest():
    clock=McuStatusClock(warmup_ns=0)
    convert(clock,[offer(0)],0)
    rows=convert(clock,[offer(t) for t in (20_000,40_000,60_000)],60_000)
    assert [x['receipt_age_ns'] for x in rows]==[40_000_000,20_000_000,0]
    assert [x['mcu_now_us'] for x in rows]==[20_000,40_000,60_000]
    assert not convert(clock,[offer(60_000)],100_000)


def test_boot_generation_old_packets_and_host_clock_reset():
    c=McuStatusClock(warmup_ns=0)
    x=offer(100_000);x['generation']=3
    old=convert(c,[x],100_000)[0]
    x=offer(120_000);x['generation']=2
    assert not convert(c,[x],120_000)
    y=offer(0);y['boot']='8765abcd'
    current=convert(c,[y],140_000)[0]
    assert current['clock_epoch']>old['clock_epoch']
    assert not convert(c,[offer(140_000)],160_000)
    clock_reset=c.convert_batch([dict(y,status_now_us=20_000,lease_issued_us=20_000)],
                                monotonic_ns=10_180_000_000,realtime_ns=2_000_000_000_000)
    assert c.counts['clock_reset']==1
    assert clock_reset[0]['clock_epoch']>current['clock_epoch']


@pytest.mark.parametrize('key,value', [('status_now_us',True),('generation',True),('estop_held',None),
    ('output_ready',1),('status_now_us',-1),('status_now_us',10),('v',True),('state','UNRECOGNIZED')])
def test_missing_or_invalid_status_never_fabricates_fields(key,value):
    c=McuStatusClock(warmup_ns=0);row=offer(0);row[key]=value
    assert not convert(c,[row],0)
    assert c.counts['invalid']==1


@pytest.fixture(scope='module')
def binaries(tmp_path_factory):
    d=tmp_path_factory.mktemp('status_tx')
    source=(R/'scripts/actuation_tx_driver.cpp').read_text()
    result={}
    for label,enabled in [('original',False),('candidate_off',False),('candidate_on',True)]:
        cpp=d/(label+'.cpp')
        cpp.write_text(source.replace('aa::TxConfig tx_cfg;',f'aa::TxConfig tx_cfg;'+(' tx_cfg.offer_status_v1=true;' if enabled else '')))
        includes=[] if label=='original' else ['-I'+str(A/'portable')]
        exe=d/label
        subprocess.run(['g++','-std=c++17','-Os','-Wall','-Wextra','-Werror','-fsanitize=undefined',
                        *includes,'-I'+str(R/'src/arena_vehicle_interface/portable'),str(cpp),'-o',str(exe)],check=True)
        result[label]=exe
    return result


def sequence(binary):
    m=Mcu(binary,diag_period_us=0);m.cmd('C 0')
    for t in range(0,100_001,10_000):m.cmd(f'T {t}');m.pump(t)
    m.cmd('S 110000 1');m.pump(110000);m.pump(111000)
    m.cmd('S 120000 0');m.pump(120000);m.pump(130000)
    outputs=copy.deepcopy(m.payloads);raw=list(m.raw)
    assert all(v<=1536 for v in m.driver.init['worst_payload'].values())
    m.close();return outputs,raw


def test_default_bytes_unchanged_and_selected_status_matches_receiver(binaries):
    original,raw=sequence(binaries['original'])
    off,off_raw=sequence(binaries['candidate_off'])
    assert raw==off_raw and off==original
    on,_=sequence(binaries['candidate_on'])
    offers=[x for x in on if x['type']=='offer']
    assert offers and all(x['status_now_us']==x['lease_issued_us'] for x in offers)
    assert all(x['output_ready'] is False for x in offers)
    assert any(x['estop_held'] and x['state']=='ESTOP_LATCHED' for x in offers)
    assert any(not x['estop_held'] and x['state']=='ESTOP_LATCHED' for x in offers)
    for row in on:
        if row['type']=='offer':
            for key in ('status_now_us','estop_held','reason'):row.pop(key)
    assert on==original
