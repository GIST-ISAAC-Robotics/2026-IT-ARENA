from dataclasses import replace
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys

import pytest

from arena_vehicle_interface.clock_bounds import MappedInterval, ClockAssumptions, ClockExchange, fit_clock
from arena_vehicle_interface.clock_freshness import HostTimeWindow, assess_freshness
from audit_clock_freshness import SCHEMA, review_bytes


def mapped(lo=100, hi=110):
    return MappedInterval('boot-A', 1, lo, hi, hi-lo, 0)


def assess(lo, hi, now=130, limit=30, inclusive=False):
    return assess_freshness(mapped(lo,hi),HostTimeWindow('host-A',now,now),
                            mapping_host_epoch='host-A',max_age_ns=limit,inclusive=inclusive)


@pytest.mark.parametrize('lo,hi,now,inclusive,status', [
    (101,110,130,False,'fresh_under_assumptions'),
    (100,110,130,False,'uncertain_age'),
    (100,110,130,True,'fresh_under_assumptions'),
    (90,100,130,False,'stale_under_assumptions'),
    (90,100,130,True,'uncertain_age'),
    (90,99,130,True,'stale_under_assumptions'),
    (120,140,130,False,'uncertain_age'),
    (131,140,130,False,'future_under_assumptions'),
    (130,140,130,False,'uncertain_age'),
    (130,130,130,False,'fresh_under_assumptions')])
def test_all_interval_extremes_not_midpoint_or_newest_point(lo,hi,now,inclusive,status):
    result = assess(lo,hi,now,inclusive=inclusive)
    assert result['status'] == status
    assert result['timestamp_policy_satisfied'] == (status == 'fresh_under_assumptions')
    for key in ('ready_to_drive','ros_stamp_verified','clock_mapping_applied','sensor_value_validated',
                'hardware_assumptions_verified'):
        assert result[key] is False


def test_now_read_uncertainty_and_very_large_uptime_are_not_lost():
    base = 2**60+123
    result = assess_freshness(mapped(base,base+10),HostTimeWindow('h',base+25,base+31),
                             mapping_host_epoch='h',max_age_ns=30)
    assert result['age_lower_ns'] == 15 and result['age_upper_ns'] == 31
    assert result['status'] == 'uncertain_age'


def test_random_integer_intervals_agree_with_exhaustive_endpoint_policy():
    rng = random.Random(12)
    for _ in range(400):
        lower = rng.randrange(100,200)
        upper = lower+rng.randrange(6)
        now_lower = lower+rng.randrange(-8,45)
        now_upper = now_lower+rng.randrange(6)
        inclusive = bool(rng.randrange(2))
        result = assess_freshness(mapped(lower,upper),HostTimeWindow('h',now_lower,now_upper),
                                 mapping_host_epoch='h',max_age_ns=30,inclusive=inclusive)
        ages = [n-c for c in range(lower,upper+1) for n in range(now_lower,now_upper+1)]
        assert result['age_lower_ns'] == min(ages) and result['age_upper_ns'] == max(ages)
        expected = all(0 <= age <= 30 if inclusive else 0 <= age < 30 for age in ages)
        assert result['timestamp_policy_satisfied'] == expected


@pytest.mark.parametrize('fault',['boot','width','reverse','bool','nan','epoch','zero_limit','inclusive','now_epoch'])
def test_invalid_contract_rejected_not_defaulted(fault):
    value, now, host, limit, inclusive = mapped(),HostTimeWindow('h',130,130),'h',30,False
    if fault=='boot': value=replace(value,boot='')
    elif fault=='width': value=replace(value,width_ns=4)
    elif fault=='reverse': value=replace(value,host_lower_ns=120,width_ns=0)
    elif fault=='bool': value=replace(value,host_lower_ns=True)
    elif fault=='nan': value=replace(value,host_upper_ns=float('nan'))
    elif fault=='epoch': host='old-host'
    elif fault=='zero_limit': limit=0
    elif fault=='inclusive': inclusive=1
    elif fault=='now_epoch':
        with pytest.raises(ValueError): HostTimeWindow('',130,130)
        return
    with pytest.raises(ValueError):
        assess_freshness(value,now,mapping_host_epoch=host,max_age_ns=limit,inclusive=inclusive)


def capture_and_review():
    header = dict(kind='header',schema='arena_clock_exchange_audit_v1',origin='synthetic',
        host_clock='host_monotonic_ns',device_clock='mcu_monotonic_us',assumptions=dict(
            max_rate_error_ppm=0,host_timestamp_error_ns=0,mcu_timestamp_error_us=0,
            max_window_us=5_000_000,max_extrapolation_us=100_000))
    records = [header,dict(kind='exchange',boot='boot-A',sequence=0,host_send_ns=8_999_000_000,
        mcu_receive_us=1_000_000,mcu_send_us=1_000_100,host_receive_ns=9_009_100_000),
        dict(kind='query',boot='boot-A',capture_us=1_000_050)]
    data = ('\n'.join(json.dumps(r) for r in records)+'\n').encode()
    review = dict(schema=SCHEMA,origin='synthetic',exchange_sha256=hashlib.sha256(data).hexdigest(),
        host_epoch='host-A',clock='host_monotonic_ns',queries=[dict(mapped_query_index=0,boot='boot-A',
            host_epoch='host-A',now_lower_ns=9_031_000_000,now_upper_ns=9_031_000_000,
            max_age_ns=30_000_000,limit_inclusive=False,policy_note='synthetic 30 ms diagnostic only')])
    return data,review


def test_asymmetric_exchange_midpoint_would_accept_but_interval_remains_uncertain():
    data,review = capture_and_review()
    result = review_bytes(data,json.dumps(review).encode())
    row = result['queries'][0]
    assert row['age_lower_ns']==21_950_000 and row['age_upper_ns']==31_950_000
    assert (row['age_lower_ns']+row['age_upper_ns'])//2 < 30_000_000
    assert result['review_valid'] and row['status']=='uncertain_age'
    assert not result['all_timestamp_policies_satisfied'] and not result['ready_to_drive']


@pytest.mark.parametrize('fault',['hash','origin','clock','boot','epoch','index','duplicate','no_note','missing','extra','no_query','nan','duplicate_key'])
def test_review_rejects_cross_evidence_or_malformed_spec(fault):
    data,review = capture_and_review()
    q = review['queries'][0]
    if fault=='hash': review['exchange_sha256']='0'*64
    elif fault=='origin': review['origin']='hardware_capture'
    elif fault=='clock': review['clock']='ros_time_ns'
    elif fault=='boot': q['boot']='boot-B'
    elif fault=='epoch': q['host_epoch']='host-B'
    elif fault=='index': q['mapped_query_index']=True
    elif fault=='duplicate': review['queries'].append(q.copy())
    elif fault=='no_note': q['policy_note']=''
    elif fault=='missing': q.pop('limit_inclusive')
    elif fault=='extra': q['guess']=True
    elif fault=='no_query': review['queries']=[]
    encoded=json.dumps(review).encode()
    if fault=='nan': encoded=encoded.replace(b'9031000000',b'NaN')
    elif fault=='duplicate_key': encoded=encoded.replace(b'"limit_inclusive": false',b'"limit_inclusive": false, "limit_inclusive": true')
    with pytest.raises(ValueError): review_bytes(data,encoded)


def test_cli_valid_uncertain_is_not_drive_success_and_never_overwrites(tmp_path):
    data,review=capture_and_review()
    capture,spec,output=tmp_path/'capture.jsonl',tmp_path/'review.json',tmp_path/'result.json'
    capture.write_bytes(data)
    spec.write_text(json.dumps(review))
    command=[sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/audit_clock_freshness.py'),
             str(capture),str(spec),'--output',str(output)]
    first=subprocess.run(command,capture_output=True,text=True)
    assert first.returncode==0,first.stderr
    saved=output.read_bytes()
    report=json.loads(saved)
    assert report['review_valid'] and report['source_unchanged'] and report['input_unchanged']
    assert not report['all_timestamp_policies_satisfied'] and not report['ready_to_drive']
    assert subprocess.run(command,capture_output=True).returncode!=0
    assert output.read_bytes()==saved and capture.read_bytes()==data


def test_no_production_node_imports_diagnostic_only_module():
    import ast
    root=Path(__file__).resolve().parents[1]
    for package in ('arena_autonomy','arena_vehicle_interface'):
        for path in (root/'src'/package/package).glob('*.py'):
            if path.name=='clock_freshness.py': continue
            names=[node.module for node in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
                   if isinstance(node,ast.ImportFrom)]
            assert 'arena_vehicle_interface.clock_freshness' not in names


def test_post_read_hash_does_not_bypass_byte_budget_by_loading_entire_file(tmp_path, monkeypatch):
    from audit_clock_freshness import digest
    payload = b'capture' * 400_000
    path = tmp_path/'large_capture.bin'
    path.write_bytes(payload)
    def forbidden(*_):
        raise AssertionError('digest must stream, not allocate a whole input')
    monkeypatch.setattr(Path, 'read_bytes', forbidden)
    assert digest(path) == hashlib.sha256(payload).hexdigest()
