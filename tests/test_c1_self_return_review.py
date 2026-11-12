import copy
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from review_c1_self_returns import review
from arena_autonomy.local_scene import blind_parameter
from arena_autonomy.local_path import mask_scan
from arena_autonomy.native_lidar_input import decode_measurements


NOMINAL=dict(sectors_deg=[[160.,2.]],range_limits_m=[[.16,.18]])


def records(count=105):
    result=[]
    for i in range(count):
        angle=163.+.2*math.sin(i)
        radius=.17+.004*math.sin(i*.7)
        payload=dict(schema='arena_native_lidar_v1',ranges_m=[radius,.4],
            angles_rad=[math.radians(angle),math.radians(angle+.02)],
            acquired_at_s=[1+i*.1,1+i*.1+.0002],source_frame_id='c1_installed',source_epoch=1,
            range_min_m=.05,range_max_m=12.,time_basis='unverified_estimate')
        result.append(dict(payload=payload,receipt_monotonic_ns=2_000_000_000+i*100_000_000))
    return result


def call(rows,nominal=NOMINAL,**kw):
    args=dict(stationary=True,clear_nearby=True,frame_id='c1_installed')
    args.update(kw)
    return review(rows,nominal,**args)


def test_measured_limits_add_reserve_and_never_shrink_nominal():
    rows=records();original=copy.deepcopy(rows)
    result=call(rows)
    assert result['candidate_ready'] and result['applied'] is False
    sector=result['mask']['sectors_deg'][0];lo,hi=result['mask']['range_limits_m'][0]
    assert sector[1]>4 and lo<=.16 and hi>=.18
    assert result['residual_motion_m']==.003 and result['unknown_filled'] is False
    assert result['unmatched_or_ambiguous_returns']==105
    assert rows==original
    scan=decode_measurements(json.dumps(rows[0]['payload']),expected_time_basis='unverified_estimate')
    masked=mask_scan(scan,blind_parameter(0.,json.dumps(result['mask'])))
    assert math.isnan(masked.ranges[0]) and masked.ranges[1]==.4


def test_missing_mount_is_retained_and_incomplete():
    nominal=dict(sectors_deg=[[160.,2.],[-160.,2.]],range_limits_m=[[.16,.18],[.16,.18]])
    r=call(records(),nominal)
    assert not r['candidate_ready'] and r['details'][1]['hits']==0
    assert r['mask']['sectors_deg'][1]==nominal['sectors_deg'][1]
    assert r['mask']['range_limits_m'][1]==nominal['range_limits_m'][1]


@pytest.mark.parametrize('kwargs',[dict(stationary=False),dict(clear_nearby=False),dict(frame_id='wrong'),
    dict(residual_motion_m=.001),dict(range_noise_m=.001),dict(angular_noise_deg=.1)])
def test_missing_conditions_or_shrunken_reserve_rejected(kwargs):
    with pytest.raises(ValueError):call(records(),**kwargs)


@pytest.mark.parametrize('kind',['short','epoch','clock','few','mask_without_range'])
def test_bad_capture_or_mask_rejected(kind):
    rows=records();nominal=NOMINAL
    if kind=='short':
        for i,r in enumerate(rows):r['receipt_monotonic_ns']=1_000_000_000+i*50_000_000
    if kind=='epoch':rows[70]['payload']['source_epoch']=2
    if kind=='clock':rows[70]['receipt_monotonic_ns']=rows[69]['receipt_monotonic_ns']
    if kind=='few':rows=rows[:90]
    if kind=='mask_without_range':nominal=dict(sectors_deg=[[160.,2.]])
    with pytest.raises(ValueError):call(rows,nominal)


def test_cli_synthetic_record_and_no_overwrite(tmp_path):
    path=tmp_path/'input.jsonl';mask=tmp_path/'mask.json';dest=tmp_path/'candidate.json'
    path.write_text(''.join(json.dumps(r)+'\n' for r in records()))
    mask.write_text(json.dumps(NOMINAL))
    script=Path(__file__).resolve().parents[1]/'scripts/review_c1_self_returns.py'
    args=[sys.executable,str(script),'--input',str(path),'--nominal-mask',str(mask),
          '--frame-id','c1_installed','--stationary-confirmed','--nearby-clear-confirmed','--output',str(dest)]
    first=subprocess.run(args,capture_output=True,text=True)
    assert first.returncode==0,first.stderr
    content=dest.read_bytes()
    assert not json.loads(content)['applied']
    assert subprocess.run(args,capture_output=True).returncode!=0
    assert dest.read_bytes()==content
