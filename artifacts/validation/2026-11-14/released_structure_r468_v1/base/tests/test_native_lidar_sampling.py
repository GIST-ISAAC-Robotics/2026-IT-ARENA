"""원천 시뮬 취득/전송: 방향과 거리 짝, 과거 프레임, 시각 출처를 검사한다."""
import json
import math
from pathlib import Path
from dataclasses import replace

import numpy as np
import pytest

from arena_vehicle_interface.native_lidar_sampling import NativeRevolutionAssembler,encode_measurements
from arena_autonomy.native_lidar_input import decode_measurements
from arena_autonomy.measurement_scan import sampling_gaps
from arena_autonomy.scan_input import scan_max_gap,scan_cell_widths

ROOT=Path(__file__).resolve().parents[1]


def patterns():
    return json.loads((ROOT/'config/tests/c1_standard10_native_angles_v1.json').read_text())['patterns_q14']


def run_frames(assembler,start=0.,stop=.2):
    output=[]
    for stamp in np.arange(start,stop+.0001,.002):
        # 값 자체는 인덱스/시각 보존용이며 실제 거리 모델이 아니다.
        values=np.arange(5000,dtype=float)+stamp*10000
        output.extend(assembler.feed(float(stamp),values,-math.pi,math.tau/5000))
    return output


def test_native_assembly_preserves_paired_ray_direction_and_past_time():
    p=patterns();a=NativeRevolutionAssembler(p)
    output=run_frames(a)
    assert len(output)==2
    for batch in output:
        assert len(batch['ranges_m'])==len(p[batch['template_index']])
        assert batch['max_capture_time_error_s']<=.00200001
        assert batch['max_source_angle_quantization_rad']<=math.pi/5000+1e-6
        assert all(v<0 for v in sampling_gaps(batch['angles_rad']))
        for distance,angle,stamp in zip(batch['ranges_m'],batch['angles_rad'],batch['acquired_at_s']):
            index=round((angle+math.pi)/(math.tau/5000))
            used_source=(distance-index)/10000
            assert used_source<=stamp+1e-10
            assert 0<=stamp-used_source+.00000001<=.00200002


def test_gap_discards_partial_scan_and_clock_rewind_changes_epoch():
    a=NativeRevolutionAssembler(patterns());assert not run_frames(a,0.,.04)
    assert not a.feed(.06,[1.]*5000,-math.pi,math.tau/5000)
    assert a.discarded==1
    result=run_frames(a,.062,.162)
    assert len(result)==1 and result[0]['acquired_at_s'][0]==.06
    assert not a.feed(.0,[1.]*5000,-math.pi,math.tau/5000)
    assert a.epoch==1


def test_native_codec_preserves_unknown_and_near_invalid_without_timing_promotion():
    sample=run_frames(NativeRevolutionAssembler(patterns()),stop=.1)[0]
    sample['ranges_m']=[1.]*len(sample['ranges_m'])
    sample['ranges_m'][:4]=[math.nan,math.inf,-math.inf,.01]
    text=json.dumps(encode_measurements(sample,range_min=.05,range_max=12.,frame_id='laser_frame'),allow_nan=False)
    decoded=decode_measurements(text,expected_time_basis='simulation_acquisition')
    assert math.isnan(decoded.ranges[0]) and decoded.ranges[1]==math.inf
    assert decoded.ranges[2]==-math.inf and decoded.ranges[3]==.01
    assert decoded.acquired_at_s==tuple(sample['acquired_at_s'])
    assert decoded.angles_rad==tuple(sample['angles_rad'])
    with pytest.raises(ValueError,match='timing basis'):
        decode_measurements(text,expected_time_basis='unverified_estimate')
    with pytest.raises(ValueError,match='not approved'):
        decode_measurements(text,expected_time_basis='validated_sensor_acquisition')
    assert scan_max_gap(decoded)>=max(scan_cell_widths(decoded))-1e-12


@pytest.mark.parametrize('bad',['extra','nan_time','reversed_time','negative_epoch','bool_range'])
def test_native_payload_faults_rejected(bad):
    sample=run_frames(NativeRevolutionAssembler(patterns()),stop=.1)[0]
    d=encode_measurements(sample,range_min=.05,range_max=12.,frame_id='laser_frame')
    if bad=='extra':d['truth_pose']=[0,0,0]
    if bad=='nan_time':d['acquired_at_s'][1]=math.nan
    if bad=='reversed_time':d['acquired_at_s'][1]=-.1
    if bad=='negative_epoch':d['source_epoch']=-1
    if bad=='bool_range':d['ranges_m'][1]=True
    with pytest.raises(ValueError):decode_measurements(json.dumps(d),expected_time_basis='simulation_acquisition')


def test_coarse_frame_is_not_relabelled_as_exact_raw_angles():
    a=NativeRevolutionAssembler(patterns())
    with pytest.raises(ValueError,match='dense'):
        a.feed(0.,[1.]*500,-math.pi,math.tau/500)
