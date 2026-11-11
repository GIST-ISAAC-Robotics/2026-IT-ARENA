"""모든 선택 배열을 실제 시뮬 취득/직렬화 경로로 통과시킨다. 주행 시험 아님."""
import hashlib
import json
import math
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'src/arena_autonomy')]
from arena_vehicle_interface.native_lidar_sampling import NativeRevolutionAssembler,encode_measurements
from arena_autonomy.native_lidar_input import decode_measurements
from arena_autonomy.measurement_scan import sampling_gaps

record=json.loads((A/'profile.json').read_bytes());patterns=record['patterns_q14']
sampler=NativeRevolutionAssembler(patterns,rate_hz=record['rotation_rate_hz'])
frames=[];source=[1.+i/5000 for i in range(5000)]
for k in range(50*len(patterns)+1):frames.extend(sampler.feed(k*.002,source,-math.pi,math.tau/5000))
assert len(frames)==len(patterns)
for i,row in enumerate(frames):
    assert row['template_index']==i and len(row['ranges_m'])==len(patterns[i])
    assert row['max_capture_time_error_s']<=.00200001
    assert row['max_source_angle_quantization_rad']<=math.pi/5000+1e-9
    encoded=encode_measurements(row,range_min=.05,range_max=12.,frame_id='laser_frame')
    decoded=decode_measurements(json.dumps(encoded,allow_nan=False),expected_time_basis='simulation_acquisition')
    assert all(g<0 for g in sampling_gaps(decoded.angles_rad))
    assert decoded.acquired_at_s==tuple(row['acquired_at_s'])
    for value,angle in zip(decoded.ranges,decoded.angles_rad):
        index=round((angle+math.pi)/(math.tau/5000))%5000
        assert value==source[index]
result=dict(templates=len(patterns),assembled_revolutions=len(frames),source_frames=50*len(patterns)+1,
    paired_angle_range_and_time_preserved=True,clockwise=True,discarded=sampler.discarded,
    counts=sorted(set(map(len,patterns))),profile_sha256=hashlib.sha256((A/'profile.json').read_bytes()).hexdigest(),
    scope='Synthetic dense scans through simulation assembler and native codec. No measured ray timing or driving proof.')
with (A/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
