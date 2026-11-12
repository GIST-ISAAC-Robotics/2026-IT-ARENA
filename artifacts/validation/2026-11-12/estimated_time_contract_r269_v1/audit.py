"""실제 입력/선택/출력 경계의 독립 감사. 미장착·엔코더 없음은 그대로 남긴다."""
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_vehicle_interface')]
from probe_esp32_imu import Decoder, summarize

out=ROOT/'build/r269_recovered/run'
read=lambda p:json.loads(p.read_bytes())
node,imu,c1=[read(out/n/'summary.json') for n in ('node','imu','c1')]
source=[json.loads(x)['payload'] for x in (out/'c1/converted.jsonl').read_text().splitlines()]
received=read(out/'node/scans.json');states=read(out/'node/states.json')
assert len(source)==len(received)==node['scans']==c1['counts']['published']==c1['counts']['received']
for a,b in zip(source,received):
    assert (a['source_frame_id'],a['source_epoch'],a['time_basis'],len(a['ranges_m']),a['acquired_at_s'][0],a['acquired_at_s'][-1]) == (
        b['frame'],b['epoch'],b['time_basis'],b['points'],b['first_s'],b['last_s'])
assert all(s['motion_time_policy']=='host_estimated_v1' and s['motion_timing_verified'] is False for s in states)
assert node['motion']['gyro']==imu['clock']['published']==3798
assert node['motion']['drive_motor']==node['motion']['wheels']==node['motion']['rejected']==0
assert node['nonzero_private_commands']==0 and node['drive_output'] is False
assert imu['inactive_status_observed'] and c1['sensor_stop_ack']
decoder=Decoder();wire=decoder.feed((out/'imu/uart.bin').read_bytes())
rows=[json.loads(x) for x in (out/'imu/frames.jsonl').read_text().splitlines()]
assert len(wire)==len(rows)
assert all(all(r[k]==v for k,v in raw.items()) for raw,r in zip(wire,rows))
summary=summarize(rows,decoder,imu['duration_s'])
assert not summary['sequence_gaps'] and not decoder.counters.get('crc_errors',0)
supervisor=read(out/'supervisor.json')
assert supervisor['ports_free'] and supervisor['source_unchanged'] and not supervisor['forced']
assert all(v==0 for v in supervisor['children'].values())
result=dict(c1_array_metadata_unchanged=len(source),gyro_published_and_common_accepted=node['motion']['gyro'],
    imu_raw_frames=len(wire),crc_errors=0,sequence_gaps=0,
    encoder_count=0,positive_private_commands=0,time_contract_verified=False,mounting_calibrated=False,
    explicit_estimate_selection_preserved=True,children_normal=True,ports_free=True,
    full_motion_calculation_proven=False,scope='actual raw inputs and private zero-output boundary only; unmounted frame/no encoder/no camera')
(A/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
