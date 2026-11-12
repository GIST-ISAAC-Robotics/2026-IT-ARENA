"""C++가 만든 혼합 wire를 기존 Python 공통 decoder로 독립 검사한다."""
import collections
import hashlib
import json
from pathlib import Path
import struct
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from arena_vehicle_interface.sensor_link_wire import SensorLinkDecoder
from arena_vehicle_interface.actuation_wire import Decoder
from probe_esp32_imu import Decoder as ImuDecoder


def inspect(case):
    raw=(A/f'{case}.bin').read_bytes();summary=json.loads((A/f'{case}.json').read_bytes())
    reference=None;split_reports=[]
    for split in (1,37,4096):
        decoder=SensorLinkDecoder();packets=[]
        for offset in range(0,len(raw),split):packets.extend(decoder.feed(raw[offset:offset+split]))
        assert not decoder.buffer
        assert sum(len(p) for _,p in packets)==len(raw)
        assert not any(v for k,v in decoder.counters.items() if k not in ('control','imu','nonframe_bytes'))
        assert decoder.counters.get('nonframe_bytes',0)==0
        if reference is None:reference=packets
        else:assert packets==reference
        split_reports.append(dict(bytes_per_feed=split,counters=dict(decoder.counters)))
    controls=[];imus=[];ia=Decoder();imu=ImuDecoder()
    for kind,packet in reference:
        if kind=='control':controls.extend(ia.feed(packet))
        else:imus.extend(imu.feed(packet))
    assert ia.errors==0 and not ia.buffer and not imu.buffer
    assert [p['sequence'] for p in imus]==list(range(summary['imu_generated']))
    observations=[c for c in controls if c.get('type')=='runtime_observation_v1']
    assert len(observations)==summary['observation_generated']-summary['observation_dropped']
    assert all(c['output_ready'] is False and c['encoder_motion_verified'] is False for c in observations)
    assert all(c['pcnt_count']==0 and c['pcnt_end_us']==c['pcnt_begin_us']+100 for c in observations)
    assert summary['control_ticks']==1100 and summary['imu_dropped']==0
    types=collections.Counter(c.get('type','unknown') for c in controls)
    assert types['offer']>0
    return dict(case=case,passed=True,raw_sha256=hashlib.sha256(raw).hexdigest(),summary=summary,
        split_checks=split_reports,control_types=dict(types),imu_count=len(imus),observation_count=len(observations),
        payload_tag_counts=dict(collections.Counter(p.get('sensor_tag') for p in imus)))


if __name__=='__main__':
    result=dict(scope='synthetic921600baud budget with400IMU frames/s and50raw observations/s; no hardware timing/drive proof',
                cases=[inspect(name) for name in ('steady','burst')])
    (A/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
