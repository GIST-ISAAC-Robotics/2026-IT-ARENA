"""실제 생성량을 반영한 1회/제한 반복 pump의 동일 대역 비교."""
from pathlib import Path
import hashlib,json,sys
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_vehicle_interface'),str(R/'scripts')]
from arena_vehicle_interface.sensor_link_wire import SensorLinkDecoder
from arena_vehicle_interface.actuation_wire import Decoder
from probe_esp32_imu import Decoder as ImuDecoder
reports=[]
for case in ('single_steady','single_burst','bounded_steady','bounded_burst'):
    raw=(A/(case+'.bin')).read_bytes();summary=json.loads((A/(case+'.json')).read_bytes())
    assert raw==(A/'sanitizer'/(case+'.bin')).read_bytes()
    reference=None
    for split in (1,37,4096):
        mux=SensorLinkDecoder();packets=[]
        for i in range(0,len(raw),split):packets.extend(mux.feed(raw[i:i+split]))
        assert not mux.buffer and sum(len(p) for _,p in packets)==len(raw)
        assert not mux.counters.get('nonframe_bytes',0)
        assert not any(v for k,v in mux.counters.items() if k not in ('control','imu','nonframe_bytes'))
        if reference is None:reference=packets
        else:assert packets==reference
    ia=Decoder();imu=ImuDecoder();cs=[];ms=[]
    for kind,p in reference:
        if kind=='imu':ms.extend(imu.feed(p))
        else:cs.extend(ia.feed(p))
    assert ia.errors==0 and not ia.buffer and not imu.buffer
    assert len(ms)==summary['imu_generated']-summary['imu_dropped']
    assert all(b['sequence']>a['sequence'] for a,b in zip(ms,ms[1:]))
    obs=[c for c in cs if c['type']=='runtime_observation_v1']
    assert len(obs)==summary['observation_generated']-summary['observation_dropped']
    assert all(not c['output_ready'] and not c['encoder_motion_verified'] for c in obs)
    if case.startswith('bounded'):
        assert summary['imu_dropped']==0
        assert [m['sequence'] for m in ms]==list(range(18000))
    else:assert summary['imu_dropped']>0
    if case=='bounded_steady':assert summary['observation_dropped']==0
    reports.append(dict(case=case,summary=summary,sha256=hashlib.sha256(raw).hexdigest(),split_checks=[1,37,4096],sanitizer_bytes_equal=True))
(A/'host_assessment.json').write_text(json.dumps(dict(cases=reports,scope='600 IMU frames/s plus 50 raw observations/s, 921600 baud synthetic; no hardware timing proof'),indent=2)+'\n')
print(json.dumps([{'case':r['case'],'imu_dropped':r['summary']['imu_dropped'],'observation_dropped':r['summary']['observation_dropped']} for r in reports]))
