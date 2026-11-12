#!/usr/bin/env python3
"""수신기와 별도로 원시 UART/JSONL·모터 종료 기록을 대조한다."""
import argparse
import binascii
import json
from pathlib import Path
import re


def audit(folder):
    data=(folder/'uart-private.bin').read_bytes();raw=[];prefix_bytes=0
    lines=data.split(b'\n')
    for i,line in enumerate(lines):
        if i==len(lines)-1 and not line:continue
        if not raw:
            m=re.search(rb'[0-9a-fA-F]{8} \{',line)
            if m:prefix_bytes+=m.start();line=line[m.start():]
            else:prefix_bytes+=len(line)+1;continue
        assert len(line)>9 and line[8:9]==b' ',('frame',i)
        assert int(line[:8],16)==binascii.crc32(line[9:]),('crc',i)
        raw.append(json.loads(line[9:]))
    logged=[json.loads(s) for s in (folder/'frames.jsonl').read_text().splitlines()]
    clean=[{k:v for k,v in r.items() if k not in ('host_ns','phase')} for r in logged]
    assert raw==clean,'raw versus JSONL mismatch'
    assert len({r['boot'] for r in raw})==1,'MCU reboot'
    assert all((b['seq']-a['seq'])&0xffffffff==1 for a,b in zip(raw,raw[1:])),'sequence gap'
    statuses=[r for r in raw if r['event']=='status']
    motors=[r for r in raw if r['event']=='motor']
    closed=json.loads((folder/'closed.json').read_text())
    final=closed['last_motor']
    assert closed['port_closed'] and final['active'] is False and final['armed'] is False
    assert final['pwm_pin']==0 and final['duty_permille']==0 and final['reason']=='host_stop'
    trials={}
    for path in sorted(folder.glob('result-*.json')):
        d=json.loads(path.read_text());m=d['output_end']
        assert m['active'] is False and m['armed'] is False
        assert m['duty_permille']==0 and m['pwm_pin']==0 and m['reason']=='deadline'
        assert m['deadline_us']-m['started_us']==d['duration_ms']*1000
        assert 0<=m['ended_us']-m['deadline_us']<=20000
        assert d['cutoff_lateness_us']==m['ended_us']-m['deadline_us']
        wire=next(r for r in raw if r['seq']==m['seq'])
        assert wire=={k:v for k,v in m.items() if k not in ('phase','host_ns')}
        for label in ('before','after'):
            r=d[label];wire=next(q for q in raw if q['seq']==r['seq'])
            assert wire=={k:v for k,v in r.items() if k not in ('phase','host_ns')}
        assert d['encoder_delta']=={k:d['after'][k]-d['before'][k] for k in d['encoder_delta']}
        trials[path.stem.removeprefix('result-')]={k:d[k] for k in
            ('duty_permille','duration_ms','cutoff_lateness_us','encoder_delta')}
    return {'raw_jsonl_identical':True,'crc_failures':0,'sequence_gaps':0,'boot_count':1,
            'boot_prefix_bytes':prefix_bytes,'frames':len(raw),'status_frames':len(statuses),
            'edge_frames':sum(r['event']=='edge' for r in raw),'motor_frames':len(motors),
            'max_invalid':max(r['invalid'] for r in statuses),
            'max_ring_dropped':max(r['ring_dropped'] for r in statuses),
            'max_tx_errors':max(r['tx_errors'] for r in statuses),
            'last_pulse_count_including_unpowered_preflight':final['pulses'],
            'powered_trials':trials,'final_pwm_zero':True,'final_disarmed':True,'port_closed':True,
            'physical_motion_or_braking_verified_by_this_audit':False}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=audit(a.folder);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in r.items() if k!='powered_trials'}))
