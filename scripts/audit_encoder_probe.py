#!/usr/bin/env python3
"""수신기를 import하지 않고 CRC 원문/JSONL·quadrature 에지 계보를 대조한다."""
import argparse
import collections
import json
from pathlib import Path
import zlib


def audit(folder):
    raw=(folder/'uart-private.bin').read_bytes();packets=[];prefix=0
    for line in raw.splitlines():
        if len(line)<10 or line[8:9]!=b' ':
            prefix+=1;continue
        try:expected=int(line[:8],16)
        except ValueError:prefix+=1;continue
        assert zlib.crc32(line[9:])==expected,'CRC mismatch'
        packets.append(json.loads(line[9:]))
    saved=[json.loads(line) for line in (folder/'frames.jsonl').read_text().splitlines()]
    assert len(saved)==len(packets)
    for wire,record in zip(packets,saved):
        assert wire=={k:v for k,v in record.items() if k not in ('phase','host_monotonic_ns')}
    assert len({r['boot'] for r in packets})==1
    assert all((b['seq']-a['seq'])&0xffffffff==1 for a,b in zip(packets,packets[1:]))
    table=(0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0)
    edges=[r for r in saved if r['event']=='edge'];direction=collections.Counter()
    for r in edges:
        assert 0<=r['before']<=3 and 0<=r['after']<=3
        assert r['delta']==table[(r['before']<<2)|r['after']]
        direction[r['delta']]+=1
    phases={}
    for phase in dict.fromkeys(r['phase'] for r in saved):
        states=[r for r in saved if r['phase']==phase and r['event']=='status']
        if not states:continue
        first,last=states[0],states[-1]
        phases[phase]=dict(samples=len(states),pcnt_delta=last['pcnt_count']-first['pcnt_count'],
            software_delta=last['software_count']-first['software_count'],
            reported_edge_delta=last['edges']-first['edges'],
            last_pcnt=last['pcnt_count'],last_software=last['software_count'],
            raw_ab_values=sorted({r['raw_ab'] for r in states if 'raw_ab' in r}),
            cached_ab_values=sorted({r['ab'] for r in states}))
    statuses=[r for r in saved if r['event']=='status']
    return dict(frames=len(packets),status_samples=len(statuses),edge_frames=len(edges),
        nonframe_prefix_lines=prefix,crc_and_jsonl_match=True,sequence_gaps=0,
        directions=dict(direction),phases=phases,
        last_invalid= statuses[-1]['invalid'],last_ring_dropped=statuses[-1]['ring_dropped'],
        last_tx_errors=statuses[-1]['tx_errors'],
        stop_acknowledged=not statuses[-1]['streaming'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result=audit(args.folder);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
