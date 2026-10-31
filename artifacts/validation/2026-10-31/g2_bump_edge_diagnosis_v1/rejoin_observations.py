"""방지턱 접촉 이전의 복귀 띠 거절과 현재 안쪽 면 관측. 새 명령은 발행하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path
from statistics import median

BASE=Path(__file__).resolve().parent
CASES=('g1_forward_same_path_gazebo_v1','g2_motion_pause_gazebo_v1','g2_motion_pause_level_v1')


def digest(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def analyze(case):
    path=BASE.parent/case/'run1/follow_decisions.jsonl';before=digest(path)
    rows=[];last_key=None;transitions=[]
    with path.open() as f:
        for line in f:
            e=json.loads(line)
            if e['kind']!='call':continue
            d=e['data'];t=d['input']['reference_s']
            if t>23.2:break  # All three saved trials are before the first possible bump overlap here.
            o=d['output'];detail=o[2];b=detail.get('bypass') or {}
            state=(detail.get('handoff_state'),b.get('entry_phase'))
            if not transitions or transitions[-1]['state']!=state:transitions.append(dict(t=t,state=state))
            if state!=('BYPASS','entry') or not b.get('rejoin_band'):continue
            key=(b.get('source_epoch'),b.get('source_first_s'))
            if key==last_key:continue
            last_key=key
            band=b['rejoin_band'];sides=b.get('corridor') or {}
            surfaces={k:{f:v.get('surface',{}).get(f) for f in ('along_range_m','normal_m','method')}
                      for k,v in sides.items() if isinstance(v,dict)}
            rows.append(dict(t=t,source=key[1],command=o[0],
                speed=(detail.get('normal') or {}).get('measured_speed_mps'),
                band=band,surfaces=surfaces))
    window=[r for r in rows if 20<=r['t']<=23.2]
    return dict(case=case,input_sha256=before,input_unchanged=digest(path)==before,
        time_window_s=[0,23.2],transitions=transitions,distinct_entry_scan_count=len(rows),
        late_window_count=len(window),late_command_median=median([r['command'] for r in window]) if window else None,
        late_supported=sum(r['band']['supported'] for r in window),
        late_current_occupied=sum(r['band'].get('current_occupied_returns',0)>0 for r in window),
        first=rows[0] if rows else None,last=rows[-1] if rows else None,observations=rows,
        scope='saved sensor-based decisions before bump; not predicted new trajectory or moving-object identity proof')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=dict(cases=[analyze(c) for c in CASES],source_sha256=digest(Path(__file__)))
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps([{k:v for k,v in r.items() if k not in ('observations','first','last','transitions')} for r in result['cases']]))
