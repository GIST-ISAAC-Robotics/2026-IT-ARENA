"""저장 명령에서 같은 스캔 내 제어 변화를 센다. 재생/캐시 후보/주행은 실행하지 않는다."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def stamp(scan):
    if scan.get('representation')=='native_measurements_v1':
        s=scan['measurements']
        return s['source_frame_id'],s['source_epoch'],s['acquired_at_s'][0]
    return scan['frame_id'],None,scan['sec']+scan['nanosec']*1e-9


def describe(row):
    d=row['data'];i=d['input'];o=d['output'];n=o[2].get('normal',o[2])
    return dict(seq=row['seq'],reference_s=i['reference_s'],source_first_s=stamp(i['scan'])[-1],
        speed_mps=o[0],steering_rad=o[1],measured_mps=i['speed_mps'],
        handoff=o[2].get('handoff_state'),reason=n.get('selector_reason') or n.get('reason'))


def audit(path):
    sha=lambda:hashlib.sha256(path.read_bytes()).hexdigest()
    before=sha();last=None;calls=0;counts=Counter();kinds=Counter();examples=[];periods=[];dt_values=set();footer=None
    with path.open(encoding='utf8') as stream:
        for row in map(json.loads,stream):
            kind=row['kind'];kinds[kind]+=1
            if kind=='reset':last=None
            if kind=='footer':footer=row['data']
            if kind!='call':continue
            calls+=1;i=row['data']['input'];o=row['data']['output'];dt_values.add(i['dt_s'])
            key=stamp(i['scan'])+(i['epoch'],)
            if last is not None:
                old,last_key=last
                prior=old['data']['input'];out=old['data']['output']
                gap=i['reference_s']-prior['reference_s']
                if gap<0:raise ValueError('reference reversed without reset')
                periods.append(gap)
                if key==last_key:
                    counts['same_scan_transitions']+=1
                    speed=not math.isclose(o[0],out[0],rel_tol=1e-10,abs_tol=1e-10)
                    steer=not math.isclose(o[1],out[1],rel_tol=1e-10,abs_tol=1e-10)
                    changed=speed or steer
                    counts['same_scan_speed_changes']+=speed
                    counts['same_scan_steering_changes']+=steer
                    counts['same_scan_any_changes']+=changed
                    counts['same_scan_positive_to_zero']+=out[0]>0 and o[0]<=0
                    counts['same_scan_zero_to_positive']+=out[0]<=0 and o[0]>0
                    if changed and (len(examples)<5 or out[0]>0 and o[0]<=0 and len(examples)<12):
                        examples.append(dict(before=describe(old),after=describe(row)))
                else:counts['new_scan_transitions']+=1
            last=row,key
    if not footer or footer.get('failed') or footer.get('dropped') or footer.get('capture_errors'):
        raise ValueError('complete decision log required')
    if sha()!=before:raise ValueError('source changed')
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=before,input_unchanged=True,calls=calls,
        events=dict(kinds),transitions=dict(counts),command_dt_values_s=sorted(dt_values),
        reference_gap_s=dict(minimum=min(periods),maximum=max(periods)),examples=examples)


if __name__=='__main__':
    path=Path(sys.argv[1]).resolve(strict=True);output=Path(sys.argv[2])
    assert not output.exists()
    result=audit(path)
    result['scope']='stored commands only; no replay, wall-time benchmark, cached controller or closed-loop result'
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('examples',)}))
