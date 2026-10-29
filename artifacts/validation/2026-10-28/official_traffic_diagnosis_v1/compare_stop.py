"""완료된 두 실행의 같은 진행 위치에서 경계/회복 이유를 비교한다.

정답 위치는 사후 정렬에만 쓴다. 제어 입력이나 판정을 변경하지 않는다.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
RUNS = {
    'temporary': ROOT/'artifacts/validation/2026-10-28/traffic_side_scope_v1/run1_course',
    'official': ROOT/'artifacts/validation/2026-10-28/official_traffic_v1/run1_course',
}


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def analyze(run):
    paths = [run/name for name in ('report.json','trajectory.jsonl','follow_decisions.jsonl','autonomy_status.jsonl')]
    hashes = {str(p.relative_to(ROOT)):digest(p) for p in paths}
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    selected = []
    with (run/'trajectory.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line)
            if 37.0 <= row['progress_m'] <= 38.0 and row['t'] <= report['stop_request_sim_s']:
                selected.append(row)
    if not selected:
        raise ValueError('no comparable position window')
    start,end=selected[0]['t'],selected[-1]['t']
    counters={name:Counter() for name in ('reason','baseline_reason','corridor_reason','partial_reason','selector_reason','wall','fresh_corridor_reason')}
    examples=[];keys=set();calls=positive=0
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            data=event['data'];t=data['input']['reference_s']
            if not start <= t <= end:continue
            calls+=1;positive+=data['output'][0]>0
            meta=data['output'][2]
            corridor=meta.get('corridor_candidate') or {}
            values=dict(reason=meta.get('reason'),baseline_reason=meta.get('baseline_reason'),
                corridor_reason=corridor.get('reason'),partial_reason=(meta.get('partial_observation')or{}).get('reason'),
                selector_reason=meta.get('selector_reason'),wall=data['input']['side'])
            for key,value in values.items():counters[key][str(value)]+=1
            if corridor.get('fresh'):counters['fresh_corridor_reason'][str(corridor.get('reason'))]+=1
            signature=tuple(values.values())
            if signature not in keys:
                keys.add(signature)
                examples.append(dict(seq=event['seq'], t=t, output=data['output'],
                    input_summary={k:data['input'][k] for k in ('speed_mps','side','previous_steering_rad','dt_s')}))
    representative = min(selected,key=lambda r:abs(r['progress_m']-37.474))
    return dict(source_hashes=hashes,raw_unchanged=all(digest(ROOT/p)==h for p,h in hashes.items()),
        original_passed=report['passed'],original_run_valid=report['run_valid'],
        window=dict(progress_m=[37.0,38.0],start_s=start,end_s=end,calls=calls,positive=positive),
        counters={k:dict(v) for k,v in counters.items()},representative_pose={k:representative[k] for k in ('t','x','y','yaw','progress_m','ground_speed_mps')},
        distinct_reason_examples=examples)


if __name__=='__main__':
    result={name:analyze(path) for name,path in RUNS.items()}
    with (BASE/'comparison.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    for name,value in result.items():
        print(name,json.dumps({k:value[k] for k in ('window','counters','representative_pose','raw_unchanged')},ensure_ascii=False))
