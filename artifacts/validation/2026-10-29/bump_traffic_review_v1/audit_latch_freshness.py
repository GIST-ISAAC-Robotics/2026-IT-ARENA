"""기존 래치가 새 스캔에서 생겼는지 같은 스캔 재사용 중 생겼는지 분류."""
import hashlib
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
RUNS={name:ROOT/'artifacts/validation/2026-10-28'/path/'follow_decisions.jsonl' for name,path in {
    'bump_run2':'bump_traffic_v1/run2','bump_run3':'bump_traffic_v1/run3',
    'rotation_run4':'rotation_stress_v1/run4_course'}.items()}


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def main():
    result={}
    for name,path in RUNS.items():
        before=sha(path);previous=None;seen=set();rows=[];calls=0
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                event=json.loads(line)
                if event['kind']=='reset':previous=None
                if event['kind']!='call':continue
                data=event['data'];inputs=data['input'];scan=inputs['scan'];calls+=1
                stamp=(scan['sec'],scan['nanosec']);fresh=stamp!=previous;previous=stamp
                now=inputs['reference_s'];part=data['output'][2].get('partial_observation') or {}
                latch=part.get('latch_time_s');key=(latch,part.get('latch_reason'))
                if latch is None or key in seen:continue
                seen.add(key)
                rows.append(dict(first_logged_s=now,latch_time_s=latch,reason=key[1],
                    detected_on_creation_call=abs(latch-now)<1e-8,
                    new_scan_vs_previous_call=fresh,source_first_s=stamp[0]+stamp[1]*1e-9,
                    scan_age_s=now-(stamp[0]+stamp[1]*1e-9),
                    pairs=(part.get('latch_evidence') or {}).get('pairs')))
        result[name]=dict(calls=calls,input_sha256=before,input_unchanged=sha(path)==before,rows=rows)
    with (BASE/'latch_freshness.json').open('x',encoding='utf-8') as stream:
        json.dump(dict(scope='recorded planner first-latch calls, not independent ROS guard timing',runs=result),
            stream,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=True))


if __name__=='__main__':main()
