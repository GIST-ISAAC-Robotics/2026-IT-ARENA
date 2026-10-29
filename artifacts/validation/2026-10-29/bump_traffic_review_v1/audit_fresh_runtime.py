"""같은 스캔 대기의 실제 판단 기록. STOP 이후 기록과 주행 구간을 분리한다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def audit(run):
    paths=[run/name for name in ('report.json','follow_decisions.jsonl','trajectory.jsonl')]
    before={p.name:sha(p) for p in paths}
    report=json.loads(paths[0].read_text(encoding='utf-8'))
    first,last=(report['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    counts=Counter(); waits=[]; latches=[]; seen=set(); releases=[]; previous=None
    with paths[1].open(encoding='utf-8') as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='reset':previous=None
            if event['kind']!='call':continue
            data=event['data'];now=data['input']['reference_s'];output=data['output']
            detail=output[2];partial=detail.get('partial_observation') or {}
            source=detail.get('source_first_s');fresh=source!=previous;previous=source
            if not first<=now<=last:continue
            counts['calls']+=1;counts['positive_planner_commands']+=output[0]>0
            key=(partial.get('latch_time_s'),partial.get('latch_reason'))
            if key[0] is not None and key not in seen:
                seen.add(key);latches.append(dict(t=now,latch_time_s=key[0],reason=key[1],
                    new_scan_vs_prior_planner_call=fresh,source_first_s=source))
            if partial.get('reason')=='same_scan_missing_wait':
                waits.append(dict(t=now,command_mps=output[0],detail=partial.get('same_scan_wait')))
            for field in ('anchor_diagnostic','clearance_diagnostic'):
                value=partial.get(field) or {}
                if value.get('released'):releases.append(dict(t=now,field=field,reason=value.get('reason')))
    with paths[2].open(encoding='utf-8') as stream:
        sampled=[r for line in stream if first<=(r:=json.loads(line))['t']<=last]
    return dict(scope='recorded planner events and sampled independent-guard reasons; not complete guard transitions',
        window_s=[first,last],counts=dict(counts),planner_same_scan_waits=waits,
        planner_first_latches=latches,planner_releases=releases,
        sampled_guard_reasons=dict(Counter(str(r['safety_reason']) for r in sampled)),
        input_sha256=before,input_unchanged=all(sha(p)==before[p.name] for p in paths))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=audit(args.run)
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(dict(calls=result['counts'].get('calls'),waits=len(result['planner_same_scan_waits']),
        latches=len(result['planner_first_latches']),releases=len(result['planner_releases']),
        input_unchanged=result['input_unchanged'])))
