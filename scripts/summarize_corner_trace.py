#!/usr/bin/env python3
"""판단 기록의 코너 증거/정지 전이를 요약한다. 재생 검증·새 주행 판정은 별도다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def summarize(events):
    rows=[]
    previous_sequence=-1
    footer=None
    mode=None
    for event in events:
        if event['seq'] != previous_sequence+1 or footer is not None:
            raise ValueError('trace order/gap/trailing record')
        previous_sequence=event['seq']
        kind,data=event['kind'],event['data']
        if event['seq']==0:
            if kind!='header' or data.get('schema')!=1:
                raise ValueError('unsupported header')
            mode=data['follow_config']['boundary_evidence_mode']
        elif kind=='footer':
            footer=data
        elif kind=='call':
            speed,steering,detail=data['output']
            distance_mode = mode == 'corner_distance_candidate'
            evidence=detail.get('corner_distance_evidence' if distance_mode else 'corner_boundary_evidence')
            rows.append(dict(seq=event['seq'],time_s=data['input']['reference_s'],
                speed_mps=speed,steering_rad=steering,measured_speed_mps=data['input']['speed_mps'],
                reason=detail['reason'],selection=detail.get('boundary_selection'),
                fresh_accepted=detail.get('fresh_boundary_accepted'),
                path_source_s=detail.get('path_source_s'),boundary_age_s=detail.get('boundary_age_s'),
                memory_agrees=detail.get('corner_distance_agrees_memory' if distance_mode else 'corner_agrees_memory'),
                corner_reason=evidence.get('reason') if evidence else None,
                corner_supported=evidence.get('supported') if evidence else None,
                supported_prefix_m=evidence.get('supported_prefix_m') if evidence else None,
                path_end_exclusive=evidence.get('path_end_exclusive') if evidence else None,
                path_points_count=len(detail['path_points']) if detail.get('path_points') is not None else None))
        elif kind!='reset':
            raise ValueError('unexpected trace kind')
    if footer is None or not rows:
        raise ValueError('incomplete capture')
    if (footer.get('capture_errors')!=0 or footer.get('failed')!=0 or footer.get('dropped')!=0
        or footer.get('pending')!=0 or footer.get('worker_alive') is not False
        or footer.get('written')!=previous_sequence or footer.get('accepted')!=previous_sequence):
        raise ValueError('capture footer not complete')
    changes=[row for i,row in enumerate(rows) if i==0 or row['reason']!=rows[i-1]['reason']]
    selected=[row for row in rows if row['selection'] in
              ('bilateral_observed_corner_anchor','bilateral_observed_corner_distance')]
    return dict(scope='recorded_decisions_only_not_replay_or_physics_proof',mode=mode,calls=len(rows),
        reason_counts=dict(Counter(row['reason'] for row in rows)),
        corner_reason_counts=dict(Counter(row['corner_reason'] for row in rows if row['corner_reason'])),
        selected_count=len(selected),selected=selected,transitions=changes,rows=rows,
        last_time_s=rows[-1]['time_s'],driving_authorized=False)


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):value.update(chunk)
    return value.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    before=digest(args.input)
    with args.input.open(encoding='utf-8') as stream:
        result=summarize(json.loads(line) for line in stream)
    result.update(input_sha256=before,input_unchanged=before==digest(args.input),
                  script_sha256=digest(Path(__file__)))
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','selected','transitions')}))
    return 0 if result['input_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
