"""완료 입력의 첫 후반 래치·스캔별 대응을 추출한다. 판단 재실행은 아니다."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def extract(trace, output):
    if output.exists():
        raise FileExistsError(output)
    before = digest(trace)
    calls, header = [], None
    with trace.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'header':
                header = row['data']
            if row['kind'] == 'call' and 130 <= row['data']['input']['reference_s'] <= 136:
                calls.append(row['data'])
    first = next(row for row in calls if (row['output'][2].get('partial_observation') or {}).get('latch_evidence'))
    evidence = first['output'][2]['partial_observation']['latch_evidence']
    summary, last_source = [], None
    for row in calls:
        data, result = row['input'], row['output']
        if data['reference_s'] < evidence['reference_s']:
            continue
        scan = data['scan']
        source = (scan['sec'], scan['nanosec'])
        if source == last_source:
            continue
        last_source = source
        partial = result[2].get('partial_observation') or {}
        anchor = partial.get('anchor_diagnostic') or {}
        clear = partial.get('clearance_diagnostic') or {}
        summary.append(dict(t=data['reference_s'], source=source, command=result[:2],
            reason=result[2].get('reason'), partial_reason=partial.get('reason'),
            unknown=partial.get('unknown_samples'),
            anchor={k: anchor.get(k) for k in ('reason','matched','consecutive_fresh_sources',
                'corrected_points','nearest_distances_m','observed','fit')},
            clearance={k:v for k,v in clear.items() if k not in ('scene','points')}))
    brief_evidence = {k:v for k,v in evidence.items() if k != 'correspondence_context'}
    output.mkdir()
    (output/'summary.json').write_text(json.dumps(dict(scope='recorded output extraction; not replay or new motion',
        input_sha256=before,input_unchanged=digest(trace)==before,
        evidence=brief_evidence,rows=summary),ensure_ascii=False,indent=2),encoding='utf-8')
    # 상세 좌표와 입력은 로컬 보존하고 공개 요약에 점군 전체를 펼치지 않는다.
    (output/'fixture.json').write_text(json.dumps(dict(header=header,evidence=evidence,calls=calls),
        ensure_ascii=False),encoding='utf-8')
    print(json.dumps(dict(output=str(output),first_latch_s=evidence['reference_s'],
        witness_pairs=evidence['pairs'],scans=len(summary),input_unchanged=digest(trace)==before)))


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('trace',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    extract(args.trace,args.output)
