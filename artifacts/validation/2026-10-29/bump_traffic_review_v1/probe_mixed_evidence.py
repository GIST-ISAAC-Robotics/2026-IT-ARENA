"""저장된 승인 정합의 점별 재관측 OR 빈 위치 확인 대조. 주행 연결 없음."""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
BUMP = BASE.parent.parent/'2026-10-28/bump_traffic_v1'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    source = BUMP/'run3/follow_decisions.jsonl'
    before = sha(source)
    rows=[];releases=[];count=0;last_source=None;previous_key=None
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            data=event['data'];now=data['input']['reference_s']
            if now<133.8:continue
            if now>135.3:break
            partial=data['output'][2].get('partial_observation') or {}
            anchor=partial.get('anchor_diagnostic') or {}
            clearance=partial.get('clearance_diagnostic') or {}
            rolling=clearance.get('rolling_clearance') or {}
            if anchor.get('released'):
                releases.append(dict(t=now,kind='original_reobservation',
                    points=(anchor.get('threat_set') or {}).get('unique_points')))
            observed=anchor.get('observed')
            proofs=rolling.get('proofs')
            threat=anchor.get('threat_set') or {}
            key=(threat.get('reference_s'),threat.get('source_first_s'))
            if key!=previous_key:
                count=0;last_source=None;previous_key=key
            scan=data['input']['scan'];stamp=(scan['sec'],scan['nanosec'])
            if stamp==last_source:continue
            last_source=stamp
            if (not isinstance(observed,list) or not isinstance(proofs,list)
                    or len(observed)!=len(proofs) or len(observed)!=threat.get('unique_points')):
                count=0
                if anchor:rows.append(dict(t=now,anchor=anchor.get('reason'),
                    rolling=rolling.get('reason'),available=False,mixed_fresh_count=count))
                continue
            clear=[p['clear'] for p in proofs]
            mixed=[a or b for a,b in zip(observed,clear)]
            count=count+1 if all(mixed) else 0
            rows.append(dict(t=now,available=True,original_reference_s=key[0],
                observed=observed,clear=clear,all_reobserved=all(observed),all_clear=all(clear),
                per_point_accounted=mixed,all_mixed=all(mixed),mixed_fresh_count=count,
                mixed_two_fresh_candidate=count>=2,
                query_counts=[dict(supported=p['supported_queries'],required=p['queries'],occupied=p['occupied']) for p in proofs]))
    result=dict(scope='read-only recorded geometry; per-point evidence combination; no controller connection or new driving',
        original_releases=releases,rows=rows,input_sha256=before,input_unchanged=sha(source)==before,
        source_sha256=sha(Path(__file__)),two_fresh_candidate_rows=sum(r.get('mixed_two_fresh_candidate',False) for r in rows))
    with (BASE/'mixed_evidence_probe.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=True))


if __name__=='__main__':main()
