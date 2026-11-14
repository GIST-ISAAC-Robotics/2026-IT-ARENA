"""r515 원 로그의 판정·일시 정지·후반 대표만 발췌한다. 재생/제어 호출 없음."""
import collections
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent.parent
RUN = A / 'run1'
OUT = A / 'failure_inputs'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    OUT.mkdir(exist_ok=False)
    report = read(RUN / 'report.json')
    original_hash = sha(RUN / 'report.json')
    first, last = (report['case_window'][k] for k in ('green_sim_s', 'end_sim_s'))
    source = RUN / 'follow_decisions.jsonl'
    raw = source.read_bytes().splitlines(keepends=True)
    rows = [json.loads(line) for line in raw]
    plans = [r for r in rows if r.get('kind') == 'call' and r['data']['compute']['stage'] == 'plan'
             and first <= r['data']['input']['reference_s'] <= last]
    hold_indexes = [i for i, r in enumerate(plans) if r['data']['output'][2].get('reason') == 'moving_pass_current_path_unavailable']
    chosen = set(range(max(0, len(plans) - 5), len(plans)))
    for index in hold_indexes:
        chosen.update(range(max(0, index - 2), min(len(plans), index + 3)))
    ids = {plans[i]['seq'] for i in chosen}
    headers = {'header', 'runtime_evaluator_header', 'runtime_factory_policy',
               'decision_submission_policy', 'current_path_recheck_policy', 'expiry_reference_policy'}
    (OUT / 'selected_records.jsonl').write_bytes(b''.join(line for line, row in zip(raw, rows)
                                                          if row['seq'] in ids or row['kind'] in headers))
    compact = []
    for index in sorted(chosen):
        row = plans[index]
        speed, steer, meta = row['data']['output']
        moving = meta.get('moving_pass', {})
        entry = moving.get('entry', {})
        compact.append(dict(seq=row['seq'], runtime_sequence=row['data']['compute']['sequence'],
            reference_s=row['data']['input']['reference_s'], speed_mps=speed, steering_rad=steer,
            reason=meta.get('reason'), behavior=meta.get('behavior'), entry_reason=entry.get('reason'),
            entry_phase=entry.get('entry_phase'), selected=entry.get('selected'),
            candidate_reasons=dict(collections.Counter(c.get('reason') for c in entry.get('candidates', []))),
            target={k:moving.get('target', {}).get(k) for k in ('observed','fresh','rear_clear','ahead_clear','reason')}))
    reasons = dict(collections.Counter(r['data']['output'][2].get('reason') for r in plans))
    entry_reasons = dict(collections.Counter(str(r['data']['output'][2].get('moving_pass', {}).get('entry', {}).get('reason')) for r in plans))
    value = dict(source='run1/follow_decisions.jsonl', source_sha256=sha(source),
        scope='Observed log extraction only; no controller replay or failure-cause attribution',
        raw_plan_call_rows=len(plans), reason_counts=reasons, entry_reason_counts=entry_reasons,
        former_wall_too_short_count=entry_reasons.get('entry_guide_current_wall_too_short', 0),
        all_nonpositive_plan_rows=sum(r['data']['output'][0] <= 0 for r in plans),
        path_unavailable_plan_rows=len(hold_indexes),
        selected_call_sequences=sorted(ids), records_sha256=sha(OUT / 'selected_records.jsonl'),
        selection='All temporary unavailable plans with adjacent two rows, plus final five plans; original JSONL bytes preserved',
        replay_note='Stateful replay requires original full prefix; this excerpt is not an isolated replay fixture',
        raw_failed_gates=[k for k,v in report['gates'].items() if not v],
        raw_road=report['evaluation']['road'], summaries=compact,
        original_report_sha256=original_hash, original_report_unchanged=sha(RUN / 'report.json') == original_hash,
        controller_reexecuted=False, actual_run_count=1)
    with (OUT / 'summary.json').open('x', encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps({k:value[k] for k in ('raw_plan_call_rows','reason_counts','entry_reason_counts',
        'former_wall_too_short_count','path_unavailable_plan_rows','raw_failed_gates')},ensure_ascii=False))


if __name__ == '__main__':
    main()
