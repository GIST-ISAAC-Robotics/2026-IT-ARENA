"""저장 입력 정책 비교(shadow_v0/v1)를 집계한다. 새 주행 아님.

평가용 정답 분류(features1의 ego/actor 자세)는 사후 해석에만 쓴다. 감속 '제안'은
저장 궤적에서 계산한 명령 차이이며 실제 주행 시간 손실이나 이득이 아니다.
"""
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import statistics

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
RUN3 = ROOT/'artifacts/validation/2026-10-29/g2_handoff_course_shadow_v1/run3'


def load(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def label(feature):
    """조각 중앙값과 평가 상자의 거리 0.10 m 이내면 해당 actor, 아니면 비행위자."""
    if feature is None or feature.get('face_x') is None:
        return 'no_current_face'
    gaps = {k: v['face_gap'] for k, v in feature['truth'].items()}
    nearest = min(gaps, key=gaps.get)
    if gaps[nearest] <= .10:
        return nearest+('_moving' if feature['truth'][nearest]['speed'] > .05 else '_still')
    return 'non_actor'


def capped(row):
    return row['reason'] == 'handoff_follow' and row['requested'][0] < row['expected'][0]-1e-8


def intervals(rows):
    spans = []
    for row in rows:
        if not spans or row['reference_s']-spans[-1][-1]['reference_s'] > .031:
            spans.append([row])
        else:
            spans[-1].append(row)
    return spans


def main():
    run3 = load(RUN3/'rows.jsonl'); v0 = load(BASE/'shadow_v0/rows.jsonl'); v1 = load(BASE/'shadow_v1/rows.jsonl')
    features = {row['seq']: row for row in load(BASE/'features1/features.jsonl')}
    r0 = json.loads((BASE/'shadow_v0/report.json').read_text(encoding='utf-8'))
    r1 = json.loads((BASE/'shadow_v1/report.json').read_text(encoding='utf-8'))
    # 기존 정책 재현: run3와 호출별 출력·사유가 같아야 한다.
    same = sum(a['seq'] == b['seq'] and a['requested'] == b['requested'] and a['reason'] == b['reason']
               and a['persistence'] == b['persistence'] for a, b in zip(run3, v0))
    by_seq1 = {row['seq']: row for row in v1}
    legacy_capped = [row for row in v0 if capped(row)]
    new_capped = [row for row in v1 if capped(row)]
    removed = Counter(); removed_label = defaultdict(Counter)
    for row in legacy_capped:
        new = by_seq1[row['seq']]
        if capped(new):
            continue
        reason = (new.get('preparation') or {}).get('reason', new['reason'])
        removed[reason] += 1
        removed_label[reason][label(features.get(row['seq']))] += 1
    remaining = []
    for span in intervals(new_capped):
        first = span[0]; feature = features.get(first['seq'])
        labels = Counter(label(features.get(row['seq'])) for row in span)
        remaining.append(dict(first_reference_s=first['reference_s'], last_reference_s=span[-1]['reference_s'],
            calls=len(span), truth_labels=dict(labels),
            lead_truth_speed_mps=[round(features[row['seq']]['truth']['lead']['speed'], 3) for row in span[::max(1, len(span)//5)]],
            low_motion_scans=[(row.get('persistence') or {}).get('sample_count') for row in span[::max(1, len(span)//5)]],
            g1_mps=[round(row['expected'][0], 3) for row in span[::max(1, len(span)//5)]],
            reduction_sum_mps=sum(row['expected'][0]-row['requested'][0] for row in span)))
    def total(rows):
        return dict(calls=len(rows), speed_reduction_sum_mps=sum(r['expected'][0]-r['requested'][0] for r in rows),
            reduction_call_s_equivalent_m=.02*sum(r['expected'][0]-r['requested'][0] for r in rows),
            truth_labels=dict(Counter(label(features.get(r['seq'])) for r in rows)))
    observed = [row for row in v1 if (row.get('persistence') or {}).get('observed')]
    result = dict(
        inputs=dict(run3_rows_sha256=hashlib.sha256((RUN3/'rows.jsonl').read_bytes()).hexdigest(),
            shadow_v0_rows_sha256=hashlib.sha256((BASE/'shadow_v0/rows.jsonl').read_bytes()).hexdigest(),
            shadow_v1_rows_sha256=hashlib.sha256((BASE/'shadow_v1/rows.jsonl').read_bytes()).hexdigest(),
            features_sha256=hashlib.sha256((BASE/'features1/features.jsonl').read_bytes()).hexdigest()),
        legacy_reproduces_run3=dict(calls=len(v0), identical_calls=same, passed=same == len(run3) == len(v0)),
        normal_detail=dict(v0=r0['counts'].get('normal_detail_matched'), v1=r1['counts'].get('normal_detail_matched'),
            calls=r1['counts']['calls']),
        proposals=dict(v0=len(r0['proposals']), v1=len(r1['proposals']), v1_detail=r1['proposals']),
        reverse_hold=dict(v0=r0['counts'].get('handoff_reverse_motion_hold'), v1=r1['counts'].get('handoff_reverse_motion_hold')),
        legacy_capped=total(legacy_capped), new_capped=total(new_capped),
        legacy_capped_removed_by=dict(removed), legacy_capped_removed_by_label={k: dict(v) for k, v in removed_label.items()},
        new_preparation_reasons_all_calls=r1['preparation_reasons'],
        observed_patch_calls_v1=len(observed),
        remaining_new_capped_intervals=remaining,
        processing_ms=dict(v0=r0['processing_ms'], v1=r1['processing_ms']),
        label_rule='face median within 0.10 m of evaluation actor box (lead=actor_poses, side=eval_e3_side); speed>0.05 m/s moving. Post-hoc only.',
        scope='Stored-trajectory command proposals only; not applied, not a driving-time result.')
    (BASE/'shadow_comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('remaining_new_capped_intervals', 'proposals', 'inputs')}, indent=2))
    print('remaining intervals', len(remaining))
    for item in remaining[:30]:
        print(json.dumps(item))


if __name__ == '__main__':
    main()
