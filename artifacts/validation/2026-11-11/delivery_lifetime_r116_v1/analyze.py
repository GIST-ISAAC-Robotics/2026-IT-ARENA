"""r114 원본을 바꾸지 않는 입력 무결성/명령 수명 분리 진단."""
from bisect import bisect_right
import collections
import hashlib
import json
from pathlib import Path

from native_traffic_contract import NativeSamplingAudit

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
RUN = A.parent / 'fresh_submission_r114_v1/two_actor_run1/run1'


def main():
    files = ('report.json', 'acquisition.json', 'scans_received.jsonl',
             'follow_decisions.jsonl', 'trajectory.jsonl')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    hashes = {name: sha(RUN / name) for name in files}
    report = json.loads((RUN / 'report.json').read_bytes())
    acquired = json.loads((RUN / 'acquisition.json').read_bytes())
    read_rows = lambda name: [json.loads(s) for s in (RUN / name).open(encoding='utf8')]
    received = read_rows('scans_received.jsonl')
    calls = [r['data'] for r in read_rows('follow_decisions.jsonl') if r['kind'] == 'call']
    trajectory = read_rows('trajectory.jsonl')
    profile = json.loads((ROOT / 'config/tests/c1_standard10_native_angles_v1.json').read_bytes())
    audit = NativeSamplingAudit(profile, 10.)
    # 이 사본은 누적 유실 횟수와 개별 완성 배열의 정확성을 분리하는 데만 사용한다.
    # 원 report의 verified=false, 유실 실패, 원시 파일은 변경하지 않는다.
    payload_rows = [{**r, 'discarded_source_gaps': 0} for r in acquired]
    payload = audit.audit(payload_rows, received)
    previous = 0
    changes = []
    for i, row in enumerate(acquired):
        current = row['discarded_source_gaps']
        if current != previous:
            changes.append(dict(index=i, next_complete_first_s=row['first_ray_stamp_s'],
                previous_complete_last_s=acquired[i-1]['last_ray_stamp_s'] if i else None,
                old_count=previous, new_count=current))
            previous = current

    candidates = [c for c in calls if c['delivery']['kind'] == 'candidate']
    apply = [c['input']['scan']['measurements']['acquired_at_s'][0] +
             c['delivery']['source_age_s'] for c in candidates]
    publish = [r['source_stamp_at_publish_s'] for r in acquired]
    green = report['green_observed_sim_s']
    end = report['case_window']['end_sim_s']
    categories = collections.Counter()
    examples = []
    for row in trajectory:
        t = row['t']
        if not green <= t <= end or row.get('safety_reason') != 'stale_input':
            continue
        i, j = bisect_right(apply, t) - 1, bisect_right(publish, t) - 1
        if i < 0 or j < 0:
            categories['before_first_candidate_or_scan'] += 1
            continue
        reference_age = t - candidates[i]['input']['reference_s']
        source_age = t - acquired[j]['first_ray_stamp_s']
        key = ('command_and_scan_age' if reference_age > .2 and source_age > .3 else
               'command_age_only' if reference_age > .2 else
               'scan_age_only' if source_age > .3 else 'not_explained_by_reconstructed_sim_ages')
        categories[key] += 1
        if len(examples) < 5:
            examples.append(dict(t=t, category=key, candidate_reference_age_s=reference_age,
                latest_published_scan_age_s=source_age, drive_speed=row.get('drive_speed_mps'),
                safe_speed=row.get('safe_speed_mps')))
    result = dict(original_native_verified=report['native_acquisition']['verified'],
        completed_payload_comparison_ignoring_cumulative_loss_counter=payload,
        source_gap_counter_changes=changes, stale_sample_reconstruction=dict(categories),
        examples=examples, source_sha256=hashes,
        source_unchanged=all(sha(RUN/k) == v for k, v in hashes.items()),
        scope='Saved diagnosis only. Payload match is not loss-free acquisition. Reconstructed candidate/publication ages are not guard callback receipt times; unexplained samples remain unresolved. Original trial remains failed.')
    with (A / 'analysis.json').open('x', encoding='utf8') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
