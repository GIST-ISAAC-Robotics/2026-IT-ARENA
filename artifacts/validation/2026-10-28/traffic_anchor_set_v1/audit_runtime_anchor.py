"""완료된 주행의 재관측과 별도 보호층 진단을 읽는다. 원시 판정을 바꾸지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def summarize(rows):
    reasons, latches, releases = Counter(), {}, []
    for now, diagnostic, command in rows:
        if not isinstance(diagnostic, dict):
            continue
        anchor = diagnostic.get('anchor_diagnostic') or {}
        if not anchor:
            continue
        reasons[str(anchor.get('reason'))] += 1
        evidence = anchor.get('threat_set') or {}
        key = diagnostic.get('latch_time_s')
        if key is not None:
            item = latches.setdefault(str(key), dict(first_seen_s=now, last_seen_s=now,
                pairs=evidence.get('pairs'), unique_points=evidence.get('unique_points'),
                source_first_s=evidence.get('source_first_s'), reference_s=evidence.get('reference_s')))
            item['last_seen_s'] = now
        if anchor.get('released'):
            observed = anchor.get('observed') or []
            releases.append(dict(time_s=now, command_mps=command, all_points_observed=bool(observed) and all(observed),
                consecutive_fresh_sources=anchor.get('consecutive_fresh_sources'),
                current_recheck_reason=anchor.get('current_recheck_reason'), threat_set=evidence,
                nearest_distances_m=anchor.get('nearest_distances_m')))
    return dict(reasons=dict(reasons), latches=latches, releases=releases)


def analyze(run):
    trace_path, decisions_path = run / 'trace.json', run / 'follow_decisions.jsonl'
    before = {p.name: sha(p) for p in (trace_path, decisions_path)}
    def decisions():
        with decisions_path.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('kind') == 'call':
                    data = row['data']
                    yield data['input']['reference_s'], data['output'][2].get('partial_observation'), data['output'][0]
    controller = summarize(decisions())
    trace = json.loads(trace_path.read_text(encoding='utf-8'))
    safety = summarize((r['t'], (r.get('partial_observation_trial') or {}).get('safety'),
                        r.get('safe_speed_mps')) for r in trace)
    return dict(controller_decision_stage=controller, safety_status_samples=safety,
        source_sha256=before, inputs_unchanged=all(sha(run / p) == digest for p, digest in before.items()),
        scope='controller calls versus asynchronous status samples; safety releases may not appear in sampled status; not object identity proof')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = analyze(args.run)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({key: dict(reasons=value['reasons'], releases=len(value['releases']))
          for key, value in result.items() if key in ('controller_decision_stage', 'safety_status_samples')}))
