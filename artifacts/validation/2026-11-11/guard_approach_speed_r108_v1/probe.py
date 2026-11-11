"""측방 접근으로 막힌 GUARD의 기존 저속 후보를 고정 입력에서만 비교한다."""
import hashlib
import json
import sys
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy import pass_release_trial as pr
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import freeze, replay_call, equal_output, comparable_decision


def run(source, candidate):
    original = pr.PassReleaseSelector._guard_curve
    active = {}
    attempts = []

    def retry(self, *args, **kwargs):
        result = original(self, *args, **kwargs)
        if result is None or not self.follow_config.traffic_recovery_trial:
            return result
        scene, view, baseline, parallel, speed, previous, dt, age, detail, raw = args
        info = result[2].get('selector_guard_candidate') or {}
        region = info.get('region') or {}
        b = baseline[2]
        partial = b.get('partial_observation') or {}
        target = info.get('target_speed_mps', 0.)
        if (result[2].get('selector_reason') not in ('guard_curve_rejected_toward_left',
                                                    'guard_curve_rejected_toward_right')
                or partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'
                or b.get('boundary_from_memory') is not False
                or b.get('path_source_s') != scene.source_first_s
                or region.get('unobserved') != 0 or region.get('masked_front_samples') != 0
                or target <= .15 or speed >= target):
            return result
        before = result
        checked = []
        for cap in (.25, .15):
            if cap >= target or cap > self.config.guard_speed_cap_mps:
                continue
            lower = self._guard_curve_at_cap(*args, **kwargs, target_cap=cap)
            if lower is None:
                continue
            accepted = lower[0] > 0 and lower[2].get('selector_guard_candidate', {}).get('accepted') is True
            checked.append(dict(cap=cap, accepted=accepted, reason=lower[2].get('selector_reason')))
            if accepted:
                result = lower
                break
        attempts.append(dict(seq=active['seq'], before=list(before[:2]), after=list(result[:2]),
                             checks=checked))
        return result

    if candidate:
        pr.PassReleaseSelector._guard_curve = retry
    rows = [json.loads(line) for line in source.read_text('utf8').splitlines()]
    follow, motion = make_controller(rows[0]['data'])
    mismatches, commands, changes, outputs = [], [], [], []
    try:
        for row in rows[1:]:
            if row['kind'] == 'reset':
                follow.reset()
            elif row['kind'] == 'call':
                active['seq'] = row['seq']
                actual = freeze(replay_call(follow, row['data']['input'], motion))
                outputs.append((row['seq'], actual))
                expected = row['data']['output']
                if not equal_output(comparable_decision(actual), comparable_decision(expected)):
                    mismatches.append(row['seq'])
                if not equal_output(actual[:2], expected[:2]):
                    commands.append(row['seq'])
                    normal = actual[2].get('normal', actual[2])
                    changes.append(dict(seq=row['seq'], reference_s=row['data']['input']['reference_s'],
                        before=expected[:2], after=actual[:2], reason=normal.get('selector_reason', normal.get('reason'))))
    finally:
        pr.PassReleaseSelector._guard_curve = original
    return dict(calls=sum(r['kind']=='call' for r in rows), mismatches=mismatches,
                commands=commands, changes=changes, attempts=attempts), outputs


def main():
    paths = [A.parent/'async_corner_r105_v1/two_actor_run1/run1/follow_decisions.jsonl',
             A.parent/'control_10hz_r91_v1/two_actor_run4/run1/follow_decisions.jsonl']
    results = []
    for source in paths:
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        before, baseline_outputs = run(source, False)
        if 'async_corner_r105' in str(source):
            assert not before['mismatches'], before['mismatches'][:10]
        after, candidate_outputs = run(source, True)
        comparison = []
        for (seq, old), (new_seq, new) in zip(baseline_outputs, candidate_outputs):
            assert seq == new_seq
            if not equal_output(comparable_decision(old), comparable_decision(new)):
                comparison.append(dict(seq=seq, before=old[:2], after=new[:2],
                    command_changed=not equal_output(old[:2], new[:2])))
        assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
        results.append(dict(source=source.relative_to(ROOT).as_posix(), sha256=digest,
                            baseline=before, candidate=after, compared_to_current=comparison))
    with (A/'probe.json').open('x', encoding='utf8') as stream:
        json.dump(results, stream, indent=2)
    print(json.dumps([dict(source=r['source'], calls=r['baseline']['calls'],
        original_baseline_mismatches=len(r['baseline']['mismatches']),
        changes=sum(c['command_changed'] for c in r['compared_to_current']), retries=len(r['candidate']['attempts']),
        zero_to_positive=sum(c['before'][0]==0 and c['after'][0]>0 for c in r['compared_to_current']),
        positive_to_zero=sum(c['before'][0]>0 and c['after'][0]==0 for c in r['compared_to_current'])) for r in results]))


if __name__ == '__main__':
    main()
