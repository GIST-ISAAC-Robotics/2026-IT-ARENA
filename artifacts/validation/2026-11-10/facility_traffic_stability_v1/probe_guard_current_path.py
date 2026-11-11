"""저장 입력에서 평행 후보의 큰 횡이동 거절과 현재 G1 경로 검사를 분리한다. 런타임 소스 불변."""
import argparse
import copy
import hashlib
import json
import math
import sys
from collections import Counter
from contextlib import contextmanager
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / 'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call
import arena_autonomy.pass_release_trial as ps

ORIGINAL = ps.PassReleaseSelector._guard_curve
CAPTURE = {}


def eligible(ctl, scene, baseline, parallel):
    detail = baseline[2]
    return (parallel[2].get('reason') == 'parallel_hold_lateral_motion_not_small'
            and ctl.follow_config.traffic_recovery_trial
            and detail.get('boundary_from_memory') is False
            and detail.get('path_source_s') == scene.source_first_s)


@contextmanager
def variant(name):
    def probe(ctl, scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan):
        use = name == 'current_path' and eligible(ctl, scene, baseline, parallel)
        examined = parallel
        if use:
            examined = (parallel[0], parallel[1], {
                **parallel[2], 'reason': 'parallel_hold_reference_not_straight'})
        out = ORIGINAL(ctl, scene, view, baseline, examined, speed, previous, dt, age, detail, raw_scan)
        if use and out is not None:
            if out[2].get('reason') == 'parallel_hold_reference_not_straight':
                out[2]['reason'] = parallel[2]['reason']
            if 'selector_guard_candidate' in out[2]:
                out[2]['selector_guard_candidate']['trigger'] = parallel[2]['reason']
        CAPTURE.update(eligible=eligible(ctl, scene, baseline, parallel),
                       baseline=list(baseline[:2]), baseline_reason=baseline[2].get('reason'),
                       parallel=list(parallel[:2]), parallel_reason=parallel[2].get('reason'),
                       measured_speed_mps=speed, previous_steer_rad=previous,
                       sides={k: dict(v) for k,v in ctl.sides.items()},
                       observed_sides={k: dict(view[k]) for k,_ in ps.SIDES} if view else None,
                       candidate=freeze(out[2].get('selector_guard_candidate')) if out else None)
        return out
    ps.PassReleaseSelector._guard_curve = probe
    try:
        yield
    finally:
        ps.PassReleaseSelector._guard_curve = ORIGINAL


def command(output):
    d = output[2]
    normal = d.get('normal') or d
    return dict(command=output[:2], behavior=d.get('behavior'), reason=d.get('reason'),
                normal_reason=normal.get('reason'), selector_reason=normal.get('selector_reason'),
                selector_choice=normal.get('selector_choice'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    sources = [args.input, Path(ps.__file__), Path(__file__)]
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    hashes = {str(p): digest(p) for p in sources}
    controllers, counts, changes, faults = {}, Counter(), [], {}
    with (args.output/'rows.jsonl').open('w', encoding='utf8') as stream:
        for event in read_events(args.input):
            data = event['data']
            if event['kind'] == 'header':
                for name in ('baseline', 'current_path'):
                    controllers[name], motion = build_controller(data)
            elif event['kind'] == 'reset':
                for ctl in controllers.values(): ctl.reset()
            elif event['kind'] == 'call':
                row = {'t': data['input']['reference_s']}
                for name, ctl in controllers.items():
                    inspect_fault = (name == 'current_path' and not faults
                                     and row['baseline']['command'][0] <= 1e-6
                                     and row['baseline']['probe'].get('eligible'))
                    before = copy.deepcopy(ctl) if inspect_fault else None
                    CAPTURE.clear()
                    with variant(name): out = freeze(replay_call(ctl, data['input'], motion))
                    row[name] = {**command(out), 'probe': dict(CAPTURE)}
                    counts[name+'_zeros'] += out[0] <= 1e-6
                    if name == 'baseline':
                        counts['baseline_mismatches'] += not equal_output(
                            comparable_decision(out), comparable_decision(data['output']))
                    elif (inspect_fault and out[0] > .1 and CAPTURE.get('eligible')):
                        for fault in ('front_obstacle', 'front_missing'):
                            inp = copy.deepcopy(data['input']); scan = inp['scan']; affected = 0
                            for i in range(len(scan['ranges'])):
                                angle = scan['angle_min']+i*scan['angle_increment']
                                if abs(angle) < math.radians(35):
                                    scan['ranges'][i] = .07 if fault == 'front_obstacle' else {'nonfinite': 'nan'}
                                    affected += 1
                            with variant(name): answer = freeze(replay_call(copy.deepcopy(before), inp, motion))
                            faults[fault] = dict(t=row['t'], changed_rays=affected, **command(answer))
                counts['calls'] += 1
                if not equal_output(row['baseline']['command'], row['current_path']['command']):
                    counts['command_changes'] += 1
                    changes.append(row)
                stream.write(json.dumps(row, ensure_ascii=False)+'\n')
    summary = dict(scope='saved sensor inputs; no vehicle dynamics response; production source unchanged',
                   input=str(args.input), counts=dict(counts), faults=faults, first_changes=changes[:8],
                   source_sha256=hashes, source_unchanged=all(digest(Path(p)) == h for p,h in hashes.items()))
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('first_changes','source_sha256')}, ensure_ascii=False))


if __name__ == '__main__': main()
