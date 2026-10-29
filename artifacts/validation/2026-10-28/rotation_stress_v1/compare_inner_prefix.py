"""이전 명령 동일 재현과 새 짧은 안쪽 지지/독립 보호의 저장 입력 대조."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT / 'src/arena_autonomy'), str(ROOT / 'src/arena_vehicle_interface'),
    str(BASE.parent / 'onroad_candidate_v1'), str(BASE.parent / 'traffic_residual_v1')]
from compare_saved import preflight, sha
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import replay_call, thaw, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
import arena_autonomy.corridor_observer as observer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    header, expected = preflight(args.trace)
    config, motion = FollowConfig(**header['follow_config']), MotionConfig(**header['motion_config'])
    assert config.short_boundary_prefix_enabled and config.traffic_recovery_trial
    args.output.mkdir(exist_ok=False)
    before = sha(args.trace)
    paths = [p for base in ('src/arena_autonomy', 'src/arena_vehicle_interface')
        for p in (ROOT / base).rglob('*.py') if '__pycache__' not in p.parts]
    paths += [Path(__file__), BASE.parent / 'traffic_residual_v1/safety_shadow.py',
        BASE.parent / 'onroad_candidate_v1/compare_saved.py']
    sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}
    with zipfile.ZipFile(args.output / 'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in paths:
            archive.write(p, p.relative_to(ROOT).as_posix())
    (args.output / 'parameters.json').write_text(json.dumps(dict(header=header,
        trace=str(args.trace), input_sha256=before, source_sha256=sources,
        old_branch='only short_inner_prefix_evidence disabled; all recorded outputs compared'),
        ensure_ascii=False, indent=2), encoding='utf-8')
    controllers = {name: LocalFollow(config) for name in ('old', 'candidate')}
    guards = {}
    statistics = Counter()
    reasons = {name: Counter() for name in controllers}
    actual = observer.short_inner_prefix_evidence
    calls = 0
    with args.trace.open(encoding='utf-8') as stream, (args.output / 'decisions.jsonl').open('x', encoding='utf-8') as sink:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'reset':
                for controller in controllers.values():
                    controller.reset()
                guards.clear()
            if event['kind'] != 'call':
                continue
            calls += 1
            data = thaw(event['data']['input'])
            outputs = {}
            for name, controller in controllers.items():
                observer.short_inner_prefix_evidence = ((lambda geometry, _: geometry) if name == 'old' else actual)
                result = replay_call(controller, data, motion)
                if name == 'old':
                    recorded = comparable_decision(thaw(event['data']['output']))
                    assert equal_output(comparable_decision(freeze(result)), recorded), (event['seq'], data['reference_s'])
                    statistics['old_recorded_matches'] += 1
                if name not in guards:
                    guards[name] = ShadowSafety(RecoveryPolicy(True, True, True), config, data['rear_half_angle_deg'])
                    assert guards[name].node.partial_policy is not controller.partial_policy
                safe, status = guards[name].step(data, motion, result[:2])
                assert 0 <= safe[0] <= result[0] + 1e-12
                detail = result[2]
                outputs[name] = dict(command=list(result[:2]), safe=list(safe), reason=detail.get('reason'),
                    safety_reason=status.get('reason'), corridor_selected=detail.get('corridor_selected'),
                    short_prefix=(detail.get('corridor_candidate') or {}).get('short_prefix'))
                reasons[name][str(detail.get('reason'))] += 1
            old, new = outputs['old']['safe'], outputs['candidate']['safe']
            statistics['changed_commands'] += int(any(abs(a-b) > 1e-10 for a, b in zip(old, new)))
            statistics['restored_positive'] += int(old[0] == 0 and new[0] > 0)
            statistics['suppressed_positive'] += int(old[0] > 0 and new[0] == 0)
            statistics['candidate_positive'] += int(new[0] > 0)
            sink.write(json.dumps(dict(seq=event['seq'], t=data['reference_s'], outputs=outputs), allow_nan=False) + '\n')
    observer.short_inner_prefix_evidence = actual
    assert calls == expected and sha(args.trace) == before
    assert all(sha(ROOT / p) == digest for p, digest in sources.items())
    result = dict(scope='stateful fixed-input comparison; no new driving or physical recovery proof',
        calls=calls, counts=dict(statistics), reasons={k:dict(v) for k,v in reasons.items()},
        input_sha256=before, input_unchanged=True, source_unchanged=True)
    (args.output / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
