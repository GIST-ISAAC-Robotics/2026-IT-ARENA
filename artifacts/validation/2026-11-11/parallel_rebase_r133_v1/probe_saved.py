"""r130 선택된 평행 경로를 같은 입력/20ms ZOH에서 대조. 미래 입력을 만들지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

from rebase import rebase, supported
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, equal_output, freeze
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import motion_scene

A = Path(__file__).resolve().parent
SOURCE = A.parent/'recent_boundary_r130_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    events = [json.loads(line) for line in SOURCE.open(encoding='utf8')]
    owner, motion = make_controller(events[0]['data'])
    counts = Counter()
    baseline_mismatch = []
    violations = []
    rows = []
    for event in events:
        if event['kind'] != 'call':
            continue
        raw = event['data']
        if 'current_rebase' in raw:
            saved = raw['current_rebase']
            data, scan, history = restore_call(saved['input'], motion)
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                 epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            result = rebase(raw['output'], raw['input'], data, scene, history, owner.config)
            counts['baseline_recorded_current'] += 1
            if not equal_output(freeze(result), saved['output']):
                baseline_mismatch.append(event['seq'])
        normal = raw['output'][2].get('normal') or {}
        if normal.get('selector_choice') != 'parallel' or not supported(raw['output']):
            continue
        counts['parallel_positive_original'] += 1
        for delay in (0., .02):
            data, scan, history = restore_call(raw['input'], motion)
            data['reference_s'] += delay
            try:
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                result = rebase(raw['output'], raw['input'], data, scene, history, owner.config)
            except (MotionUnavailable, ValueError) as error:
                result = dict(speed_mps=0., reason='motion_unavailable', input_error=str(error))
            counts[f'{delay:.2f}s/{result["reason"]}'] += 1
            if result['speed_mps'] > 0:
                if (result['speed_mps'] > raw['output'][0]+1e-10
                        or result['maximum_body_corner_lateral_m'] > .01+1e-10
                        or not 0 <= data['reference_s']-result['path_source_s'] < owner.config.memory_ttl_s):
                    violations.append([event['seq'], delay])
            rows.append(dict(seq=event['seq'], delay_s=delay, source_reference_s=raw['input']['reference_s'],
                original_speed_mps=raw['output'][0], **{k:v for k,v in result.items() if k!='path_points'}))
    report = dict(counts=dict(counts), baseline_mismatch_sequences=baseline_mismatch,
                  contract_violations=violations, trace_sha256=digest,
                  source_unchanged=hashlib.sha256(SOURCE.read_bytes()).hexdigest()==digest,
                  scope='Same saved input and explicit20ms bounded extrapolation; not actual arrival history, live current scan or closed loop')
    with (A/'comparison.json').open('x', encoding='utf8') as stream:
        json.dump(report, stream, indent=2)
    with (A/'details.jsonl').open('x', encoding='utf8') as stream:
        for row in rows:
            stream.write(json.dumps(row)+'\n')
    print(json.dumps(report))
    return 0 if not baseline_mismatch and not violations and report['source_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
