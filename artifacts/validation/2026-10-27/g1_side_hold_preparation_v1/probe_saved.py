"""좌우 피추월 저장 입력에서 현행 G1의 판단을 비교한다. 새 주행·옆차 보호 허가가 아니다."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
CASES = {
    'left': 'artifacts/validation/2026-10-25/m3_execution_review_v1/run17_observed_release_closed_loop/run/follow_decisions.jsonl',
    'right': 'artifacts/validation/2026-10-25/m3_execution_review_v1/run21_right_encounter/run/follow_decisions.jsonl',
}


def lateral_motion(speed, age, steering, previous, dt, wheelbase=.135):
    """현재/직전각의 반응+제동 구간에서 네 차체 모서리의 횡변위. 관측 증명은 아니다."""
    distance = abs(speed)*(max(0., age)+.1+dt)+speed*speed/4
    samples = np.linspace(0., distance, max(2, math.ceil(distance/.02)+1))
    maximum = 0.
    for angle in (steering, previous):
        k = math.tan(angle)/wheelbase
        yaw = samples*k
        rear_y = (1-np.cos(yaw))/k if abs(k)>1e-9 else np.zeros_like(yaw)
        center_y = rear_y+wheelbase/2*np.sin(yaw)
        for x in (-.1, .1):
            for y in (-.075, .075):
                maximum = max(maximum, float(np.max(abs(center_y+x*np.sin(yaw)+y*np.cos(yaw)-y))))
    return maximum


def one(name, relative):
    source = ROOT/relative
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    counters = {p: Counter() for p in ('strict', 'bounded_local_v4')}
    positive, threat_positive, costs = Counter(), Counter(), {p: [] for p in counters}
    threat_reasons = {p: Counter() for p in counters}
    rows = HERE/f'{name}_rows.jsonl'
    controllers = {}
    calls = 0
    last_seq = -1
    footer = None
    with rows.open('x', encoding='utf-8') as out, source.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            assert event['seq'] == last_seq+1
            last_seq = event['seq']
            data = event['data']
            if event['kind'] == 'header':
                assert data['controller_mode'] == 'm3_execution_trial'
                motion = MotionConfig(**data['motion_config'])
                for policy in counters:
                    controllers[policy] = LocalFollow(FollowConfig(**{
                        **data['follow_config'], 'partial_observation_policy': policy}))
            elif event['kind'] == 'reset':
                for ctl in controllers.values():
                    ctl.reset()
            elif event['kind'] == 'footer':
                footer = data
            elif event['kind'] == 'call':
                calls += 1
                old = data['output']
                threats = (old[2].get('m3_lateral') or {}).get('threats') or {}
                observed_threat = any(v.get('present') for v in threats.values())
                result = {'seq': event['seq'], 'reference_s': data['input']['reference_s'],
                          'stored_m3_output': old[:2], 'stored_sensor_threat': observed_threat}
                for policy, ctl in controllers.items():
                    start = time.perf_counter()
                    output = replay_call(ctl, data['input'], motion)
                    costs[policy].append(1e3*(time.perf_counter()-start))
                    detail = output[2]
                    counters[policy][detail['reason']] += 1
                    positive[policy] += int(output[0]>0)
                    if observed_threat:
                        threat_reasons[policy][detail['reason']] += 1
                        threat_positive[policy] += int(output[0]>0)
                    age = data['input']['reference_s']-detail.get('source_first_s', data['input']['reference_s'])
                    result[policy] = {'command': output[:2], 'reason': detail['reason'],
                        'clearance_m': detail.get('swept_clearance_m'),
                        'target_before_hold_mps': detail.get('target_speed_before_hold_mps'),
                        'maximum_corner_lateral_m': lateral_motion(
                            max(abs(data['input']['speed_mps']), output[0]), age, output[1],
                            data['input']['previous_steering_rad'], data['input']['dt_s'])}
                out.write(json.dumps(result, allow_nan=False)+'\n')
    assert footer and footer['capture_errors'] == footer['failed'] == footer['dropped'] == footer['pending'] == 0
    assert footer['worker_alive'] is False and footer['written'] == footer['accepted'] == last_seq
    return {'calls': calls, 'reasons': {p: dict(c) for p,c in counters.items()},
        'positive': dict(positive), 'positive_during_stored_threat': dict(threat_positive),
        'reasons_during_stored_threat': {p:dict(c) for p,c in threat_reasons.items()},
        'maximum_ms': {p:max(v) for p,v in costs.items()},
        'input_sha256': before, 'input_unchanged': before == hashlib.sha256(source.read_bytes()).hexdigest()}


if __name__ == '__main__':
    target = HERE/'run1_probe.json'
    if target.exists():
        raise FileExistsError(target)
    files = sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    hashes = lambda: {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    before = hashes()
    results = {name: one(name, path) for name,path in CASES.items()}
    result = {'scope': 'G1_on_saved_M3_sensor_inputs_not_original_M3_replay_or_new_trajectory',
        'source_sha256': before, 'source_unchanged': before == hashes(), 'results': results,
        'limits': ['stored threat tags select analysis rows only, never a controller input',
            'commands diverge from the trajectory that generated later inputs',
            'lateral motion is sampled Ackermann geometry, not observed empty rear space',
            'no independent protection, actuation or new physical outcome is replayed']}
    with target.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result['results'], indent=2))
