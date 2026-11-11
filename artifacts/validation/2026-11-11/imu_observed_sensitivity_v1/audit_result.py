"""저장 결과만 검사. 마지막 연속 정지 직전에만 bias 제거 반사실을 분기한다."""
import copy
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import replay_call, replay_events, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow, FollowConfig

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUN = HERE / 'solo_run1'
OUTPUT = HERE / 'postrun_audit.json'
if OUTPUT.exists():
    raise SystemExit('preserve existing audit')
events = [json.loads(line) for line in (RUN / 'follow_decisions.jsonl').open()]
exact = replay_events(events)  # footer/sequence/기록된 판단까지 먼저 대조
calls = [r for r in events if r['kind'] == 'call']
start = len(calls)-1
while start > 0 and calls[start-1]['data']['output'][0] <= 0:
    start -= 1
start_seq = calls[start]['seq']
stop_s = calls[start]['data']['input']['reference_s']
header = events[0]['data']
assert header['schema'] == 1
motion = MotionConfig(**header['motion_config'])
follow = LocalFollow(FollowConfig(**header['follow_config']))
bias = json.loads((HERE / 'motion.json').read_text())['imu']['bias']
variant = None
outcomes = []
for event in events[1:]:
    kind, data = event['kind'], event['data']
    if kind == 'reset':
        follow.reset()
        if variant is not None:
            variant.reset()
    elif kind == 'call':
        if event['seq'] == start_seq:
            variant = copy.deepcopy(follow)
        if variant is None:
            replay_call(follow, data['input'], motion)
        else:
            # 같은 궤적/속도/과거 조향. 양수 명령이 나와도 새 주행의 증명은 아니다.
            changed = copy.deepcopy(data['input'])
            changed['history']['gyro']['values'] = [v-bias for v in changed['history']['gyro']['values']]
            result = freeze(replay_call(variant, changed, motion))
            outcomes.append(dict(seq=event['seq'], time_s=changed['reference_s'],
                original_speed=data['output'][0], corrected_speed=result[0],
                original_reason=data['output'][2].get('reason'), corrected_reason=result[2].get('reason'),
                corrected_corridor=result[2].get('corridor_candidate')))
            if len(outcomes) >= 30:  # 정지 진입 약0.6초만 확인, 성공할 때까지 확장하지 않음
                break
injection = [json.loads(line) for line in (RUN / 'motion_injection.jsonl').open()]
imu = [r for r in injection if r.get('stream') == 'imu' and r['event'] == 'publish']
stamps = np.array([r['source_stamp_s'] for r in imu])
biases = np.array([r['after'][0]-r['before'][0] for r in imu])
registered = json.loads((HERE / 'registered.json').read_text())
hash_errors = [p for p,h in registered['inputs'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
report = json.loads((RUN/'report.json').read_text())
summary = dict(
    exact_replay=dict(calls=exact['calls'], matched=exact['matched'], passed=exact['passed'],
                      mismatch_sequences=[r['seq'] for r in exact['mismatches']]),
    registration_hash_errors=hash_errors,
    runtime_source_unchanged=report['source_unchanged'],
    gyro_source=dict(count=len(imu), rate_hz=(len(stamps)-1)/(stamps[-1]-stamps[0]),
        interval_min_s=float(np.diff(stamps).min()), interval_max_s=float(np.diff(stamps).max()),
        bias_min_rad_s=float(biases.min()), bias_max_rad_s=float(biases.max()),
        bias_mean_rad_s=float(biases.mean())),
    terminal_zero=dict(first_s=stop_s, last_s=calls[-1]['data']['input']['reference_s'],
        count=len(calls)-start, reasons=dict(Counter(r['data']['output'][2].get('reason') for r in calls[start:]))),
    bias_removal_at_stop=dict(scope='decision-stage counterfactual on recorded trajectory; not corrected-bias driving',
        calls=len(outcomes), positive=sum(r['corrected_speed']>0 for r in outcomes), outcomes=outcomes),
    limitations=['200Hz source is an approximation to 199.068509Hz requested/measured',
        'one combined-IMU trial; no paired Gazebo baseline or hardware proof',
        'current 500-ray simulation path; new native C1 measurement adapter not used'])
OUTPUT.write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='bias_removal_at_stop'}))
print(json.dumps({'counterfactual_calls':len(outcomes),'positive':summary['bias_removal_at_stop']['positive']}))
