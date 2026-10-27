"""run1 왼쪽과 run2 오른쪽의 같은 항목 대조(읽기 전용). 원본 판정은 바꾸지 않는다.

정답 자세는 report.json·사후 감사 파일의 평가값만 사용한다. 판단 사유 분포와 예측 모서리 횡변위는 판단 기록에서 센다.
대형 원시 파일은 해시만 기록한다.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def decisions(path):
    reasons, corner, alongside_hold = Counter(), [], 0
    with path.open(encoding='utf-8') as stream:
        header = json.loads(stream.readline())['data']
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            detail = event['data']['output'][2]
            reasons[detail.get('reason')] += 1
            trial = detail.get('parallel_hold_trial') or {}
            if detail.get('reason') == 'parallel_hold_follow':
                corner.append(trial.get('maximum_body_corner_lateral_m') or 0.)
    return {'follow_config_parallel': header['follow_config'].get('parallel_hold_trial'),
            'policy': header['follow_config'].get('partial_observation_policy'),
            'reasons': dict(reasons), 'calls': sum(reasons.values()),
            'selected_max_corner_lateral_m': max(corner) if corner else None}


def summary(run_dir, prefix):
    report = load(run_dir/'report.json')
    side = report['additional_actors']['actors']['eval_e3_side']
    encounter = side['encounter']
    evaluation = report['evaluation']
    gap = evaluation.get('gap', {})
    wheel = load(HERE/f'{prefix}_wheel_centers.json')
    front = load(HERE/f'{prefix}_front_target.json')
    replay = load(run_dir/'follow_trace_replay.json')
    return {
        'passed': report['passed'], 'run_valid': report['run_valid'], 'harness_valid': report['harness_valid'],
        'gates': report['gates'], 'config_sha256': report['config_sha256'],
        'actor_overlay_sha256': report['actor_overlay']['sha256'],
        'progress_window_end_m': evaluation['progress']['at_window_end_m'],
        'case_metrics': evaluation['case_metrics'],
        'primary_lead': {'min_sampled_gap_m': gap.get('min_sampled_gap_m'),
                         'min_swept_lower_bound_m': gap.get('min_swept_lower_bound_m'),
                         'overlap_samples': gap.get('sampled_overlap_samples'),
                         'contact_messages': gap.get('actor_contact_messages_with_ego'),
                         'final_gap_m': evaluation['progress'].get('sampled_gap_at_window_end_m')},
        'side_actor': {'min_sampled_gap_m': side['min_sampled_gap_m'],
                       'min_swept_lower_bound_m': side['min_swept_lower_bound_m'],
                       'registered_gap_m': side['minimum_body_gap_m'], 'gates': side['gates'],
                       'behind_observed_s': encounter['behind_observed_s'],
                       'ahead_after_behind_s': encounter['ahead_after_behind_s'],
                       'behind_to_ahead_observed': encounter['behind_to_ahead_observed'],
                       'alongside_samples': encounter['alongside_samples'],
                       'first_below_gap_s': encounter['first_below_gap_s'],
                       'initial_ego_road_lateral_m': encounter['initial_ego_road_lateral_m'],
                       'maximum_ego_road_lateral_change_m': encounter['maximum_ego_road_lateral_change_m']},
        'wall_collision_samples': report['wall_collision_samples'],
        'wheel_centers': {k: wheel[k] for k in ('samples', 'wheel_count_histogram', 'sampled_three_plus_events',
                                                 'minimum_center_to_road_boundary_m', 'max_sample_gap_s')},
        'stop': report['stop'], 'stop_checks': report['stop_checks'],
        'actuation_stop_passed': report['actuation_stop_passed'],
        'shutdown': {k: report['shutdown_audit'][k] for k in ('clean', 'started_child_count', 'clean_child_count',
                                                              'failed_children', 'unaccounted_child_pids')},
        'remaining_test_pids': report['remaining_test_pids'], 'source_unchanged': report['source_unchanged'],
        'parallel_hold_trial': report['parallel_hold_trial'],
        'partial_observation_evidence': report['partial_observation_trial']['evidence'],
        'replay': {k: replay[k] for k in ('calls', 'matched', 'passed') if k in replay},
        'front_target': {k: front[k] for k in ('valid', 'front_actor', 'front_body_gap_m', 'alongside_actors',
                                               'original_primary_gap_m', 'front_gap_within_registered_upper_bound')},
        'decisions': decisions(run_dir/'follow_decisions.jsonl'),
        'raw_sha256': {name: hashlib.sha256((run_dir/name).read_bytes()).hexdigest()
                       for name in ('follow_decisions.jsonl', 'ego_poses.jsonl', 'actor_poses.jsonl')},
    }


left = summary(HERE/'run1_left', 'run1')
right = summary(HERE/'run2_right', 'run2')
out = {'scope': 'run1 left vs run2 right of the default-OFF straight parallel hold trial; single fixed schedule each; '
                'not closed-loop safety proof, curve or full-lap evidence',
       'left': left, 'right': right}
(HERE/'run2_left_right_comparison.json').write_text(json.dumps(out, ensure_ascii=False, indent=1)+'\n',
                                                    encoding='utf-8', newline='\n')
for key in ('passed', 'run_valid', 'progress_window_end_m'):
    print(key, left[key], right[key])
print('case', left['case_metrics'], right['case_metrics'])
print('lead', left['primary_lead'], right['primary_lead'])
print('side', json.dumps(left['side_actor']), '\n     ', json.dumps(right['side_actor']))
print('wall', left['wall_collision_samples'], right['wall_collision_samples'])
print('wheel', left['wheel_centers'], right['wheel_centers'])
print('stop', json.dumps(left['stop']), '\n     ', json.dumps(right['stop']))
print('stop_checks', left['stop_checks'], right['stop_checks'], left['actuation_stop_passed'], right['actuation_stop_passed'])
print('shutdown', left['shutdown'], right['shutdown'], left['remaining_test_pids'], right['remaining_test_pids'])
print('source', left['source_unchanged'], right['source_unchanged'], 'replay', left['replay'], right['replay'])
print('front', left['front_target'], right['front_target'])
print('decisions', left['decisions'], '\n          ', right['decisions'])
print('parallel', left['parallel_hold_trial']['evidence'], right['parallel_hold_trial']['evidence'])
print('partial', left['partial_observation_evidence'], right['partial_observation_evidence'])
