"""좌우 피추월 초기 양보 후보의 실제 거절 질의. 입력/출력은 바꾸지 않는 계측이다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/p) for p in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'tests')]
import arena_autonomy.m3_execution_trial as m3
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import depth_support


def one(name):
    path = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run/follow_decisions.jsonl'
    raw = path.read_bytes()
    events = [json.loads(line) for line in raw.decode('utf-8').splitlines()]
    header = events[0]['data']
    ctl = m3.M3ExecutionFollow(FollowConfig(**header['follow_config']),
                             m3.ExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**header['motion_config'])
    expected = json.loads((HERE.parent/'m3_rear_residual_v1/cachefix_runs/run7_premises_cachefix'/
                           f'{name}__plain.json').read_text(encoding='utf-8'))
    original_plan, original_support = m3.plan_check, m3.support
    original_command = ctl._evaluate_command
    scope = {'tag': None, 'sample': False, 'row': None}
    rows = []; checked = 0; differences = []; last_source = None

    def support(scene, frames, queries, cfg, execution):
        result = original_support(scene, frames, queries, cfg, execution)
        if scope['sample'] and scope['tag'] and not result['supported']:
            q = np.asarray(queries).reshape(-1, 2)
            pad = m3.ray_padding(scene, cfg, execution.history_motion_bound_mps)
            missing = q[~depth_support(scene, q, pad)]
            # 계측 비용을 제한하기 위해 새 스캔마다만 실행. 같은 함수로 최종 이력 지지를 재확인한다.
            final = np.asarray([point for point in missing if not original_support(
                scene, frames, point[None, :], cfg, execution)['supported']]).reshape(-1, 2)
            masked = m3.masked_only(scene, final, pad)
            outside_x = np.maximum(np.abs(final[:, 0])-cfg.length_m/2, 0.)
            outside_y = np.maximum(np.abs(final[:, 1])-cfg.width_m/2, 0.)
            scope['row']['observation'].append({'tag': scope['tag'], 'summary': result,
                'final_missing': final.tolist(), 'masked_only_count': int(masked.sum()),
                'max_outside_body_m': float(np.max(np.maximum(outside_x, outside_y), initial=0.)),
                'rear_half_count': int(np.sum(final[:, 0] < 0)),
                'pointwise_agrees': len(final) == result['unsupported']})
        return result

    def plan(scene, frames, candidate, context, cfg, execution, speed):
        previous = scope['tag']
        scope['tag'] = 'yield_plan' if candidate['name'] == 'yield' else None
        try:
            result = original_plan(scene, frames, candidate, context, cfg, execution, speed)
            if scope['sample'] and candidate['name'] == 'yield':
                scope['row']['plan'] = {'delta_m': candidate['delta_m'], 'blend_m': candidate['blend_m'],
                    'target_m': candidate['d_target_m'], 'max_steering_rad': candidate['max_abs_steering_rad'],
                    'required_progress_m': result.get('required_progress_m'), 'result': result,
                    'poses': candidate['poses'].tolist(), 'progress': candidate['progress'].tolist()}
            return result
        finally:
            scope['tag'] = previous

    def command(candidate_name, *args, **kwargs):
        previous = scope['tag']
        scope['tag'] = 'yield_command' if candidate_name == 'yield' else None
        try:
            result = original_command(candidate_name, *args, **kwargs)
            if scope['sample'] and candidate_name == 'yield':
                scope['row']['command'] = result
            return result
        finally:
            scope['tag'] = previous

    m3.plan_check, m3.support, ctl._evaluate_command = plan, support, command
    try:
        for i, event in enumerate(events[1:-1]):
            if event['kind'] == 'reset':
                ctl.reset()
                continue
            data, scan, history = restore_call(event['data']['input'], motion)
            if data['reference_s'] > 5.05:
                break
            scope.update(sample=False, row=None)
            try:
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                fresh = scene.source_first_s != last_source
                last_source = scene.source_first_s
                scope['sample'] = fresh and 3.9 <= data['reference_s'] <= 5.05
                scope['row'] = {'time_s': data['reference_s'], 'source_first_s': scene.source_first_s,
                    'measured_speed_mps': data['speed_mps'], 'previous_steering_rad': data['previous_steering_rad'],
                    'observation': [], 'plan': None, 'command': None}
                out = ctl.command(scene, history, data['speed_mps'], data['side'],
                                  data['previous_steering_rad'], data['dt_s'])
                if scope['sample']:
                    scope['row'].update(output=out, threats=(out[2].get('m3_lateral') or {}).get('threats'))
                    rows.append(freeze(scope['row']))
            except (ValueError, MotionUnavailable) as error:
                ctl.reset()
                out = (0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD',
                    'input_error': str(error), 'mode': 'm3_execution_trial'})
            digest = hashlib.sha256(json.dumps(freeze(out), sort_keys=True).encode()).hexdigest()
            if digest != expected[i]['sha']:
                differences.append(i)
            checked += 1
    finally:
        m3.plan_check, m3.support = original_plan, original_support
    observations = [o for row in rows for o in row['observation']]
    return rows, {'decision_calls': checked, 'output_differences': differences,
        'sampled_scans': len(rows), 'yield_plan_scans': sum(row['plan'] is not None for row in rows),
        'missing_groups': len(observations),
        'all_pointwise_agree': all(o['pointwise_agrees'] for o in observations),
        'all_final_missing_masked': all(len(o['final_missing']) == o['masked_only_count'] for o in observations),
        'input_sha256': hashlib.sha256(raw).hexdigest(), 'input_unchanged': path.read_bytes() == raw}


def main(out):
    if out.exists():
        raise FileExistsError(out)
    sources = list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    data, summaries = {}, {}
    for name in ('run17_observed_release_closed_loop', 'run21_right_encounter'):
        data[name], summaries[name] = one(name)
        print(json.dumps({name: summaries[name]}), flush=True)
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    out.mkdir(parents=True)
    (out/'rows.json').write_text(json.dumps(data)+'\n', encoding='utf-8')
    (out/'summary.json').write_text(json.dumps({'source_sha256': hashes, 'source_unchanged': unchanged,
        'results': summaries, 'scope': 'saved-input early yield queries, no control changes or new driving'},
        indent=2)+'\n', encoding='utf-8')
    return 0 if unchanged and all(not v['output_differences'] and v['input_unchanged'] and
                                 v['all_pointwise_agree'] for v in summaries.values()) else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True)
    sys.exit(main(p.parse_args().output))
