"""저장 입력의 현행 출력 동일성과 복귀 출력 외곽 진단. 운영 연결/폐루프가 아니다."""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OLD = HERE.parent/'m3_rear_residual_v1'
sys.path[:0] = [str(ROOT/p) for p in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts', 'tests')]
from arena_autonomy.follow_trace import restore_call, freeze, controller_mode
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig, SceneHistory
from arena_autonomy.m3_return_guard import assess_return_command, ReturnGuardAudit


def source_hashes():
    files = sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def one(path_text):
    path = ROOT/path_text
    name = path.parent.parent.name
    raw = path.read_bytes()
    events = [json.loads(line) for line in raw.decode('utf-8').splitlines()]
    header = events[0]['data']
    ctl = M3ExecutionFollow(FollowConfig(**header['follow_config']),
        ExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**header['motion_config'])
    baseline_call = ctl.base.command
    captured = {}
    def capture(*args, **kwargs):
        out = baseline_call(*args, **kwargs)
        captured['output'] = out
        return out
    ctl.base.command = capture
    recent = SceneHistory()
    audit = ReturnGuardAudit()
    expected = json.loads((OLD/'cachefix_runs/run7_premises_cachefix'/f'{name}__plain.json').read_text())
    rows, comparisons, previous_state = [], [], None
    ever_threat = False
    eligible = {'local_follow', 'follow_gap_hold', 'follow_release_wait', 'local_follow_corridor_candidate'}
    for index, event in enumerate(events[1:-1]):
        if event['kind'] == 'reset':
            ctl.reset(); recent.reset(); audit.reset(); ever_threat = False
            comparisons.append(expected[index] == {'reset': True})
            previous_state = None
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        captured.clear()
        scene = None
        try:
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            frames = recent.update(scene, history) if ctl.execution.masked_history_support else []
            out = ctl.command(scene, history, data['speed_mps'], data['side'],
                data['previous_steering_rad'], data['dt_s'])
        except (MotionUnavailable, ValueError) as error:
            ctl.reset(); recent.reset()
            out = (0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD',
                            'input_error': str(error), 'mode': controller_mode(ctl)})
        frozen = freeze(out)
        digest = hashlib.sha256(json.dumps(frozen, sort_keys=True).encode()).hexdigest()
        comparisons.append(digest == expected[index].get('sha'))
        lateral = out[2].get('m3_lateral') or {}
        post_release = audit.observe_release(out[2])
        present = any(v.get('present') for v in (lateral.get('threats') or {}).values())
        ever_threat |= present
        state = (lateral.get('mode'), lateral.get('selected'), out[2].get('reason'))
        sample = index % 5 == 0 or state != previous_state or lateral.get('released_to_baseline')
        previous_state = state
        base = captured.get('output')
        if not (sample and ever_threat and scene is not None and base and base[2]['reason'] in eligible):
            continue
        entry = {'index': index, 't': data['reference_s'], 'mode': lateral.get('mode'),
            'actual_output': out[:2], 'baseline_output': base[:2], 'post_release': post_release,
            'hypothetical_baseline_only': not post_release, 'output_sha': digest}
        # 입력은 기록된 제어기 입력뿐이다. 평가 actor/정답 자료는 읽지 않는다.
        for key, candidate in [('baseline', base), ('actual', out)]:
            start = time.perf_counter()
            result = assess_return_command(scene, frames, base[2].get('path_points'),
                target_speed=float(candidate[0]), steering=float(candidate[1]),
                measured_speed=data['speed_mps'], previous_steering=data['previous_steering_rad'],
                control_dt=data['dt_s'], cfg=ctl.lateral_config, execution=ctl.execution)
            entry[key] = result
            entry[key+'_cost_s'] = time.perf_counter()-start
        entry['output_unchanged_by_audit'] = freeze(out) == frozen
        rows.append(freeze(entry))
    return name, rows, {'calls': len(comparisons), 'all_match_previous': all(comparisons),
        'first_difference': next((i for i, ok in enumerate(comparisons) if not ok), None),
        'audited_calls': len(rows), 'post_release_calls': sum(r['post_release'] for r in rows),
        'baseline_reasons': dict(Counter(r['baseline']['reason'] for r in rows)),
        'actual_reasons': dict(Counter(r['actual']['reason'] for r in rows)),
        'baseline_rejected_actual_supported': sum(not r['baseline']['sampled_sweep_ok'] and r['actual']['sampled_sweep_ok'] for r in rows),
        'audit_output_unchanged': all(r['output_unchanged_by_audit'] for r in rows),
        'source_input_sha256': hashlib.sha256(raw).hexdigest(),
        'input_unchanged': path.read_bytes() == raw}


def main(output, workers):
    if output.exists(): raise FileExistsError(output)
    before = source_hashes()
    plan = json.loads((OLD/'run2_plan.json').read_text())
    results, rows = {}, {}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for name, detail, summary in pool.map(one, list(plan)):
            results[name], rows[name] = summary, detail
            print(json.dumps({name: summary}, ensure_ascii=False), flush=True)
    output.mkdir(parents=True)
    report = {'scope': 'saved-input shadow command inspection, no policy/output change or closed-loop claim',
        'sampling': 'every fifth call plus mode/selection/reason transitions after first side threat',
        'source_sha256': before, 'source_unchanged': before == source_hashes(), 'results': results,
        'limits': ['Hypothetical baseline is not an adopted release policy.',
                   'Current rear mask, lateral budget, braking and surface criteria unchanged.',
                   'No hidden-body size or rear-absence premise; sampled geometry is not continuous proof.']}
    (output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    (output/'rows.json').write_text(json.dumps(rows, ensure_ascii=False)+'\n', encoding='utf-8')
    return 0 if report['source_unchanged'] and all(r['all_match_previous'] and r['input_unchanged'] and r['audit_output_unchanged'] for r in results.values()) else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True); p.add_argument('--workers', type=int, default=5)
    a = p.parse_args(); sys.exit(main(a.output, a.workers))
