"""실제 해제 분기를 타는 기존 합성 회귀에 출력 외곽 진단을 붙인다. 물리 주행 아님."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/p) for p in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'tests')]
from arena_autonomy.follow_trace import freeze
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig, SceneHistory
from arena_autonomy.m3_return_guard import assess_return_command, ReturnGuardAudit
from test_m3_baseline_release import car_x, D0
from test_m3_observed_follow import timed_scene
from test_lateral_intent import car
from test_local_follow import history


def run(sign):
    ctl = M3ExecutionFollow(execution=ExecutionConfig(release_to_baseline=True), allow_simulation_trial=True)
    base = LocalFollow(FollowConfig(occupancy_timing_mode='per_ray_candidate'))
    recent = SceneHistory(); audit = ReturnGuardAudit(); h = history(v=0.)
    side = 'left' if sign > 0 else 'right'
    offset = D0*sign
    sequence = [(round(.4+.1*k, 3), []) for k in range(3)]
    sequence += [(round(.7+.1*k, 3), [car(car_x(1.+.1*k), .13*sign, offset)]) for k in range(20)]
    rows = []
    for now, objects in sequence:
        scene, _ = timed_scene(now, offset, extra=objects)
        frames = recent.update(scene, h)
        actual = ctl.command(scene, h, 0., side)
        expected = base.command(scene, h, 0., side)
        post_release = audit.observe_release(actual[2])
        row = {'t': now, 'actual': actual[:2], 'post_release': post_release,
               'mode': (actual[2].get('m3_lateral') or {}).get('mode')}
        if post_release:
            before = freeze(actual)
            row['baseline_output_equal'] = actual[:2] == expected[:2]
            row['guard'] = assess_return_command(scene, frames, expected[2].get('path_points'),
                target_speed=actual[0], steering=actual[1], measured_speed=0., previous_steering=0.,
                control_dt=.02, cfg=ctl.lateral_config, execution=ctl.execution)
            row['audit_did_not_change_output'] = before == freeze(actual)
        rows.append(freeze(row))
    post = [r for r in rows if r['post_release']]
    return {'release_events': audit.release_events, 'post_release_calls': len(post),
            'positive_unconfirmed': sum(r['actual'][0] > 0 and not r['guard']['sampled_sweep_ok'] for r in post),
            'baseline_match': all(r['baseline_output_equal'] for r in post),
            'audit_unchanged': all(r['audit_did_not_change_output'] for r in post), 'rows': rows}


def main(out):
    if out.exists(): raise FileExistsError(out)
    files = list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    results = {side: run(sign) for side, sign in [('left', 1.), ('right', -1.)]}
    report = {'scope': 'existing stationary-ego synthetic release sequence with shadow command check only',
        'source_sha256': hashes, 'source_unchanged': all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items()),
        'results': results, 'limits': ['Ego is held fixed independently of requested velocity.',
        'A rejected sweep is insufficient observation/geometry, not a demonstrated collision.',
        'No new policy or actuator output; full loop and hardware not tested.']}
    out.mkdir(parents=True)
    (out/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:{x:v for x,v in result.items() if x!='rows'} for k,result in results.items()}),flush=True)
    return 0 if report['source_unchanged'] and all(r['release_events'] and r['baseline_match'] and r['audit_unchanged'] for r in results.values()) else 1


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    sys.exit(main(p.parse_args().output))
