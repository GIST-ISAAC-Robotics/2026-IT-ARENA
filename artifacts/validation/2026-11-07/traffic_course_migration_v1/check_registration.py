"""재등록 제안의 위치 타당성·옛 각본 거부·실행 인자 일관성을 Gazebo 없이 확인한다.

결과: checks/registration_check.json. 같은 이름 결과가 있으면 덮어쓰지 않는다.
"""
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from audit_local_pursuit_road import road_polygon  # noqa: E402
from build_experimental_track import obstacle_polygons  # noqa: E402
import official_traffic_course as official  # noqa: E402
import validate_g1_traffic as harness  # noqa: E402
from validate_basic_autonomy import TRACK_WORLDS  # noqa: E402

TRACK = 'official_v2026_11_06'
OLD_CONFIGS = ('config/tests/official_traffic_v1.json',
               'artifacts/validation/2026-11-05/moving_pass_withdrawal_v1/course.json')
OUT = HERE/'checks/registration_check.json'
ARGS = SimpleNamespace(mode='g1g2', handoff_reserve='paced', handoff_yield_progress=True,
                       handoff_rear_sweep=True, record_follow_decisions=True, seed=0)


def positions(config, world):
    centerline, scene = harness.load_centerline(world)
    road, walls = road_polygon(world/'world.sdf'), obstacle_polygons(world/'world.sdf')
    slot = {int(s['index']): s for s in scene['starting_grid']['slots']}[config['launch']['grid_slot']]
    rows = {}
    for case in config['cases']:
        s0 = harness.actor_start_s(config, scene, 0, case)
        route = harness.check_route(config, centerline, case, s0, road, walls)
        start = harness.primary_pose_at(config, centerline, s0, case, 0.)
        end = harness.primary_pose_at(config, centerline, s0, case, case['duration_s'])
        rows[case['name']] = dict(
            route_check=route, actor_start_pose=start, actor_end_pose=end,
            ego_slot=dict(index=slot['index'], s_m=slot['s_m'], x=slot['x'], y=slot['y'], yaw_rad=slot['yaw_rad']),
            ego_to_actor_center_m=math.dist((slot['x'], slot['y']), start[:2]),
            actor_end_to_corner_m=route['first_corner_s_m']-route['actor_end_s_m'],
            actor_start_inside_road=road.contains(harness.rectangle(*start, config['actor']['length_m'],
                                                                    config['actor']['width_m'])))
    return rows


def refused(call):
    try:
        call()
    except ValueError as error:
        return dict(refused=True, message=str(error))
    return dict(refused=False)


def dry(name, arguments):
    folder = HERE/'dryargs'/name
    run = subprocess.run([sys.executable, '-B', str(HERE/'dryargs_runner.py'), str(folder), *arguments],
                         cwd=ROOT, capture_output=True, text=True, timeout=300)
    argv = folder/'dry_launch_argv.json'
    result = dict(exit=run.returncode, stdout_tail=run.stdout[-400:], stderr_tail=run.stderr[-400:])
    if argv.exists():
        result['argv'] = json.loads(argv.read_text(encoding='utf-8'))
        snapshot = folder/'source_snapshot.zip'
        if snapshot.exists():
            snapshot.unlink()  # 실행 전 사본은 실제 등록 때 다시 만든다. 여기서는 argv만 근거로 남긴다.
    return result


def launch_options(argv):
    return {item.split(':=', 1)[0]: item.split(':=', 1)[1] for item in argv if ':=' in item}


def main():
    if OUT.exists():
        raise FileExistsError(OUT)
    proposal = harness.load_config(HERE/'new_track_course.json')
    new_world = TRACK_WORLDS[TRACK]
    report = dict(scope='Gazebo 미실행. 평가 전용 위치·거부·launch 인자 확인이며 주행 결과가 아님',
                  proposal_positions_new_track=positions(proposal, new_world))
    case = proposal['cases'][0]
    report['refusals'] = {
        'proposal_on_default_track': refused(lambda: official.inputs(ARGS, proposal, case)),
        **{f'{path}_on_new_track': refused(lambda path=path: official.inputs(
            ARGS, json.loads((ROOT/path).read_text(encoding='utf-8')), {}, new_world, TRACK)) for path in OLD_CONFIGS}}
    report['proposal_accepted_on_new_track'] = sorted(official.inputs(ARGS, proposal, case, new_world, TRACK))[:3]
    runs = json.loads((HERE/'proposed_runs.json').read_text(encoding='utf-8'))
    r16 = json.loads((ROOT/runs['r16_registered']).read_text(encoding='utf-8'))['args']
    strip = lambda args: [a for a in args[1:args.index('--output')]]  # noqa: E731
    report['dry'] = {
        'r16_old_track_current_code': dry('r16_old_track', strip(r16)),
        **{f'new_{name}': dry(f'new_{name}', strip(args)) for name, args in runs['runs'].items()},
        'old_r16_config_new_track': dry('old_r16_config_new_track', strip(r16)+['--track', TRACK]),
    }
    base = launch_options(report['dry']['r16_old_track_current_code'].get('argv', []))
    sensor = str((ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml').resolve())
    for name in runs['runs']:
        options = launch_options(report['dry'][f'new_{name}'].get('argv', []))
        differs = sorted(k for k in set(base) | set(options) if base.get(k) != options.get(k))
        report['dry'][f'new_{name}']['launch_option_diff_vs_r16'] = {k: [base.get(k), options.get(k)] for k in differs}
        report['dry'][f'new_{name}']['checks'] = dict(
            track=options.get('track') == TRACK, sensor_b0390=options.get('local_sensor_config') == sensor,
            only_expected_keys_differ=set(differs) <= {'track', 'local_sensor_config', 'evaluation_actor_sdf',
                                                       'g1_trace_path', 'runtime_artifacts'},
            r16_flags_kept=all(options.get(k) == v for k, v in base.items()
                               if k.startswith(('evaluation_handoff', 'evaluation_moving', 'g1_', 'pilot_'))
                               and k != 'g1_trace_path'))
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str)+'\n', encoding='utf-8')
    summary = {name: row.get('checks') for name, row in report['dry'].items()}
    print(json.dumps(dict(refusals={k: v['refused'] for k, v in report['refusals'].items()},
                          dry_exit={k: v['exit'] for k, v in report['dry'].items()}, checks=summary), ensure_ascii=False))
    ok = (all(v['refused'] for v in report['refusals'].values())
          and report['dry']['old_r16_config_new_track']['exit'] == 3
          and all(all(row['checks'].values()) for name, row in report['dry'].items() if 'checks' in row))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
