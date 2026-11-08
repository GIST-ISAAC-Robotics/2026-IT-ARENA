"""새 follow-profile 묶음을 격리 추출해 현재 저장소와 같은 판단 명령·비용을 1회씩 비교한다.

부모: 묶음 검사(verify) → build/ 아래 새 폴더에 추출 → 저장소/추출본 각각을 python -I -B, ROS·PYTHON
환경 제거 상태의 자식으로 실행 → 명령 동일성 비교. 원 기록 출력과의 차이는 별도 항목으로만 보고한다.
Jetson·ROS·Gazebo를 실행하지 않으며 비용은 이 PC/WSL의 오프라인 재생 비용이다.
"""
from hashlib import sha256
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tarfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
ARCHIVE = REPO/'build/jetson_follow_20261108_v1.tar.gz'
TARGET = REPO/'build/jetson_follow_20261108_v1_extract'
MANIFEST = HERE/'bundle_manifest.json'
HANDOFF = 'g1_g2_handoff_decision_stage_only'


def file_sha(path):
    value = sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            value.update(chunk)
    return value.hexdigest()


def replay_once(pft, path):
    """기록 1개를 처음부터 끝까지 한 번 재생. 기록 순서/footer 무결성은 replay_events와 같은 조건."""
    from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze
    from arena_autonomy.lidar_motion import MotionConfig
    events = pft.read_events(path)
    head = next(events)
    data = head['data']
    standard = data.get('schema') == 1 and data.get('scope') == 'g1_decision_stage_only'
    handoff = (data.get('schema') == 5 and data.get('scope') == HANDOFF
               and data.get('controller_mode') == 'g1_g2_handoff_trial')
    if head.get('seq') != 0 or head.get('kind') != 'header' or not (standard or handoff):
        raise ValueError('unsupported trace header')
    if handoff:
        from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
        follow = BypassHandoffFollow(pft.FollowConfig(**data['follow_config']),
            HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
    else:
        follow = pft.LocalFollow(pft.FollowConfig(**data['follow_config']))
    if follow.config.partial_observation_policy == 'strict':
        raise ValueError('timed_call always passes raw_scan; strict traces are outside this check')
    motion = MotionConfig(**data['motion_config'])
    count, footer, rows, commands, mismatches = 1, None, [], [], []
    stream_hash = sha256()
    for event in events:
        if footer is not None or event.get('seq') != count:
            raise ValueError('trace gap, reorder, or trailing data')
        count += 1
        if event['kind'] == 'reset':
            follow.reset()
        elif event['kind'] == 'call':
            output, costs = pft.timed_call(follow, event['data']['input'], motion)
            frozen = freeze(output)
            comparable = comparable_decision(frozen)
            if not equal_output(comparable, comparable_decision(event['data']['output'])):
                mismatches.append(event['seq'])
            digest = sha256(json.dumps(comparable, sort_keys=True, allow_nan=True).encode()).hexdigest()
            stream_hash.update(digest.encode())
            detail = frozen[2]
            commands.append([event['seq'], frozen[0], frozen[1], detail.get('reason'), digest])
            rows.append(dict(costs, handoff_state=detail.get('handoff_state')))
        elif event['kind'] == 'footer':
            footer = event['data']
        else:
            raise ValueError('unknown trace event')
    if (footer is None or not commands or footer.get('capture_errors') != 0 or footer.get('failed') != 0
            or footer.get('dropped') != 0 or footer.get('pending') != 0 or footer.get('worker_alive') is not False
            or footer.get('written') != count-1 or footer.get('accepted') != count-1):
        raise ValueError('incomplete trace capture')
    groups = {'all': rows}
    for row in rows:
        if row['handoff_state']:
            groups.setdefault('handoff_'+row['handoff_state'].lower(), []).append(row)
    costs = {name: {stage: pft.distribution([r[stage] for r in items])
                    for stage in ('total_ms', 'decision_ms', 'thread_cpu_ms')}
             for name, items in groups.items()}
    speeds = [c[1] for c in commands]
    return dict(scope=data['scope'], calls=len(commands), decision_stream_sha256=stream_hash.hexdigest(),
                recorded_matched=len(commands)-len(mismatches), recorded_mismatch_count=len(mismatches),
                recorded_first_mismatch_seqs=mismatches[:20], zero_speed_commands=sum(s == 0 for s in speeds),
                failed_stage_calls=sum(r['failed_stage'] is not None for r in rows), costs=costs), commands


def child(root, label):
    root = root.resolve()
    sys.path[:0] = [str(root/'scripts'), str(root/'src/arena_autonomy'), str(root/'src/arena_vehicle_interface')]
    import numpy
    import profile_follow_trace as pft
    manifest = json.loads(MANIFEST.read_text(encoding='utf-8'))
    traces = sorted(name for name in manifest['files'] if name.startswith('artifacts/') and name.endswith('.jsonl'))
    replays, all_commands = {}, {}
    for name in traces:
        before = file_sha(root/name)
        report, commands = replay_once(pft, root/name)
        report['input_unchanged'] = before == file_sha(root/name) == manifest['files'][name]['sha256']
        replays[name], all_commands[name] = report, commands
    project = {}
    for name, module in list(sys.modules.items()):
        location = getattr(module, '__file__', None)
        # 이 실행기(__main__)는 artifacts에 있으므로 제외한다. 나머지 저장소 하위 파일은 모두 대상이다.
        if name != '__main__' and location and Path(location).resolve().is_relative_to(REPO):
            project[name] = Path(location).resolve()
    build = (REPO/'build').resolve()
    if root == REPO:
        isolated = all(not path.is_relative_to(build) for path in project.values())
    else:
        isolated = all(path.is_relative_to(root) for path in project.values())
    expected = {f'arena_autonomy.{m}' for m in pft.SNAPSHOT_MODULES['arena_autonomy']}
    hashes = {name: file_sha(root/name) == item['sha256'] for name, item in manifest['files'].items()}
    result = dict(label=label, root=str(root), modules_from_selected_root_only=isolated and bool(project),
                  project_modules_loaded=len(project), snapshot_modules_loaded=len(expected & set(project)),
                  snapshot_modules_expected=len(expected), manifest_files_match=all(hashes.values()),
                  manifest_mismatches=sorted(k for k, v in hashes.items() if not v),
                  sys_flags=dict(isolated=sys.flags.isolated, ignore_environment=sys.flags.ignore_environment,
                                 no_user_site=sys.flags.no_user_site),
                  env_python_ros=sorted(k for k in os.environ if k.startswith(('PYTHON', 'ROS', 'AMENT'))),
                  environment=dict(python=platform.python_version(), numpy=numpy.__version__,
                                   system=platform.system(), release=platform.release(), machine=platform.machine()),
                  modules={k: str(v) for k, v in sorted(project.items())}, replays=replays,
                  scope='offline decision replay on this PC/WSL; not Jetson/ROS deadline or closed-loop driving')
    with (HERE/f'result_{label}.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    with (HERE/f'commands_{label}.json').open('x', encoding='utf-8') as stream:
        json.dump(all_commands, stream)
    print(json.dumps({k: v for k, v in result.items() if k not in ('modules', 'replays')}, ensure_ascii=False))
    return 0


def parent():
    sys.path.insert(0, str(REPO/'scripts'))
    from verify_follow_profile_bundle import verify
    manifest = json.loads(MANIFEST.read_text(encoding='utf-8'))
    check = verify(ARCHIVE, manifest)
    with (HERE/'verify_result.json').open('x', encoding='utf-8') as stream:
        json.dump(check, stream, ensure_ascii=False, indent=2)
    assert check['passed'] and check['dependency_closure']['checked'], check
    assert TARGET.resolve().is_relative_to((REPO/'build').resolve())
    TARGET.mkdir(exist_ok=False)
    with tarfile.open(ARCHIVE, 'r:gz') as archive:
        archive.extractall(TARGET, filter='data')
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(('PYTHON', 'ROS', 'AMENT', 'COLCON', 'RMW', 'CYCLONEDDS', 'FASTRTPS'))}
    for label, root in (('repo', REPO), ('bundle', TARGET)):
        code = subprocess.run([sys.executable, '-I', '-B', str(Path(__file__).resolve()), '--child',
                               str(root), label], cwd=root, env=env).returncode
        if code:
            return code
    results = {label: json.loads((HERE/f'result_{label}.json').read_text(encoding='utf-8')) for label in ('repo', 'bundle')}
    commands = {label: json.loads((HERE/f'commands_{label}.json').read_text(encoding='utf-8')) for label in ('repo', 'bundle')}
    traces = {}
    for name, repo in results['repo']['replays'].items():
        bundle = results['bundle']['replays'][name]
        first = next((a[0] for a, b in zip(commands['repo'][name], commands['bundle'][name]) if a != b), None)
        traces[name] = dict(commands_identical=commands['repo'][name] == commands['bundle'][name],
            first_different_seq=first, calls=repo['calls'],
            decision_stream_identical=repo['decision_stream_sha256'] == bundle['decision_stream_sha256'],
            recorded_mismatch_count=dict(repo=repo['recorded_mismatch_count'], bundle=bundle['recorded_mismatch_count']),
            total_ms={label: {k: results[label]['replays'][name]['costs']['all']['total_ms'][k]
                              for k in ('p50', 'p95', 'p99', 'maximum', 'over_20ms')} for label in ('repo', 'bundle')})
    summary = dict(passed=all(t['commands_identical'] and t['decision_stream_identical'] for t in traces.values())
                   and all(r['modules_from_selected_root_only'] and r['manifest_files_match'] for r in results.values()),
                   bundle_modules_from_extract_only=results['bundle']['modules_from_selected_root_only'],
                   repo_and_extract_match_manifest=all(r['manifest_files_match'] for r in results.values()),
                   traces=traces, archive=ARCHIVE.relative_to(REPO).as_posix(), remote_execution=False,
                   note='recorded_mismatch_count는 원 실행 당시 코드와 현재 코드의 차이이며 묶음 동일성 판정과 별개다.')
    with (HERE/'parity.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    if '--child' in sys.argv:
        index = sys.argv.index('--child')
        raise SystemExit(child(Path(sys.argv[index+1]), sys.argv[index+2]))
    raise SystemExit(parent())
