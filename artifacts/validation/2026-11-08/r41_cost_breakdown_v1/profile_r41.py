"""r41 코너 기록 1개를 r42 parity와 같은 입력·상태 구성으로 재생하고 함수별 비용을 분해한다.

순서(한 프로세스, python -I -B, ROS/PYTHON 환경 제거):
  A. 기준 재생: r42 bundle_parity.replay_once 그대로(계측 없음). 명령을 r42 commands_repo.json과 비교.
  B. cProfile 재생: 같은 루프에서 timed_call 구간만 profiler.enable/disable. 명령을 A·r42와 비교.
제어기·보호층·설정을 바꾸지 않는다. Gazebo·ROS·Jetson을 사용하지 않는다. 비용은 이 PC/WSL 오프라인 값이다.
"""
from hashlib import sha256
import cProfile
import importlib.util
import json
import os
from pathlib import Path
import platform
import pstats
import sys
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
REF = REPO/'artifacts/validation/2026-11-08/jetson_bundle_refresh_v1'
TRACE_NAME = 'artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl'
TRACE = REPO/TRACE_NAME
RAW = HERE/'raw'


def file_sha(path):
    value = sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            value.update(chunk)
    return value.hexdigest()


def load_parity():
    spec = importlib.util.spec_from_file_location('r42_bundle_parity', REF/'bundle_parity.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # 정의만 있다. parent()/child()는 호출하지 않는다.
    return module


def profiled_replay(pft, path, profiler, per_call):
    """replay_once와 같은 헤더 검사·제어기 생성·reset 순서. timed_call만 profiler로 감싼다."""
    from arena_autonomy.follow_trace import comparable_decision, freeze
    from arena_autonomy.lidar_motion import MotionConfig
    events = pft.read_events(path)
    head = next(events)
    data = head['data']
    if not (head.get('seq') == 0 and data.get('schema') == 5 and data.get('scope') == 'g1_g2_handoff_decision_stage_only'
            and data.get('controller_mode') == 'g1_g2_handoff_trial'):
        raise ValueError('expected r41 handoff trace header')
    from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
    follow = BypassHandoffFollow(pft.FollowConfig(**data['follow_config']),
        HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
    if follow.config.partial_observation_policy == 'strict':
        raise ValueError('strict traces are outside this check')
    motion = MotionConfig(**data['motion_config'])
    count, footer, commands, resets = 1, None, [], 0
    for event in events:
        if footer is not None or event.get('seq') != count:
            raise ValueError('trace gap, reorder, or trailing data')
        count += 1
        if event['kind'] == 'reset':
            follow.reset()
            resets += 1
        elif event['kind'] == 'call':
            wall0 = time.perf_counter_ns()
            profiler.enable()
            output, costs = pft.timed_call(follow, event['data']['input'], motion)
            profiler.disable()
            wall1 = time.perf_counter_ns()
            frozen = freeze(output)
            comparable = comparable_decision(frozen)
            digest = sha256(json.dumps(comparable, sort_keys=True, allow_nan=True).encode()).hexdigest()
            detail = frozen[2]
            commands.append([event['seq'], frozen[0], frozen[1], detail.get('reason'), digest])
            per_call.append(dict(seq=event['seq'], profiled_total_ms=costs['total_ms'],
                                 profiled_restore_ms=costs['restore_ms'], profiled_projection_ms=costs['projection_ms'],
                                 profiled_decision_ms=costs['decision_ms'], enable_to_disable_ms=(wall1-wall0)/1e6,
                                 handoff_state=detail.get('handoff_state'), reason=detail.get('reason'),
                                 failed_stage=costs['failed_stage']))
        elif event['kind'] == 'footer':
            footer = event['data']
        else:
            raise ValueError('unknown trace event')
    if (footer is None or footer.get('written') != count-1 or footer.get('accepted') != count-1
            or footer.get('capture_errors') != 0 or footer.get('failed') != 0 or footer.get('dropped') != 0):
        raise ValueError('incomplete trace capture')
    return commands, resets


def main():
    RAW.mkdir(exist_ok=True)
    sys.path[:0] = [str(REPO/'scripts'), str(REPO/'src/arena_autonomy'), str(REPO/'src/arena_vehicle_interface')]
    import numpy
    import profile_follow_trace as pft
    parity = load_parity()
    manifest = json.loads((REF/'bundle_manifest.json').read_text(encoding='utf-8'))
    trace_before = file_sha(TRACE)

    # A. 계측 없는 기준 재생(r42와 같은 함수).
    t0 = time.perf_counter()
    report_a, commands_a = parity.replay_once(pft, TRACE)
    wall_a = time.perf_counter()-t0
    r42_commands = json.loads((REF/'commands_repo.json').read_text(encoding='utf-8'))[TRACE_NAME]
    r42_result = json.loads((REF/'result_repo.json').read_text(encoding='utf-8'))['replays'][TRACE_NAME]

    # B. cProfile 재생. 기본 타이머(perf_counter), builtins 포함.
    profiler = cProfile.Profile()
    per_call = []
    t0 = time.perf_counter()
    commands_b, resets = profiled_replay(pft, TRACE, profiler, per_call)
    wall_b = time.perf_counter()-t0
    profiler.dump_stats(str(RAW/'r41_timed_call.pstats'))
    for key in ('cumulative', 'tottime'):
        with (RAW/f'r41_pstats_{key}.txt').open('w', encoding='utf-8') as stream:
            stats = pstats.Stats(str(RAW/'r41_timed_call.pstats'), stream=stream)
            stats.strip_dirs().sort_stats(key).print_stats(80)
    with (RAW/'r41_pstats_callers.txt').open('w', encoding='utf-8') as stream:
        stats = pstats.Stats(str(RAW/'r41_timed_call.pstats'), stream=stream)
        stats.sort_stats('tottime').print_callers(40)
    with (RAW/'r41_per_call_profiled.jsonl').open('w', encoding='utf-8') as stream:
        for row in per_call:
            stream.write(json.dumps(row)+'\n')

    project = {}
    for name, module in list(sys.modules.items()):
        location = getattr(module, '__file__', None)
        if name not in ('__main__', 'r42_bundle_parity') and location and Path(location).resolve().is_relative_to(REPO):
            path = Path(location).resolve()
            project[name] = dict(path=path.relative_to(REPO).as_posix(), sha256=file_sha(path))
    in_manifest = {v['path']: manifest['files'][v['path']]['sha256'] == v['sha256']
                   for v in project.values() if v['path'] in manifest['files']}
    first_ab = next((a[0] for a, b in zip(commands_a, commands_b) if a != b), None)
    a_costs = report_a['costs']
    result = dict(
        trace=TRACE_NAME, trace_sha256_before=trace_before, trace_sha256_after=file_sha(TRACE),
        trace_matches_r42_manifest=trace_before == manifest['files'][TRACE_NAME]['sha256'],
        calls=len(commands_b), resets=resets,
        parity=dict(reference_vs_r42_commands_identical=commands_a == r42_commands,
                    profiled_vs_reference_identical=commands_a == commands_b,
                    profiled_vs_r42_identical=commands_b == r42_commands, first_different_seq_ab=first_ab,
                    reference_stream_sha256=report_a['decision_stream_sha256'],
                    r42_stream_sha256=r42_result['decision_stream_sha256'],
                    reference_recorded_mismatch_count=report_a['recorded_mismatch_count']),
        reference_costs=a_costs, r42_repo_costs_all_total=r42_result['costs']['all']['total_ms'],
        wall_s=dict(reference_pass=wall_a, profiled_pass=wall_b),
        profiled_total_ms=pft.distribution([r['profiled_total_ms'] for r in per_call]),
        profiled_decision_ms=pft.distribution([r['profiled_decision_ms'] for r in per_call]),
        profiled_restore_ms=pft.distribution([r['profiled_restore_ms'] for r in per_call]),
        profiled_projection_ms=pft.distribution([r['profiled_projection_ms'] for r in per_call]),
        source_modules=project,
        loaded_modules_in_r42_manifest=len(in_manifest),
        loaded_modules_match_r42_manifest=all(in_manifest.values()) and bool(in_manifest),
        loaded_modules_mismatch=sorted(k for k, v in in_manifest.items() if not v),
        sys_flags=dict(isolated=sys.flags.isolated, no_user_site=sys.flags.no_user_site),
        env_python_ros=sorted(k for k in os.environ if k.startswith(('PYTHON', 'ROS', 'AMENT'))),
        environment=dict(python=platform.python_version(), numpy=numpy.__version__, system=platform.system(),
                         release=platform.release(), machine=platform.machine(), processor=platform.processor(),
                         cpu_count=os.cpu_count()),
        method='A: r42 replay_once uninstrumented; B: same loop, cProfile enabled only around timed_call',
        scope='offline decision replay on this PC/WSL; not Jetson/ROS deadline or closed-loop driving')
    with (HERE/'profile_result.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({k: result[k] for k in ('calls', 'resets', 'parity', 'wall_s', 'loaded_modules_match_r42_manifest',
                                              'trace_matches_r42_manifest')}, ensure_ascii=False))
    p = result['parity']
    ok = (p['reference_vs_r42_commands_identical'] and p['profiled_vs_reference_identical']
          and result['trace_sha256_before'] == result['trace_sha256_after'])
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
