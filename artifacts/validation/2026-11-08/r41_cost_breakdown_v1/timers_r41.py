"""r41 기록의 저부하 구간 타이머 재생(Pass C). cProfile(Pass B)이 광선당 호출이 많은 구간을 크게 부풀려서 추가했다.

순서(한 프로세스, python -I -B):
  A. r42 bundle_parity.replay_once 기준 재생(계측 없음, 명령을 r42 commands_repo.json과 비교)
  C. 같은 헤더·제어기 생성·reset 순서로 재생하되, 선택 함수(판단당 수십 회 이하)만 perf_counter_ns 래퍼로 감싼다.
     래퍼는 인자·반환값을 바꾸지 않는다. 명령을 A·r42와 비교한다.
  측정 밖(timed_call 반환 뒤): 순수 함수 보조 측정 — 광선별 rear_masked/blind_spec 반복 비용, 사전 파싱 원형의
     같은 판정 여부, ReturnGrid와 ObservationGrid의 compact 결과 동일성. 이 값은 total에 포함되지 않는다.
src/의 제어기·보호층·설정은 바꾸지 않는다(프로세스 안 함수 객체만 감싼다).
"""
from collections import defaultdict
from hashlib import sha256
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
REF = REPO/'artifacts/validation/2026-11-08/jetson_bundle_refresh_v1'
TRACE_NAME = 'artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl'
TRACE = REPO/TRACE_NAME
RAW = HERE/'raw'

# (파일, 시작 줄, 이름). 줄 번호는 Pass B pstats 키와 같다.
TARGETS = [
    ('follow_trace.py', 155, 'restore_call'),
    ('scene_motion.py', 31, 'motion_scene'),
    ('lidar_observation.py', 19, 'audit_front_observation'),
    ('scene_motion.py', 15, 'relative_poses'),
    ('local_scene.py', 155, 'scene_from_scan'),
    ('local_scene.py', 76, 'LocalScene.__post_init__'),
    ('bypass_handoff.py', 608, 'BypassHandoffFollow.command'),
    ('bypass_handoff.py', 633, 'BypassHandoffFollow._command'),
    ('bypass_initial_reference.py', 102, 'ReadyInitialReference.update'),
    ('local_follow.py', 470, 'LocalFollow.command'),
    ('local_follow.py', 499, 'LocalFollow._observed_command'),
    ('known_structure.py', 107, 'StructureAware.evaluate'),
    ('local_follow.py', 577, 'LocalFollow._path_command'),
    ('corridor_observer.py', 145, 'CorridorObserver.evaluate'),
    ('local_follow.py', 621, 'LocalFollow._baseline_command'),
    ('parallel_lane_trial.py', 153, 'ParallelLaneTrial.apply'),
    ('pass_release_trial.py', 433, 'PassReleaseSelector._select'),
    ('pass_release_trial.py', 108, 'threat_view'),
    ('m3_execution_trial.py', 492, 'ReturnGrid.__init__'),
    ('lateral_intent.py', 463, 'ObservationGrid.__init__'),
    ('lateral_intent.py', 590, 'compact_return_mask'),
    ('lateral_intent.py', 558, 'cell_coordinates'),
    ('m3_execution_trial.py', 499, 'side_return_masks'),
    ('lateral_intent.py', 634, 'side_threats'),
    ('pass_release_trial.py', 195, 'motion_region'),
    ('pass_release_trial.py', 324, 'PassReleaseSelector._guard_curve'),
    ('local_path.py', 77, 'local_path'),
]


def file_sha(path):
    value = sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            value.update(chunk)
    return value.hexdigest()


class Timers:
    """중첩 스택으로 포함/자체 시간을 나눈다. 같은 이름의 재귀 호출은 바깥 호출만 센다."""
    def __init__(self):
        self.stack = []
        self.reset_call()
        self.captured = {}

    def reset_call(self):
        self.incl = defaultdict(int)
        self.self_ns = defaultdict(int)
        self.count = defaultdict(int)
        self.by_parent = defaultdict(int)

    def wrap(self, name, func):
        timers = self
        def wrapper(*args, **kwargs):
            if any(frame[0] == name for frame in timers.stack):
                return func(*args, **kwargs)
            frame = [name, 0]
            timers.stack.append(frame)
            begin = time.perf_counter_ns()
            try:
                return func(*args, **kwargs)
            finally:
                elapsed = time.perf_counter_ns()-begin
                timers.stack.pop()
                timers.incl[name] += elapsed
                timers.self_ns[name] += elapsed-frame[1]
                timers.count[name] += 1
                parent = timers.stack[-1][0] if timers.stack else '<timed_call>'
                timers.by_parent[parent+' > '+name] += elapsed
                if timers.stack:
                    timers.stack[-1][1] += elapsed
                if name in ('ReturnGrid.__init__', 'ObservationGrid.__init__'):
                    timers.captured.setdefault(name, []).append(args[0])
                elif name == 'scene_from_scan':
                    timers.captured.setdefault(name, []).append((args, kwargs))
        wrapper.__wrapped__ = func
        return wrapper


def install(timers):
    """선택 함수 객체를 찾아 모든 참조(모듈 전역·클래스 속성)를 같은 래퍼로 바꾼다."""
    wanted = {(f, line): name for f, line, name in TARGETS}
    roots = (REPO/'src', REPO/'scripts')
    found = defaultdict(list)
    for module in list(sys.modules.values()):
        location = getattr(module, '__file__', None)
        if not location or not any(Path(location).resolve().is_relative_to(root) for root in roots):
            continue
        owners = [module] + [v for v in module.__dict__.values()
                             if isinstance(v, type) and getattr(v, '__module__', None) == module.__name__]
        for owner in owners:
            for attr, value in list(vars(owner).items()):
                code = getattr(value, '__code__', None)
                if code is None:
                    continue
                key = (Path(code.co_filename).name, code.co_firstlineno)
                if key in wanted:
                    found[key].append((owner, attr, value))
    wrappers, report = {}, {}
    for key, name in wanted.items():
        refs = found.get(key, [])
        functions = {id(v): v for _, _, v in refs}
        if len(functions) != 1:
            raise RuntimeError(f'target {name} {key}: {len(functions)} distinct objects')
        func = next(iter(functions.values()))
        wrapper = wrappers.setdefault(id(func), timers.wrap(name, func))
        for owner, attr, _ in refs:
            setattr(owner, attr, wrapper)
        report[name] = dict(file=key[0], line=key[1], references=sorted(
            f'{getattr(o, "__name__", "?")}.{a}' for o, a, _ in refs))
    return report


def blind_parsed_prototype(angle_rad, half, sectors, finite):
    """후보 1 원형: rear_masked와 같은 식·순서. blind_spec 파싱만 광선 밖으로 뺀다(src에는 넣지 않는다)."""
    angle = math.remainder(finite(angle_rad, 'angle_rad'), math.tau)
    if not 0 <= half < 180:
        raise ValueError('invalid rear mask angle')
    if abs(angle) > math.radians(180 - half) + 1e-12:
        return True
    return any(abs(math.remainder(angle - math.radians(center), math.tau)) <= math.radians(width) + 1e-12
               for center, width in sectors)


def side_measure(captured, ls):
    """timed_call 밖. 광선 마스크 반복 비용·원형 동일성, compact 중복 결과 동일성."""
    out = {}
    scans = captured.get('scene_from_scan', [])
    if scans:
        args, kw = scans[-1]
        n, start, inc, spec = len(args[0]), kw['angle_min'], kw['angle_increment'], kw['rear_half_angle_deg']
        angles = [math.remainder(start + i * inc, math.tau) for i in range(n)]
        t0 = time.perf_counter_ns()
        original = [ls.rear_masked(a, spec) for a in angles]
        t1 = time.perf_counter_ns()
        for _ in angles:
            ls.blind_spec(spec)
        t2 = time.perf_counter_ns()
        half, sectors = ls.blind_spec(spec)
        prototype = [blind_parsed_prototype(a, half, sectors, ls._finite) for a in angles]
        t3 = time.perf_counter_ns()
        out.update(rays=n, rear_masked_loop_ms=(t1-t0)/1e6, blind_spec_loop_ms=(t2-t1)/1e6,
                   parsed_prototype_loop_ms=(t3-t2)/1e6, prototype_identical=original == prototype,
                   masked_rays=sum(original))
    grids = captured.get('ObservationGrid.__init__', [])
    light = captured.get('ReturnGrid.__init__', [])
    if grids:
        a, b = light[-1], grids[-1]
        out['compact_duplicate_identical'] = bool(
            np.array_equal(a.returns, b.returns) and np.array_equal(a.ages, b.ages)
            and np.array_equal(a.compact_returns, b.compact_returns) and a.return_components == b.return_components)
    return out


def main():
    RAW.mkdir(exist_ok=True)
    sys.path[:0] = [str(REPO/'scripts'), str(REPO/'src/arena_autonomy'), str(REPO/'src/arena_vehicle_interface')]
    import profile_follow_trace as pft
    spec = importlib.util.spec_from_file_location('r42_bundle_parity', REF/'bundle_parity.py')
    parity = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(parity)
    trace_before = file_sha(TRACE)

    t0 = time.perf_counter()
    report_a, commands_a = parity.replay_once(pft, TRACE)
    wall_a = time.perf_counter()-t0
    r42_commands = json.loads((REF/'commands_repo.json').read_text(encoding='utf-8'))[TRACE_NAME]

    from arena_autonomy.follow_trace import comparable_decision, freeze
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy import local_scene as ls
    from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
    timers = Timers()
    installed = install(timers)  # A에서 지연 import가 모두 끝난 뒤 감싼다.

    events = pft.read_events(TRACE)
    head = next(events)
    data = head['data']
    follow = BypassHandoffFollow(pft.FollowConfig(**data['follow_config']),
        HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**data['motion_config'])
    count, footer, commands, rows = 1, None, [], []
    t0 = time.perf_counter()
    for event in events:
        if footer is not None or event.get('seq') != count:
            raise ValueError('trace gap, reorder, or trailing data')
        count += 1
        if event['kind'] == 'reset':
            follow.reset()
        elif event['kind'] == 'call':
            timers.reset_call()
            timers.captured = {}
            output, costs = pft.timed_call(follow, event['data']['input'], motion)
            frozen = freeze(output)
            comparable = comparable_decision(frozen)
            digest = sha256(json.dumps(comparable, sort_keys=True, allow_nan=True).encode()).hexdigest()
            detail = frozen[2]
            commands.append([event['seq'], frozen[0], frozen[1], detail.get('reason'), digest])
            rows.append(dict(seq=event['seq'], total_ms=costs['total_ms'], restore_ms=costs['restore_ms'],
                             projection_ms=costs['projection_ms'], decision_ms=costs['decision_ms'],
                             handoff_state=detail.get('handoff_state'),
                             incl_ms={k: v/1e6 for k, v in timers.incl.items()},
                             self_ms={k: v/1e6 for k, v in timers.self_ns.items()},
                             count=dict(timers.count), by_parent_ms={k: v/1e6 for k, v in timers.by_parent.items()},
                             side=side_measure(timers.captured, ls)))
        elif event['kind'] == 'footer':
            footer = event['data']
        else:
            raise ValueError('unknown trace event')
    wall_c = time.perf_counter()-t0
    if footer is None or footer.get('written') != count-1 or footer.get('accepted') != count-1:
        raise ValueError('incomplete trace capture')

    with (RAW/'r41_timers_per_call.jsonl').open('w', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row)+'\n')

    def dist(values):
        values = np.asarray(values, dtype=float)
        return dict(mean=float(values.mean()), p50=float(np.percentile(values, 50)),
                    p95=float(np.percentile(values, 95)), maximum=float(values.max()))
    n = len(rows)
    names = [name for _, _, name in TARGETS]
    functions = {}
    for name in names:
        incl = [r['incl_ms'].get(name, 0.) for r in rows]
        own = [r['self_ms'].get(name, 0.) for r in rows]
        calls = [r['count'].get(name, 0) for r in rows]
        functions[name] = dict(total_calls=int(sum(calls)), decisions_with_call=int(sum(c > 0 for c in calls)),
                               inclusive_per_decision_ms=dist(incl), self_per_decision_ms=dist(own),
                               inclusive_sum_s=sum(incl)/1e3, self_sum_s=sum(own)/1e3)
    edges = defaultdict(float)
    for r in rows:
        for k, v in r['by_parent_ms'].items():
            edges[k] += v
    sides = [r['side'] for r in rows]
    stages = {k: dist([r[k] for r in rows]) for k in ('total_ms', 'restore_ms', 'projection_ms', 'decision_ms')}
    result = dict(
        trace=TRACE_NAME, trace_sha256_before=trace_before, trace_sha256_after=file_sha(TRACE), calls=n,
        parity=dict(reference_vs_r42_identical=commands_a == r42_commands, timed_vs_reference_identical=commands == commands_a,
                    timed_vs_r42_identical=commands == r42_commands,
                    first_different_seq=next((a[0] for a, b in zip(commands_a, commands) if a != b), None)),
        reference_costs=report_a['costs']['all'], timer_pass_stages=stages, wall_s=dict(reference=wall_a, timers=wall_c),
        installed=installed, functions=functions, edges_sum_s={k: v/1e3 for k, v in sorted(edges.items())},
        side_measure=dict(
            rear_masked_loop_ms=dist([s['rear_masked_loop_ms'] for s in sides if 'rays' in s]),
            blind_spec_loop_ms=dist([s['blind_spec_loop_ms'] for s in sides if 'rays' in s]),
            parsed_prototype_loop_ms=dist([s['parsed_prototype_loop_ms'] for s in sides if 'rays' in s]),
            prototype_identical_all=all(s.get('prototype_identical', True) for s in sides),
            scans_measured=sum('rays' in s for s in sides),
            compact_duplicate_checked=sum('compact_duplicate_identical' in s for s in sides),
            compact_duplicate_identical_all=all(s.get('compact_duplicate_identical', True) for s in sides)),
        method='A: r42 replay_once uninstrumented; C: perf_counter_ns wrappers on selected functions (inclusive/self via stack)',
        scope='offline decision replay on this PC/WSL; not Jetson/ROS deadline or closed-loop driving')
    with (HERE/'timers_result.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(dict(parity=result['parity'], wall_s=result['wall_s'],
                          prototype_identical_all=result['side_measure']['prototype_identical_all'],
                          compact_duplicate_identical_all=result['side_measure']['compact_duplicate_identical_all'])))
    ok = result['parity']['reference_vs_r42_identical'] and result['parity']['timed_vs_reference_identical'] \
        and trace_before == result['trace_sha256_after']
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
