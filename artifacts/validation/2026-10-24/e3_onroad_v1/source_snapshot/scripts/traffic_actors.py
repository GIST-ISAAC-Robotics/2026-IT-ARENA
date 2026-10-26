"""평가 전용 복수 actor 관리. 일정/정답/접촉은 제어기에 전달하지 않는다.

기존 G1의 주 actor는 그대로 두고 추가 actor마다 명령·자세·접촉을 분리한다.
같은 접촉이 두 센서에서 보고될 수 있으므로 원시 기록은 보존하고 사건만 중복 제거한다.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import g1_traffic_eval as ev


def overlay_case(case, path):
    """별도 사전 등록 파일을 명시한 경우만 추가 actor를 적용한다."""
    if path is None:
        return case
    overlay = json.loads(Path(path).read_text(encoding='utf-8'))
    if (overlay.get('schema_version') != 1 or overlay.get('base_case') != case['name']
            or not overlay.get('scope') or not overlay.get('registered_at')
            or not isinstance(overlay.get('additional_actors'), list) or not overlay['additional_actors']):
        raise ValueError('actor overlay must name the matching base case and preregister its scope')
    result = deepcopy(case)
    result['additional_actors'] = overlay['additional_actors']
    result['additional_actor_scope'] = overlay['scope']
    return result


def actor_specs(config, case):
    """주 actor와 선택형 추가 actor를 정규화한다. 기존 설정은 한 항목만 반환한다."""
    primary = deepcopy(config['actor'])
    primary.update(schedule=deepcopy(case['schedule']),
                   start_center_gap_m=case.get('actor_start_center_gap_m', primary['start_center_gap_m']),
                   lateral_offset_m=case.get('actor_lateral_offset_m', primary.get('lateral_offset_m', 0.)),
                   contact_topic='/eval/g1b_actor/contact', role='lead')
    specs = [primary]
    for extra in case.get('additional_actors', []):
        required = {'model_name', 'role', 'start_center_gap_m', 'lateral_offset_m', 'schedule'}
        if not isinstance(extra, dict) or not required.issubset(extra):
            raise ValueError('additional actor requires name, role, start offset, lateral offset and schedule')
        actor = deepcopy(config['actor'])
        actor.update(deepcopy(extra))
        actor['contact_topic'] = f"/eval/traffic/{actor['model_name']}/contact"
        specs.append(actor)
    if not 1 <= len(specs) <= 4:
        raise ValueError('evaluation supports one ego and one to four actors')
    names = [a['model_name'] for a in specs]
    topics = [a['contact_topic'] for a in specs]
    if len(set(names)) != len(names) or len(set(topics)) != len(topics):
        raise ValueError('actor names and contact topics must be unique')
    for actor in specs:
        if not re.fullmatch(r'eval_[A-Za-z0-9_]+', actor['model_name']):
            raise ValueError('invalid evaluation actor name')
        if actor['role'] not in ('lead', 'side', 'trailing'):
            raise ValueError('invalid evaluation actor role')
        for key in ('length_m', 'width_m', 'height_m', 'command_hz', 'pose_publish_hz'):
            if not math.isfinite(actor[key]) or actor[key] <= 0:
                raise ValueError(f'invalid actor {key}')
        for key in ('bottom_m', 'start_center_gap_m', 'lateral_offset_m'):
            if not math.isfinite(actor[key]):
                raise ValueError(f'invalid actor {key}')
        if actor['bottom_m'] < 0 or abs(actor['lateral_offset_m']) > .45:
            raise ValueError('invalid actor height or lateral offset')
        if actor is not primary:
            minimum = actor.get('minimum_body_gap_m')
            if not ev._finite(minimum) or minimum < 0 or not actor.get('gap_basis'):
                raise ValueError('additional actor requires preregistered finite gap and basis')
        ev.validate_schedule(actor['schedule'], case['duration_s'])
    return specs


def compose_sdf(specs, poses, single_actor_sdf):
    """검증된 단일 actor 생성기를 재사용하며 Contact world plugin은 한 번만 추가한다."""
    if len(specs) != len(poses) or not specs:
        raise ValueError('one initial pose per actor required')
    root = ET.Element('sdf', version='1.9')
    for index, (actor, pose) in enumerate(zip(specs, poses)):
        fragment = ET.fromstring(single_actor_sdf({'actor': actor}, pose))
        if index == 0:
            for plugin in fragment.findall('plugin'):
                root.append(plugin)
        model = fragment.find('model')
        model.find('link/sensor/contact/topic').text = actor['contact_topic']
        root.append(model)
    return ET.tostring(root, encoding='unicode')


def check_additional_routes(config, case, specs, scene, seed, centerline, road, obstacles,
                            start_s, route_check, pose_at):
    """기존 도로/잔디·벽 검사를 각 actor에 그대로 적용하고 actor끼리 초기/일정 겹침도 거절한다.

    10 ms 표본 검사는 일정의 설정 실수를 잡는 사전 검사이며 연속 충돌 검증은 아니다.
    실제 실행에서는 별도 밀집 자세·접촉 검사가 필요하다.
    """
    starts = [start_s(config, scene, seed, {'actor_start_center_gap_m': a['start_center_gap_m']}) for a in specs]
    checks = {}
    for spec, initial in zip(specs[1:], starts[1:]):
        actor_config = {**config, 'actor': spec}
        actor_case = {**case, 'schedule': spec['schedule'],
                      'actor_lateral_offset_m': spec['lateral_offset_m'],
                      'route_kind': spec.get('route_kind', 'straight')}
        checks[spec['model_name']] = route_check(actor_config, centerline, actor_case, initial, road, obstacles)
    minimum_gap = math.inf
    count = max(1, math.ceil(case['duration_s']/.01))
    for i in range(count+1):
        elapsed = case['duration_s']*i/count
        footprints = [ev.rectangle(*pose_at(centerline, initial, a['schedule'], elapsed, a['lateral_offset_m']),
                                   a['length_m'], a['width_m']) for a, initial in zip(specs, starts)]
        for j, first in enumerate(footprints):
            for second in footprints[j+1:]:
                if first.intersection(second).area > 1e-9:
                    raise ValueError(f'actor schedules overlap at {elapsed:.3f} s')
                minimum_gap = min(minimum_gap, first.distance(second))
    return starts, {'actors': checks, 'sample_interval_s': case['duration_s']/count,
                    'sampled_actor_pair_min_gap_m': minimum_gap if math.isfinite(minimum_gap) else None,
                    'scope': 'registered schedule sample check; not continuous collision proof'}


class ActorRecords:
    """actor별 순서·잘못된 frame·명령 ACK를 독립적으로 보존한다."""
    def __init__(self, specs):
        self.specs = {a['model_name']: a for a in specs}
        self.poses = {name: [] for name in self.specs}
        self.commands = {name: [] for name in self.specs}
        self.rejected_times = {name: 0 for name in self.specs}
        self.invalid_frames = []

    def pose(self, name, frame, row):
        if name not in self.specs or frame != 'it_arena_track':
            self.invalid_frames.append([frame, name])
            return False
        if len(row) != 4 or not all(math.isfinite(v) for v in row):
            raise ValueError('invalid actor pose')
        rows = self.poses[name]
        if rows and row[0] <= rows[-1][0]:
            self.rejected_times[name] += 1
            return False
        rows.append(tuple(row))
        return True


def contact_events(records, specs):
    """발행 actor의 충돌체가 없는 잘못된 토픽 자료는 거절한다.

    동일 시각·동일 충돌체 쌍만 합친다. 인접 시각 접촉을 임의의 시간창으로 합치지 않는다.
    수신 공백은 무접촉 근거가 아니며 별도의 연결 검사·자세 기하가 필요하다.
    """
    expected = {a['model_name']: a['contact_topic'] for a in specs}
    events, errors = {}, []
    duplicates = 0
    for row in records:
        source = row.get('actor')
        if (source not in expected or row.get('topic') != expected[source]
                or not isinstance(row.get('t'), (int, float)) or not math.isfinite(row['t'])
                or not isinstance(row.get('pairs'), list)):
            errors.append('invalid source/topic/time/pairs')
            continue
        for pair in row['pairs']:
            if (not isinstance(pair, list) or len(pair) != 2
                    or not all(isinstance(p, str) and '::' in p for p in pair)
                    or source not in [p.split('::')[0] for p in pair]):
                errors.append(f'contact source mismatch: {source}')
                continue
            key = (row['t'], *sorted(pair))
            if key in events:
                duplicates += 1
                events[key]['sources'].add(source)
            else:
                events[key] = {'t': row['t'], 'pair': sorted(pair), 'sources': {source}}
    normalized = [{**event, 'sources': sorted(event['sources'])} for _, event in sorted(events.items())]
    return {'valid': not errors, 'errors': errors, 'raw_messages': len(records),
            'duplicate_pair_reports': duplicates, 'events': normalized,
            'ego_events': sum(any(p.split('::')[0] == 'arena_car' for p in e['pair']) for e in normalized),
            'scope': 'event-only; absence of messages is not no-contact proof'}


def audit_actor(spec, rows, commands, first, end, schedule_pose, runtime_world, harness):
    """actor별 일정 이행/수신 검사. 한 actor의 정상 기록이 다른 actor의 누락을 덮지 않는다."""
    valid_rows = bool(rows) and ev._pose_rows_valid(rows)
    command_valid = all(math.isfinite(c.get('requested_sim_s', math.nan)) for c in commands) and all(
        b['requested_sim_s'] > a['requested_sim_s'] for a, b in zip(commands, commands[1:]))
    coverage = ev.stream_coverage([r[0] for r in rows] if valid_rows else [], first, end, ev.POSE_MAX_GAP_S)
    checked = [r for r in rows if first <= r[0] <= end] if valid_rows else []
    errors = ev.actor_tracking_errors(checked, schedule_pose)
    moving_errors, static_errors = [], []
    for row in errors:
        recent = [ev.actor_speed_mps(spec['schedule'], row['t']-first-dt) for dt in (0., .1, .2)]
        (moving_errors if max(recent) > 0 else static_errors).append(row['error_m'])
    try:
        world = ET.fromstring(runtime_world)
        models = [m for m in world.iter('model') if m.get('name') == spec['model_name']]
        inserted = len(models) == 1 and models[0].findtext('static') == 'true'
    except ET.ParseError:
        inserted = False
    has_motion = any(r['speed_mps'] > 0 for r in spec['schedule'])
    cmd_times = [c['requested_sim_s'] for c in commands if first <= c.get('requested_sim_s', math.nan) <= end]
    checks = {
        'finite_ordered': valid_rows and command_valid,
        'pose_before_start': valid_rows and rows[0][0] <= first,
        'pose_coverage': coverage['valid'], 'runtime_model': inserted,
        'acknowledged': (bool(commands) or not spec['schedule']) and all(c.get('acknowledged') is True for c in commands),
        'command_coverage': not spec['schedule'] or ev.stream_coverage(
            cmd_times, first, end, harness['actor_command_max_interval_s'])['valid'],
        'moving_tracking': (bool(moving_errors) or not has_motion) and all(
            e <= harness['actor_moving_tracking_error_max_m'] for e in moving_errors),
        'static_tracking': (bool(static_errors) or (has_motion and all(r['speed_mps'] > 0 for r in spec['schedule'])))
            and all(e <= harness['actor_static_error_max_m'] for e in static_errors),
    }
    return {'valid': all(checks.values()), 'checks': checks, 'coverage': coverage,
            'max_moving_error_m': max(moving_errors, default=None), 'max_static_error_m': max(static_errors, default=None)}
