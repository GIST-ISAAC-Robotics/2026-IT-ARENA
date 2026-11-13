"""CAD 기하와 소프트웨어 영점 프로필. 장치 I/O나 출력 허가는 하지 않는다."""
from dataclasses import dataclass
import csv
import hashlib
import json
import math
from pathlib import Path


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def equivalent(left, right):
    # 현 single_motor PostUpdate와 같은 모델 정의. 실차 곡률 실측이 아니다.
    if left == 0 and right == 0:
        return 0.0
    a, b = math.tan(left), math.tan(right)
    return math.atan(2*a*b/(a+b))


@dataclass(frozen=True)
class CadGoal:
    servo_rad: float
    left_rad: float
    right_rad: float
    equivalent_rad: float
    clamped: bool = False


class CadMapping:
    def __init__(self, rows):
        self.rows = tuple(tuple(row) for row in rows)
        if not 3 <= len(self.rows) <= 96:
            raise ValueError('invalid CAD sample count')
        zero = False
        for i, row in enumerate(self.rows):
            if len(row) != 3 or not all(finite(v) and abs(v) < 1.4 for v in row):
                raise ValueError('invalid CAD sample')
            s, left, right = row
            if s == 0:
                if left != 0 or right != 0:
                    raise ValueError('CAD zero must be straight')
                zero = True
            elif s*left >= 0 or s*right >= 0:
                raise ValueError('CAD sign differs from positive-right input')
            if i:
                previous = self.rows[i-1]
                if not (s > previous[0] and left < previous[1] and right < previous[2]):
                    raise ValueError('CAD mapping must be monotone')
        if not zero or not self.rows[0][0] < 0 < self.rows[-1][0]:
            raise ValueError('CAD straight and both directions required')

    @classmethod
    def default(cls):
        from .steering_cad_table import SAMPLES
        return cls(SAMPLES)

    @classmethod
    def from_csv(cls, path):
        with Path(path).open(encoding='utf-8-sig', newline='') as stream:
            return cls((math.radians(float(row['fusion_servo_deg'])),
                        float(row['left_wheel_heading_rad_left_positive']),
                        float(row['right_wheel_heading_rad_left_positive']))
                       for row in csv.DictReader(stream))

    def relative(self, requested):
        if not finite(requested):
            raise ValueError('invalid angle')
        servo = min(self.rows[-1][0], max(self.rows[0][0], requested))
        hi = 1
        while hi < len(self.rows)-1 and self.rows[hi][0] < servo:
            hi += 1
        a, b = self.rows[hi-1], self.rows[hi]
        fraction = (servo-a[0])/(b[0]-a[0])
        left = a[1]+fraction*(b[1]-a[1])
        right = a[2]+fraction*(b[2]-a[2])
        return CadGoal(servo, left, right, equivalent(left, right), servo != requested)

    def central(self, requested, *, wheelbase, track, wheel_limit):
        if not all(finite(v) for v in (requested, wheelbase, track, wheel_limit)):
            raise ValueError('nonfinite geometry or command')
        if wheelbase <= 0 or track <= 0 or not 0 < wheel_limit < 1.4:
            raise ValueError('invalid geometry')
        target = min(equivalent(*self.rows[0][1:]),
                     max(equivalent(*self.rows[-1][1:]), requested))
        if target == 0:
            return CadGoal(0, 0, 0, 0, requested != 0)
        hi = 1
        while hi < len(self.rows)-1 and equivalent(*self.rows[hi][1:]) > target:
            hi += 1
        lo, up = self.rows[hi-1][0], self.rows[hi][0]
        for _ in range(40):
            mid = (lo+up)/2
            if self.relative(mid).equivalent_rad > target:
                lo = mid
            else:
                up = mid
        goal = self.relative((lo+up)/2)
        clipped = target != requested
        if max(abs(goal.left_rad), abs(goal.right_rad)) > wheel_limit:
            inside, outside = 0.0, goal.servo_rad
            for _ in range(40):
                mid = (inside+outside)/2
                trial = self.relative(mid)
                if max(abs(trial.left_rad), abs(trial.right_rad)) <= wheel_limit:
                    inside = mid
                else:
                    outside = mid
            goal, clipped = self.relative(inside), True
        return CadGoal(goal.servo_rad, goal.left_rad, goal.right_rad, goal.equivalent_rad, clipped)

    def command_limit(self, *, wheelbase, track, wheel_limit):
        """공통 대칭 명령 범위. 양 방향 CAD 끝점과 명시 내륜 상한의 교집합."""
        left = self.central(1., wheelbase=wheelbase, track=track, wheel_limit=wheel_limit)
        right = self.central(-1., wheelbase=wheelbase, track=track, wheel_limit=wheel_limit)
        return min(left.equivalent_rad, -right.equivalent_rad)


PROFILE_SCHEMA = 'arena_steering_zero_v1'


def read_profile(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate profile key')
            result[key] = value
        return result
    data = json.loads(Path(path).read_text(encoding='utf-8'), object_pairs_hook=pairs,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
    check_profile(data)
    return data


def check_profile(data, *, ready=False, assembly_id=None):
    if not isinstance(data, dict) or data.get('schema') != PROFILE_SCHEMA:
        raise ValueError('unknown profile schema')
    if not isinstance(data.get('assembly_id'), str) or not data['assembly_id'].strip():
        raise ValueError('assembly identity required')
    if assembly_id is not None and data['assembly_id'] != assembly_id:
        raise ValueError('assembly identity changed')
    if type(data.get('zero_count')) is not int or not 0 <= data['zero_count'] <= 4095:
        raise ValueError('invalid straight count')
    if data.get('straight_confirmed') is not True or data.get('count_wrap_policy') != 'reject':
        raise ValueError('straight confirmation and no-wrap policy required')
    if data.get('hardware_angle_measured') is not False or data.get('angle_source') != 'fusion_cad_unmeasured_hardware':
        raise ValueError('CAD profile cannot claim measured wheel angles')
    if data.get('status') not in ('zero_recorded_pending_mount_checks', 'mount_checks_recorded'):
        raise ValueError('invalid calibration status')
    if data['status'] == 'mount_checks_recorded' or ready:
        if data['status'] != 'mount_checks_recorded':
            raise ValueError('direction/endpoints/mode/scale pending')
        for flag in ('direction_confirmed', 'endpoints_confirmed', 'position_mode_confirmed', 'scale_confirmed'):
            if data.get(flag) is not True:
                raise ValueError(flag+' required')
        direction = data.get('count_direction_per_cad_positive')
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('direction must be measured as -1 or +1')
        if type(data.get('counts_per_turn')) is not int or data['counts_per_turn'] != 4096:
            raise ValueError('only checked single-turn 4096 scale supported')
        low, high = data.get('safe_min_count'), data.get('safe_max_count')
        if type(low) is not int or type(high) is not int or not 0 <= low < data['zero_count'] < high <= 4095:
            raise ValueError('mounted limits must contain straight, without wrap')


def record_zero(neutral_count, *, assembly_id, straight_confirmed, evidence_note):
    # count는 사람이 관측해 전달한다. 이 함수에서 장치를 읽지 않는다.
    if not isinstance(evidence_note, str) or not evidence_note.strip():
        raise ValueError('manual observation evidence note required')
    data = dict(schema=PROFILE_SCHEMA, assembly_id=assembly_id, zero_count=neutral_count,
                straight_confirmed=straight_confirmed, status='zero_recorded_pending_mount_checks',
                count_wrap_policy='reject', hardware_angle_measured=False,
                angle_source='fusion_cad_unmeasured_hardware', evidence_note=evidence_note,
                count_direction_per_cad_positive=None, safe_min_count=None, safe_max_count=None,
                counts_per_turn=None, direction_confirmed=False, endpoints_confirmed=False,
                position_mode_confirmed=False, scale_confirmed=False)
    check_profile(data)
    return data


def save_new_profile(path, data):
    check_profile(data)
    # 새 버전 파일만 만든다. 기존 교정 덮어쓰기와 디바이스 EEPROM 조작은 없다.
    with Path(path).open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def complete_mount_profile(data, *, direction, minimum, maximum, confirmed):
    check_profile(data)
    if confirmed is not True:
        raise ValueError('mounted direction, limits and position mode confirmation required')
    value = dict(data, status='mount_checks_recorded', counts_per_turn=4096,
                 count_direction_per_cad_positive=direction, safe_min_count=minimum, safe_max_count=maximum,
                 direction_confirmed=True, endpoints_confirmed=True, position_mode_confirmed=True,
                 scale_confirmed=True)
    check_profile(value, ready=True)
    return value


def relative_goal(profile, servo_rad):
    check_profile(profile, ready=True)
    if not finite(servo_rad):
        raise ValueError('invalid relative servo angle')
    scale = profile['count_direction_per_cad_positive']*profile['counts_per_turn']/(2*math.pi)
    raw = profile['zero_count']+servo_rad*scale
    if not math.isfinite(raw):
        raise ValueError('relative count overflow')
    bounded = min(profile['safe_max_count'], max(profile['safe_min_count'], raw))
    position = math.floor(bounded+0.5)  # 양수 count: C++ lround와 동일
    return dict(position=position, clamped=raw != bounded,
                realized_relative_rad=(position-profile['zero_count'])/scale)


def covers(profile, servo_min, servo_max):
    check_profile(profile, ready=True)
    if not all(finite(v) for v in (servo_min, servo_max)) or not servo_min <= 0 <= servo_max:
        return False
    scale = profile['count_direction_per_cad_positive']*profile['counts_per_turn']/(2*math.pi)
    a, b = (profile['zero_count']+v*scale for v in (servo_min, servo_max))
    return profile['safe_min_count'] <= min(a, b) and max(a, b) <= profile['safe_max_count']


def _cad_contract(command_limit, wheelbase, track, wheel_limit):
    mapping = CadMapping.default()
    geometry = dict(wheelbase=wheelbase, track=track, wheel_limit=wheel_limit)
    limit = mapping.command_limit(**geometry) if command_limit is None else command_limit
    if not finite(limit) or limit <= 0:
        raise ValueError('invalid central command limit')
    ends = [mapping.central(sign*limit, **geometry) for sign in (-1, 1)]
    if any(goal.clamped for goal in ends):
        raise ValueError('outside CAD central command range')
    return mapping, geometry, limit, ends


def cad_servo_goal(profile, steering_rad, *, command_limit=None, wheelbase=.128,
                   track=.1239, wheel_limit=math.pi/6):
    """오프라인 목표 계산. 영점은 실장 관측, 링크 각도는 CAD 추정이며 송신하지 않는다."""
    check_profile(profile, ready=True)
    mapping, geometry, limit, endpoints = _cad_contract(command_limit, wheelbase, track, wheel_limit)
    if not finite(steering_rad) or abs(steering_rad) > limit:
        raise ValueError('outside CAD central command range')
    ends = [goal.servo_rad for goal in endpoints]
    if not covers(profile, min(ends), max(ends)):
        raise ValueError('required range crosses mounted or count limit')
    ideal = mapping.central(steering_rad, **geometry)
    goal = relative_goal(profile, ideal.servo_rad)
    actual = mapping.relative(goal['realized_relative_rad'])
    # 끝점에서 반 count 반올림이 CAD 범위 밖으로 나가면 1 count 안쪽을 사용한다.
    # 유효 최대 조향 명령을 오류/정지로 바꾸거나 count를 wrap하지 않는다.
    if actual.clamped or max(abs(actual.left_rad), abs(actual.right_rad)) > geometry['wheel_limit']:
        goal['position'] += -1 if goal['position'] > profile['zero_count'] else 1
        goal['realized_relative_rad'] = ((goal['position']-profile['zero_count'])*2*math.pi /
                                        (profile['count_direction_per_cad_positive']*4096))
        actual = mapping.relative(goal['realized_relative_rad'])
    if (goal['clamped'] or actual.clamped or
            max(abs(actual.left_rad), abs(actual.right_rad)) > geometry['wheel_limit'] or
            not profile['safe_min_count'] <= goal['position'] <= profile['safe_max_count']):
        raise ValueError('quantized target outside mounted range')
    return dict(position=goal['position'], central_requested_rad=steering_rad,
                servo_relative_rad=actual.servo_rad, left_target_rad=actual.left_rad,
                right_target_rad=actual.right_rad, central_realized_rad=actual.equivalent_rad,
                hardware_angle_measured=False, output_enabled=False)


def cpp_zero_profile(profile, *, assembly_id, command_limit=None, wheelbase=.128,
                     track=.1239, wheel_limit=math.pi/6):
    """실장 확인 JSON을 같은 C++ ZeroProfile 값으로 내보낸다. 출력/flash는 하지 않는다."""
    check_profile(profile, ready=True, assembly_id=assembly_id)
    # 미지정은 기존 CAD 전체 범위다. 운영 빌드는 board와 같은 명시 범위를 전달한다.
    # 중립뿐 아니라 선택한 양 끝의 count 양자화도 장착/기하 범위 안이어야 한다.
    _, _, limit, _ = _cad_contract(command_limit, wheelbase, track, wheel_limit)
    for steering in (-limit, 0.0, limit):
        cad_servo_goal(profile, steering, command_limit=limit, wheelbase=wheelbase,
                       track=track, wheel_limit=wheel_limit)
    canonical = json.dumps(profile, ensure_ascii=True, sort_keys=True, separators=(',', ':'), allow_nan=False)
    digest = hashlib.sha256(canonical.encode('ascii')).hexdigest()
    from .steering_cad_table import SOURCE_SHA256
    # 사용자 메모/장착명을 C++ 소스에 삽입하지 않는다. 연결된 프로필의 해시만 기록한다.
    text = ('#pragma once\n#include "steering_zero.hpp"\n'
            '// Generated mounted zero; no UART, torque, ARM or firmware output enable.\n'
            'namespace arena_steering_mount {\n'
            f'inline constexpr char kCanonicalProfileSha256[] = "{digest}";\n'
            f'inline constexpr char kCadTableSha256[] = "{SOURCE_SHA256}";\n'
            'inline arena_steering::ZeroProfile Profile() {\n'
            '  arena_steering::ZeroProfile p;\n'
            f'  p.zero={profile["zero_count"]}; p.minimum={profile["safe_min_count"]}; '
            f'p.maximum={profile["safe_max_count"]};\n'
            f'  p.direction={profile["count_direction_per_cad_positive"]}; p.counts_per_turn=4096;\n'
            '  p.straight_verified=p.direction_verified=p.endpoints_verified=true;\n'
            '  p.position_mode_verified=p.scale_verified=true;\n'
            '  return p;\n}\n}  // namespace arena_steering_mount\n')
    return text, digest
