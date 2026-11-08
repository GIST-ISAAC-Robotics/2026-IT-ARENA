"""새 코스 대표 교통 장면 재등록 제안(고정 파일) 생성. 실행 등록(registered.json)은 하지 않는다.

r16 각본(moving_pass_withdrawal_v1/course.json)을 바이트 기준으로 읽어 track 의존 항목만 바꾼다.
- track=official_v2026_11_06, first_corner_s_m은 새 중심선의 첫 직각 꼭짓점(평가 전용, geometry_probe.json)
- 장면 2개만: 0.30 m/s 중앙 앞차 추종(slow_lead), r16 오른쪽 0.12 m 저속→가속 앞차(straight_moving_lead)
- 판정·상대 크기·출발 간격·시드·시간 한도·r16 실행 인자는 그대로. 센서만 B0390 프로필로 명시
"""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
REL = HERE.relative_to(ROOT).as_posix()
SOURCE = 'artifacts/validation/2026-11-05/moving_pass_withdrawal_v1/course.json'
R16 = 'artifacts/validation/2026-11-05/moving_pass_withdrawal_v1/candidate_r16/registered.json'
TRACK = 'official_v2026_11_06'
SENSOR = 'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
OLD_SENSOR = 'src/arena_description/config/b0183_c1_jr60p20_sim.yaml'
CONFIG = f'{REL}/new_track_course.json'


def sha(path):
    return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()


def once(path, data):
    text = json.dumps(data, ensure_ascii=False, indent=2)+'\n'
    if path.exists():
        assert path.read_text(encoding='utf-8') == text, f'{path} 내용이 다르다. 덮어쓰지 않는다'
    else:
        path.write_text(text, encoding='utf-8', newline='\n')


def course():
    base = json.loads((ROOT/SOURCE).read_text(encoding='utf-8'))
    geometry = json.loads((HERE/'geometry_probe.json').read_text(encoding='utf-8'))[f'it_arena_{TRACK}']
    corner = next(row['to_s'] for row in geometry['straights_over_1m'] if row['from_s'] == 0.0)
    moving = next(case for case in base['cases'] if case['name'] == 'straight_moving_lead')
    follow = dict(name='slow_lead', behavior_kind='slow_lead', route_kind='straight', duration_s=14.0,
                  schedule=[dict(from_s=0.0, to_s=14.0, speed_mps=0.30)],
                  expected='새 출발 직선 중앙 0.30 m/s 앞차. 영구 정지 없이 간격을 유지하며 따라간다. '
                           'r16과 같은 제어 구성이므로 추월 시도 여부는 결과로 기록한다.')
    assert base['acceptance']['slow_lead']['basis'].startswith('lead travels 4.2 m') and .30*14.0 == 4.2
    return {**base,
            'registered_at': '2026-11-07 새 코스 대표 교통 재등록 제안(미실행)',
            'scope': '평가 각본 전용: v2026.11.06 출발 직선의 0.30 m/s 중앙 앞차 추종과 r16 오른쪽 0.12 m 저속→가속 앞차 각 1회.',
            'track': TRACK,
            'derived_from': dict(path=SOURCE, sha256=sha(SOURCE), changed_keys=[
                'registered_at', 'scope', 'track', 'derived_from', 'vehicle_and_sensor_unchanged', 'cases',
                'first_corner_s_m', 'first_corner_basis', 'matrix']),
            'vehicle_and_sensor_unchanged': '모터축 단일 28카운트·8.970588:1·65 mm, C1 10 Hz 순차, 카메라 B0390 '
                                            f'프로필({SENSOR}). 실물 조립값 아님.',
            'cases': [follow, moving],
            'first_corner_s_m': corner,
            'first_corner_basis': f'{REL}/geometry_probe.json: 새 중심선 s=0 직선 끝(첫 직각 꼭짓점). 평가 전용, 제어 입력 아님',
            'matrix': {'g1g2': {'cases': ['slow_lead', 'straight_moving_lead'], 'seeds': [0]}}}


def runs():
    r16 = json.loads((ROOT/R16).read_text(encoding='utf-8'))['args']
    assert r16.count(OLD_SENSOR) == 1 and r16.count(SOURCE) == 1 and '--track' not in r16
    result = {}
    for case in ('slow_lead', 'straight_moving_lead'):
        args = [CONFIG if value == SOURCE else SENSOR if value == OLD_SENSOR else value for value in r16]
        args[args.index('--case')+1] = case
        result[case] = args[:args.index('--output')]+['--track', TRACK]+args[args.index('--output'):]
    return dict(scope='제안 실행 인자. r16 등록 인자에서 --config·--local-sensor-config·--case만 바꾸고 --track을 더했다.',
                r16_registered=R16, r16_registered_sha256=sha(R16), runs=result)


if __name__ == '__main__':
    once(HERE/'new_track_course.json', course())
    once(HERE/'proposed_runs.json', runs())
    print('prepared', CONFIG)
