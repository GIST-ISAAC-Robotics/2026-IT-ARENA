"""v4 test_candidate.py(37건)를 런타임 모듈 경로로 옮기고 10/30 연결·경계 반례를 덧붙인다.

생성 결과는 tests/test_measured_lane_preference_trial.py이며, 이 스크립트는 생성 경위를 남기기 위한 것이다.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
V4 = ROOT/'artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4/test_candidate.py'
EXTRA = Path(__file__).resolve().parent/'runtime_tests_extra.py.txt'
TARGET = ROOT/'tests/test_measured_lane_preference_trial.py'

PAIRS = [
    ('"""후방 소실 직후 차선 유지 후보의 상태 계약 반례. 주입 view는 분기만 검증하며 센서 인식 증명이 아니다."""',
     '"""observed_measured_v1(10/29 오프라인 v4의 10/30 연결본) 반례. 주입 view는 분기만 검증하며 센서 인식 증명이 아니다.\n\n'
     '앞부분은 v4 test_candidate.py의 37건을 런타임 모듈 경로로 옮긴 것이고, 뒤에 연결·목표 상한 경계 반례를 더했다.\n"""'),
    ('import candidate as lk\nimport parallel_speed\n',
     'import arena_autonomy.measured_lane_preference_trial as lk\nimport arena_autonomy.measured_parallel_speed_trial as parallel_speed\n'),
    ("    return FollowConfig(**{**CHAIN, 'rear_lane_keep_trial': True, **kwargs})",
     "    return FollowConfig(**{**CHAIN, 'rear_lane_keep_trial': True, 'rear_lane_keep_policy': 'observed_measured_v1', **kwargs})"),
    ('    from parallel_state import MeasuredParallelTrial\n',
     '    from arena_autonomy.measured_parallel_trial import MeasuredParallelTrial\n'),
]


def main():
    source = V4.read_text(encoding='utf-8')
    for before, after in PAIRS:
        assert source.count(before) == 1, before
        source = source.replace(before, after)
    text = source.rstrip('\n')+'\n\n'+EXTRA.read_text(encoding='utf-8')
    TARGET.write_bytes(text.encode('utf-8'))


if __name__ == '__main__':
    main()
