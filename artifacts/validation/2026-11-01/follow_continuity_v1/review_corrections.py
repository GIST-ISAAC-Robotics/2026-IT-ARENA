"""최종 검토 권고 네 가지를 보고서에 반영한다. 측정값·원시 기록·런타임 소스는 바꾸지 않는다."""
from pathlib import Path

PATH = Path(__file__).resolve().parent / 'README.md'
EDITS = [
    ('0.5 m/s는 기존 `object_motion_bound_mps`를 다시 쓴 팀 가정이고 앞차 속도 측정이 아니다. 앞차가 더 빠르면 간격이 벌어질 뿐이다. 추종 중에는 앞차 속도로 달리므로 거리 자체가 시간 손실은 아니며, 이 거리가 아래의 우회 공간이 된다. 조정 없이 기존 값을 썼고, 판정선을 맞추려고 고르지 않았다.',
     '0.5 m/s는 기존 `object_motion_bound_mps`(물체 이동의 **상한** 가정)를 다시 쓴 시험용 진행 가정이다. 앞차 속도 측정이 아니고 **실제 앞차 속도의 하한도 아니다.** 옵션 이름 `moving_pace_floor`와 코드 주석은 그대로 두었으며, 여기서 floor는 명령 상한식에 더하는 값이라는 뜻이다. 앞차가 이보다 빠르면 간격이 벌어지고, 느리거나 멈춘 물체에는 이 값이 실제보다 클 수 있다.\n\n'
     '**짧은 미확정 구간의 한계:** 연속 3스캔이 쌓이기 전(0.2–0.3초)에는 정지 물체에도 고정 여유 대신 이 식이 적용되고, 단일 스캔에서는 상한이 아예 없다. 이 구간의 제동은 G1의 직접 충돌 판단과 독립 보호층이 맡으며, 우회 공간은 그만큼 늦게 확보된다. 추종 중에는 앞차 속도로 달리므로 거리 자체가 시간 손실은 아니고, 이 거리가 아래의 우회 공간이 된다. 조정 없이 기존 값을 썼고, 판정선을 맞추려고 고르지 않았다.'),
    ('**원래 판정 그대로:** 코너는 v1 전체 통과',
     '**표의 0.30초 최대 감속 정의:** 고밀도 자세(2–4 ms 간격)를 20 ms 격자에 놓고 ±30 ms 중심 차분한 차체 속도로, 각 구간 안에서 끝점이 구간 시작 + 0.30초 이후인 모든 창의 최댓값이다([`compare_runs.py`](compare_runs.py)). 앞선 보고서의 0.294·0.988·0.959·0.881·0.666 m/s는 10 Hz `trajectory.jsonl`의 `ground_speed_mps`를 선형 보간한 값이다([`bindings.py`](../rolling_lead_v1/bindings.py), [`compare.py`](../race_follow_closing_v1/compare.py)). 표본 간격과 창 위치가 달라 같은 실행도 값이 다르며(run1 6–13초 0.294 ↔ 0.517), 이전 보고서 수치를 교체하지 않는다. 두 표의 숫자끼리 직접 비교하지 않는다.\n\n'
     '**원래 판정 그대로:** 코너는 v1 전체 통과'),
    ('재출발 뒤 앞차 이동이 0.44 m라 1.0 m 판정은 이 각본에서 통과할 수 없는 값이다.',
     '`progress_after_restart`는 재출발 뒤 자차 진행 1.0 m 이상이라는 원래 임계를 이 각본의 잔여 1.25초 창에 그대로 쓴 결과로, 기준 0.380 m·v2 0.340 m로 미통과했다. 임계와 창이 맞지 않은 등록 각본의 부적합이다. 앞차 이동 0.44 m는 자차의 재가속·우회 진행 상한이 아니다.'),
    ('16건은 [수정 전 복원본](pytest_failed_on_before.log)에서도 같은 항목이 실패해 이번 변경과 무관하며(`vehicle.yaml` 등 앞선 미커밋 변경의 해시·의존성 사슬 검사와 C++ 시험 1건) 고치지 않았다.',
     '16건은 [수정 전 복원본](pytest_failed_on_before.log)에서도 같은 항목이 실패해 이번 변경과 무관하며 아래 [분류](#기존-실패-16건-분류)대로 고치지 않았다.'),
    ('## 미검증과 다음 재개\n',
     '## 기존 실패 16건 분류\n\n'
     '이번 후보나 대표 코스 실행에 직접 영향을 주는 항목은 없어 새 관문으로 전수 수정하지 않았다. 다만 나중에 갚아야 할 기초 부채다. 시험이 실패한 채로 남으면 진짜 회귀를 가릴 수 있다.\n\n'
     '| 분류 | 건수 | 시험 | 원인 | 정리할 때의 계약 |\n'
     '|---|---:|---|---|---|\n'
     '| 과거 계약 사슬 미연장 | 7 | `test_corner_follow`, `test_g1_dense_stop`, `test_physics_step_option`, `test_measured_lane_preference_provenance`, `test_r0_safety_contract`, `test_scheduled_apply`, `test_scheduled_command` | 각 시험의 자체 사슬 목록이 옛 시점에 멈춰, 이후 후속 계약으로 바뀐 현재 바이트를 설명하지 못함 | 시험별 목록을 없애고 `M3_SUCCESSORS` 한 경로로 되돌려 검사 |\n'
     '| 허용 변경 목록 고정 | 2 | `test_multi_vehicle_g0` 두 건 | G0 기준 목록에 11/1의 `vehicle.yaml` 변경이 없음 | 과거 기준 재현 조건과 현재 소스 조건을 분리 |\n'
     '| 전이 import 목록 누락 | 4 | `test_follow_profile`, `test_follow_profile_bundle`, `test_official_start_ros_fixture`, `test_pipeline_fault_validation` | `local_follow`가 11/1 새 파일 `turn_preview`를 import하나 각 묶음 목록에 없음 | 목록을 손으로 쓰지 않고 정적 import 스캔으로 생성 |\n'
     '| 공식 실행 월드 manifest | 1 | `test_official_track` | 입력 해시(차량 설정 등)가 달라져 재생성을 요구 | 월드 형상에 영향 없는 입력을 분리하거나, 공식 월드 재생성을 별도 결정으로 수행 |\n'
     '| 정적 문자열 검사 | 1 | `test_bypass_corridor_trial` | launch에 `g1_g2_corridor_policy` 문자열이 생겨 부분 문자열 금지에 걸림. 기본값은 여전히 `off` | 문자열 대신 기본 launch의 실제 선택 결과를 검사 |\n'
     '| legacy C++ 수식 비교 | 1 | `test_zero_speed_modes[legacy]` | 과거·현재 `SingleMotor`를 각자 기본값으로 만들어 비교하는데 기본 감속비가 15 → 8.97로 바뀜. [같은 매개변수를 명시하면 5만 표본 불일치 0](../course_compare_v1/legacy_probe_check.json)이라 구동식 변경은 아님 | 양쪽에 같은 매개변수를 명시해 "기본값 동일"이 아니라 "수식 동일"을 검사 |\n\n'
     '앞 세 분류는 소스 계약의 전제이므로 경기 구성 고정 전에 한 묶음으로 정리하는 편이 낫다. 실행 시점의 소스는 각 실행의 등록 해시와 `source_snapshot.zip`이 별도로 보존한다.\n\n'
     '## 미검증과 다음 재개\n'),
]

data = PATH.read_bytes()
newline = '\r\n' if data.count(b'\r\n') > data.count(b'\n') / 2 else '\n'
for old, new in EDITS:
    old_bytes = old.replace('\n', newline).encode('utf-8')
    new_bytes = new.replace('\n', newline).encode('utf-8')
    if data.count(old_bytes) != 1:
        raise ValueError(f'기대 원문이 한 번이 아님: {old[:50]!r}')
    data = data.replace(old_bytes, new_bytes)
PATH.write_bytes(data)
print('corrected', len(EDITS))
