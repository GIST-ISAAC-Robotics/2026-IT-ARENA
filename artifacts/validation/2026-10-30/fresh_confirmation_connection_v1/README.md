# 확인 기억 v2 명시 정책 연결

2026-10-30. [오프라인 v2](../fresh_confirmation_body_v2/README.md)를 `rear_lane_keep_trial=true`, `rear_lane_keep_policy='observed_fresh_v2'`일 때만 사용하도록 연결한다. 기본 OFF/`timed`와 기존 `observed`·`observed_measured_v1`은 유지한다. 새 정책의 선택기는 기존 측정속도 후보와 같으며, 평행 후보의 확인 기억만 바꾼다.

- `fresh_parallel_trial.py`: 같은 raw 광선 전체의 강체 변환과 차체 외곽 이동을 확인하고, 제한된 확인1회를 다음 fresh 스캔에 넘긴다. 운영 구현은 오프라인 v2의 임시 record 치환 없이 직접 조건을 계산한다.
- `fresh_parallel_speed_trial.py`: 기존 속도 제안 복사본에 **분류 전용** `padding_reference_s`를 추가한 오프라인본과 같다. 실제 명령은 이 인자를 주지 않고 현재 시각의 원래 팽창으로 계산한다.
- `local_follow.py`: 새 정책의 명시 선택만 추가했다. 기록 헤더의 정책 문자열이 재생에도 유지된다.
- 보존/재생/이관 도구4개에 새 모듈2개를 추가했다.
- [후속 계약](../../../../config/tests/fresh_confirmation_r2_sha256.json)은 기존 계약·해시를 변경하지 않고 연결 전 바이트와 오프라인 원본을 연결한다. `before_connection.zip`·`offline_sources.zip`을 보존한다.

## 연결 검증 결과

[고정 실행](check_connection.sh) r1: 관련 회귀 → 과거 OFF/2초/관측 v3/측정속도 v1 기록 동일 재생 → 오프라인 v2와 새 정책 명령·선택 이유·기억 보존 대조 → 빌드/실제 설치 대조. 어느 단계든 실패하면 다음 단계로 넘어가지 않는다. 새 Gazebo 주행은 별도 결과로 기록한다.


r1 모든 단계 종료0. 관련 회귀622건이 통과했다. 과거 OFF649/2초653/관측v3 652/측정속도v1 654판단이 각각 기록과 동일했다. 새 명시 정책과 오프라인v2는 오른쪽 Gazebo 입력662·복합 오른쪽631·앞차 정지/재개781호출에서 명령/선택 이유/확인 보존이 같았다. 이는 저장 입력 비교이며 새 주행 성과가 아니다.

`arena_autonomy` 빌드와 실제 설치11모듈의 바이트 대조, 기본 OFF/`timed`, 네 정책의 선택 클래스를 확인했다. [GPT 인계 검토](gpt_connection_review.json)는 새 계약과 현재/보존 해시, 기존 세 모듈 보존, 실행 설정 동일과 단계 결과를 독립 대조했다. 전체 저장소 회귀를 다시 실행한 것은 아니다. 원시 호출별 재생 보고서는 로컬 보존이다.

다음은 [동일 조건 오른쪽 Gazebo 한 번](../fresh_confirmation_gazebo_v1/DRIVE_PLAN.md)이다.
