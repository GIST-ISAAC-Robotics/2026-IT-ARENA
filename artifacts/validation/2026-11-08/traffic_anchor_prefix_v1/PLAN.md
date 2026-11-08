# 교통 코너 구간 44초: 경로 앵커 표현 전환 r23

사전 등록: 2026-11-08, 실행 전. 짧은 범위의 폐루프 1회다.

## 기존 실패 (보존)

[traffic_lap_anchor_v1](../traffic_lap_anchor_v1/README.md) run1(r22)은 0.5 m/s 각본 상대 2대와 96초를 주행했다. 18.98 m 지점에서 약 59초 동안 정체했다. 원인은 다음과 같다. 선택형 경로 표현이 `piecewise_corner`에서 일반 매개곡선으로 바뀌면 `path_anchor_error_m`가 빠졌다. 그 결과 옛 벽 y좌표 검사로 돌아갔다. r23(`config/tests/corner_path_anchor_transition_r23_sha256.json`, `local_path.py`·`local_follow.py`)은 같은 원점→반환 경로 거리 지표를 일반 곡선에도 제공한다. 저장 입력 재생 근거는 [corner_path_anchor_transition_v1](../corner_path_anchor_transition_v1/README.md)이다. 개루프 대조이며 폐루프 통과 증명이 아니다. 원 r22 실패와 그 소스·결과는 그대로 보존한다. 이번 결과를 96초 시험의 소급 성공으로 쓰지 않는다.

## 이번 범위

- 초록 이후 44초만 실행한다. 두 상대의 schedule은 `to_s=44`, case는 `duration_s=44`이다.
- 사례 이름은 `two_actor_corner_prefix`이다. `route_kind=first_curve`는 기존 제한 경로 검사로, 이름은 legacy지만 등록한 어떤 코너 구간에도 쓴다. 주 상대와 추가 상대에 모두 적용한다.
- 예상 경로: 주 상대 s 43.26→65.26(원 s 20.62), 추가 상대 49.26→71.26(26.62). 중심선 한 랩은 44.64 m이다.

## 파라미터 불변

r22 run1과 같은 조건을 쓴다.

- 초기 자세: grid 3, 새 공식 코스, 610 mm, 평탄 노면.
- 차량·센서: JR60P20/B0390, C1 순차 10 Hz. 모터축 28카운트, 8.970588:1, 65 mm.
- 속도·이동 프로필: `g1_open_1p5`, `medium_v1`.
- 플래그: `--handoff-wall-witness`를 포함한 기존 모든 flag.
- 상대: 초기 1.6/7.6 m 앞, 0.5 m/s, 중앙선. 크기·높이·발행 주기는 바꾸지 않는다.

경로 좌표, 12 cm 기준, 점유, 독립 보호, TTL, 속도, 물성은 바꾸지 않는다. 제어기 소스는 r23 계약 그대로다. 제어기에는 지도·정답 입력이 없으며, 각본·중심선·도로 다각형은 평가 전용이다.

## 사전 기준 (실행 전 고정)

case acceptance `two_actor_corner_prefix`는 `two_actor_course_lap` 내용을 복사한다. 여기서 `minimum_laps`·`maximum_actor_laps`를 제거하고 다음 값을 둔다.

- `required_exit_s_m=18.5`
- `maximum_actor_s_m=27.0`
- `minimum_heading_change_deg=90`
- `minimum_progress_m=21.5`

추가 상대도 `check_additional_routes`를 통해 같은 case 규칙으로 검사한다. 이 연결은 geometry.py에서 확인한다.

assess.py 판정은 기존과 같다.

- 입력·상대 유효, 추가 상대 유효.
- 접촉 0, 겹침 0. 표본 간격과 쓸림 하계는 모두 80 mm 이상.
- 바퀴 잔디는 최대 2개, 벽 0.
- 출발 뒤 연속 저속(<0.05 m/s)은 2.5초 이하.
- STOP과 정상 종료, 입력 해시 불변, 프로세스 정리.

`one_lap` 대신 `prefix_progress ≥ 21.5 m`를 쓴다. 이 값은 이전 18.98 m 정체 지점을 약 2.5 m 지나야 한다는 사전 목표다. 진행 거리는 보충 평가로 계산한다. `scripts/g1_course_eval.full_course_progress`에 조밀 `ego_poses.jsonl`/`actor_poses.jsonl`을 넣는다. 첫 코너 원평가는 반 랩을 넘으면 unwrap 한계가 있기 때문이다. 원 `raw_gates`는 별도로 보존한다. 하네스 오류는 제어기 실패와 섞지 않는다.

성공해도 전체 랩 완료, 경기 전체, 실차 증명으로 확대하지 않는다. 실패하면 재시도하지 않고 원시 증거와 첫 정체 원인만 회수한다.
