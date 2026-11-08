# 사전 계획 — r15 중속 추월을 오목 시각 상대 하나로 확인 (2026-11-08)

## 목적과 범위

[r15 원상자 대표](../medium_pass_connected_v1/README.md)와 같은 장면(새 코스 grid3, 오른쪽 12 cm·등속 0.50 m/s 상대 14초, `--moving-pass-profile medium_v1`)을 **상대 시각 외형만 `recessed_sides`로 바꿔** 한 번 실행한다. 새 추월 제어기·기본값·물성·코스·간격/성공 기준·시험 속도는 바꾸지 않는다. 2회 이상의 통계 보장이 아니다.

## 고정한 입력

- 설정: r15 `new_track_course.json`에 `actor.visual_outline='recessed_sides'` 한 키만 추가([make_config.py](make_config.py)가 다른 값 변화를 거부).
- CLI: r15 등록 `args`와 동일, `--config` 경로만 교체([register.py](register.py)가 나머지 인자 동일성을 assert).
- 코드: **현재 작업트리(r42) 코드**이다. r15 등록 868입력 중 28개 소스가 바뀌었고 이후 새 모듈도 있다([code_delta.json](code_delta.json)). 따라서 이 결과는 r15와의 완전한 단일 변수 인과 비교가 아니다.
- 실행 전 r15 저장 판단 1회 재생: 748호출 중 551일치·197불일치. 저장한 앞 20건은 속도·조향이 동일하고 진단 필드 `normal/current_lane_preference/context_lost`만 달랐다. 나머지 177건은 1회 재생 제한으로 분류하지 않았다([replay_r15_result.json](replay_r15_result.json)). 전체 메타데이터 동일성과 속도/조향 비교는 구분한다.

## 판정 (실행 전 고정, 실행 뒤 낮추지 않음)

1. **원시(raw):** 검사기 `report.json`의 `passed/gates`를 그대로 보고한다(r15도 원 최종 간격·차체 도로 안·끝까지 뒤 판정 실패, exit 1).
2. **보충 판정:** r15 [assess.py](assess.py) 그대로 — 진입→옆→앞→현재 관측 복귀→0.50 m 이상 전진, 접촉 0, 최대 두 바퀴 잔디·벽 바퀴 0, 진행 뒤 0.05 m/s 미만 0.50초 이하, 명시 STOP/정상 종료, 상대 일정 0.45–0.55 m/s, B0390·505/305 단면 적용, 입력 불변·잔여 프로세스 없음.
3. **모델 변경 범위:** 정적 XML([static_sdf_check.json](static_sdf_check.json))과 실행 SDF(`runtime_sdf_check.json`)에서 visual 외 요소(충돌 외접 상자·접촉 센서·static·pose·플러그인)가 r15와 동일하고, 오목 visual은 중앙 폭 2/3·앞뒤 끝 원폭·외접 상자 안일 것.
4. **센서가 실제 오목면을 보았는가(별도):** [outline_moving.py](outline_moving.py) — 오목 중앙 반환 ≥ 2, 그 고유 스캔 ≥ 2(다른 시각/상대 위치), 오목 중앙 반환 > 옛 외접 상자 중앙 옆면 반환, 자세 시각 ±10 ms 이동에도 앞 두 조건 유지. 성립하지 않으면 "형상이 실제로 검증됨"이라 부르지 않는다.

## helper 근거

원 [audit_actor_outline_returns.py](../../../../scripts/audit_actor_outline_returns.py)는 "상대 정지 후 3.5초" 창을 쓴다. 광선별 보간·actor 좌표 변환·8 mm 여유는 그대로 재사용하고 창만 초록~사례 종료로 넓혔으며, 광선 시각을 감싸는 자세 표본 간격 > 50 ms이면 버린다. 실행 전 대조([controls/](controls/)):

- 양성(정지 오목 r1, 원 함수와 같은 3.5초 창): 오목 중앙 592/고유 31·옛 옆면 0·끝 139 — 원 함수 결과와 동일.
- 음성(r15 움직이는 원상자): 오목 중앙 0·옛 옆면 583/고유 64 → 판정 거짓. 원 함수도 거짓. +10 ms 이동 시 오목 중앙 2건이 생기지만 옆면 573–583건보다 작아 판정 거짓 유지.
- 상대 자세는 100 Hz 기록이나 50 Hz 순간이동이라 계단형이다(표본 간 이동 0 또는 약 10 mm). 직선 경로·상대 yaw 변화율 0이라 이 오차는 상대 종방향(x)에만 생기고 오목 판정의 횡방향(y) 띠에는 직접 들어가지 않는다.

## 실행 절차

등록 뒤 [run.sh](run.sh)로 Gazebo **한 번만** 실행한다. 사전 `pgrep`·실제 import/share/실행파일 대조([check_runtime.py](check_runtime.py))가 실패하면 주행 없이 환경 오류로 기록한다. 실행 중 오류·행동 실패에도 자동 재시작하지 않는다. 종료 뒤 등록 입력·snapshot 해시 불변, 잔여 프로세스(있으면 기록 후 종료)를 확인한다. 단순 실행·대기·회수는 Claude Haiku 5.5 보조 에이전트가 맡는다.
