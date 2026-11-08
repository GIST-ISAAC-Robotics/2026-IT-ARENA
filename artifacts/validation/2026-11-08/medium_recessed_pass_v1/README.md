# r15 중속 추월의 오목 시각 상대 1회 — 2026-11-08

**시험:** [r15 원상자 대표](../medium_pass_connected_v1/README.md)와 같은 CLI·B0390/C1·`medium_v1`·새 코스 grid3·오른쪽 12 cm·등속 0.50 m/s 상대 14초에서 **상대 시각 외형만 `recessed_sides`**(중앙 폭 2/3, 앞뒤 끝 원폭, 충돌·간격 평가는 외접 상자 그대로)로 바꿔 Gazebo를 1회 실행했다. 새 추월 제어기·기본값·물성·코스·판정·속도는 바꾸지 않았다. [사전 계획](PLAN.md).

**결과:** 보충 판정 전부 통과, 원시 판정은 r15와 같은 3항목 실패. 기록 C1 반환에서 오목 중앙면을 실제로 확인했다. 한 번의 실행이며 통계 보장이 아니다.

| 항목 | 이번(오목) | r15(원상자) |
|---|---|---|
| 진입 / 추월·복귀 확인(초록 뒤) | 2.218 s / 9.118 s | 2.219 s / 9.119 s |
| 복귀 후 진행 (판정 ≥ 0.50 m) | **1.969 m** | 2.008 m |
| 14초 진행 | 12.35 m | 11.19 m |
| 추월 구간 명령 상한 / 차체 최고·최저 | 0.90 / 0.920·0.632 m/s | 0.90 / 0.923·0.584 m/s |
| 진행 뒤 0.05 m/s 미만 최장 / 0명령 표본 | 0.0 s / 0 | 0.101 s / 7 |
| 최소 표본·스윕 간격 / 접촉 | 0.2109 m / 0 | 0.2108 m / 0 |
| 바퀴 노면 | 최대 두 바퀴 잔디·벽 바퀴 0 | 같음 |
| STOP·종료 | 명시 MCU STOP·밀집 정지·정상 종료 | 같음 |
| 원시 `report.json` | 실패: `final_gap`·`inside_road`·`ego_stays_behind` (exit 1) | 같은 3항목 실패 |

원시 판정은 옛 "끝까지 앞차 뒤"·최종 간격·차체 전체 도로 안 기준이며 수정하지 않았다. 최종 횡좌표 1.853 m는 r15와 같이 첫 코너를 돈 차를 출발 직선 좌표로 표시한 값이다. 근거: [보충 평가](straight_moving_lead/assessment_run1.json)·[노면](straight_moving_lead/surface_run1.json)·[원 보고](straight_moving_lead/run1/report.json)·[종료 코드](straight_moving_lead/run1.exit).

## 모델 변경 범위와 실제 관측

- **정적 XML:** 현재 `actor_sdf`의 box 생성본이 r15 실제 `evaluation_actor.sdf`와 바이트 동일하고, 오목 생성본은 visual을 뺀 나머지(충돌 0.2×0.15×0.12 m·접촉 센서·static·pose·플러그인)가 동일하다. `gz sdf -k` Valid(최상위 Contact 플러그인 배치 경고는 r1과 같음). [static_sdf_check.json](static_sdf_check.json)
- **실행 SDF:** 이번 `run1/evaluation_actor.sdf`가 정적 오목 생성본과 바이트 동일, visual 외 요소는 r15 실행 SDF와 동일. [runtime_sdf_check.json](runtime_sdf_check.json)
- **센서 관측:** [반환 대조](straight_moving_lead/outline_returns_run1.json) — 오목 중앙면 반환 **522개/고유 스캔 48개**(초록 뒤 3.04–8.11 s, 그 사이 상대 2.54 m 이동), 옛 외접 상자 중앙 옆면 위치 반환 0개, 폭 유지 끝면 158개. 자세 시각 ±10 ms 이동에도 520–521개/48–49스캔·옛 옆면 0–1개로 유지. 사전 판정 4조건 모두 참이다. 같은 helper로 r15 원상자는 오목 중앙 0·옛 옆면 583개였다([controls/](controls/)).

`outline_moving.py`는 원 [`audit_actor_outline_returns.py`](../../../../scripts/audit_actor_outline_returns.py)의 자세 읽기·광선별 선형 보간·8 mm 여유를 재사용하고, "상대 정지 후 3.5초" 창을 초록~사례 종료로 넓힌 artifact 전용 helper다. 정지 오목 r1에 원 창을 주면 원 함수 결과(592/31/0/139)와 같다. 상대 자세는 50 Hz 순간이동의 100 Hz 계단 기록이라 종방향 약 10 mm 오차가 있으나 직선·상대 yaw 변화 0이므로 횡방향 판정 띠에는 직접 들어가지 않는다. 자세 공백(> 50 ms)으로 버린 광선은 0이다. 정답 자세는 이 사후 평가에만 썼다.

## 출처와 무결성

- 입력: 설정은 r15에 `actor.visual_outline` 한 키만 추가([make_config.py](make_config.py)). CLI는 `--config` 경로 외 r15와 동일([register.py](register.py)). 등록 912입력·[snapshot](straight_moving_lead/source_snapshot.zip)·[등록](straight_moving_lead/registered.json), 실행 뒤 912입력 변경 0·snapshot 불변([postflight](straight_moving_lead/postflight_integrity_run1.json)).
- 실제 import: 두 Python 패키지 모든 모듈의 해석 경로·주요 노드 import·launch/config share·실행 파일 152항목이 등록 해시와 일치([runtime_check.json](straight_moving_lead/runtime_check.json)).
- 실행 전후 `gz sim`/`ros2 launch` 프로세스 없음(사전·사후·보조 에이전트 최종 확인). 실행·대기·회수는 Claude Haiku 5.5 보조(별칭 `haiku`, 자체 보고 `claude-haiku-5-5`)가 맡았고 자동 재시작은 없었다.
- 실행 전 검사: 외형 관련 기존 2파일 6검사 통과([outline_tests.log](outline_tests.log)). 대규모 회귀는 하지 않았다.

## 한계

- **단일 변수 비교가 아니다.** 코드는 현재 작업트리(r42)로 r15 등록 대비 소스 28개 변경·49개 추가([code_delta.json](code_delta.json)). 실행 전 r15 저장 판단 1회 재생은 748호출 중 551일치·197불일치였다. 저장한 앞 20건은 속도·조향이 같고 진단 필드 `current_lane_preference.context_lost`만 달랐으나, 나머지 177건은 분류하지 않았다([replay_r15_result.json](replay_r15_result.json)). 0명령 7→0·진행 11.19→12.35 m 차이를 외형이나 특정 코드 변경 효과로 귀속하지 않는다.
- 오목 25 mm 시각 외형 하나의 제한 장면이다. 다른 비정형 차체·상대 속도·더 빠른 상대(r31/r32)·지속 코너 병렬(r41)·독립 다중 차량·실물 형상/동역학은 확인하지 않았다. 충돌은 외접 상자이므로 오목부로 파고든 통과가 아니다.

**다음:** 조향 설계 문제는 보이지 않았다. 중속 직선 추월의 오목 외형 확인은 이 1회로 마치고(같은 장면 무변경 반복 없음), 미해결인 0.70 m/s 이상 상대 추월과 지속 코너 병렬로 돌아간다.
