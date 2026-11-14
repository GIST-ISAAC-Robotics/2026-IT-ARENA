# r503 r502 주시거리 후보의 오른쪽 옆차 대표 1회 — 시설 앞 정체 실패

2026-11-14, Claude Opus 5.5. r499와 같은 `right_side_start` 12 sim초 조건을 r502 후보 공통으로 **단1회** 실행했다. 원 평가와 기존 별도 목표가 모두 실패했다. r502 후보 명령은 실제로 한 번 선택됐으나, 다음 계획부터 신호 지주 쪽 점유 간섭으로 다시 정지했다. 재실행이나 코드 변경은 하지 않았다.

## 입력과 실행

- r499 대비 control 4파일(`moving_pass.py` r502 후보, r498 출력동등 최적화 3파일)과 sim에서 불러오지 않는 비제어 2파일만 다르다. 상세 차이와 근거는 [PLAN](PLAN.md)과 [동결 영수증](preflight/freeze.json)에 있다.
- 1,391입력 동결. `prepared.json` SHA `7130a18b4e34fa2c71a6ade178ba222cc6a3fd80026e99d6fba7b2056117f122`, `sources.zip` SHA `842b3b2bbdfc87d4141df3e5aa451947e6d6a3cfa277ff2764ac71c9236ea88f`. root 검토 [기록](root_execution_review.json) 후 실행했다.
- 실행 UTC 02:17:55–02:22:18(wall 263초), launcher PID 2933. GREEN 7.401–19.401 sim초. 원 validator exit 1은 판정 실패이며 수집 exit 0과 구분한다.
- 8 sim초 무진행 조기 STOP 기준에 도달하기 전에 창이 끝나(최장 무진행 관측 6.98초) 별도 STOP helper는 호출하지 않았다. 원 평가기가 명시 STOP과 정상 종료를 수행했다.

## 결과

| 항목 | r503 | r499(참고) |
|---|---|---|
| 원 평가 | **FAIL**: `no_permanent_stall`, `final_gap`, `inside_road` 실패 | FAIL: `progress`, `no_permanent_stall`, `final_gap` |
| 기존 별도 목표 | **FAIL**, 장면 미성립(`actually_alongside=false`), `passed_actor`/`final_follow`/`no_long_stall`/`post_return_progress` 실패 | 같음 |
| 진행 | 2.52 m(마지막 2초 진행 −0.00003 m), 최종 상태 `MOVING_PASS` | 2.46 m |
| 최장 정지 | 6.874 sim초 | 6.880 sim초 |
| 상대 간격 | 표본 최소 296.500 mm, swept 하한 296.499 mm | 같음 |
| 접촉·도로 | actor 접촉/겹침 0, 벽 접촉 0, 현 CAD 바퀴중심 도로밖 0 | 같음 |
| 속도·전달 | 최고 0.937 m/s, 명령 만료 0 | 0.937 m/s, 0 |
| STOP·회수 | STOP 6항목 통과, `shutdown_clean`, 잔여 PID 0 | 같음 |
| 무결성 | 실행 전후 1,391입력 변경 0, ZIP 불변 | — |

원 `progress` gate는 이번에 통과했고 `inside_road`가 새로 실패했다. 원 gate 정의를 바꾸거나 r499와의 차이를 개선으로 해석하지 않는다. 바퀴 지표는 계산상 값이며 실제 접촉이나 심판 판정이 아니다.

## 최초 정지

| runtime seq | sim초 | 출력 속도 | 출력 조향(rad) | 사유 |
|---|---:|---:|---:|---|
| 97 | 11.92 | 0.9 | −0.192 | `moving_pass_entry` |
| 98 | 12.02 | **0.674312** | **−0.340031** | `moving_pass_entry`(후보 명령 간격 0.32 m) |
| 99 | 12.12 | 0 | −0.340031 | `known_occupied_command_hold`, 후보 간격 0 |
| 100–101 | 12.22–12.32 | 0 | 유지 | 같음 |
| 102 이후 | 12.42– | 0 | 유지 | `entry_guide_current_wall_too_short`(후보 없음) |

- 종료까지 창 안의 계획 이유는 `entry_guide_current_wall_too_short` 70회, `no_entry_command_candidate` 3회, `moving_pass_entry` 15회, `handoff_follow` 56회였다. r499는 끝까지 점유 간섭 후보 거절이 이어졌지만, r503은 3회 뒤 외벽 guide 길이 부족으로 바뀌었다.
- seq98의 0.674312 m/s는 r502 저장 재생에서 seq99에 나온 값과 같다. 폐루프 시점은 저장 입력과 0.1초 어긋나 있으므로 같은 입력의 재현이라고 단정하지 않는다.
- [점유 재구성](failure_inputs/occupied_attribution.json): 후보가 있는 정지 3개(seq582/585/588)를 저장 scan·이력에서 다시 구성했다. 기록된 명령 간격 0을 재현했고, 첫 간섭점 7/8/9개의 최근접 평가 형상은 모두 **신호 지주**였다. 대표 첫 점은 자차 기준 `[0.204, 0.138]` m, 월드 `[9.421, 6.136]` m, 패딩 0.092 m, 지주까지 0, 벽까지 0.126 m, actor까지 0.550 m다. 최근접 형상에 의한 귀속 추정이며 센서의 물체 식별 증거가 아니다.
- r499 도구를 재사용하되 후보가 없는 정지는 재구성 대상에서 뺐다(`attribute_occupied.py`의 필터 한 줄). 제어기 `command`와 시뮬레이터는 다시 실행하지 않았다.

## 자료

- [원 결과](run1/report.json), [실행 요약](execution_summary.json), [별도 목표](assessment_summary.json), [바퀴 기하](surface_run1.json), [무결성](integrity.json), [최종 소유 프로세스](execution_v1/owned_processes_after.json).
- [최초 정지 요약](failure_inputs/summary.json)과 원문 바이트 `selected_records.jsonl`; 원시 scan(`selected_raw_scans.json`)과 상세 로그는 기존 제외 정책에 따라 로컬에만 보존한다.
- r499 동결 artifact 48개는 변경 0이다. r499 폴더에는 쓰지 않았다.

이 결과는 220 mm 상대 가정의 우측 옆차 시뮬레이션 1회 실패다. 전체 교통 랩, 낮은 차량, 실차, 실시간 10 Hz의 증거로 확대하지 않는다. 후속 설계와 채택 판단은 root가 맡는다.
