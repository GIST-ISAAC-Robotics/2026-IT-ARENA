# r513 r512 연속 24cm 주시 후보의 오른쪽 옆차 대표 1회 — 외벽 기준 소실 정체 실패

2026-11-14, Claude Opus 5.5. r499/r503과 같은 `right_side_start` 12 sim초 조건을 r512 공통 후보로 **단1회** 실행했다. 원 평가와 기존 별도 목표가 모두 실패했다. r503의 신호 지주 점유 간섭 정지는 이번에 나타나지 않았지만, 다음 계획부터 현재 외벽 기준이 짧다는 이유로 후보가 없어져 정지했다. 재실행이나 코드 변경은 하지 않았다.

## 입력과 실행

- r503 동결 대비 control `moving_pass.py`(`f567876f`→`bcd58132`, r512 `validation.json`과 같음)와 비제어 시험 `tests/test_moving_path_preview.py`(`68c16dab`→`e956d5ed`)만 다르다. 나머지 공통·설정·case/actor/acceptance/시설·기하는 r503(=r499)과 같다([PLAN](PLAN.md), [동결](preflight/freeze.json)).
- 1,408입력 동결. `prepared.json` SHA `53cefe142ce43b7082c47d872c91f71c04aa2e703b57489dfd514debd67d10e2`, `sources.zip` SHA `468bd51b92bd6fae3c756ed0934842d89b363a561762de75d018a7ef4691f185`.
- 실행 검토 [기록](root_execution_review.json)은 root 위임에 따라 실행 담당이 작성했다. root는 recipe와 후보 소스 해시를 검토했고, 동결 뒤 ZIP·무결성 대조는 실행 담당이 수행했다. root가 ZIP을 직접 대조한 것은 아니다.
- 실행 UTC 02:35:24–02:39:53(wall 269초), launcher PID 5822. GREEN 7.401–19.401 sim초. 원 validator exit 1은 판정 실패이며 수집 exit 0과 구분한다.
- 관측된 최장 무진행은 6.98 sim초였다. 8 sim초 조기 STOP 기준에 도달하기 전에 창이 끝나 STOP helper는 호출하지 않았다. 원 평가기가 명시 STOP과 정상 종료를 수행했다.

## 결과

| 항목 | r513 | r503 / r499(참고) |
|---|---|---|
| 원 평가 | **FAIL**: `progress`, `no_permanent_stall`, `final_gap`, `inside_road` | r503: `no_permanent_stall`/`final_gap`/`inside_road`; r499: `progress`/`no_permanent_stall`/`final_gap` |
| 기존 별도 목표 | **FAIL**, 장면 미성립(`actually_alongside=false`), `passed_actor`/`final_follow`/`no_long_stall`/`post_return_progress` 실패 | 같음 |
| 진행 | 2.45 m, 최종 상태 `MOVING_PASS`, 최종 중심선 편차 0.124 m | 2.52 m / 2.46 m |
| 최장 정지 | 6.774 sim초 | 6.874 / 6.880 sim초 |
| 상대 간격 | 표본 최소 296.500 mm, swept 하한 296.499 mm | 같음 |
| 접촉·도로 | actor 접촉/겹침 0, 벽 접촉 0, 현 CAD 바퀴중심 도로밖 0 | 같음 |
| 속도·전달 | 최고 0.929 m/s, 명령 만료 0 | 0.937 m/s, 0 |
| STOP·회수 | STOP 6항목 통과, `shutdown_clean`, 잔여 소유 PID 0 | 같음 |
| 무결성 | 실행 전후 1,408입력 변경 0, ZIP 불변. r503 동결 artifact 32개·r499 35개 변경 0 | — |

바퀴 지표는 계산상 값이며 실제 접촉이나 심판 판정이 아니다. 원 gate 정의를 바꾸지 않았고, r499/r503과의 수치 차이를 개선이나 악화로 해석하지 않는다.

## 최초 정지

| runtime seq | sim초 | 출력 속도 | 출력 조향(rad) | 사유 |
|---|---:|---:|---:|---|
| 87–91 | 10.92–11.32 | 0.9→0.810 | +0.247→+0.113 | `moving_pass_entry` |
| 92–96 | 11.42–11.82 | 0.9→0.810 | −0.062→−0.455→−0.384 | `moving_pass_entry` |
| 97 | 11.92 | 0.810330 | −0.218986 | 후보 명령 간격 0.40 m |
| 98 | 12.02 | 0.9 | −0.009495 | 후보 명령 간격 1.92 m |
| 99 이후 | 12.12– | 0 | −0.009495 유지 | `entry_guide_current_wall_too_short`, 후보 없음 |

- 창 안의 계획 이유는 `entry_guide_current_wall_too_short` 73회, `moving_pass_entry` 15회, `handoff_follow` 56회였다. r503에 있던 `no_entry_command_candidate`(점유 간섭 거절)는 0회다.
- 직전 seq97/98의 현재 왼쪽 외벽 적합은 반환 164/146개, x 범위 약 −0.24–1.71 m / −0.12–1.76 m, 잔차 3.6/2.0 mm였다. seq99부터는 guide 기준과 외벽 정보가 비어 있다. 그 사이에 기준이 짧다고 판정된 정확한 내부 원인은 이번 회수에서 재생하지 않았다.
- seq99 시점의 상대 표적은 현재 반환으로 관측 중이었다(`current_associated_returns`, 자차 기준 x 0.474–0.661 m, y −0.337–−0.144 m, `ahead_clear=false`). 평가용 정답으로는 actor 중심이 자차 기준 `[+0.620, −0.233]` m였다. 실제 접촉은 없었다.
- r503용 [점유 재구성](execution_v1/attribute_occupied.py)은 후보가 있는 정지만 대상으로 한다. 이번에는 그런 정지가 0개라 입력 선택 단계에서 종료했다(exit 1, [로그](execution_v1/attribute.log), 출력 없음). 이는 분석 도구의 적용 범위 문제이며 실행 결과에 영향이 없다. 신호 지주 귀속은 이번에 하지 않았다.
- [최초 정지 요약](failure_inputs/summary.json): 선행 양수 2개와 최초 정지 5개, 상대 좌표·상태·scan 개수. 원문 바이트 `selected_records.jsonl`과 원시 scan은 기존 제외 정책에 따라 로컬에만 보존한다.

## 자료

- [원 결과](run1/report.json), [실행 요약](execution_summary.json), [별도 목표](assessment_summary.json), [바퀴 기하](surface_run1.json), [무결성](integrity.json), [최종 소유 프로세스](execution_v1/owned_processes_after.json), [감시 기록](execution_v1/observations.jsonl).
- 출처: [r512 후보](../continuous_pass_preview_r512_v1/RESULT.md)(저장 입력 재생, 폐루프 아님), [r503 원 결과](../moving_pass_facility_drive_r503_v1/RESULT.md), [r499 원 결과](../rear_overtake_current_geometry_r499_v1/RESULT.md). 이 폴더의 결과로 이전 결과를 고쳐 쓰지 않는다.

이 결과는 220 mm 상대 가정의 우측 옆차 시뮬레이션 1회 실패다. 전체 교통 랩, 낮은 차량, 실차, 실시간 10 Hz, 잔디 우선순위의 증거로 쓰지 않는다. 후속 설계와 채택 판단은 root가 맡는다.
