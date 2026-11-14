# r499 오른쪽 옆차 통과 대표 1회 — 시설 앞 정체 실패

2026-11-14 재개 지시에 따라 준비된 `right_side_start`를 **단1회** 실행했다. 원 평가와 기존 별도 목표가 모두 실패했으며, 실제 나란히·통과는 성립하지 않았다. 재실행이나 제어 코드 변경은 하지 않았다.

## 입력과 실행 범위

- HEAD `b59efcab11df06a55d85620ef24433af9b8fd521`. r496의 공통/시설 입력 1,330개 및 원 사례·recipe·정적 기하 근거 해시 변경0을 확인했다. native 6파일166검사/실패0과 동일 바이너리 SHA `1bbe89cdcff6662fda699847756ff9c776e1b70c0d406ec97df469666dc3b16c`를 재사용했다. 새 빌드는 없다.
- 기존 actor +0.50 m/오른쪽−0.23 m, 0.50 m/s, 12 sim초, `box` 200×150×220 mm. 높이120→220 mm의 **C1에 보이는 상대 가정**이며 낮은 차량 감지 증거가 아니다. 현 CAD/C1/IMU·시설3·속도/독립 보호/STOP을 유지했고 r498은 포함하지 않았다.
- 전체 actor 경로601표본·시설 관통0·최소시설간격47.5 mm의 기존 정적 근거를 재사용했다. 거절한 빠른 뒤차 후보의25표본 시설 관통은 보존했다. actor 경로의 적합성이 자차 추월 경로의 적합성을 보장하지 않는다.
- 1,375입력 동결. `prepared.json` SHA `a24528224e9b9f507ffa2f5fef2942be3cb34bfefb9195a80b8c1f2abf395033`, `sources.zip` SHA `1378464a7a512036a7427cac786654a541c5b717c13534e0665ab6db983ba663`. root의 별도 동결/실행 검토 후 실행했다.
- 실행 UTC00:37:21–00:41:51, wall270초. GREEN7.401–19.401 sim초의12초 대표이며 실시간 처리율 보장이 아니다. 원 validator exit1은 실패 판정이며 수집 exit0과 구분한다.

## 원 판정과 별도 목표

| 항목 | 관측 결과 |
|---|---|
| 원 평가 | **FAIL**: `progress`, `no_permanent_stall`, `final_gap` 실패. 150 mm 간격·뒤 유지·차체 도로 안 등 나머지 원 gate 통과 |
| 기존 별도 목표 | **FAIL**, 장면 미성립: `actually_alongside=false`, 통과없음, `passed_actor`/`final_follow`/`no_long_stall`/`post_return_progress` 실패 |
| 진행·정체 | 2.46 m, 최장정체6.880 sim초, 최종상태 `MOVING_PASS`, 마지막2초 진행−0.000478 m |
| 실제 나란히 | 0표본·0초. 옆차를 통과하지 못했으므로 복귀 성공으로 부르지 않음 |
| 상대 간격 | 표본 최소296.500 mm, swept 하한296.499 mm. 원150 mm와 별도80 mm 모두 통과 |
| 접촉·도로 | actor 접촉/겹침/벽접촉0. 현 CAD 바퀴중심 최대도로밖0. 실제 타이어 접촉/심판 판정의 증거는 아님 |
| 속도·전달 | 최고0.936896 m/s, 명령만료0 |
| STOP·회수 | STOP6항목 통과·0.5초 안정·명시MCU STOP, 14/14자식 정상종료, 강제회수없음, 소유PID0 |
| 무결성 | 실행 전후1,375입력 변경0·ZIP불변 |

8 sim초 무진행 조기STOP 기준 전에12초 창이 끝났으므로 별도 STOP helper는 호출하지 않았다. 원 평가기가 명시 STOP과 정상 종료를 수행했다. 보충 평가의 `rejoined_main=true`는 마지막 차체 도로 안 조건의 부분 결과이며, 통과/최종FOLLOW/진행 실패를 덮지 않는다.

## 최초 정지와 현재 점유 근거

직전 계획 `seq576`/`seq579`(11.82/11.92초)는 0.9/0.220489 m/s였다. 최초 지속0인 `seq582`(12.02초, runtime sequence99)부터 `moving_pass_current_path_unavailable`, 출력 `HOLD`, 내부 `handoff_state=MOVING_PASS`가 이어졌다. 외벽 guide는 현재 왼쪽171반환으로 존재했으나 유일 진입 후보가 `known_occupied_command_hold`로 거절됐다. 후보 명령 간격은 직전0.12→0.04 m, 조향−0.215999 rad이고 출력 조향은 직전−0.159982 rad를 유지했다. 전역 미관측 veto가 아니라 **현재 고정조향 점유 간섭으로 후보가 없는 구조**다.

기존 `motion_scene`·현재 패딩·고정조향 `swept_limit`만 저장 입력에서 재구성했다. 제어기의 `command`나 시뮬레이터를 재실행하지 않았으며, 최초/종료직전2개의 기록 간격0.04/0/0 m와 일치했다. 최초0은 후륜축 이동0.08 m 표본에서5점이 간섭했고 함수의4 cm 이산 여유 차감으로0.04 m가 됐다.

- 대표 첫 간섭점: 자차 기준 `[0.277655, 0.124576]` m, 평가 월드 `[9.422036, 6.144602]` m, 기존 패딩0.091430 m. 평가용 시설 단면까지 거리0, actor까지0.539449 m, 벽까지0.127040 m다.
- 최초5점과 종료직전9/10점의 가장 가까운 평가 형상은 모두 **signal 지주**였다. 최근접 형상에 의한 귀속 추정이며 센서가 물체 ID를 증명한 것은 아니다. 시설 C1높이134.5–134.7 mm 단면과 대조했다.
- 최초0에서 actor 중심은 자차 기준 `[+0.325978, −0.508607]` m였다. 실제 자차–actor 접촉은 없고, actor는 계속 앞으로 갔다.
- 이 근거는 현재 관측된 다른 경로의 비교를 검토할 재현 입력이다. 패딩/점유 판정선을 완화하거나 시설을 제거한 결과가 아니다.

## 후속 인계와 자료

- [원 결과](run1/report.json), [실행 요약](execution_summary.json), [분리된 보충 판정](assessment_summary.json), [현재 바퀴 기하](surface_run1.json).
- [최초 정지 요약](failure_inputs/summary.json): 선행양수2+최초0 5개, 상대좌표·상태·후보·scan 개수. `selected_records.jsonl`은 원문 바이트이며 원시 scan/이력은 로컬 보존한다.
- [차단점 대조](failure_inputs/occupied_attribution.json): 최초0과 case 종료직전 `seq798`/`seq801`(19.22/19.32초). `last_stall_records.jsonl`에 두 원문을 보존했다. 첫 분석의 종료창 이후20.02/20.12초 표본은 `occupied_attribution_with_postwindow.json`과 `postwindow_stall_records.jsonl`로 구분해 보존하며 위 종료직전 결과에 섞지 않는다.
- 완전한 상태 재생에는 원본 `run1/follow_decisions.jsonl`의 선행 순서가 필요하다. 소량 발췌만으로 내부 selector 상태가 복원되었다고 가정하지 않는다.
- [최종 소유 프로세스](execution_v1/owned_processes_after.json)에서 UTC00:48:00 PID0, 최종1,375입력 불변을 확인해 WSL을 반환했다. 공통 `src`/`docs`, Jetson, 펌웨어/전원은 변경하지 않았다. root가 후속 제어 수정·검증 범위를 결정한다.

이 결과는 지정된 높이의 우측 옆차 단1회 시뮬레이션 실패다. 전체 교통랩·낮은 차량·실차 주행·실시간10 Hz의 증거로 확대하지 않는다.
