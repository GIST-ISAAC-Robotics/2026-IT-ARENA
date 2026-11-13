# r490 교통 10 sim초 단일 실행 결과

원 판정은 **실패**입니다. 정상 출발 후 2.35 m에서 정체했고, 앞차가 움직이는 동안 최장 정지는 5.162 sim초입니다. r486 저장 입력 회귀 통과를 실제 교통 성공으로 확대하지 않습니다. 이번 실행은 1회이며 재시도하지 않았습니다.

- 원 r478과 같은 actor .30→.58 m/s, 오른쪽 .12 m, 높이 220 mm, 초기 중심 간격 1.6 m, 시설 3개 및 물성·센서·판정 기준을 유지했습니다. 승인한 제어 5파일만 달라졌으며 r489 guard는 미포함입니다.
- GREEN 7.401 → 17.401 sim s의 10초 구간입니다. 첫 운동은 8.155 sim s입니다. 원 실패 gate는 `progress`, `no_permanent_stall`, `final_gap`, `inside_road`입니다.
- actor 접촉·벽 접촉·sampled overlap은 0입니다. 최소 sampled gap .465109 m, swept lower bound .464667 m입니다. 현 차량 기하의 별도 바퀴 중심 감사에서는 최대 1개 이탈, 3개 이상 이탈은 0이며 원 `inside_road=false`를 변경하지 않았습니다.
- command_expired 0, 모든 STOP 검사 통과입니다. 정상 10초 종료 후 명시 MCU STOP 및 .5 sim초 정지 증거를 확보했고 변위는 .000004044 m입니다. 사용자 중단이나 8초 무진행 abort가 아닙니다.
- 14/14 child 정상 종료, `shutdown_clean=true`, 소유 PID 765/partition 잔류 0입니다. 입력 1,356개 변경 0, source/archive 불변입니다. 회수 명령 exit 0 후 WSL을 반환했습니다.

## 최초 정지 구분

원 `run1/follow_decisions.jsonl`의 1-based 행 번호입니다. generation은 360입니다.

| 구간 | plan 입력 / 완료 / call 행 | sequence / 기준시각 | 기록된 이유 |
|---|---|---|---|
| 짧은 정지 | 527 / 528 / 529 | 74 / 11.02 s | `moving_pass_current_path_unavailable`; 단일 entry 후보 `large_motion_region_unobserved`, clearance .48 m. seq75에서 다시 양수 |
| 지속 정지 시작 | 566 / 567 / 568 | 87 / 12.32 s | 같은 상위 reason이지만 후보 `known_occupied_command_hold`, clearance .04 m, target 0, occupied points 481 |

실제 `/drive=0` 첫 샘플은 각각 11.091 s / 12.405 s이며 `/drive/safe=0`, guard reason은 `clear`입니다. 지속 정지 이후 기록된 양수 plan 복귀는 없고 같은 상위 0 reason이 59회입니다. 이 지속 정체를 자기마스크 거절로 단정할 근거는 없습니다. 어느 물체/벽 반환이 점유 검사를 막았는지는 추가 재생 전 미확정입니다.

## 자료·재개 위치

- 원 결과: `run1/report.json`, `execution_summary.json`, `integrity.json`, `execution_v1/owned_processes_after.json`.
- 발췌: `failure_inputs/follow_records.jsonl`은 헤더 및 정지 전후 원 행 24개를 byte 그대로 보존합니다. 행 대응은 `record_index.json`, 최초 후보 상세는 `first_stops.json`, 원본 SHA는 `README.json`입니다.
- 발췌를 다시 만드는 Windows 명령: `py -X utf8 artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1/seal_result.py`. 이는 로그 발췌이며 native 재생이 아닙니다.
- 최소 scene 복원 재사용 지점은 `../traffic_current_route_r478_v1/analysis_v1/inspect_failure.py:restore_scene`입니다. `follow_trace.restore_call` → 원 시각의 measurement transform → `scene_from_measurements` 순서를 그대로 사용합니다. r478 전용 진입점/동결 경로를 r490에 무수정 실행하면 안 됩니다. r490에 검증된 단일 native 재생 CLI는 아직 만들지 않았으며 root가 후속 분석을 담당합니다.

기존 `PAUSED.md`는 실행 전 중단 기록으로 보존합니다. 현재 상태는 본 결과와 실행 영수증이 대체합니다. 다음 r492는 변경된 guard의 원 단독 변형 1회 검증을 위한 준비 초안만 허용되며, root의 통합/검사/동결 지시 전 실행하지 않습니다.
