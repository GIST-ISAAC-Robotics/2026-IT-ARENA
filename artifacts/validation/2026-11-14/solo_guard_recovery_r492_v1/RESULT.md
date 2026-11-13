# r492 시작 위치·각도 변형 단독 1랩 결과

**원 평가 PASS / full_lap=true**입니다. r485와 같은 `start_left20_yaw_plus3`(+20 mm, +3 deg), actor 0·시설 3개 조건에서 실제 1회 실행했고 44.77 m / 랩 길이 44.64 m를 완료했습니다. r485 이후 승인한 r486 2생산파일과 r489 5생산파일을 반영했으며 r491은 포함하지 않았습니다. 속도·차체·물성·센서·80 sim초 상한·8 sim초 무진행 STOP·기존 판정선을 유지했습니다.

- GREEN 후 실제 구간 46.536 sim초, 평균 진행속도 .962051 m/s입니다. 기존 평가기의 최장 정지 값은 0.0초, 총 정지시간 값은 .506 sim초입니다. 전체 실행 벽시계 시간 640.843초와 설정 RTF .25를 실제 구간 시간으로 혼용하지 않습니다.
- 기존 41.98 m 정지 지점을 통과했습니다. 50.615 sim s / 42.04 m 표본에서 `/drive=1.0`, `/drive/safe=1.0 m/s`, guard `clear`였습니다. 41.5–42.7 m 구간의 14개 기록 표본에서 보호 출력 0은 없었습니다. 이는 이번 주행 관측이며 이전 입력의 완전 동일 재생이나 다른 조건의 성공을 뜻하지 않습니다.
- wall_collision_samples 0, dense wall overlap 0입니다. 바퀴 중심 표본 23,817개에서 노면 밖 바퀴 최대 0개였습니다. 평가용 평면 기하 판정이며 실물 접촉 안전 증명은 아닙니다.
- command_expired 0, 원 STOP 검사 모두 통과, abort null입니다. 완주 뒤 평가기가 명시 STOP을 요청했고 최종 MCU는 `ESTOP_LATCHED/software_stop`, 목표속도 0입니다. 안정 정지 확인까지 1.11 sim초, STOP 요청 이후 변위 .314057 m, 마지막 .5초 안정 구간 최대 속도 .009345 m/s입니다. 이미 달리던 차량의 제동 변위를 0으로 보고하지 않습니다.
- 14/14 child 정상 종료, launch return 0, 강제 회수 없음입니다. 종료 감사의 PID 759 / `arena_autonomy_test_759` partition 잔류 0이고 WSL 소유권을 반환했습니다.
- 원 `sensor_only_inputs`, `safety_sensor_only_inputs`, `source_unchanged` 모두 true입니다. 동결 입력 1,375개 변경 0, archive 동일입니다.

실행 전 WSL 서비스 `E_UNEXPECTED` 진입 실패 1건은 `entry_failure_v1.json`에 보존했습니다. 당시 등록/run1/PID가 없어 실제 주행은 0회였으며, root의 Ubuntu 복구 뒤 `run_entry_v2`에서 실제 최초 1회만 실행했습니다. 준비 v1의 launch 순회 도구 오류도 보존했고, 좁은 도구 수정 뒤 준비 v2가 통과했습니다. pytest 180개·ROS constructor·native build는 반복하지 않았습니다.

runtime header는 r485와 동일하게 `known_structure_sectors_json` 문자열의 직렬화 차이로 `header_exact=false`입니다. 기존 비교기의 값·타입·순서 등가성은 true이며 원 audit 값은 변경하지 않았습니다. 준비 source 불변 판정과 구분합니다.

원 결과는 `cases/start_left20_yaw_plus3/run1/report.json`, 집계와 이전 실패 구간 표본은 `execution_summary.json`, 종료 감사는 `cases/start_left20_yaw_plus3/execution_v1/owned_processes_after.json`입니다. 이 단독 변형 성공은 별도 r490 교통 실패를 해결한 것으로 취급하지 않습니다. 추가 주행은 시작하지 않았습니다.
