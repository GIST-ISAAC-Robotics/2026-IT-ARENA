# r485 결과 — 독립 보호층 정지로 미완주

**동일 +20mm·+3° 출발 조건을1회 실행했고41.98m/44.64m에서 미완주했다.** r481 v2의 계획/current 양수 명령은 실제 `/drive` 발행까지 연결됐으나, 독립 보호층의 `lateral_metric_outside_trial`이 `/drive/safe`를0으로 제한했다. 보호층 우회·판정선 변경·재실행은 하지 않았다.

- GREEN7.401sim초, 첫 움직임8.067sim초. 최초 보호 정지 표본은 **50.245sim초·41.88m**, `/drive=.80m/s`, `/drive/safe=0`이다. 관성 정지 후 최대/최종 진행은41.98m다.
-50.2sim초 이후 양수 계획89, 양수 current175, 양수 route 발행88개가 기록됐다. 양수 `/drive`와 보호층0을 함께 관측한 표본은83개다. 서로 다른 기록 단계를 실제 구동 횟수로 합쳐 세지 않는다.
- 무진행8초 기준을 충족한 관측은58.997sim초/8.552sim초였고 기존 owned STOP을 요청했다. 원 평가 **최장 정지8.954sim초**, `passed=false/full_lap=false`, runner exit1을 보존한다.
- 벽접촉 표본0, 조밀 평면 벽 겹침0, 도로 밖 바퀴 중심 최대0, 명령만료 진단0이다. 두 노드의 sensor-only 입력을 확인했다. 이는 이번 시뮬레이션의 표본 판정이다.

원 `abort=mcu_latched_before_stop: software_stop`은 owned helper가 평가기 자체 STOP보다 먼저 latch한 결과로 보존한다. `/autonomy/enable=false`와 `/actuation/stop` ACK, `ESTOP_LATCHED`·출력0을 확인했다. STOP 안정.5sim초·변위.000113586m, 자식14/14 정상종료·`shutdown_clean=true`다. 마지막 감사는 PID765와 `GZ_PARTITION=arena_autonomy_test_765` 잔류0/exit0이며 WSL을 반환했다.

기존 속도·센서·차체·물성·시설·노면/접촉/STOP 기준, RTF .25·최대80sim초, actor0/old610 시설3개를 유지했다. 제어 변경은 승인된 r481 두 파일과 r482 `current_lane_preference.py`뿐이다. `jetson_access.py`는 비실행 보조 변경으로 분리했다. 실제 import를 확인하고 기존 native SHA `1bbe89cd…`를 재사용했으며 새 빌드는 없다. **동결1,444입력 변경0·archive 불변·source_unchanged=true**를 회수했다. prepared SHA `216f0465c035f1ece070c6e617704c61b13987385645a31ab3e97f0014265876`, archive SHA `b80b6ef6f5b423736b4921a3c577fa4dc4ad3e41273833ed3eed81e37781372e`다. 초기 헤더 줄바꿈 실패를 보존하고 원바이트 복원 후 preflight를 완료했다. 실행 벽시간693.599초와 GREEN 이후 평가52.128sim초를 구분한다.

## r489 최소 입력 인계

[failure_inputs/README.json](failure_inputs/README.json)에49.5~50.65sim초의 원 scan, plan/current 입력·signed wheel/gyro history·epoch, 명령, MCU 피드백과 원행/seq·SHA를 보존했다. `record_index.json`에서 `follow_records.jsonl`로 연결한다. 평가 정답 pose/progress는 `evaluation_observation.json`에 분리했고 제어 입력에 쓰지 않는다.

MCU 첫 target0 표본은50.220s, 직전 양수 목표 조향은.1248121113rad다. **관측된 하류 목표이며 정확한 guard previous-safe 조향/callback 순서의 증명은 아니다.** full `/safety/status` partial 상세·독립 보호 내부 기억·정확한 timer/수신 순서는 이번 trace에 저장되지 않았다. 첫 `safety_clearance_m=0`은 partial 거절 뒤 초기값이며 실제 점유 충돌거리0을 뜻하지 않는다. 전체 guard exact 재현이나 실차 안전 증거로 확대하지 않는다. 후속 담당에게 경로와 한계를 직접 전달했다.

근거: [기계 요약](execution_summary.json), [원 report](cases/start_left20_yaw_plus3/run1/report.json), [무결성](cases/start_left20_yaw_plus3/integrity.json), [소유 감사](cases/start_left20_yaw_plus3/execution_v1/owned_processes_after.json). 추가 주행·재발행·정책 수정 없이 봉인한다.
