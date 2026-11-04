# 최소 경기 운영: 상태 화면·같은 경기 STOP/RESET/START·사건 기록

상태: **PC·ROS·Gazebo 대표 흐름 a6가 사전 평가 관문을 모두 통과했다.** 브라우저 버튼은 대역 차량으로 따로 확인했다. 기능은 기본 OFF 후보다. 전체 본선, 공용 Wi-Fi, Jetson, 실제 MCU·차체 정지 검증이 아니다.

구현은 Claude Opus 5.5, 시뮬 실행·회수는 Claude Haiku 5.5 보조가 맡았다. 설계 기준은 [복구 계획](../../../../docs/autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md)이다. 인계 사항은 [HANDOFF](HANDOFF.md)에 있다.

## 만든 것

| 항목 | 내용 |
|---|---|
| 경기 감독 | `race_session.py`: `recovery_reset`·`resume_start`·`cancel_start`·`restore_race`. 복구 RESET은 경기와 출발 사실을 보존한다. START 수락은 입력 경계 생성까지이며 래치를 유지한다 |
| 입력 경계 | 스캔 첫 광선, 영상 촬영, 자이로, 엔코더의 **원천 취득 시각**으로 거른다. 수신 시각이 아니다. 영상 세대를 올려 진행 중 결과를 버린다. 독립 보호층은 `/race/input_boundary`로 관측 기억만 지운다. 허가는 늘지 않는다 |
| START 단계 | `inputs`(경계 뒤 네 입력) → `route`(출력 0으로 경로 성립 확인) → `mcu`(경계 뒤 ARM 목격) → 허가. 제한시간 시뮬 3초, 추가 10초·빨강 대기 없음 |
| 취소 | 새 STOP·고장·새 경기·입력 제한 초과·MCU boot 변경·HW 정지·운영 연결 상실. 취소된 START는 큐에 남지 않고 재연결해도 실행되지 않는다 |
| 기준 벽 | 이동 전 벽·경로 기하는 버린다. 논리적 `side`는 유지한다. 출발 뒤 STOP 중(운반 중) 영상의 마커는 `side`를 바꾸지 않고, START 경계 뒤 첫 영상부터 다시 쓴다 |
| 프로세스 재시작 | 어댑터가 `race_id`·출발 사실·`side`만 파일에 저장한다. 새 프로세스의 출발 전 상태에서만 후보로 표시하고, 운영자 확인이 있어야 복원한다. 복원은 `restored_operator_confirmed`로 표시하며 시각 신호를 본 것으로 가장하지 않는다 |
| 운영 어댑터 | `race_ops`: 별도 프로세스의 표준 라이브러리 HTTP + ROS. 토큰·차량 이름 검사, 읽기 전용 토큰, 요청 번호 멱등 처리, 화면 번호의 원자적 바인딩. `/drive`·`/drive/safe` 발행자 없음. 파일 기록은 별도 스레드 |
| STOP 연결 | `/race/command` stop과 기존 `/actuation/stop`을 함께 보낸다. 요청·정책 수락·MCU 응답·MCU 상태·모터축 값을 구분하고 실제 차체 정지는 항상 `미확인`으로 표시한다 |
| 화면 | 한 장, 한국어. 계획·보호 후·MCU 적용 목표·모터축 환산을 구분한다. 상태 오래됨, 센서 나이, 최근 사건, CSV/HTML 내보내기. 새 경기는 접힌 메뉴와 체크로 분리했다 |
| 사건 기록 | `ops_log.jsonl`(표본+사건), `scripts/race_event_export.py`. 기존 판단 기록을 재사용하고 평가 정답은 별도 절로 분리한다 |
| 재획득 후보 | 기본 OFF `g1_standing_wall_reacquire`. 아래 결함을 푸는 추가 옵션 |

CLI에는 `recovery-reset`·`resume-start`를 추가했다. 복구 RESET 뒤 `acknowledge`는 `recovery_requires_start`로 거절해 입력 경계를 우회하지 못한다.

## 시험 중 찾아 고친 결함

- **ARM 직후 DISARM(a2):** 5 Hz 상태가 늦어 어댑터가 `rearm_required`를 옛값으로 읽고 방금 올린 ARM을 내렸다. ARM 응답을 받으면 추가 호출을 멈추도록 고쳤다.
- **합류부 영구 정체(a5):** 재개 뒤 s≈35.2에서 `follow_boundary_unconfirmed`로 멈췄다. 기억 없는 재획득은 양측 벽 일치를 요구하는데 왼쪽이 지름길 합류로 열려 있어 풀리지 않는다. 평상 주행은 경로 기억으로 지나가 드러나지 않았다. 측정 속도 0.10 m/s 이하에서 선호 벽의 긴 현재 관측으로 재획득하도록 했다. 확인 횟수·점유·재개 여유·독립 보호는 그대로다. [재생](../../../../tests/test_standing_wall_reacquire.py)과 a6에서 확인했다.
- **기타:** `rclpy.Node.services` 이름 충돌(a1), 평가기 인자 충돌(a4). a3는 내가 동시에 돌린 대역 시험의 `pkill`이 어댑터를 종료해 무효다. 모두 폴더째 보존한다.

## 대표 흐름 a6

[평가기](../../../../scripts/race_ops_eval.py)는 운영자 조작을 브라우저와 같은 HTTP API로만 한다. 재배치·정답 진행 확인은 평가기만 하며 제어기에 좌표·번호·좌우 지정을 주지 않는다. 판정은 [goal_eval.py](goal_eval.py), 결과는 [a6_goal.json](a6_goal.json)이다. **37개 관문 전부 통과**, 원본은 [a6/report.json](a6/report.json).

구성은 선정 단독 프로필 `g1_open_10kmh_solo_gap05_arc_preview_bridge`, 그리드 3, 공식 v2026.11.01 파생 월드, 가상 MCU, 재획득 옵션 ON이다.

| 장면 | 확인한 결과 |
|---|---|
| 최초 START | 어댑터 `race_not_started`, CLI로 보낸 경기 감독도 같은 이유로 거절. 신호 전 이동 없음. 실제 RGB로 출발 사실 `visual` |
| 왼쪽 복구 | STOP → RESET → 운반(ID30 앞 s=31.6, 마커 30 관측) → s=24 배치 → START 1회. 운반 중 본 ID30이 `side`를 바꾸지 않음. **START→허가 시뮬 0.432초**, 경로 `left`, 본선 4.02 m, 지름길 표본 0 |
| 오른쪽 복구 | 자연 주행으로 ID30을 지난 뒤 STOP → s=33.7 배치 → RESET. ID30 미관측, `side=right` 유지. **허가 0.434초**, 본선 s=33.69→38.51(4.82 m), 지름길 표본 0, 최대 2.22 m/s |
| 준비 중 새 STOP | START 취소, 중복 STOP 수락, 이후 이동 없음 |
| 운영 연결 상실 | `operator_link_lost` 취소, 재연결 뒤 뒤늦은 실행 없음 |
| 응답 유실 START | 응답을 읽지 않고 끊음. 재전송 없이 상태로 결과 확인, 1회 실행 |
| 프로세스 재시작 | 제어 프로세스 종료·재배치·재기동. 복원 전 START 거절, 확인 없는 복원 거절, 복원 뒤 이동 없음. START 뒤 **허가 0.395초**, s≈38.5 도달, 지름길 표본 0, ID30 미관측 |
| 입력 경계 | 경계 2초 안의 판단 275건 모두 경계 뒤 스캔 사용. 경계 전 스캔 2건 폐기 |

최대 중심선 오차는 약 0.03 m다. 접촉 센서는 이 평가에 넣지 않았으므로 접촉 0을 주장하지 않는다. 시간은 시뮬 시간이다. 평가 환경의 실시간 비율은 약 0.09라 어댑터 벽시계 한도를 평가에서만 늘렸다(`status_stale_s` 20, `result_timeout_s` 30, `mcu_timeout_s` 240). 기본값 1/3/30초는 실시간 조건에서 시험하지 않았다.

사건 내보내기 예: [a6_first_stop_timeline.html](a6_first_stop_timeline.html)·[CSV](a6_first_stop_timeline.csv). 표시된 누락 77건은 벽시계 간격 기준이라 낮은 실시간 비율의 영향이 크다.

## 브라우저 확인

[page_check.mjs](page_check.mjs)로 실제 Chrome에서 로그인·버튼 클릭을 했다. 차량은 [대역](stub_vehicle.py)이며 실제 `RaceSupervisor`와 실제 `race_ops`를 쓴다. 준비 → 출발 → STOP → RESET → START → 허가 → 새 경기가 표시와 함께 이어졌다([기록](stub_page.json), 화면 `stub_page_*.png`). **주행 증거가 아니며, Gazebo 주행 중 실제 브라우저로 누른 적은 없다.** 이 뒤 favicon 204·긴 문구 줄바꿈·START 처리 중 문구를 고쳤고 a6에는 포함됐지만 브라우저 재확인은 하지 않았다. 당시 콘솔 오류 1건은 favicon 409였다.

## 한계

- **마커를 건너뛴 재배치:** 왼쪽 상태로 멈춘 뒤 ID30을 보지 못한 채 그 너머로 옮기면 `side`가 왼쪽으로 남는다. 합류부에서 잘못된 벽을 따를 수 있다(추측, 미실행). 진단 장면 `--scenario across_marker`를 만들었으나 실행하지 않았다.
- 운전자 조작·정답 입력으로 이 분기를 피하지 않는다는 원칙은 유지했다. 해결은 관측 근거로 해야 한다.
- 대표 1회. 교통 후보 G1/G2, 앞차가 있는 재개, 여섯 그리드, 비스듬한 재배치는 시험하지 않았다.
- 경로 준비의 앞차 예외는 `follow_gap_hold` 하나뿐이다. 다른 정지 이유로 3초가 지나면 START가 취소되고 다시 눌러야 한다.
- 공용 AP 접속·client isolation·토큰 강도, 실제 MCU STOP/ARM, 실제 정지, Jetson 부하는 현장 미검증이다.
- 평가 종료 때 어댑터가 SIGINT 10초 안에 끝나지 않아 SIGKILL됐다(exit -9). 원인 미조사. 잔존 프로세스는 0이다.
- 이관 묶음(`prepare_jetson_bundle.py`)에 새 파일을 넣지 않았다.

## 검증

| 명령 | 결과 |
|---|---|
| `bash .../build.sh` | colcon 0, 관련 99+51 통과 |
| `bash .../post_a6.sh` | 평가 통과, 내보내기 0 |
| `bash .../contract.sh` | `official_ops_r1` 생성. 전체 회귀 16 실패·7456 통과·19 건너뜀([로그](tests_full.log)) |
| `bash .../final_checks.sh` | 새 실패 3건 수정 뒤 관련 **238 통과**([로그](tests_final.log)) |

전체 회귀의 16건 중 13건은 직전 목록과 같은 기존 실패다. 새 3건(`test_local_follow` 2, B3 계약 1)은 시험 대역·사슬 대조 문제였고 고쳤다. **수정 뒤 전체 회귀는 다시 돌리지 않았다.** 소스 파일은 a6 실행 때와 같다.
