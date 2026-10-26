# B3 선택형 경기 감독 v2 — 제한 ROS 1회 (사전 등록)

2026-10-24. GPT 검토 지적 반영 후 실행 전 작성. 결과는 "결과" 절에만 추가하며 사전 기준을 고치지 않는다. [v1](../b3_race_session_ros_v1/README.md)은 이전 정책의 실행 결과로 그대로 보존한다.

## v1 대비 바뀐 정책·시험

- 프로세스 시작과 새 경기도 개입 래치로 시작한다. 사람 확인과 래치 이후의 별도 MCU ARM이 필요하며, 이전 프로세스·이전 경기의 ARMED를 재사용하지 않는다.
- 재ARM 목격: 래치 시각(fence, 같은 `/clock` 가정, 여유 20 ms) 이후 원본 시각의 MCU 상태만 쓰고, 그 boot에서 fence 이후 처음 본 generation보다 높은 ARMED만 인정한다. ARMED↔ACTIVE는 DRIVE로 바뀌며 generation이 오르지 않는다.
- STOP 요청 번호: 현재 활성 개입을 만든 같은 STOP의 재전송만 새 개입 없이 수락한다. 다른 op의 번호와 겹치거나 이미 해제된 STOP 번호면 새로 고정한다.
- 시험 전용 중계: 제어기의 `/actuation/status` 입력만 `-r`로 `/validator/controller_mcu_status`에 바꿔 시험기가 중계한다. host·출력 보호는 원래 토픽을 쓴다. 이 중계로 래치 전 DISARM→ARM 상태를 붙잡았다가 래치 뒤 전달한다.

범위·입력·한계는 v1과 같다: 가상 직선 센서·임시 종방향 모형·가상 MCU, Gazebo/전체 코스/실물/Jetson 아님, `/drive/safe` 직접 발행·정답/신호 상태 입력 없음, MCU STOP/CLEAR/ARM 자동 발행 없음.

## 사전 기준 (모두 충족해야 통과)

1. 사전 회귀(`tests.log`) exit 0.
2. 시험기 `report.json`의 `passed=true`: 모든 사례 통과, 정리 감사 통과, 런타임 소스 해시 일치, 그래프 감사(중계 토픽 포함) 통과.
3. 시작: `process_start` 래치·개입 1. 시작 뒤 ARM과 신호가 있어도 해제 전 0출력, 해제 뒤 출발.
4. 같은 경기: 센서 HOLD는 래치 없이 자동 재개. 상위 STOP 뒤 활성 STOP 재전송은 같은 개입, enable true·형식 오류·이전 개입/다른 세션 해제 거절. 해제 뒤에도 `mcu_rearm_required` 0출력.
5. 지연 상태: 래치 전 DISARM→ARM 상태(둘 다 붙잡힘)를 래치 뒤 전달해도 해제 뒤 `mcu_rearm_required` 0출력. fence 이후 DISARM→ARM 뒤에만 신호 재확인 없이 재개.
6. MCU STOP은 새 개입 1개, ESTOP 중 ARM 거절, CLEAR·ARM만으로 재개하지 않고 해제 뒤 재개.
7. STOP 번호: 이전 해제 요청 번호와 같은 STOP, 이미 해제된 STOP 번호 재전송 모두 실제 래치.
8. 새 경기: 틀린 경기 번호·다른 세션 거절 중 주행 유지. 정상 명령은 `new_race` 래치·개입 +1·경기 +1, 초록만·이전 시각 영상 미출발, 해제+새 신호 뒤에도 재ARM 전 0출력, 재ARM 뒤 출발.
9. 구형 reset: 새 경기+래치, 이전 경기 번호 해제 거절, 해제+새 신호 뒤 재ARM 전 0출력.
10. 영상 SIGKILL: `fault:vision` 래치·해제 거절. 제어기 재시작: 새 session·`process_start`·개입 1, 신호만으로 미출발, 이전 세션 해제 거절, 해제 뒤에도 이전 ARMED로는 0출력, DISARM→ARM 뒤 출발.
11. 최종 MCU STOP·신선한 0출력·모형 정지·단일 그래프 수렴, 모든 역할 기대 종료, 잔존 프로세스 0.

실패하면 이 폴더를 보존하고 원인을 분리한 뒤 새 폴더에서 재실행한다. 판정선을 낮추지 않는다.

## 결과 (실행 후 추가)

`run.sh` 1회, `runner_status.txt` `EXIT_STATUS=0`, `source_hash_status.txt` `SOURCE_UNCHANGED`. 실행·회수는 Claude Haiku 5.5 보조(별칭 `haiku`)가 맡았다. 보조는 사례 수를 46개로 적었지만 `report.json`을 Claude Opus 5.5가 다시 확인한 값은 36개이며, 아래 수치는 모두 이 재확인 값이다.

- 사전 회귀 **530개 통과**. 시험기 `passed=true`, 사례 **36/36**, 명령/서비스 기대 판정 **26/26**, `runtime_source_hashes_match=true`, `cleanup_errors=[]`, 그래프 감사(중계 토픽 포함) `valid=true`. 역할 6개(host·mcu·sink·safety·controller 2회) 모두 exit 0·강제 정리 없음. 실행 뒤 시험 프로세스 0, `/dev/shm`의 `fastrtps` 항목 0.
- 무재개 사례 0출력 수신 간격 최대 sim 0.010 s·wall 0.336 s(한도 0.25 s/3 s). MCU 마감 진단 0건은 지연 부재의 증명이 아니다.

| 기준 | 관측 |
|---|---|
| 3 시작 | `process_start`·개입 1. 공통 접두의 시작 뒤 ARM(세대 1)과 신호가 있어도 `RACE_LATCHED`, 해제 뒤 출발 |
| 4 같은 경기 | 센서 HOLD 래치 없음·자동 재개. 상위 STOP 개입 2, 활성 STOP 재전송 `duplicate_stop_already_applied`·개입 불변, 구형 enable·형식 오류·이전 개입/다른 세션 거절, 해제 뒤 `mcu_rearm_required` |
| 5 지연 상태 | 붙잡은 상태 7개(DISARMED·ARMED, 세대 1–3) 뒤 STOP(개입 3, fence 9.965 s). 전달 뒤 관측기 수용·`clock_mismatch` 0, fence 이전 무시 11→23. 기준 세대 3, 해제 뒤에도 `mcu_rearm_required`. fence 뒤 DISARM(세대 4)→ARM 뒤 신호 재확인 없이 재개 |
| 6 MCU STOP | 개입 4 `mcu:ESTOP_LATCHED:software_stop`, ESTOP 중 ARM 거절, CLEAR·ARM(세대 8) 뒤에도 래치, 해제 뒤 재개 |
| 7 STOP 번호 | 이전 해제 요청 번호의 STOP → 개입 5 래치, 해제된 첫 STOP 번호 재전송 → 개입 6 래치. 각각 해제+재ARM 뒤 재개 |
| 8 새 경기 | 틀린 번호·다른 세션 거절 중 주행. 정상 명령 → 경기 1·`new_race`·개입 7, 초록만·이전 시각 영상 미출발, 해제+새 신호 뒤 `ACTUATOR_WAIT`, 재ARM 뒤 출발 |
| 9 구형 reset | 경기 2·`legacy_reset`·개입 8, 이전 경기 번호 해제 거절, 해제+신호 뒤 재ARM 대기, 재ARM 뒤 재개 |
| 10 고장·재시작 | `fault:vision`·개입 9, 해제 거절. 재시작 뒤 새 session·개입 1, 신호만으로 래치 유지, 이전 세션 해제 거절, 해제 뒤 이전 ARMED(세대 16)로 `mcu_rearm_required`, DISARM→ARM 뒤 출발 |
| 11 종료 | 최종 STOP `ESTOP_LATCHED`(세대 19)·그래프 수렴 통과 |

**한계:** 제어기의 MCU 상태 입력만 시험기 중계를 거친다(`-r` 재매핑, 시험 전용). 가상 직선 센서·임시 종방향 모형·가상 MCU의 같은 `/clock` 가정이다. 교통 HOLD·시계 되감기·MCU 재부팅은 순수/노드 회귀에서만 확인했다. 실물 MCU 시계 변환·전기 STOP·실제 제동·Gazebo/전체 코스·Jetson은 미검증이다.

**전체 회귀:** `full_regression/run_checks.sh` 1회, **4,769통과·실패 0·skip 0**, 실행 중 소스 해시 불변(`source_hash_diff.txt` 0 바이트). 10/23 `integration_review_v1`의 C++ 대조 바이너리 6개는 `binaries.sha256`와 일치했고, 그때 빌드에 쓴 C++ 원본 22개도 현재와 같았다.
