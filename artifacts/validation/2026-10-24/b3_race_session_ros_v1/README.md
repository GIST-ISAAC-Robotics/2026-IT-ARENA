# B3 선택형 경기 감독 — 제한 ROS 1회 (사전 등록)

2026-10-24. 실행 전 작성. 결과는 아래 "결과" 절에만 추가하며 사전 기준을 고치지 않는다.

## 범위

- `local_pursuit`의 `race_control_mode:=b3_candidate`(기본 `legacy`)와 공식 4빨강 후보 영상, 별도 영상 프로세스, 독립 `lidar_safety`, `actuation_host`·가상 MCU(모터축 28카운트·15:1·65 mm)·출력 보호를 실제 ROS 프로세스로 연결한다.
- 입력은 기존 `validate_pipeline_faults_ros.py`의 가상 직선 RGB/LiDAR/IMU/모터축 센서와 ±2 m/s² 임시 종방향 모형이다. Gazebo·전체 코스·실물·Jetson 시험이 아니다. 정답 위치/신호 상태/타이머는 제어 입력이 아니다.
- `/race/command`는 사람 운영 도구를 대신하는 시험 입력이다. `/drive/safe` 직접 발행은 없다.

## 사전 기준 (모두 충족해야 통과)

1. 사전 회귀(`tests.log`) exit 0.
2. 시험기 `report.json`의 `passed=true`: 모든 사례 통과, 정리 감사 통과, 런타임 소스 해시 일치.
3. 같은 경기: 센서 HOLD는 래치 없이 자동 복구. 상위 STOP만 걸고 MCU는 ARMED인 상태에서 구형 enable true·형식 오류 명령·이전 개입 번호/다른 세션 해제는 거절되고 0출력 유지. 올바른 해제 뒤에도 같은 ARM 세대에서는 `mcu_rearm_required`로 0출력 유지, 실제 DISARM→ARM 뒤 신호 재확인 없이 재개.
4. MCU STOP 직접 배송은 새 개입 1개가 되며, ESTOP 중 ARM 거절, CLEAR·ARM만으로는 재개하지 않고 사람 해제 뒤 재개.
5. 새 경기: 다른 세션/틀린 경기 번호 명령은 거절되고 주행 유지. 정상 명령 뒤 WAIT_GREEN·출발 사실 없음, 초록만·이전 시각 영상으로 출발하지 않고 새 빨강→초록 뒤 출발. 래치·개입 번호 불변.
6. 구형 `/autonomy/reset`은 새 경기+래치. 이전 경기 번호 해제 거절, 해제+새 신호 뒤에도 재ARM 전 `ACTUATOR_WAIT` 0출력.
7. 영상 작업자 SIGKILL은 `fault:vision` 래치, 해제 거절(`fixed_fault_present`). 제어기 재시작은 새 세션·개입 0, 새 신호를 요구하고 이전 세션 해제는 거절.
8. 최종 MCU STOP(`ESTOP_LATCHED`)·신선한 0출력·모형 정지·단일 그래프 수렴, 모든 역할 기대 종료, 잔존 프로세스 0.

판정은 상위 래치(`race.latch`), MCU 상태(`/actuation/status`), 출력/모형 정지를 각각 기록한다. 하나로 다른 것을 대신하지 않는다. 실패하면 이 폴더를 보존하고 원인을 분리한 뒤 새 폴더에서 재실행한다. 판정선을 낮추지 않는다.

## 결과 (실행 후 추가)

`run.sh` 1회, `runner_status.txt` `EXIT_STATUS=0`, `source_hash_status.txt` `SOURCE_UNCHANGED`. 실행·대기·회수는 Claude Haiku 5.5 보조(별칭 `haiku`, 자체 보고 `claude-haiku-5-5`)가 맡았고 수치는 Claude Opus 5.5가 `run/report.json`에서 다시 확인했다.

- 사전 회귀 **448개 통과**(`tests.log`, `junit.xml`). 시험기 `passed=true`, 사례 **30/30**, 명령/서비스 기대 판정 **16/16**(`race_expectations`), `runtime_source_hashes_match=true`, `cleanup_errors=[]`, 그래프 감사 `valid=true`.
- 역할 6개(host·mcu·sink·safety·controller 2회) 모두 exit 0·강제 정리 없음. 의도적 SIGKILL 영상 작업자 1개와 재시작 뒤 작업자 1개를 소유 확인 후 정리했다. 실행 뒤 프로세스 표에 시험 프로세스 잔존 0. 보고서의 `dev_shm_after_cleanup` 22개 항목은 시험기 자신의 노드 종료 전 FastDDS 구획이며, 종료 후 `/dev/shm`에는 `fastrtps` 항목이 남지 않았다.
- 무재개 사례들의 0출력 수신 간격 최대 sim 0.010 s·wall 0.293 s(시험 한도 0.25 s/3 s). 최종 명시 STOP 뒤 가상 모형 정지·`ESTOP_LATCHED`·단일 그래프 수렴. MCU 마감 진단 0건은 지연 부재의 증명이 아니다.

| 기준 | 관측 |
|---|---|
| 3 같은 경기 | 센서 HOLD `SENSOR_STOP`·래치 없음·자동 재개. 상위 STOP 뒤 `RACE_LATCHED`·MCU `ARMED` 세대 1 유지. enable true·형식 오류·이전 개입/다른 세션 해제 거절. 올바른 해제 뒤 `ACTUATOR_WAIT`(`mcu_rearm_required`), DISARM(세대 2)→ARM 뒤 신호 재확인 없이 재개 |
| 4 MCU STOP | 개입 1→2, `mcu:ESTOP_LATCHED:software_stop`. ESTOP 중 ARM 거절(`not_disarmed`). CLEAR·ARM(세대 6) 뒤에도 `RACE_LATCHED`, 해제 후 재개 |
| 5 새 경기 | 틀린 경기 번호·다른 세션 거절 중 주행 유지. 정상 명령 뒤 경기 1·`WAIT_GREEN`·개입 2 유지. 초록만·이전 시각 영상 미출발, 새 빨강→초록 뒤 출발 |
| 6 구형 reset | 경기 2·`legacy_reset` 래치·개입 3. 이전 경기 번호 해제 거절. 해제+새 신호 뒤 `ACTUATOR_WAIT`, DISARM→ARM 뒤 재개 |
| 7 영상 고장·재시작 | `fault:vision` 래치·개입 4, 해제 `fixed_fault_present` 거절. 재시작 뒤 새 세션·개입 0·MCU `ARMED` 유지, 초록만 미출발, 새 신호 뒤 출발. 이전 세션 해제 `session_mismatch` |
| 8 종료 | `final_explicit_software_stop`·`final_graph_convergence_stays_stopped` 통과 |

**한계:** 가상 직선 센서·임시 종방향 모형·가상 MCU의 같은 `/clock` 가정이다. 상위 래치는 프로세스 수명 한정이며 재시작으로 사라진다(이 사례는 MCU `ARMED` 유지 상태에서 새 신호만 요구함을 확인). 교통 HOLD·시계 되감기·MCU 재부팅은 이번 ROS 실행이 아니라 순수/노드 회귀에서만 확인했다. 실물 MCU 상태 중계·시계 변환·전기 STOP·실제 제동·Gazebo/전체 코스는 미검증이다.

## 회귀 기준선 (같은 날 별도 실행)

`regression/related_after.log`는 변경 후 관련 45파일 묶음(새 B3 시험 2파일 포함)의 로그이며 **1,086통과·3실패**다. 변경 전 같은 43파일 묶음은 **1,055통과·5실패**였으나, 그 로그 파일은 이후 실행이 같은 임시 경로에 덮어써 보존하지 못했다. 변경 전 수치와 실패 이름은 작업 중 출력에서 옮긴 것이다.

남은 3실패는 변경 전부터 있던 R0+ 미등록 소스 해시 계약이다: `test_corner_follow::test_corner_source_contract_has_explicit_parent_and_unchanged_previous_bytes`(`local_pursuit.py`), `test_g1_dense_stop::test_evaluation_changes_have_explicit_successor_not_rewritten_old_contract`·`test_physics_step_option::test_contract_records_evaluation_script_changes`(`validate_g1_traffic.py`). 이번 B3 변경도 `local_pursuit.py` 해시를 바꾸지만, 해당 계약의 후속 등록은 R0+ 변경의 승인 기록이므로 이 작업에서 하지 않았다. 시험기 manifest의 `r0_plus` 누락 2실패(`test_official_start_ros_fixture`·`test_pipeline_fault_validation`)는 manifest 보완으로 해소했다.

**전체 회귀(ROS 실행과 같은 소스, 실행 중 소스 해시 불변):** 4,737통과·4실패·19 skip(`regression/full_suite_tail.log`). 4실패는 위 3개와 `test_follow_profile_bundle::test_minimal_bundle_has_all_static_internal_dependencies_and_no_ros_node`이다. 마지막 것은 R0+ 변경의 `follow_trace.py`가 `r0_plus`를 import하지만 Jetson 최소 묶음 목록에 없어서 생긴 것이며, 이번 B3 파일은 그 묶음에 포함되지 않는다. 19 skip은 C++ 대조 바이너리 환경변수(`ARENA_*_BINARY`)를 주지 않은 시험이다. 10/23의 4,656통과·skip 0과는 소스·실행 조건이 달라 비교·합산하지 않는다.

**이후:** 같은 날 GPT 검토로 STOP 번호 중복 판정, 새 경기·시작의 ARMED 재사용, 재ARM 목격의 지연 상태 오인, generation 설명 오류를 발견해 정책을 고쳤다. 이 v1 결과는 이전 정책의 실행으로 그대로 두며, 현재 정책의 결과는 [v2](../b3_race_session_ros_v2/README.md)다. 위의 "변경 전 1,055통과·5실패"는 [작업 기록 발췌](regression/related_before_journal_excerpt.txt)로 원 도구 출력이 확인된다.
