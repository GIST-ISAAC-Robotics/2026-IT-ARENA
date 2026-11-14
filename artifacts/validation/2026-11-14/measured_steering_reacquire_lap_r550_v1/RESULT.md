# r550 revision2 단독 결과

**이전19.56 m 정체는 통과했지만 새25.80–25.81 m 지점에서 정체해 전체랩은 미완주다.** 원 `passed=false`, `run_valid=true`, `harness_valid=true`를 유지한다. 등록된8 sim초 무진행 조건으로 STOP했고 실제 안정 정지·14개 자식 정상 종료·잔여 소유 PID0을 확인했다. 같은 조건 추가 실행은 하지 않았다.

| 원 평가 지표 | r544 attempt2 | r550 |
|---|---:|---:|
| 최종 진행 거리 | 19.56 m | 25.80 m |
| 전체 코스 길이 | 44.64 m | 44.64 m |
| GREEN 이후 평가 기록 | 40.716 sim초 | 64.040 sim초 |
| 최장 정지 | 9.256 sim초 | 10.106 sim초 |
| 전체랩 판정 | FAIL | FAIL |

[원 보고서](run1/report.json)의 `full_lap`/`no_permanent_stall`은 false이며 신호·자료 범위·바퀴 노면 기준은 true다. 최종 진행은25.80 m이고 감시 중 최대 진행25.81 m에서 멈췄다. 조밀 pose32,278표본의 벽 외곽 겹침은0이었다. 도로 밖 바퀴 중심은 최대1개이고 해당 표본5,460개이며, 기존 최대2개 허용 판정을 그대로 적용했다. 접촉0이라는 표현은 기록된 시뮬레이션 검사 범위이며 연속 실물 접촉 검증으로 확대하지 않는다.

이번 입력은 [최종 revision2](FINAL_READY.md)다. 반대벽 fallback v1은 제거됐으며 `current_wall_reacquire.py` SHA `598e1d70a61b5fb9757eacba6fd1f4c8bff42d375df2accb7ed9802171b75d1b`, `local_path.py` SHA `9ee7d0e3761d96084b3c71c4ef3781c50484f7ea9e2c2f14a93cc3afe0c816ad`를 실제 import했다. r543 옵션 연결은 기본 `full`을 유지했다. [상태 집계](run_supplement.json)에서3,597행 모두 full 형태였고 compact 상세 참조는 없었다. 원 회귀42개·최종 helper15개는 root의 JUnit을 재사용했다.

`arena_autonomy` 단일 패키지 빌드가9.61초에 통과했고 [빌드 전후 입력](build_result.json)은 모두 불변이었다. native 바이너리 SHA `1bbe89cdcff6662fda699847756ff9c776e1b70c0d406ec97df469666dc3b16c`도 같았다. 새 광선 방향 정렬 stage의 native 검사는 실행하지 않았다. [실제 import/경쟁 검사](preflight/result.json) 후 필수 입력462개·관측985개를 동결했다. prepared SHA `2aae2b55aa5798489831c40166f4d72425ae9d2c28d8eaa03037a6e4770f6bbe`, ZIP SHA `6dfc3632dfd84b24010dec8bf73eb41c92934137137cd6d56cae3ad63bc3474f`다.

월드 SHA `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`, measured YAML SHA `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021`과 actor0·grid3·시설3개·80 sim초 상한·RTF 설정.25·g1_open_1p5·medium_v1·원 STOP/판정을 유지했다. 첫 직선 두 벽만 미실측 가정으로 변형한 월드이며 바퀴 조향각도 실측이 아닌 count 범위의 CAD 파생값이다.

[실제 조향 감사](steering_runtime_receipt.json) 7검사 모두 통과했다. controller/guard 상한 .28475436568260193 rad, MCU/actuator 물리 중앙 상한 .28475438456006436 rad, SDF 내륜 joint/plugin .30666234131856146 rad다. 실제 경로 발행1,049개 중439개가 명령 상한의1e-6 rad 이내였다. 기록상 최고 지면 속도는1.5413 m/s다. 이 수치만으로 새 정체의 원인을 조향 상한이라고 단정하지 않는다.

실제 RTF는 GREEN 구간의 simulation reference 차이/evaluator monotonic 수신 시각 차이로 약 **0.08506**이었다(해당 wall 구간752.409초). 메시지 전달 시차를 포함한 기록 기반 비율이며 설정.25와 구분한다. 원 보고서 전체 wall 시간은860.180초다.

[새 정체 발췌](stall_extract/transition.json)의 마지막 양수 call seq3555(ref61.12)는 candidate .1 m/s·`local_follow`다. 최초0 수락 seq3560(ref61.22)은 candidate0·`parallel_hold_baseline_unavailable`, baseline `follow_boundary_unconfirmed`다. 이번 최종 양수 이후에는 전달 취소(discarded)된0 call이 없어 `first_zero_discarded_present=false`로 명시했다. 존재하는 두 경우의 compute/current 입력·call·원 plan·근접 guard·첫 drive0 trajectory를 보존했고 원인 분석을 위한 제어기 재생은 하지 않았다.

독립 감시가 t70.185에서 마지막 진행t61.295 이후8.89 sim초 무진행을 관측한 뒤 [기존 STOP](early_stop.json)을 요청했다. `/autonomy/enable=false`와 `/actuation/stop`은 모두 성공했고 MCU `ESTOP_LATCHED`·출력0을 확인했다. 외부 STOP이 평가기 자체 STOP보다 먼저였으므로 원 `abort=mcu_latched_before_stop: software_stop`과 `no_mcu_latch_before_stop=false`는 [순서 기록](stop_sequence.json)에 그대로 보존한다. 실제 안정 STOP·조밀 pose·명시 MCU STOP·명령 만료 없음·벽 겹침 없음과 이 부가 실패를 구분한다.

`shutdown_clean=true`,14/14 자식 정상 종료·강제 정리 없음, [독립 잔여 PID0](execution_v1/owned_processes_after.json)을 확인한 즉시 WSL을 root에 반환했다. PID40118의 [원 실행 로그](execution_v1/runner.log), 원 exit1과 [수집 종료 코드](execution_v1/collection_exit.json) process0/integrity0/collection0을 보존했다.

[전체 수집](evidence_collection.json)은 필수 근거 누락0·수집 오류0·원 보고서 불변이다. 실제 snapshot454개 모두 동결 필수 범위에 들어가며 실행 전후 불변·snapshot SHA 불일치0이다. 필수462개·관측985개·ZIP도 회수 시 모두 불변이었다. trace 제출/완료638/638·call 입출력 불일치0·footer 정상 종료, 기대 헤더의 기존 정규화 동등성을 확인했다. r544 첫 시작 오류·attempt2 정체·이번 원 실패를 각각 보존하며 추가 trial이나 실물 장비 접근은 하지 않는다.
