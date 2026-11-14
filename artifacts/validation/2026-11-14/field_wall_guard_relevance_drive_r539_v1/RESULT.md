# r539 같은 조건 교통 비교 결과

**실제 1회 원 주행 판정은 PASS이며, 전체 관측 파일 목록의 무결성 판정은 FAIL로 별도 보존한다.** 12 sim초 동안 5.660 m 진행했고 최종 앞차 간격은 0.636879 m였다. 원 판정 11개·STOP 6개가 모두 통과했고 정상 종료·잔여 소유 PID 0을 확인했다. 같은 조건 재실행은 하지 않았다.

| 동일 조건 비교 | r530 | r539 |
|---|---:|---:|
| 원 주행 판정 | FAIL(final_gap) | PASS |
| 진행 거리 | 2.880 m | 5.660 m |
| 최종 앞차 간격 | 3.421735 m | 0.636879 m |
| 원 기준 최장 정지 | 0.100 sim초 | 0.000 sim초 |

right_side_start·상대 높이220 mm/속도0.5 m/s·초기 +0.5 m/−0.23 m·시설3개·g1_open_1p5·medium_v1·RTF .25를 유지했다. 월드 SHA는 `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`로 r530과 같다. 변형은 첫 직선 두 벽의 미실측 가정(30 mm 굴곡·20 mm 접합 틈)이며 다른 코스 벽이나 실차 검증으로 확대하지 않는다. 제어 변경은 r538 `guard_memory_command.py` 1파일과 관련 시험2파일이다. 실측 조향 파생 설정과 r540 최적화는 이번 입력에 포함하지 않았다.

[원 보고서](run1/report.json)의 `passed`, `run_valid`, `harness_valid`는 모두 true이고 GREEN 구간은 7.401–19.401 sim초다. [별도 추월 평가](assessment_summary.json)는 실제 나란히 주행·추월을 성립시키지 못했으므로 false를 유지한다. 별도 평가의 정지 정의로 계산한 0.404초와 원 판정의 0.000초는 서로 다른 지표다. 이번 PASS를 추월 성공이나 단독 전체랩 완주로 해석하지 않는다.

[제어/보호층 기록 집계](controller_guard_supplement.json)에서 저속 재획득은 1계획이고 이후 정상 양수 FOLLOW는 125계획·249발행이다. 요청 속도가 0.1 m/s보다 큰데 retained memory가 0.1 m/s로 제한한 구간은 0개다. r530에서는 마지막 5.36 sim초 구간을 포함해 이 제한이 지속되었다. 저장된 `/safety/status`와 실제 발행을 집계했으며 입력 재생이나 새 실행은 하지 않았다. [첫 래치](guard_event_extract/first_latch.json), [첫 양수 출력·정상 상한 복귀](guard_event_extract/first_memory_normal_cap_restored.json)에 원문 행을 보존했다. 일반 수집기의 `positive_normal_follow_plans`는 selector와 handoff 상태를 혼동하므로 이 지표에는 별도 집계의 명시적 정의를 적용한다.

실행 PID는 21399이며 [실행 로그](execution_v1/runner.log)·[원 실행 종료 코드](execution_v1/run.exit)는 정상 0이다. [독립 프로세스 감사](execution_v1/owned_processes_after.json)의 잔여 소유 작업은 0이고 `shutdown_clean=true`다. STOP 안정 구간의 조밀 pose 근거도 통과했다. WSL 소유권은 회수 직후 root에 반환했다.

[무결성 원문](integrity.json)은 동결 목록 1,486개 중 `scripts/probe_esp32_motor.py`가 다른 세션에서 변경되어 실패했다. [원 차이](unused_tool_drift.diff)를 보존하고 해당 도구를 복구하거나 덮어쓰지 않았다. 이 때문에 실행 wrapper와 최종 수집 wrapper는 1로 종료했으며 원 주행 PASS와 구분한다. [실제 입력 감사](actual_input_audit.json)에서는 실제 snapshot의 267개 파일에 archive/live/frozen 불일치가 없고, 도구는 실제 snapshot·기록된 launch·명시 import 참조에 없었다. 차량·월드·별도 센서 YAML도 동결값과 일치했다. 전체 목록 불변을 성공으로 바꾸지는 않는다.

[전체 수집](evidence_collection.json)은 필수 근거 누락0·수집 오류0·원 보고서 불변을 확인했다. [수집 종료 코드](execution_v1/collection_exit.json)는 process0/integrity1/summary0/assessment0/collection0이다. 원 보고서 SHA256은 `61df9b7affc92db307840d6ece5f3c836e0a09acba92cce9ec2ee366f3a2e3d5`이며, 원 r520/r530 실패와 이번 원 결과를 보존한다.
