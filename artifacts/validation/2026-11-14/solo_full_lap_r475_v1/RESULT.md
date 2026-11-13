# r475 동일 조건 단독 전체랩 결과

**단독 전체랩 1회 통과**. r474 현재 경로 우선 판정, r470 native 수치 커널을 포함한 동결 공통 소스로 이전 9.90 m·23.35 m 정체 지점을 모두 통과했다. 마커 observer는 OFF다. 원 `report.json`의 `passed`, `harness_valid`, `run_valid`, `behavior_gates_passed`가 모두 true이며 실행 종료 코드는 0이다. 자동 재주행하지 않았다.

이는 actor 0·시설물 3개인 고정 Gazebo 조건의 결과다. 실차, 교통 차량 조우, 마커 인식 또는 모든 코스 조건의 증명으로 확대하지 않는다. 평가 위치/자세는 평가에만 사용했고 제어·독립 보호 입력 경계는 원 검사에서 통과했다.

## 주행·속도

| 항목 | 결과 |
| --- | --- |
| 코스 길이 | 44.64 m |
| 랩 검출 시각 | simulation clock 57.283 s |
| GREEN 시각 | simulation clock 7.401 s |
| **GREEN 이후 랩 구간시간** | **49.882 sim s** |
| STOP 요청 시각 | simulation clock 57.287 s |
| 평가 종료 진행거리 | dense pose 기준 44.67 m; 랩 검출 trajectory 44.66 m |
| 9.90 m 초과 최초 저장 표본 | clock 16.667 s, GREEN 이후 9.266 s, 9.92 m |
| 23.35 m 초과 최초 저장 표본 | clock 31.423 s, GREEN 이후 24.022 s, 23.66 m |
| 기록된 최고 ground speed | **1.539498 m/s**, clock 48.429 s, 38.15 m |
| 기존 평가기 평균 진행 속도 | **0.895442 m/s** |
| 출발 후 가장 긴 정지 | 0.202 sim s |
| 기존 평가기 전체 정지 합계 | 0.606 sim s |
| 실행 wall time | 674.447 s; 랩 sim 시간과 다른 값 |

평균은 기존 평가기 정의 그대로 `중심선 진행거리 / GREEN→STOP 49.886 sim s`다. 최고값은 같은 평가 구간에 포함되는 기존 trajectory의 ground-speed 495개 표본에서 얻었다(표본 범위 clock 7.459–57.283 s). 명령 속도의 최고값이나 실차 계측값이 아니다. 임의 보간·새 속도 추정은 수행하지 않았다. 마지막 약 42 m 구간에서 감속했으나 진행이 이어졌고 완주했다.

## 접촉·STOP·정상 종료

- 벽 접촉 표본 0, dense footprint 벽 겹침 0/25,480, 보수적 footprint 노면 이탈 0, 이탈 바퀴 중심 최대 0개. 모두 모델의 표본 기반 평가 범위이며 연속 실물 접촉 검증은 아니다.
- `command_expired` 상태 표본 0, 제어 deadline 진단 0. MCU 종료 상태는 `ESTOP_LATCHED / software_stop`, target 0이며 STOP 요청 이전 latch 0이다.
- STOP 6개 원 검사 모두 통과했다. 정지 확인 대기 1.076 sim s, STOP 동안 변위 0.271990 m, 최종 0.5 sim s의 dense pose 최고 속도 0.009029 m/s다.
- 외부 조기 STOP helper는 필요하지 않았다. 랩 검출 뒤 기존 평가기의 STOP을 사용했다.
- 자식 프로세스 14/14 정상 종료, 강제 회수 없음, launch/readers 종료 코드 `[0, 0]`, 잔여 시험 PID 0. Windows launcher도 exit 0이다.
- WSL의 소유 PID 770 및 `GZ_PARTITION=arena_autonomy_test_770` 잔여 0을 확인한 뒤 소유권을 root에 반환했다. 이후 수집은 Windows에서만 수행했다.

## 원기록·계산 비용

계획 508개 제출/508개 완료, call 990개, current 계산 984개, 경로 발행 502개(양수 501개)다. 제출/완료 누락·중복과 call 입출력 canonical JSON 해시 불일치가 모두 0이다. footer accepted/written 3,356개가 일치하고 dropped/failed/pending/capture_errors가 모두 0이며 worker가 종료됐다.

| 통합 실행의 host 계산 시간 | 수 | 중앙값 | p95 | 최대 | 100 ms 초과 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 완료된 고유 plan | 508 | 20.865 ms | 35.539 ms | 110.855 ms | 1 |
| current | 984 | 4.674 ms | 6.161 ms | 70.932 ms | 0 |

이 값은 이번 PC/WSL 통합 부하에서 기록된 monotonic wall 시간이다. Jetson 성능이나 r470 단독 커널의 인과적 개선량으로 환산하지 않는다. `parallel_discard` 1개는 `retained_route_not_proven`이며 명령 만료와 구분한다.

기존 `runtime_audit.json` 원문도 보존했다. `known_structure_sectors_json` 직렬화 표현 때문에 `header_exact=false`지만, 기존 비교기가 값·타입·배열 순서 보존을 확인하여 `configuration_equivalent=true`다. 기존 crop retry 진단/계산 사용은 0이며 그 전용 auditor의 `configuration_evidence_valid=false`, `parent_publication_evidence_valid=false`를 성공으로 바꾸지 않았다. 실제 전체 주행·보호·종료 판정은 원 `run1/report.json`에 별도로 기록돼 있다.

## 소스 보존·준비 검증

동결 입력 1,365개 변경 0, `sources.zip` 동일, 원 보고서 `source_unchanged=true`다. 공통 기준 입력 1,323개와 root가 검토한 31개 delta를 사용했다. 주행 중 공통 소스 수정이나 Jetson 접근은 없었다.

준비 단계 colcon 빌드는 한 번만 수행했다. 생성된 `.so` 한 개가 바뀌면서 최초 import의 이전 binary 해시 검사가 실패한 원기록을 보존했고, root가 해당 단일 생성물 변경을 검토·반영한 뒤 재빌드 없이 계속했다. 재생성 native entrypoint import, 모듈/CLI 확인, 신규 native pytest **10 passed / 0 skipped**를 실제 수행했다. 빌드 성공을 주행 성공으로 대신 판정하지 않았다.

| 봉인 파일 | SHA-256 |
| --- | --- |
| prepared.json | `1adf724c6403eb6f5a7e57940674749c1eb0dac05bbd5dad33ff97c53c8e7fe1` |
| sources.zip | `8609c5d9dfc8f1f8924270cd1c6f0c92960cc3e5ae099873f67f39f87aa65191` |
| run1/report.json | `4df70c6de04a93f2dae35606d27d1a2892f96d03597cabceb232cd36a4143836` |
| run1/follow_decisions.jsonl | `98fe9c140e19a1b92ef9c846a4d022872a9e6de2aef59e532a5b0e85360bb11f` |

근거: [원 보고서](run1/report.json), [집계](execution_summary.json), [계산별 집계](timing_rows.json), [입력 무결성](integrity.json), [STOP 시퀀스](stop_sequence.json), [소유 프로세스 회수](execution_v1/owned_processes_after.json), [준비 검증](build_v1/receipt.json), [root 실행 검토](root_execution_review.json). 저장 로그 집계만 수행하는 [collect_result.py](collect_result.py)는 종료 후 작성했으며 동결 실행 입력에 포함되지 않는다.
