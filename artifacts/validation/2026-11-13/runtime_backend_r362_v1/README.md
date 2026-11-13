# r362 선택형 두 프로세스 실행 계약 추출

2026-11-13. GPT가 r336/r334의 실행 경계를 공통 패키지의 선택형 준비물로 추출했다. **기존 노드·프로필·알고리즘·기본 활성화는 바꾸지 않았다.** 실제 센서/Jetson/Gazebo/ROS 실행이 아니며, r358의 10Hz 미달이나 r353의 주행 실패를 해결한 결과가 아니다.

## 범위

| 파일 | 소유하는 계약 |
|---|---|
| `runtime_dual_process.py` | 계획/현재 검사 각각 단일 진행 슬롯, 원 JSON 바이트 소유, 세대 변경 뒤 늦은 결과 폐기, 완료 시각/작업 제한, 두 자식 정리 보고 |
| `runtime_dual_state.py` | r266 경로 보관/현재 증거, r286 동일 양수 의도, r323/r325 제출 여유, r334 같은 의도의 감속 상한 |
| `runtime_dual_delivery.py` | r273 최신 미사용 스캔 제출, r282 후보 인계, r336 실제 발행 확인 뒤 경로 보관 |

`src/arena_autonomy/arena_autonomy/runtime_dual_*.py`에는 시험 폴더 import, 지연 주입, 센서/ROS/구동 활성화가 없다. 원천300ms·명령200ms·경로500ms·양수발행100ms와 현재 계산 제출 예산100ms를 유지한다. 경로 보관은 원 기하를 다음 **새 현재 검사**에 제공하는 것이며, 옛 점유 허가나 명령 시각을 연장하지 않는다.

`RuntimeDualProcess`는 `evaluator_factory`와 `supported`를 필수로 받는다. factory는 spawn 가능한 최상위 callable로 `factory(stage, header)`를 통해 해당 단계 evaluator를 만들고, evaluator는 `reset(reason)`·`__call__(data)`를 제공해야 한다. 계획 단계의 data는 원 snapshot, 현재 단계는 `output/submitted/current/retained`다. `supported(output)`는 선택한 현재 검사 정책의 원 지원 범위를 판정한다. 원 r278/r352의 정책 추출/연결은 별도이며, 임의 기본 정책으로 대체하지 않았다.

## 호출자에 남은 연결

1. 원 알고리즘과 같은 evaluator factory 및 지원 판정을 준비한다. 기하/가림/시각 계약을 함께 등록한다. r278/r352 정책의 시험 import 폐포를 이 실행 경계에 몰래 포함하지 않는다.
2. 명시 선택형 노드에서 기존 공통 신호·센서·발행 경계를 거쳐 `poll`한 후보를 발행한다. 같은 콜백에서 실제 `reference_s/speed/steering`으로 `acknowledge_publication`을 호출한다. 발행 예외는 `invalidate`한다.
3. STOP/RESET/시계 역행 때 delivery를 `invalidate`한다. 실제 발행마다 노드의 기존 `last_request_time_s`와 delivery의 `last_application`을 갱신한다. 보관/제출 여부와 실제 발행을 혼동하지 않는다.
4. 완료 알림은 두 pipe를 모은 `worker.connection`을 사용한다. 종료 때 알림을 먼저 닫고 `worker.close()`의 `lanes/forced/alive/exitcode`를 검사한다. `worker.process` 한 개를 가정하지 않는다.
5. 운영 시각/실제 엔코더/독립 보호/MCU 권한은 별도 연결이다. 현재 live 엔코더 부재를 0속도 이력으로 채우지 않는다.

## 검증

- Windows AST: 새 제품 모듈3개·회귀2개·저장 대조 스크립트 구문 통과.
- 초기 순수 회귀: `tests/test_runtime_dual_contract.py`, `tests/test_runtime_dual_slots.py`의48검사를 WSL 담당자가 실행했다. 동시 확인한 카메라/preview의 별도2개 파일과 합쳐 **4파일69검사 통과(1.33초)**다. 기존 r334 감속/r336 발행 반례와 원 입력 소유, 두 슬롯 동시 진행, STOP 뒤 늦은 결과, 제출 시각/예산, 정리 예외를 확인한다. 초기 묶음에는 실제 자식 spawn이 없다. [실행 로그](../released_crawl_r361_v1/external_contract_tests.log).
- 저장 대조: `check_saved_contract.py --limit 12 --output .../saved_contract.json`를 같은 WSL 담당자가 실행했다. **원 r353의12계산 결과×10조건=120비교 전부 일치**, 발행 확인 조건72건을 포함한다. 현재/만료/epoch/시계 역행/미발행/감속/조향 변경/늦은 확인/STOP의 조건을 비교한다. 원 입력/출력은 바꾸지 않으며 [보고서](saved_contract.json)는 해시·비교 개수·불일치만 보존한다. 계획 재계산이나 성능 측정은 아니다.

GPT 실행 담당의 반환 뒤 로그와 JSON을 독립 확인했다. 이 작업은 기존 `g1_g2_node.py`·hardware shadow runner/probe/config를 변경하지 않았고, live에서 이 backend가 실행됐다고 주장하지 않는다. [정적 출처 감사](source_audit.json)는 핵심8메서드의 원본 AST 동일과 원본9개/신규6개 파일의 해시를 기록한다. 초기 감사는 [spawn 전 기록](source_audit.before_spawn.json)에 보존했다.

## 실제 spawn 후속 확인

`Lane.__init__`의 시작 handshake가 `poll`/`recv` 예외를 내면 기존 분기의 `close()`를 건너뛰는 경로를 확인했다. handshake를 `try/except BaseException`으로 감싸 해당 Lane의 자식/pipe를 회수한 뒤 원 예외를 다시 전달하도록 한정 수정했다. 출력·기한·정책은 바꾸지 않았다.

`tests/test_runtime_dual_spawn.py`의 작은 최상위 evaluator로 WSL에서 실제 두 자식을 생성했다. 첫 계획 완료 뒤 다음 계획과 현재 검사를 각각 Event로 잠시 보류하고 두 자식의 진입을 확인한 다음 STOP 세대를 올렸다. 해제 뒤 구세대 완료2개가 모두 폐기되고 새 계획의 evaluator reset 횟수가2가 되며, 두 자식 모두 `exitcode=0`·`alive=false`·`forced=false`로 종료했다. 별도의 작은 자식은 handshake 이전 `exit23`을 주입하여 EOF가 나도 pipe closed·자식 회수가 확인됐다. 이 비정상 종료는 의도한 시험이며 정상 결과로 위장하지 않았다.

새 smoke2개와 기존48회귀를 합쳐 **50검사 통과(2.18초)**, 실행 명령 exit0이다. [spawn 실행 로그](spawn_tests.log). 이는 작은 테스트 evaluator의 프로세스 계약 검증이며 실제 r278/r352 계산·ROS·Jetson·장치 검증이 아니다. 실제 알고리즘 factory와 노드 연결은 여전히 남는다. WSL 소유권은 실행 종료 뒤 주 에이전트에 반환했다.

## 원 계약

필요 메서드는 `retained_route_execution_r266_v1/overlap.py`·`route_delivery.py`, `current_priority_r286_v1/priority.py`, `current_phase_r323_v1/phase.py`, `current_deadline_r325_v1/deadline.py`, `current_deceleration_r334_v1/deceleration.py`, `latest_scan_submission_r273_v1/submission.py`, `current_region_runtime_r282_v1/delivery.py`, `current_ros_bridge_r336_v1/ros_delivery.py`에서 가져왔다. 이 원본은 수정하지 않았다. 초기 선택 메서드 추출 과정은 `extract_backend.py`에 남겼으며 **이후 수동 회귀 추가를 포함한 완전한 재생성 도구가 아니므로 통상 실행하지 않는다.**
