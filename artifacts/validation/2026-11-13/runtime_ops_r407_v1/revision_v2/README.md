# r407 revision_v2 — STOP 응답과 표시 상태 연결 보완

2026-11-13. **원 verify_v1은 실패 그대로 보존**한다. 이번 수정은 시험 하네스만의 상태 동기화와 거절 진단이며, 실제 B3/RaceOps/host 코드·운영 정책 변경은 없다. Windows에서 저장 근거 감사·순수 회귀·AST/해시 검사를 수행했다. 새 ROS/PTY 시험은 아직 실행하지 않았다. WSL/Jetson/실물 포트/센서/firmware에 접근하지 않았다.

## 원 실패의 원인과 확인 범위

원 `verify_v1/extracted/results/report.json`은 `passed=false`, `TimeoutError: resume input boundary`다. STOP은 `intervention=2`로 수락됐지만, 바로 다음 HTTP resume 요청의 view는 이전 `intervention=1`이었다. 실제 노드는 `intervention_mismatch`로 정확히 거절했다. `boundaries.jsonl`에는 `restore_race` 1개만 있고, 31개 상태 행 모두 resume 단계가 없다. 따라서 입력 경계 생성 실패나 host ARM 계약 결함으로 판단하지 않는다.

| 원 `ops/ops.jsonl`·HTTP/STOP receipt의 순서 | 같은 호스트 monotonic seconds |
|---|---:|
| STOP 정책 수락, intervention 2 | 2420.964414584 |
| 일치 host STOP tx/ACK 기록 | 2420.979725607 |
| 이전 intervention 1로 resume 요청 | 2420.985599424 |
| resume `intervention_mismatch` 거절 | 2421.003104813 |
| 새 `explicit_stop`, intervention 2 상태 수신 | 2421.122748577 |

새 상태는 STOP 수락 약158.334 ms 뒤 수신됐고, resume은 그보다 약137.149 ms 먼저 요청됐다. 원 대기문은 `resume_start_ready=true`만 보았는데 이 값은 복원 직후의 이전 `process_start` 상태에서도 이미 true였다. MCU 상태/ACK와 주기적인 autonomy 상태의 갱신 순서가 달라 생긴 하네스 전제 오류다. 실제 UI에서도 상태 갱신 전 짧은 표시 구간이 존재할 수 있지만, 이 시험에서 stale 요청의 실제 수신부 거절은 정상 보호 근거다. 그 표시 구간만으로 새 운영 계약 결함을 주장하지 않는다.

원 native/preflight exit0, source208/준비물 전후 불변, 외부 project import0을 확인했다. 종료 matching STOP tx/ACK, host의 의도한 signal2/exit1, ops exit0, 계산 자식·영상 자식 정상 회수, cleanup error0/강제 종료0은 별도 유지한다. 이는 실물 정지나 전체 시험 성공이 아니다. `original_audit.json`에 원 오류·정확한 요청/응답·행 번호·종료 근거와 12개 근거 파일의 SHA256을 기록했다. 원 manifest/archive/result는 덮지 않았다.

## 최소 변경

- `staged/r407/run_once.py`: 동일 STOP의 `/race/command_result`가 수락되었고, HTTP 표시의 session/race_epoch/intervention이 그 응답과 정확히 같으며 race_id가 원 요청과 같은지 확인한다. `explicit_stop`, 준비 가능, resume 없음, 기존 STOP 서비스 성공/MCU ESTOP 조건까지 확인한 단일 snapshot의 binding을 캡처한다. 그 view로 resume을 정확히 한 번 보낸다. 상태 재조회로 요청 view를 바꾸거나 거절 후 재시도하지 않는다.
- `staged/r407/operator_receipt.py`: 위 연결 조건과 거절 사유를 처리하는 하네스 전용 순수 helper다. 정책/어댑터 거절은 즉시 `RuntimeError`에 실제 사유를 남긴다. 준비 중인 응답은 기존 제한시간 안 기다린다.
- `tests/`: 원 실패의 두 상태/STOP 응답/resume 거절을 보존한 fixture와 좁은 순수 회귀다. 테스트의 HTTP display 구조는 동결 `OpsCore.view` 필드 투영이다. 완전한 HTTP GET 원문을 수집했다고 주장하지 않는다. 이후 상태 역시 원 실패 실행의 행이며 새 resume 성공으로 쓰지 않는다.
- `execute_isolated.py`: fresh root 접두사를 v2로 구분하고 하네스 회귀를 먼저 수행한다. 원 native/preflight/ROS 실행·종료·결과 회수 구조는 유지한다.

부모 검토용 차이는 `harness.patch`에 있다. `reference_original/`에는 원 run_once/executor/두 manifest의 바이트 사본을 둔다. 현재 common을 다시 수집하지 않고 **원 동결 archive**를 사용했다. 원208 중 run_once만 교체하고207개는 동일하다. helper/테스트/fixture 3개를 추가하여 새 bundle은211파일이다. `src/`, `scripts/`, profile, bindings, clock policy, FakeMcu, registration, preflight, run_ops는 원문 그대로다.

기존 대기3초, startup12초, 내부 전체45초/외부70초, lease/TTL100ms 및 시계 오차 가정, B3/독립 guard·센서 부재·양수 명령 없음·RESET 없음·추가 ARM 없음 조건을 바꾸지 않았다. 최초 CLEAR/ARM은 원 시험처럼 신선한 PTY에서만 명시 호출한다. 센서 입력을 합성하거나 encoder로 가짜 카운트를 발행하지 않는다. r401 기하/시간/frame와 r407의 등록된 B3·개발 상한 차이도 동일하다.

## 수행한 정적·순수 검사

Windows 명령:

```powershell
python -B artifacts/validation/2026-11-13/runtime_ops_r407_v1/revision_v2/tests/test_operator_receipt.py
```

8개 통과, 오류/실패0 (`windows_pure_tests.json`). 원 stale-ready 거절, 정확한 새 binding 캡처·사본 유지, 다른 session/epoch/intervention/race 거절, stale/다른 latch/이미 진행 중인 resume 제외, 응답 대기와 거절 구분, 원 `intervention_mismatch` 즉시 진단을 확인했다. 전체 ROS 동작의 통과로 세지 않는다.

`static_preparation.json`은211파일 AST와 원207파일·결과 근거 해시 보존을 확인한다. `audit_original.py`, `prepare_revision.py`는 생성 전용이고 기존 출력이 있으면 거절한다. 인계 후 다시 실행하지 않는다.

## 지정 실행자에게 인계할 후속 1회

root의 실행 배정 후 아래 명령만 수행한다. domain219/localhost를 원 실행 종료 후 다시 독점 사용하며, **새 isolated root와 새 PTY**만 만든다. 기존 root/receipt가 있으면 거절한다. 원 실패 폴더를 지우거나 재사용하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B artifacts/validation/2026-11-13/runtime_ops_r407_v1/revision_v2/execute_isolated.py \
  --root /home/jinhyeong/it-arena-r407-ops-v2-20261113-a \
  --receipt artifacts/validation/2026-11-13/runtime_ops_r407_v1/revision_v2/verify_v1
```

순서: 하네스 회귀10초 상한→native build120초→preflight30초→실제 ROS/HTTP/PTY **1회**, 내부45초/외부70초. 이식된 ROS Python 경로를 independent source 뒤에 유지한다. 단계 실패는 이후 진행을 중단한다. 외부 시간상한이면 원 방식대로 SIGINT10초 후 필요하면 KILL3초로 회수하며, 강제 회수나 잔여 process group은 실패다. 명령 시간/TTL을 늘리거나 자동 반복하지 않는다.

수용 범위는 STOP receipt와 캡처 view 일치, 정지 중0.3→0.2 m/s 상한 설정, 신호 대기 밖 수동 출발 거절, RESET 없는 resume 준비와 새 planner/guard 입력 경계 일치, 네 입력 누락에서 추가 ARM0/양수 명령0, guard 단독 `/drive/safe` 발행과 원 source 보존, matching 종료 STOP tx/ACK·자식 회수다. 실제 영상 출발·실물 출력·실차 정지·주행 성능은 모두 미검증이다. 합성 과거 경기 fixture와 기존 r366 근거의 범위는 원 README와 동일하다.

회수: 새 `verify_v1/execution.json`, `results.zip` 내 원 행/traceback/log/host summary/UART, `results/report.json`의 `stop_binding_confirmed`와 `resume_without_reset`, 준비물/추출 source 전후 해시. 다시 실패하면 첫 결과를 보존하고 멈춘다. source/receipt JSON 추가만으로 실패를 성공으로 바꾸지 않는다.

동결 bundle SHA256: `13291b23561fc13c828ff13775804753bb2c9c81e814854568a2df2937fe08ff`.
bundle manifest SHA256: `01bfeb59dbb6aa22fc72dde3fdfcf5a672f658d9d278b651613aaa138c4570a8`.
executor SHA256: `445cb20a176127cfe67e0aba0ff5a34b5c3a65c6b2429527f26866826cc1e16c`.

원 manifest SHA256은 `5d0718dc4246939383399211f29351a37ab2f572e531cbb22659b3eab59c8d2d`, 원 bundle은 `152dd6a50703ca09c6bfdc7363383b1dda017db3c79ebb570c15e184a1384db1`로 유지한다. 최종 revision manifest 생성·인계 후에는 부모의 명시 반환 전까지 문서도 수정하지 않는다.
