# r449 중지 인계

후속: 사용자의 명시 재개 뒤 검증을 완료했습니다. 현재 stage 결과는 [RESULT.md](RESULT.md)를 따릅니다. 아래는 12:33 UTC 중지 당시 인계 원문입니다.

사용자 중지 요청에 따라 독립 stage 저장 직후 중단했습니다. 공통 이관·새 시험 실행은 하지 않았습니다.

- 수정 3개: `runtime_dual_delivery.py`, `runtime_node_backend.py`, `g1_g2_node.py`.
- 신규 시험 초안 1개: `tests/test_runtime_plan_trace.py`.
- 기본 `g1_complete_plan_trace=false`; true일 때만 실제 성공 offer 직후 부모 소유 input과 lane 식별자/시각/reset 이유를 기록합니다. 실제 수집한 plan 완료는 input 중복 없이 별도 이벤트로 기록하고, 실제 evaluator header도 선택 ON에서 보존합니다.
- 기존 FollowTrace.emit과 bounded writer를 그대로 사용합니다. 정책/runtime_dual_state/제어 발행 경계는 수정하지 않았습니다.
- 시험 초안은 OFF 경로, 부모 폐기, mutable 소유권, 실패 offer, 늦은 세대 결과, current 반복, writer/freeze 오류 footer, header/옵션, 실제 Lane fake evaluator spawn과 stateful 재생을 포함합니다. **구문 검사와 pytest 모두 아직 실행하지 않았으므로 동작을 확인한 구현이 아닙니다.**
- 진행 중 시험/자식 PID 0, WSL/Jetson/ROS/장치 사용 0, 보유 자원 없음. 원본 3개 SHA는 source_before.json과 동일합니다.
- `apply.patch`는 현재 미검증 초안 diff이며 적용하지 않았습니다. 정확한 파일 SHA는 pause_receipt.json에 있습니다.

재개 시 주 담당이 작업 재개를 명시한 뒤, 먼저 4파일 diff를 검토합니다. 별도 validation 사본에 공통 Python package를 복사하고 stage 4파일을 덮어쓴 뒤, 해당 사본만 PYTHONPATH에 넣어 아래 5파일을 Windows에서 한 번 실행하는 것이 다음 단계입니다. 아직 validation 사본과 실행 wrapper는 만들지 않았습니다.

```powershell
python -B -m pytest -q -p no:cacheprovider tests/test_runtime_plan_trace.py tests/test_runtime_dual_slots.py tests/test_runtime_dual_contract.py tests/test_runtime_node_backend.py tests/test_runtime_dual_spawn.py
```

위 명령은 **stage overlay가 끝난 validation 사본 작업 디렉터리**에서 실행해야 합니다. 공통 루트에서 바로 실행하면 신규 stage 시험/코드를 검사하지 않으므로 그대로 실행하지 않습니다. 실제 Windows multiprocessing 결과와 종료/잔존을 회수한 후 주 담당이 공통 이관을 검토합니다. 이번 중지 중에는 이를 진행하지 않습니다.
