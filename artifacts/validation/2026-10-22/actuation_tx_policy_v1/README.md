# 송신 정책 후보 v1 — 변이체 1개 미검출로 실패

`run_checks.sh`의 **`checks_exit.txt=2`**. 소스 해시 차이 0 B(`source_hash_diff.txt`). 같은 소스로 v2를 다시 실행했으며 이 폴더는 실패 근거로 보존한다.

- 감사(`audit/report.json`, `audit.log`): 위반 0, 소스 불변. 수치는 v2와 같은 소스·같은 결정론 입력이며 [v2](../actuation_tx_policy_v2/README.md)에서 해석한다.
- 관련 회귀(`related_tests.log`): **130 passed**.
- 변이체(`mutants.log`, `mutants/*/pytest.log`): 원본 사본 58 passed, 12개 중 **11개 검출**. `stop_ack_not_first`(ack 큐에서 STOP 결과를 먼저 고르지 않음)는 58 passed로 **미검출**이었다. 기존 시험은 비STOP 결과가 모두 축출되어 큐가 STOP 결과로만 찬 경우만 보았기 때문에 FIFO와 STOP 우선이 같은 순서를 냈다.
- `set -e`로 전체 회귀 단계는 실행되지 않았다(`full_tests.log` 없음).

후속: 비STOP 3개 뒤 STOP 1개를 넣어 STOP 결과가 먼저 나가는지 보는 `test_stop_ack_overtakes_queued_non_stop_acks`를 추가했다. 이 변이체에서 실패하고 원본에서 통과함을 예비 확인한 뒤 v2를 새 폴더에서 실행했다. 보드·UART·ROS 노드 실행 없음, `firmware_verified=false`, `ready_to_drive=false`.
