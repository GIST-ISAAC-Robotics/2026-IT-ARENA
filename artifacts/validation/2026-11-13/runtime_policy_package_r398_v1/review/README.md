# r398 이관 비교 인계

상태: **준비/Windows 정적 검사만 완료, pytest·spawn 비교는 미실행**. 정책 생성기를 다시 실행하지 않는다. 작성 범위는 r398 `review/`뿐이며 공통과 과거 artifact는 불변이다. [독립 검토](REVIEW.md).

## 고정 실행

Terra 전담 WSL 환경에서 저장소 루트를 cwd로 사용한다. 기존 검증된 Python 환경과 `native_v1` 확장이 필요하다. `review_support.py`가 staged package를 앞에 두고 동일 common source를 뒤에 놓는다. native 확장은 기존 설치 경로에서 찾되 모듈의 Python 구현은 공통 src를 사용한다. 설치/빌드가 필요하면 root가 별도로 배정하며 이 명령은 환경을 설치하거나 변경하지 않는다.

```bash
export PYTHONDONTWRITEBYTECODE=1
python3 -m pytest -q -p no:cacheprovider artifacts/validation/2026-11-13/runtime_policy_package_r398_v1/review/test_policy_boundary.py
python3 artifacts/validation/2026-11-13/runtime_policy_package_r398_v1/review/compare_spawn.py --numeric native_v1 --output artifacts/validation/2026-11-13/runtime_policy_package_r398_v1/review/verify_native_v1
```

첫 명령 실패 시 두 번째를 실행하지 않는다. 전체 묶음 상한은 외부 실행 담당이 300초로 두며 시험 자식이 남으면 해당 묶음의 소유 PID만 정상 종료/회수한다. 하네스의 각 Lane 계산 제한20초/대기25초는 시험 회수용이며 운영 TTL이 아니다. 한 번에 자식 하나이고 단계별로 정상 종료를 확인한다. 출력 경로는 새 경로만 허용한다. 재실행이 필요하면 실패 폴더를 보존하고 root 검토 뒤 `verify_native_v2`처럼 새 이름을 쓴다.

Python 구현은 위 pytest에서 사용한다. 같은 spawn 하네스의 `--numeric python`도 지원하지만 native 정확비교가 통과한 뒤 근거 없이 같은 전 묶음을 반복하지 않는다. 두 backend의 처리시간 비교나 새 성능 시험을 포함하지 않는다.

## 수용과 중단

- `manifest.json`의 시험 소스·원 ledger 정책·현재 common Python 파일·final YAML·원 trace가 시작/끝에 같아야 한다. hash 불일치는 기존 manifest를 조용히 다시 만들지 말고 root에게 수정 원인을 보고한다.
- pytest 전부 통과와 계획24/현재223/시각 정책 대표의 전체 결과·상태 일치가 필요하다. 전체 숫자는 exact이며 허용오차를 늘리지 않는다.
- 모든 Lane `forced=false`, `alive=false`, `exitcode=0`이어야 한다. 입력 불변과 모든 JSONL 원 결과를 보존한다.
- 최초 차이는 `report.json`의 경로와 해당 JSONL에서 읽는다. 불일치 후 나머지 단계는 자동 진행하지 않는다. 조건 완화·표본 교체·정답 덮어쓰기를 하지 않는다.
- 결과가 같아도 새 기본 정책 채택, ROS 실제 연결, 실물 출력 허가는 아니다. root의 공통 이관과 후속 backend 연결 판단에만 사용한다.

입력은 r377 `run1/follow_decisions.jsonl`, `source_ledger.json`의 원본17개, 현행 common, `vehicle.final_cad_2026_11_13.yaml`이다. `recorded_output`은 참고자료일 뿐 새 common/final 기하의 기대값으로 쓰지 않는다. 새 체인의 기준은 같은 입력·같은 common/final 기하를 새로 계산한 원 체인이다.
