# r485 첫 출발 변형 회복 1회

r477 `start_left20_yaw_plus3`의 원 recipe·설정·시설·평가기·STOP 도구를 새 경로에 재사용한다. 출발은 동일 slot3에서 왼쪽20mm·yaw+3°이며 actor0, old610 시설3개, 동일 센서/차체/물성/속도, RTF .25, 최대80sim초,8sim초 무진행 owned STOP과 기존 노면/접촉 판정을 유지한다. 벽 변형은 이 묶음에 없다. 원 r477 실패와 사용자 STOP 결과는 보존한다.

현재는 **Windows recipe 준비만 완료**했다. `source_selection.json`은 공통 통합/동결 허용/WSL 소유를 모두 false로 둔다. root의 실제 인계 후에만 세 값을 갱신하고 준비 명령을 수행한다. 새 source/archive는 아직 동결하지 않았다.

주행 제어 변경 범위는 r481 v2의 `local_follow.py`, `race_runtime_policy/current_route_priority.py`와 r482의 `current_lane_preference.py`이다. 원 r477 manifest에 포함된 `scripts/jetson_access.py`의 별도 Wi-Fi stdin 변경은 **시험에서 실행하지 않는 보조도구 delta**로 분리 기록한다. 나머지 기존 동결 입력은 같아야 한다. 기존 native binary SHA를 재사용하며 새 native 빌드나 전수 회귀를 하지 않는다. 실제 import가 공통 src의 승인된 파일을 선택하는지 preflight에서 확인한다.

공통 통합 완료·WSL 인계 뒤 준비 명령(아직 미실행):

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_variation_recovery_r485_v1/execution/prepare_once.sh'
```

준비 후 생성될 `execution_prepared.json`/`execution_sources.zip` SHA를 root에게 전달한다. root가 `cases/start_left20_yaw_plus3/root_execution_review.json`에 `root_reviewed=true`, `case=start_left20_yaw_plus3`, `execution_limit=1`, `repeat_on_failure=false`, `wsl_exclusive_owner_confirmed=true`와 두 SHA를 기록하고 실행을 지시한 뒤 아래 명령을 정확히1회 수행한다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_variation_recovery_r485_v1/execution/execute_once.sh' start_left20_yaw_plus3
```

기존 `execution/observe.py`, `execution/stop_once.sh`, `execution/check_owned_processes.py`를 같은 첫 조건으로 사용해 모니터·owned STOP·PID/partition 회수한다. 등록/run1이 생기면 성공/실패와 무관하게 반복 실행하지 않는다. 최초 다른 실패 또는1랩까지의 원 report/plan-current 입력, 최장 정지·접촉·STOP·입력 무결성을 회수한다. 정답 위치·경로는 평가에만 쓰고 제어기에는 넣지 않는다.

재사용 근거와 파일별 출처는 `recipe_provenance.json`에 있다. 이 준비 자체는 주행 성능/완주 또는 실차 증거가 아니다.
