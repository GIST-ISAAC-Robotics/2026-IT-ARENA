# r233 — 선택 구성의 일반 기록 재생 연결

11/12 main `e54527f7`의 실행 계획에 따라 r231에서 확인한 초기화 의미를 공통 재생기에 연결했다. `follow_trace.replay_reset`을 일반 재생과 `profile_follow_trace`의 측정 반복에서 함께 쓴다. `expiry_route_reference_trial=true`이고 원인이 `decision_source_expired`인 경우에만 기존 r218의 최근 기하 참조 보존 함수를 호출한다. STOP·RESET·센서 세대 변경 및 옵션 OFF는 전체 초기화한다. 명령이나 활성 경로를 보존하는 변경은 아니다.

- 관련 회귀 **269 통과**: [tests.log](tests.log).
- r225 저장 입력 **632회 일반 재생 및 632회 측정 반복 모두 일치**, 원시 입력 SHA 불변: [saved_result.json](saved_result.json), [검증 스크립트](verify_saved.py).
- 이전 소스: [before_sources.zip](before_sources.zip), [SHA](before_sha256.json). 후속 소스 계약은 `config/tests/selected_trace_r233_sha256.json`이며 r229 이후 변경을 등록했다.
- ROS/Gazebo/Jetson 새 주행은 수행하지 않았다. PC 측정 반복은 재현성 확인이며 Jetson 성능 개선 증거가 아니다. r231의 당시 미연결 상태와 r230의 원 판정은 그대로 보존한다.
