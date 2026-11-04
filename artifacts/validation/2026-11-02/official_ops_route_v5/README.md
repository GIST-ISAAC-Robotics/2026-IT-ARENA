# 앞차 대기 상태에서 같은 경기 START

`LocalPursuit.prepared_output`은 양의 계획 속도 또는 `follow_gap_hold`를 경로 성립으로 받는다. G1/G2의 현재 FOLLOW는 이 정보를 `normal` 안에 넣고 최상위 이유를 `handoff_follow`로 바꾸므로, 경로가 있어도 앞차 때문에 0인 경우 START가 시간 초과될 수 있었다. 코드 경로에서 확인한 결함이며 새 실차 현상이 아니다.

현재 상태가 FOLLOW이고 이유가 정확히 `handoff_follow`일 때만 `normal`의 경로 이유를 읽는다. BYPASS·REACQUIRE에 남은 이전 `normal`이나 입력 오류·미확정 경로를 쓰지 않는다. 준비 중 출력은 계속 0이고, 허가 후 속도는 기존 추종 제어·독립 보호층이 정한다. 경로 성립과 당장 전진할 수 있음은 구별한다.

원본은 [before.zip](before.zip), 연결 계약은 [official_ops_route_r5_sha256.json](../../../../config/tests/official_ops_route_r5_sha256.json)이다. 직접/중첩 앞차 대기, 미확정·입력 오류, 이전 상태 오용, 일반 주행 불변을 회귀로 확인한다. 관련 검사 결과는 `tests.txt`에 남긴다. 이 변경 뒤 새 폐루프 주행은 아직 하지 않았으며 다음 통합 주행에 포함한다.
