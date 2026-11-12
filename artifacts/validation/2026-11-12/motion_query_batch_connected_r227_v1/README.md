# 두 조향 원호 질의 일괄 계산 공통 연결 r227

[r226](../motion_query_batch_r226_v1/README.md)의 저장 비교와 Jetson 비용 감소를 확인한 뒤 `pass_release_trial.motion_queries`에 같은 후보를 연결한다. 두 원호의 자세/바퀴와 unknown 교차를 모두 유지하고 질의 생성에 필요한 중복 설정·정렬만 줄인다. 새 정책/옵션·물성·보호 기준을 추가하지 않는다.

r225 종료·소스 불변을 확인한 뒤 변경했다. [변경 전 소스](before_sources.zip)·[기준 해시](before_manifest.json)·[출처 계약](../../../../config/tests/motion_query_batch_r227_sha256.json)을 남겼다. 원 함수와 좌표 전체 대조, 현재 재검사/선택기/출처 회귀를 진행한다. 전체 ROS 실시간 달성이나 주행 실패 해결을 의미하지 않는다.

관련389검사를 통과했다([로그](tests.log)). 같은 소스의2패키지 빌드는 [r229 빌드](../stale_command_observation_connected_r229_v1/build.log)에서 함께 완료했다. 이후 주행은 r229의 관측 갱신 변경과 결합한 r230으로 구분하며, r227만의 물리 주행 효과를 따로 측정한 것은 아니다.
