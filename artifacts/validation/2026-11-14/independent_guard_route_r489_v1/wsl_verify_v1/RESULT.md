# r489 공통 이관 뒤 WSL 연결 확인

root의 `integration_v1/applied.json` 이관본 6파일 해시를 대조한 뒤 기존 ROS Jazzy/설치 메시지 환경과 common Python source import로 실행했습니다. Python 변경이므로 colcon/native 재빌드는 필요하지 않았습니다. 성능 행렬·센서·Gazebo·실차 실행은 하지 않았습니다.

실제 `LidarSafety` constructor는 `selected_r181 + traffic_recovery + bounded_local_v4 + both + simulation verified timing`을 받아 `command_observation_priority=true`, `partial_policy.command_observation_priority=true`, `partial_class=RecoveryPolicy`를 확인했습니다. executor/spin/control 호출 없이 `destroy_node/shutdown`을 완료했습니다. 구동 토픽 발행은 없었습니다. `constructor.json`에 import 경로/해시와 결과를 저장했습니다.

지정 6파일의 pytest 결과는 **100 passed / 80 failed**입니다. 80개 모두 기존 `tests/test_lidar_safety.py::node_double`의 `SimpleNamespace`에 r466 이후 `vehicle_marker_channel`이 없는 동일 AttributeError입니다. `test_partial_observation`의 `candidate_node`와 `test_known_structure_policy`도 이 대역을 공유합니다. marker OFF 대역 누락으로 제어 판정에 도달하지 못한 실패이며, 이를 r489 정책 통과로 세지 않았습니다. 신규 r489 fixture에는 OFF channel이 있고 해당 대표들은 통과했습니다.

원 `pytest.log/pytest.xml`, exit1, 실패별 분류 `failure_summary.json`을 보존했습니다. common 추가 수정은 하지 않았습니다. fixture 보완은 root 검토에 인계했습니다.

실행 wrapper exit0으로 결과 회수를 마쳤고, `collected.json`에 common source 불변·pytest/constructor 자식 잔존 PID 0을 확인했습니다. shell도 종료됐으며 WSL 소유권을 root에 반환했습니다. constructor 연결 성공과 기존 fixture 때문에 실패한 회귀 전체 판정은 구분해야 합니다.
