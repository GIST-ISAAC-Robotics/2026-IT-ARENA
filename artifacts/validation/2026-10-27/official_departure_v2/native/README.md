# 신호 시험용 지속 연결 클라이언트

3대 run1에서 광원별 `gz service` 프로세스의 응답에 `Host unreachable`/timeout이 발생했다. 이 도구는 Gazebo Transport `Node` 하나를 유지하며, 한 입력 줄마다 `gz.msgs.Visual` 요청을 딱 한 번 보낸다. ACK의 executed·service result·Boolean data를 각각 기록한다.3초 요청 한도는 기존과 같고 자동 재시도는 없다. 오류 응답/불명확한 결과에서는 종료한다. 근본 원인 확정이나 재발 방지 보장이 아니다.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S artifacts/validation/2026-10-27/official_departure_v2/native \
      -B artifacts/validation/2026-10-27/official_departure_v2/native/build
cmake --build artifacts/validation/2026-10-27/official_departure_v2/native/build
GZ_PARTITION=arena_material_selftest_local \
  artifacts/validation/2026-10-27/official_departure_v2/native/build/material_client --self-test
```

자체 검사는 동일 프로세스의 서비스 수락·거절·service result 실패·없는 서비스·잘못된 protobuf를 대조한다. 이 검사만으로 Gazebo 서버와의 실제 연결을 검증했다고 하지 않는다. 출력 바이너리는 Git 제외하며 각 실행 소스 ZIP에 바이너리/원본 해시를 함께 보존한다. 실제 신호 연결은 `signal_driver_v2.py`가 담당하고 물리 주행 결과는 상위 README와 해당 보고서를 따른다. 시험 코드이며 차량의 출발 신호 상태를 직접 주입하는 도구가 아니다.
