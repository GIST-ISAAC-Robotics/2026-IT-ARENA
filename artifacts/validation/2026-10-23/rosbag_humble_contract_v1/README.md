# Humble writer API 원문 대조

2026-10-23 조회한 [ros2/rosbag2 humble](https://github.com/ros2/rosbag2/tree/0aa2e05fe9cb3983812fec6c078c9d0bb4590353) SHA `0aa2e05fe9cb3983812fec6c078c9d0bb4590353`로 고정했다. `upstream/_storage.cpp`, `_writer.cpp`, Apache-2.0 LICENSE를 원문 그대로 보존했다. 실제 Jetson 설치 버전이라는 뜻은 아니다.

`_storage.cpp`의 `TopicMetadata` binding은 name/type/serialization_format/offered_qos_profiles 4문자열 생성자이고 `id` 속성/인수가 없다. `_writer.cpp`의 `SequentialWriter`는 open/write/create_topic/close binding을 제공한다. 기존 기록기의 Jazzy `TopicMetadata(id=...)` 호출을 그대로 적용할 수 없으므로 **설치된 클래스의 id 속성 존재 여부에 따라 해당 인수만 분리**하고, 명시 close 등 필수 API가 없으면 거절한다. 광범위한 TypeError 재시도로 다른 결함을 감추지 않는다.

Humble 서명을 따른 stub 회귀와 PC Jazzy 실행은 별개다. Jetson/Humble의 실제 설치·MCAP plugin·대용량 기록 성능은 미검증이며 원격 접속/설치를 하지 않았다. 소스 검사만으로 호환성을 완료했다고 표시하지 않는다.
