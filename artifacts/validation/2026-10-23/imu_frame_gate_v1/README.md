# IMU 명시 frame 검사 옵션의 회귀

- `motion_imu_frame_id`를 설정했을 때만 정확히 같은 `header.frame_id`를 받는다. 불일치는 `gyro_frame_mismatch`이며 이력 시각/값을 갱신하지 않는다. 실제 축/장착 교정·TF 변환·하드웨어 드라이버는 아니다.
- 기본 빈 문자열은 기존 동작을 보존한다. 현재 launch/기존 bag의 frame 이름을 가정해 바꾸지 않았다. 실물 연결 시에는 실제 기대 이름과 장착 변환을 명시해야 한다.
- 추가 20사례: 다른/빈 이름, 잘못된 설정, 명시 일치 뒤 기존 단위/finite·장착 투영 유지, 거절 시각의 정상 표본 재수용, 기본 호환, 반복 불일치에 따른 이력 만료. ROS 메시지 객체와 NodeDouble로 검사하며 실제 DDS/장치 시험이 아니다.
- `run_checks.sh` 1회: **4,301 passed·실패/skip 0**, 168.91 s, exit 0. 이전 `integration_review_v1`의 ASAN/UBSAN C++ 검사 바이너리 6개와 원문 해시를 확인해 재사용했다. 새 C++ 빌드/새 Gazebo 주행을 주장하지 않는다.
- 전체 `src/scripts/tests/config` 전후 해시 차이 0 B, 당시 소스 스냅샷 로컬 보존. 검사 프로세스 잔존 없음. 원래 G1/실물/종료 실패는 해결된 것으로 바꾸지 않았다.

[입력 이관 계약](../../../../docs/hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#93-현재-소비자와-남은-경계)을 따른다. 실제 관측 frame과 축의 검증이 없으면 이 옵션만으로 운동 보정을 허가하지 않는다.
