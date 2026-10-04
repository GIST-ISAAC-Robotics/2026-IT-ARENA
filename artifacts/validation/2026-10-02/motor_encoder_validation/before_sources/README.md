# 수정 전 소스

`boundary_before`의 8개 실패 및 `count_before`의 고착 미검출 당시 소스다. 작업 트리에 이미 이전 모터축 구성 변경이 있었으므로 Git HEAD만으로 당시 상태를 복원할 수 없어 수정 전 파일을 복사했다. 원본 바이트/줄바꿈을 보존한다.

- `actuation_contract.py`, `actuation_ros.py`, `mcu_speed_loop.py`, `drive_feedback.py`: `src/arena_vehicle_interface/arena_vehicle_interface/`의 동명 파일.
- `lidar_motion_ros.py`: `src/arena_autonomy/arena_autonomy/`의 동명 파일.
- `validate_motor_encoder.py`: `scripts/`의 동명 파일. `--sensor-only`로 당시 고착 실패를 재현한다. 당시에는 `update_mean`이 없었으므로 PI 시험은 아직 실행하지 않았다.

독립된 작업 사본에서 해당 위치로 복원한 뒤 시험해야 한다. 현재 작업 파일을 덮어쓰지 않는다. `drive_feedback.py`는 초기 경계/고착 보완 때에는 불변이었고, 후속 최종 검토에서 거대 유한 위치의 overflow 경계만 추가했다. 이 사본은 그 이전 버전이다. 의존 파일 전체나 실행 가능한 환경 이미지가 아니라 이번 수정 전 경계를 보존한 사본이다. 결과별 JSON과 `sha256.txt`의 해시로 대조한다.
