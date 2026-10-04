# 최종 검토

- `count_after_v2`의 JSON은 통과 결과와 소스 해시를 담고 있다. 다만 실행 담당의 후속 중복 호출이 이미 존재하는 출력 폴더에 쓰려다 `FileExistsError`로 실패했다. 해당 console 말미의 `ACTUAL_EXIT_CODE=0`은 이 오류와 모순되므로 종료 코드의 신뢰할 근거로 사용하지 않는다.
- 이를 분리하기 위해 최종 검토에서 `C:\Python314\python.exe scripts/validate_motor_encoder.py --output artifacts/validation/2026-10-02/motor_encoder_validation/count_final`을 새 폴더에 한 번 실행했다. 실행 도구가 반환한 실제 종료 코드 **0**, `count_final/report.json`의 `passed=true`를 확인했다. Gazebo를 재실행한 것은 아니다.
- 최종 회귀는 `pytest_broad.log`의 **579 passed**다. `tests`와 `src/arena_vehicle_interface/test`를 실행하되 기존 정적 검사 집합과 같이 `test_lidar_shutdown.py`, `test_local_pursuit.py`, `test_replay_imu_probe.py`, `test_replay_observer.py`, `test_wsl_d3d12_lifetime.py`를 제외했다. 별도 실제 Gazebo/ROS 실행 결과와 합산하지 않는다.
- `count_before/report.json`의 수정 전 소스 해시와 `before_sources`를 대조했고 모두 일치했다. `count_after_v2/report.json`의 소스 해시도 최종 소스와 일치했다.
- 첫 Gazebo는 `system`·3초 wall 감시 조건에서 실패했고 종료만 통과했다. 재시험은 `wsl_nvidia`·30초 오프라인 wall 감시·12 m 목표로 주행/정지/종료를 통과했다. 전자는 전체 랩 요청, 후자는 부분 검사로 범위가 다르다.
- `gazebo_12m_v2/report.json`의 `actuation_observation.latched_fault_samples`에는 요청한 STOP 뒤의 정상 `ESTOP_LATCHED`도 포함된다. 이를 모두 주행 중 고장으로 세지 않는다.
