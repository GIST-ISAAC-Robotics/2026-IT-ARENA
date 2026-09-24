# A1-2 구동 계약 ROS 연결 검증 자료

상세 구현/경계/최종 결과는 [A1-2 보고서](../../../../docs/hardware/ACTUATION_ROS_2026_09_22.md)에 정리한다.

최종 결과: `ros_final` 14/14조건 및 6프로세스 정상 종료, `gazebo_host_exit_v1` 12.15847 m·실제 정지·정상 종료, 전체 회귀 365개 통과. 최종판 이전 `gazebo_stop_v1`도 12.21847 m·실제 STOP 정지·정상 종료를 통과했으며 이후 시계/ACK/준비 판정 보완 전의 별도 소스 버전이다. `ros_v1`·`ros_v2` 실패를 포함한 모든 실행은 보존한다. 코스 한 바퀴 검증이나 실차 안전 결과가 아니다.

## 자료 해석

- `build_v*`, `final_regression`: 빌드·단위/회귀 검사다. 주행 결과가 아니다.
- `ros_v*`, `ros_final`: 별도 ROS 프로세스와 합성 엔코더·단순 종방향 모형의 고장 시험이다. Gazebo 또는 실물 모터가 아니다. `report.json`의 전체 `passed`, 오류, 각 조건, 프로세스 종료·강제 정리·잔존 여부를 함께 확인한다.
- `gazebo_*`: 기존 공식 월드·지역 Pure Pursuit·LiDAR 보호층 뒤에 가상 MCU를 넣은 코스 시험이다. `report.json`, `trajectory.jsonl`, `stop_trace.json`, `actuation_trace.json`, 소스 스냅샷, 실행 로그를 함께 본다. 부분 주행을 전체 코스 완주로 표현하지 않는다.
- `source_sha256`/`input_sha256`는 실행 시점 코드이며 후속 수정 코드와 같다는 뜻이 아니다. 실패 디렉터리를 덮어쓰지 않는다.

## 재현

WSL의 ROS Jazzy와 이 작업 공간의 install 환경을 로드한 후, 존재하지 않는 새 출력 폴더를 지정한다.

```bash
python3 scripts/validate_actuation_ros.py --output artifacts/validation/2026-09-22/actuation_ros/ros_new
python3 scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit \
  --actuation-mode virtual_mcu --lidar-acquisition sequential --lidar-compensation both \
  --control-rate-hz 50 --speed-profile local_fast --disable-speed-bump --grid-slot 3 \
  --sensor-wall-timeout-s 30 --shutdown-mode service --target-progress-m 12 \
  --max-sim-seconds 90 --wall-timeout-s 900 --stop-wall-timeout-s 120 \
  --output artifacts/validation/2026-09-22/actuation_ros/gazebo_new
```

호스트 종료에 따른 정지 시험은 두 번째 명령에 `--actuation-stop host_exit`를 추가한다. 시험 프로세스가 생성한 독립 launch 로그에서 단일 호스트 PID를 확인한 뒤 종료하며, 다른 사용자의 ROS/Gazebo 실행을 종료하지 않는다. 기본 시험은 MCU STOP 서비스를 사용한다. 속도/정지 한도를 통과시키기 위해 완화하지 않는다.

상세 실패/수정 기록은 보고서와 [당일 활동](../../../../docs/activity/2026-09-22.md)에 보존한다. 실물 ESP32 펌웨어·ESC 전기 신호·하드웨어 차단을 검증한 자료가 아니다.
