# 2026 IT ARENA 자율주행 경주

GIST-ISAAC-Robotics의 2026 IT ARENA 자율주행 대회 참가를 위한 소프트웨어·시뮬레이션 작업 공간입니다.

주최 측에서 제공한 트랙 자료를 출발점으로 삼으며, 다음을 목표로 합니다.

- 팀이 확정한 전체 외형 20 cm x 15 cm를 사용하고, 축거·윤거 등 미확정 치수를 매개변수로 조정할 수 있는 Ackermann 조향 차량
- RealSense D435i와 호환되는 시뮬레이션 카메라·깊이·IMU 인터페이스
- Jetson과 ESP32의 구동 제어 역할 분리 및 매개변수로 조정 가능한 휠 엔코더 시뮬레이션
- 필요에 따라 낮은 위치에 장착한 2D LiDAR와 근거리 ToF 센서로 감지 범위 보완
- 카메라 영상만을 이용한 신호등·ArUco 마커 인식
- 단독 차량의 경로 추종을 먼저 구현한 뒤, 여러 차량이 함께 달릴 때의 안전 주행과 회피로 확장
- Gazebo에서만 얻을 수 있는 정답값과 실제 Jetson에서 실행할 자율주행 코드의 명확한 분리

## 현재 상태

- 원본 트랙 ZIP은 `assets/track/original/`에 보존했으며, SHA-256 해시를 기록했습니다.
- 압축을 푼 트랙 자료는 `assets/track/source/`에 있습니다.
- 시뮬레이션 실행 환경으로 WSL2 Ubuntu 24.04를 선택했습니다.
- PC 시뮬레이션 소프트웨어로 ROS 2 Jazzy + Gazebo Harmonic을 선택했습니다.
- 전체 평면 외형은 길이 20 cm·폭 15 cm로 팀이 확정한 설계 목표입니다. 축거 14.5 cm, 윤거 13.5 cm, 바퀴 지름 5 cm·폭 1.2 cm와 센서 배치 등은 실측 전 임시값입니다.
- 현재 기본 모델은 20 cm x 15 cm 외형을 사용합니다. 초기 수동 주행·센서 검증 및 아래 화면은 변경 전 18 cm x 12 cm 모델 기준이며, 크기 변경 후 검증 범위는 활동 기록에서 구분합니다.
- 원본 재현용 본선 35 cm·지름길 12 cm 지도는 보존했습니다. 기본 실행은 별도 실험 지도인 본선 45 cm·지름길 25 cm이며, 중심선과 경로 길이는 유지합니다. 어느 쪽도 최신 공식 코스라고 확정한 상태가 아닙니다.
- 수동 전진·조향 제어, 오도메트리(이동량 추정), D435i RGB·깊이·포인트 클라우드와 엔코더 토픽을 구현했습니다. 최종 검증 결과는 활동 기록에 남깁니다.
- 확대된 축거·윤거로 원본·실험 지도 각각의 짧은 전진·조향·RGB·깊이·IMU·엔코더·명령 중단 정지와 종료를 확인했습니다. 자동 검사 22개가 통과했으며, 단독 완주나 지름길 진입 궤적을 검증한 것은 아닙니다.

현재 인수인계 내용은 [프로젝트 현황](docs/PROJECT_CONTEXT.md)을 참고하십시오. 주최 측 트랙 README를 근거로 사용하기 전에는 [트랙 감사 기록](docs/track/TRACK_AUDIT.md)을 먼저 읽어야 합니다.

계속 참고할 원문은 [2026-06-30 미팅](https://maddening-cause-ce7.notion.site/2026-06-30-38f99fd42e3080f6956fe5a5b90d0824)입니다. 당시 기준과 현재 파일을 구분하며, 지도 전환·재생성과 차량 치수 근거는 [실험 트랙 안내](docs/track/EXPERIMENTAL_TRACK.md)에 정리했습니다.

![제공된 IT ARENA 트랙을 불러온 Gazebo 기초 구현 화면](artifacts/screenshots/gazebo_baseline.png)

## 기본 실행 절차

```bash
sudo bash scripts/setup_wsl.sh # Ubuntu 24.04 안에서 최초 한 번만 실행
bash scripts/configure_wsl_user.sh
bash scripts/doctor.sh
colcon build --symlink-install
source install/setup.bash
ros2 launch arena_bringup simulation.launch.py
```

Gazebo 창 없이 실행하려면 `headless:=true`를 사용합니다. `/drive`에 `ackermann_msgs/AckermannDriveStamped` 형식의 명령을 발행하며, 시뮬레이션 엔코더 피드백은 `/wheel_states`와 `/wheel_encoder_ticks`에서 확인할 수 있습니다.

기본 실행은 실험 지도입니다. 원본 재현용은 아래와 같이 선택합니다. 차량은 두 지도 모두 20×15 cm이므로 원본의 12 cm 지름길은 이용할 수 없습니다.

```bash
ros2 launch arena_bringup simulation.launch.py track:=original
```

환경 설정을 불러온 별도의 WSL 터미널에서 다음 저속 주행 명령을 실행할 수 있습니다.

```bash
ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {steering_angle: 0.15, speed: 0.25}}"
```

명령 발행을 중단하려면 `Ctrl+C`를 누릅니다. 차량 측 명령 감시 기능(watchdog)은 새로운 명령이 0.5초 동안 들어오지 않으면 값이 0인 명령을 보냅니다.
