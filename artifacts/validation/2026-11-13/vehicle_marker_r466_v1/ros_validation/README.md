# r466 무구동 ROS 대표 준비

이 문서는 **v1 준비 명세와 실패 이력**이다. 중첩 quad 수정 뒤 별도 v2의 ROS import 2개·제한 대표가 통과했으며 최신 결과와 후속 PID 감사의 진입 실패는 [현재 인계](../RESULT.md)와 [v2 상세](../revision_v2/README.md)를 따른다. v1은 정면 중복 quad 제외로 cap이 없었던 실패로 보존하며 아래 v1 명령을 다시 실행하지 않는다. 공통 소스·설치·서비스 변경이나 build는 없었다.

## 입력과 실행 범위

- 배포판 `Ubuntu-24.04`, `/opt/ros/jazzy/setup.bash`, 기존 공통 `install/setup.bash`를 읽는다. Python 대상 모듈은 별도 `/home/jinhyeong/it-arena-r466-ros-a`에 푼 동결 ZIP에서 import하며 실제 import 경로를 결과에 기록/검사한다.
- `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=96`. 한 Python executor 안의 producer/실제 `LidarSafety`/collector 3노드만 사용한다. 기존 `/drive`→`/drive/safe` 토픽을 전용 domain 안에서 시험한다. graph에 다른 노드가 있으면 중단한다.
- 장치 카메라·센서 포트·ESP·Motor bridge·ARM·flash·Gazebo를 띄우지 않는다. 범위는 합성 입력의 소프트웨어 연결이다.
- profile은 `run_ros_validation.py --profile 절대경로`에서 받고, `rclpy.init`의 ROS parameter `vehicle_marker_profile_path`로 producer와 실제 guard 양쪽에 같은 파일을 주입한다.
- 합성 LaserScan은 720개/3 m 유한 반환, wheel/IMU는 0 운동이다. 실제 ROS wall clock과 각 입력 stamp를 사용하고 `lidar_compensation=both`, `motion_scan_timing_verified=true`를 **합성 시각 계약에 한해서** 선택한다. 하드웨어 시각 검증으로 쓰지 않는다.
- 실제 `VisionProcess`가 합성 4×4/1 RGB를 검출하고 `VisionPipeline`→`MarkerChannel` publish→guard subscribe→실제 guard timer→`/drive/safe`까지 연결한다. pose packet의 취득 시각을 새 시각으로 바꾸어 재전송하지 않는다.

## 한 번의 순서와 판정

1. Windows에서 제외된 기존 ROS import 검사 2개를 먼저 실행한다. 실패하면 뒤의 대표를 시작하지 않는다.
2. 패킷 부재에서 기존 유한 LiDAR 입력과 요청 .75 m/s를 유지한다.
3. 정면 마커가 있을 때 guard가 별도 점유 cap을 적용하여 .75 m/s 미만으로 제한한다.
4. 측방으로 벗어난 마커에서는 기존 .75 m/s 출력과 사유를 유지한다.
5. 정면 마커를 다시 짧게 제공한 뒤 RGB 제공을 멈춘다. 원 stamp부터 .25 s가 지난 보조가 소거되고 기존 .75 m/s로 돌아오는지 확인한다.
6. 종료 전 요청 0을 보내 `/drive/safe`의 0을 관측하고 VisionProcess를 join, 노드를 destroy, executor/context를 종료한다. 실제 차량 정지의 증명은 아니다.

초기 discovery/운동 이력 준비 구간을 지난 각 단계의 후반 표본 3개 이상을 판정한다. 총 대표는 약 5.2 wall s이고 내부 상한 12 s, 외부 상한 30 s이다. 실패 시 원 결과를 보존하고 자동 재실행하지 않는다. 첫 WSL 서비스 진입 실패라면 gate/started 영수증으로 시작 여부부터 구분한다.

## 고정 실행 명령 — 자원 인계 전 실행 금지

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/vehicle_marker_r466_v1/execution_v1/run_once.sh'
```

스크립트가 실제 사용하는 Python 인자는 다음과 같다. 위 스크립트와 별개로 중복 실행하지 않는다.

```bash
python3 -B /home/jinhyeong/it-arena-r466-ros-a/validation/run_ros_validation.py \
  --profile /home/jinhyeong/it-arena-r466-ros-a/config/vehicle_marker_approx_b0390.json \
  --output /home/jinhyeong/it-arena-r466-ros-a/run1 \
  --import-tests /home/jinhyeong/it-arena-r466-ros-a/tests/test_local_follow.py
```

실제 운용에 통합한 뒤에는 기존 planner(`g1_g2_handoff_trial`)와 `lidar_safety` 양쪽 기존 `--ros-args` 안에 같은 `-p vehicle_marker_profile_path:=/절대/profile.json`을 추가한다. 이 검증의 합성 `motion_scan_timing_verified=true`를 실물 운용 명령으로 복사하지 않는다. 실물의 기존 `host_estimated_v1`/timing false·설치 상한·B3·독립 보호 설정은 그대로 유지한다.

## 산출물과 미검증

- `execution_v1/prepared.json`: 232개 입력의 SHA·출처, archive SHA, 고정 명령.
- `execution_v1/source.zip`: 읽기 전용 공통 Python snapshot + r466 stage override + 대표 도구/표본 profile. C++/native build 변경 없음.
- 실행 뒤 `execution_v1/run1/{result.json,safety_status.jsonl,drive_safe.jsonl,marker_packets.jsonl,ros_import_tests.xml}` 및 `completion.json`을 회수할 예정이다.
- 이 대표는 native plan/current의 전체 saved trace 재생, 전체 자율 노드, 실시간 10 Hz 통합 부하, 실제 ArUco 출력물·카메라 설치·주행을 검증하지 않는다. 그 범위를 이번 소규모 연결 확인에 추가하지 않는다.
- 기존 `stages_ms.markers`는 공유 4×4 검출 구간이다. 추가 7×7 검출/PnP는 그 값 밖에 있으며 전체 `duration_ms`에는 포함된다. 이후 비용 계측에서 두 값을 혼동하지 않는다. 이번 대표는 처리 비용의 정밀 비교가 아니다.

현재 실행 영수증·run1·completion 및 최종 PID 감사가 보존돼 있다. 원 runner/영상 child 모두 absent이며 WSL을 반환했다. 재개 후 수정한다면 v1을 보존하고 새 v2 archive/명령을 만들어 root에 전달해야 한다. 아직 v2 수정·재시험은 없다.
