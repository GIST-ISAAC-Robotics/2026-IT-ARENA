# 신호등 출발·본선 반복 주행 데모

이 데모는 **한 대의 차량이 빨간 신호에서 기다렸다가 초록불을 보고 출발하고, 지름길을 타지 않은 채 본선을 계속 도는** 최소 구현입니다. 완주 횟수 제한은 없습니다. 대회용 위치 추정·추월·다중 차량 회피까지 구현한 것은 아닙니다.

## 실행

WSL Ubuntu 24.04 터미널에서 실행합니다. 기존 시뮬레이터가 같은 통신 영역에서 실행 중이라면 먼저 그 실행을 종료하십시오.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=164
export GZ_PARTITION=it_arena_basic_demo
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 launch arena_bringup demo.launch.py
```

차량의 첫 RGB 영상이 준비되면 빨강 8초 → 노랑 2초 → 초록 유지 순서로 바뀝니다. 시간은 **시뮬레이션 시간**입니다. 컴퓨터 부하에 따라 실제 대기 시간은 더 길어집니다. 차량은 이 시간표를 읽지 않고 RGB 영상에서 빨강을 먼저 확인한 뒤 초록을 연속 세 번 확인해야 출발합니다.

- `headless:=true`: GUI 없이 실행합니다.
- `red_duration_s:=20`: 첫 빨간 신호 대기를 늘립니다.
- `grid_slot:=0`: 출발 위치를 선택합니다. 기본값 0의 연속 주행부터 검증하며, 모든 출발 위치의 반복 완주를 보장하지 않습니다.
- `depth_camera:=true`: 사용하지 않는 깊이 영상·점군도 함께 켭니다.

데모는 처리 부담을 줄이기 위해 **RGB 848×480·30 Hz, 라이다 10 Hz**를 사용하고 깊이 센서를 끕니다. IMU·엔코더 인터페이스는 남아 있지만 이 자율주행의 입력으로 사용하지 않습니다. `simulation.launch.py`의 일반 기본값은 기존 RGB 60·깊이 90 Hz를 유지합니다.

## 가장 간단한 정지·재개

Gazebo 왼쪽 아래의 일시정지 버튼은 시뮬레이션 전체를 멈춥니다. 실행한 터미널의 `Ctrl+C`는 해당 실행을 종료합니다.

차량만 멈추려면 두 번째 WSL 터미널에서도 같은 작업 폴더·ROS 환경을 준비합니다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=164
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: false}'
```

같은 명령에서 `false`를 `true`로 바꾸면 재개합니다. 이미 초록불로 출발한 주행에서는 다시 신호등을 찾지 않습니다. 수동 `/drive` 명령을 보내기 전에는 반드시 자율주행을 꺼야 합니다. 두 제어기가 같은 토픽에 동시에 명령을 보내면 충돌합니다.

## 신호등을 직접 바꾸기

위와 같이 준비한 두 번째 터미널에서 다음 서비스를 사용합니다.

```bash
ros2 service call /sim/traffic_light/set_red std_srvs/srv/Trigger '{}'
ros2 service call /sim/traffic_light/set_yellow std_srvs/srv/Trigger '{}'
ros2 service call /sim/traffic_light/set_green std_srvs/srv/Trigger '{}'
ros2 service call /sim/traffic_light/reset std_srvs/srv/Trigger '{}'
```

`set_*`는 선택한 색을 유지하고, `reset`은 자동 출발 순서를 다시 시작합니다. 서비스 응답은 변경 요청 접수이며 `/sim/traffic_light/state`의 `applied`가 실제 Gazebo 적용 완료 상태입니다. 렌즈의 재질·발광색을 바꾸므로 차량의 RGB 영상에서도 색이 변합니다.

출발 후 신호를 빨강으로 바꾸더라도 차량은 계속 주행합니다. 이 구현에서 신호등은 **출발 허가 장치**입니다. 비상정지 신호로 사용하지 마십시오. 정지는 `/autonomy/enable` 또는 시뮬레이션 일시정지를 사용합니다.

다시 출발 위치에서 시험하려면 실행을 종료하고 새로 시작하는 방법이 가장 확실합니다. `/autonomy/reset`은 인식 상태만 초기화하며 차량을 출발 위치로 옮기지 않습니다. 출발 신호가 보이지 않는 곳에서 초기화하면 새 빨강→초록 신호를 볼 때까지 움직이지 않습니다.

## 사용한 알고리즘

1. RGB 영상에서 빨간 렌즈를 찾고, 같은 신호등의 초록 렌즈가 연속으로 켜진 것을 확인합니다.
2. 2D 라이다에서 한쪽 벽의 점들을 골라 짧은 2차 곡선으로 맞춥니다. 벽과의 거리·기울기·곡률로 조향하고 커브에서는 감속합니다.
3. RGB 영상에서 ArUco를 읽어 지름길 반대편 벽을 선택합니다.

| 인식한 마커 | 이후 따라갈 벽 | 의미 |
| --- | --- | --- |
| ID 0, 20, 45 | 왼쪽 | 첫 분기는 오른쪽 지름길을 피합니다. 출발·복귀 기본값도 왼쪽입니다. |
| ID 30 | 오른쪽 | 두 번째 분기의 왼쪽 지름길을 피합니다. |

현재 실험 지도의 **본선 노면 0.45 m + 양쪽 녹색 영역 각각 0.20 m** 때문에 목표 벽 간격은 중심선에서 약 0.425 m입니다. 다른 지도에서는 그대로 사용할 수 없습니다. `track:=original`에서 이 데모를 켜는 것은 실행 단계에서 막습니다.

조향 입력은 `/scan`과 `/camera/color/image_raw`뿐입니다. `/odom`, 정답 위치·자세, 중심선 CSV, 지도 파일, 신호등 상태 토픽을 자율주행 노드가 읽지 않습니다. 검사 도구는 정답 위치를 별도로 관측해 완주·이탈 여부를 판정하지만 차량 제어에는 전달하지 않습니다.

직선 목표 최고 속도는 **0.35 m/s**, 일반적인 커브 최저 명령은 0.14 m/s입니다. 가까운 장애물·벽 소실·센서 데이터 누락·사용자 정지 시에는 0을 명령합니다. 가속·감속과 실제 속도가 순간적으로 같지는 않습니다. 튜닝 값은 `src/arena_bringup/config/wall_follow.yaml`에 있습니다.

## 라이다와 안전 한계

RPLIDAR C1의 명목 사양을 참고한 360도·10 Hz·500표본·0.05–12 m 센서입니다. 스캔 평면은 차량 기준 지면에서 약 12.5 cm이며 자기 차체를 피한 **임시** 위치입니다. 실물 구매·장착 확정이 아닙니다. [사양과 단순화 범위](../sensors/RPLIDAR_C1_SIMULATION.md)를 참고하십시오.

- 스캔 평면 아래의 낮은 차량·장애물은 놓칠 수 있습니다. 이 라이다 하나로 사방의 모든 차를 안전하게 피한다고 주장할 수 없습니다.
- 마커 ID와 분기 방향은 이 실험 코스에 맞춘 규칙입니다. 표지판 가림·오인식·새 지도에서의 올바른 분기 선택은 보장하지 않습니다.
- 신호등의 렌즈 간격과 색 임계값도 현재 실험 시설에 맞췄습니다. 실물 조명·노출·신호등 배치에는 재조정이 필요합니다.
- 라이다의 반사율·잡음·회전 중 운동 왜곡, 실물 구동계의 지연·슬립 등은 아직 충실히 재현하지 않습니다.
- 다중 차량 회피·추월·막힌 길에서의 복구는 없습니다. 장애물 정지는 기초 보호 기능이지 검증된 충돌 방지 시스템이 아닙니다.

## 검증

자동 검사는 별도의 통신 영역에서 새 Gazebo를 띄워 RGB 신호 출발, 두 분기 통과, 지정한 바퀴 수 이상 진행, 차량 외형과 벽의 겹침, 지속적인 정지, 정상 종료를 확인합니다.

```bash
python3 scripts/validate_basic_autonomy.py --laps 2
python3 -m pytest -q tests src/arena_vehicle_interface/test
```

기본 출력은 `artifacts/tests/basic_autonomy/`의 `report.json`, `trace.json`, 차량 RGB 사진과 마지막 스캔입니다. `--output`으로 실행별 폴더를 구분할 수 있습니다. 진행 중에는 `progress.json`과 한 줄씩 저장되는 `trajectory.jsonl`에 중간 기록을 남깁니다. 세션이 끊겨도 이미 저장된 기록을 확인할 수 있지만, `completed: false` 또는 최종 보고서가 없는 실행을 통과로 간주하지 않습니다. 최종 검증 수치와 확인된 한계는 당일 활동 기록에 남깁니다.

### 2026-08-31 최종 통합 결과

[최종 통합 보고서](../../artifacts/screenshots/2026-08-31/basic_demo/final_integrated_report.json)는 95.2856 m·2바퀴+약 2 m, 기록 표본상 초록 전 움직임 없음, 고정 20×15 cm 외형과 정적 벽의 겹침 0회, 마커 4개와 좌·우 벽 선택, 지속 정지와 **정상 종료까지 한 실행에서 통과**했습니다. 정지 요청 뒤 약 0.592 s에 지속 정지했고 부모 프로세스 종료 코드 0/0·오류 0·`shutdown_clean: true`였습니다.

검증기의 조기 출발 판정은 토픽 수신 순서와 무관하게 허가/속도 명령을 확인하도록 보강했습니다. 최종 실행 도중 소스만 수정했으며 이미 실행 중이던 검사기에는 소급 적용되지 않았습니다. 생산 제어 입력은 바꾸지 않았고 이번 실행의 출발은 초록 전 107개 원시 표본의 별도 감사로 보강했습니다. 새 판정의 회귀 검사를 포함한 자동 검사 62개와 패키지 5개 빌드, 두 월드의 엄격한 SDF 검사를 통과했습니다.

앞선 [연속 주행 보고서](../../artifacts/screenshots/2026-08-31/basic_demo/continuous_laps_report.json)의 종료 실패와 [단기 종료 검사](../../artifacts/screenshots/2026-08-31/basic_demo/shutdown_check_report.json) 3회는 원인 분리의 과거 근거로 그대로 보존합니다. 과거 보고서를 성공으로 수정하지 않았습니다.

[출발 대기 화면](../../artifacts/screenshots/2026-08-31/basic_demo/start_grid_red.jpg) · [차량과 라이다](../../artifacts/screenshots/2026-08-31/basic_demo/car_lidar_oblique.jpg) · [실제 RGB 초록 신호](../../artifacts/screenshots/2026-08-31/basic_demo/signal_green.png)

[사선 조감도 한 바퀴 영상](../../artifacts/videos/2026-08-31/basic_autonomy_overhead_one_lap.mp4)은 같은 최종 실행을 기록했습니다. 녹화 중 두 체크포인트 사이에만 약 47.57 m를 진행해 한 바퀴 46.6329 m 이상을 포함합니다. Gazebo 3D 사용자 카메라만 녹화했으며 2548×1448·약 25 FPS·165.88 s입니다. 재생 길이는 시뮬레이션 시간 기준으로, 실제 PC 녹화 시간 약 13분 20초와 다릅니다. 저장 뒤 모든 검사·GUI 프로세스를 종료했습니다.
