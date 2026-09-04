# 신호등 출발·본선 반복 주행 데모

이 데모는 **한 대의 차량이 빨간 신호에서 기다렸다가 초록불을 보고 출발하고, 지름길을 타지 않은 채 본선을 계속 도는** 최소 구현입니다. 완주 횟수 제한은 없습니다. 대회용 위치 추정·추월·다중 차량 회피까지 구현한 것은 아닙니다.

2026-09-04 현재 `autonomy_mode:=lidar|stereo` 선택을 구현했습니다. `lidar`는
C1급 `/scan`+ToF 여섯 개, `stereo`는 전방 D435i급 RGB-D+좌우 ToF 네 개를
사용합니다. 이 배치는 사용자의 개인 검토안이며 팀원과 아직 협의하지 않았습니다.
두 모드 모두 본선 한 바퀴+약 2 m와 실제 정지, 3인칭 촬영을 확인했습니다.
스테레오는 이전 헛돎 이후 RGB-D 노면 목표점 추종으로 변경했습니다. 다만
완주 실행의 깊이 전달률 약 24 Hz·Gazebo 종료 실패가 남아 종합 검사는 실패입니다.
[두 모드 검증 결과와 영상](../../artifacts/validation/2026-09-04/selectable_autonomy/README.md),
[결정 0015](../decisions/0015-selectable-lidar-stereo-autonomy-prototypes.md)를 함께 확인하십시오.
주행·영상·종료 성공은 서로 구분합니다.

2026-08-31 공식 자료 반영 후 `demo.launch.py`는 **공식 자료 기반 `official` 트랙(본선 45 cm·지름길 20 cm)**을 선택합니다. 마커 벽 부착 보정과 임시 신호등·방지턱이 포함된 파생 실행 월드이며, 공개 SDF를 아무 수정 없이 실행한 것은 아닙니다. [공식 마커·시설 근거와 보정](../track/OFFICIAL_MARKERS_AND_FACILITIES.md)을 함께 확인하십시오. 새 `official`에서 짧은 센서·주행 검사와 `brisk` 본선 한 바퀴·정지·정상 종료를 별도로 통과했으며 아래에 고정 보고서를 연결했습니다. 지름길 25 cm인 `experimental`의 과거 기록은 보존하되 새 공식 검증으로 승계하지 않습니다.

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

선택형 실행은 다음과 같습니다. `depth_camera:=auto`는 LiDAR 모드에서는 깊이를
끄고 스테레오 모드에서는 켭니다.

```bash
ros2 launch arena_bringup demo.launch.py autonomy_mode:=lidar
ros2 launch arena_bringup demo.launch.py autonomy_mode:=stereo
```

차량의 첫 RGB 영상이 준비되면 빨강 8초 → 노랑 2초 → 초록 유지 순서로 바뀝니다. 시간은 **시뮬레이션 시간**입니다. 컴퓨터 부하에 따라 실제 대기 시간은 더 길어집니다. 이는 시험용 순서이며, 실제 대회는 랜덤한 시각 신호를 인식하여 출발하고 무선 출발 신호는 없습니다. 차량은 이 시간표를 읽지 않고 RGB 영상에서 빨강을 먼저 확인한 뒤 초록을 연속 세 번 확인해야 출발합니다.

- `headless:=true`: GUI 없이 실행합니다.
- `red_duration_s:=20`: 첫 빨간 신호 대기를 늘립니다.
- `grid_slot:=0`: 출발 위치를 선택합니다. 기본값 0의 연속 주행부터 검증하며, 모든 출발 위치의 반복 완주를 보장하지 않습니다.
- `depth_camera:=true`: 사용하지 않는 깊이 영상·점군도 함께 켭니다.
- `tof_profile:=low_latency_4x4_60`: 기본 8×8·15 Hz 하부 ToF 점군을 4×4·60 Hz 근거리·지연 비교 프로필로 바꿉니다.
- `speed_profile:=brisk`: 직선 요청을 0.70 m/s 수준으로 올립니다. 이전 `experimental`과 새 `official`에서 각각 별도 한 바퀴 검증을 마쳤습니다. 모든 출발 위치·가림·차량 조건에서의 완주 보장은 아닙니다. 기본값은 `cautious`입니다.

데모의 벽 추종 본체는 처리 부담을 줄이기 위해 **RGB 848×480·30 Hz, 라이다 10 Hz**를 사용하고 깊이 센서를 끕니다. 벽 추종은 ToF·IMU·엔코더를 조향 입력으로 쓰지 않습니다. 대신 독립된 ToF 보호층이 하부 여섯 점군과 좌우 엔코더를 읽어 벽 추종 또는 수동 `/drive` 요청을 감속·정지시킨 뒤 `/drive/safe`로 전달합니다. `simulation.launch.py`의 일반 기본값은 기존 RGB 60·깊이 90 Hz를 유지합니다.

이 데모 실행 파일은 `official`을 지정합니다. 이전 지도는 일반 실행의 `ros2 launch arena_bringup simulation.launch.py track:=experimental` 또는 `track:=original`로 계속 선택할 수 있으며, 아래 짧은 검사 도구도 세 지도를 모두 지원합니다. `original`에서 신호등·자율주행 데모를 켜는 것은 지원하지 않습니다.

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

### C1 모드

1. RGB 영상에서 빨간 렌즈를 찾고, 같은 신호등의 초록 렌즈가 연속으로 켜진 것을 확인합니다.
2. 2D 라이다에서 한쪽 벽의 점들을 골라 짧은 2차 곡선으로 맞춥니다. 벽과의 거리·기울기·곡률로 조향하고 커브에서는 감속합니다.
3. RGB 영상에서 ArUco를 읽어 지름길 반대편 벽을 선택합니다.

| 인식한 마커 | 이후 따라갈 벽 | 의미 |
| --- | --- | --- |
| ID 0, 20, 45 | 왼쪽 | 첫 분기는 오른쪽 지름길을 피합니다. 출발·복귀 기본값도 왼쪽입니다. |
| ID 30 | 오른쪽 | 두 번째 분기의 왼쪽 지름길을 피합니다. |

현재 `official`과 이전 `experimental`은 모두 **본선 노면 0.45 m + 양쪽 녹색 영역 각각 0.20 m**를 사용하여, 일반 구간의 목표 벽 간격은 중심선에서 약 0.425 m입니다. 분기 벽·마커 배치까지 같다는 의미는 아니므로 기존 튜닝과 완주 결과를 새 지도에 그대로 보장하지 않습니다. 다른 폭의 지도에는 이 목표 거리를 그대로 적용할 수 없으며, 일반 실행에서도 `track:=original`의 신호등·자율주행 데모 활성화는 막습니다.

조향 입력은 `/scan`과 `/camera/color/image_raw`뿐입니다. `/odom`, 정답 위치·자세, 중심선 CSV, 지도 파일, 신호등 상태 토픽을 자율주행 노드가 읽지 않습니다. 검사 도구는 정답 위치를 별도로 관측해 완주·이탈 여부를 판정하지만 차량 제어에는 전달하지 않습니다.

### D435i 모드

RGB 영상의 회색 노면 후보를 깊이로 평지인지 확인하고, 전방 약 0.34 m의
노면 중심 목표점을 기본 Pure Pursuit로 추종합니다. 한쪽 경계만 보이면 본선
명목 폭 0.45 m를 이용합니다. 가까운 목표점이 없을 때만 깊이의 열린 통로·
선택한 벽 거리 기반 조향을 최저속도로 사용합니다. 신호와 마커의 규칙은 같습니다.

`/camera/color/image_raw`, `/camera/depth/image_rect_raw`, 깊이 `CameraInfo`가
입력이며 `/scan`·지도·정답 위치는 읽지 않습니다. 데모 RGB/깊이는 각각 명목
30 Hz, 제어 주기는 20 Hz입니다. ToF는 좌우 전·후측 네 개만 남기므로 정후방
관측과 전방 근거리 ToF를 포기합니다. 이를 전방 깊이만으로 완전히 보완했다고
가정하면 안 됩니다.

이 방식은 회색 도로·녹색 잔디, 평지와 현재 시뮬레이션 카메라 정렬을 이용한
단순 기준선입니다. 잔디와 도로는 깊이만으로 구분하지 못합니다. 실물 RGB-깊이
정렬, 조명·노출, 흰색 표시·방지턱, 다른 차량의 가림은 후속 과제입니다.

### 공통 속도와 정지

기본 `cautious`의 직선 목표 최고 속도는 **0.35 m/s**, 일반적인 커브 최저 명령은 0.14 m/s입니다. `brisk`·`exploratory`·`hardware_target` 프로필도 있지만 ToF 가시거리·벽 곡률·횡가속 가정이 추가로 제한합니다. 가까운 장애물·벽 소실·센서 데이터 누락·사용자 정지 시에는 0을 명령합니다. 가속·감속과 실제 속도가 순간적으로 같지는 않습니다. 튜닝 값은 `src/arena_bringup/config/wall_follow.yaml`과 `src/arena_bringup/config/tof_safety.yaml`에 있습니다.

## 라이다와 안전 한계

RPLIDAR C1의 명목 사양을 참고한 360도·10 Hz·500표본·0.05–12 m 센서입니다. 스캔 평면은 차량 기준 지면에서 약 12.5 cm이며 자기 차체를 피한 **임시** 위치입니다. 낮은 차량 보완용 ToF 링은 벽 추종 조향과 분리된 감속·정지층에 연결했습니다. [C1 단순화 범위](../sensors/RPLIDAR_C1_SIMULATION.md), [ToF 링 안내](../sensors/TOF_RING.md), [ToF 안전층](TOF_SAFETY.md)을 참고하십시오.

- 스캔 평면 아래의 낮은 차량·장애물은 라이다가 놓칠 수 있습니다. 독립 ToF 층은 정적 저상 장애물 감속·정지만 하며, 이동 차량 추적·회피·추월 구현과 구분합니다.
- 마커 ID별 벽 선택은 현재 코스의 분기 방향에 맞춘 팀 구현 규칙입니다. 공식 가이드의 의무 알고리즘이 아니며, 표지판 가림·오인식·변경된 마커 배치에서의 올바른 분기 선택은 보장하지 않습니다.
- 마커 벽 부착 보정은 차량의 정적 경로 표본 통과를 확보한 것입니다. ID 30 지지판과 노면 폴리곤의 평면 겹침은 약 0.000109 m² 남으며, 본선 한 바퀴 성공도 지름길의 연속 진입·합류 검증을 대신하지 않습니다.
- 신호등의 렌즈 간격과 색 임계값도 현재 실험 시설에 맞췄습니다. 실물 조명·노출·신호등 배치에는 재조정이 필요합니다.
- 라이다의 반사율·잡음·회전 중 운동 왜곡과 실물 구동계의 전기·열·상세 타이어 특성은 아직 충실히 재현하지 않습니다. 단일 모터 토크·이상/손실/LSD 차동 근사·접지 슬립은 구현했지만 실측 보정 전입니다.
- 다중 차량 회피·추월·막힌 길에서의 복구는 없습니다. 장애물 정지는 기초 보호 기능이지 검증된 충돌 방지 시스템이 아닙니다.

## 검증

연속 주행 검사는 별도의 통신 영역에서 새 Gazebo를 띄워 RGB 신호 출발, 두 분기 통과, 지정한 바퀴 수 이상 진행, 차량 외형과 벽의 겹침, 지속적인 정지, 정상 종료를 확인합니다. `validate_basic_autonomy.py`는 데모와 같은 `it_arena_official`의 중심선·씬·벽 충돌 형상을 읽고 보고서에 `track: official`을 남깁니다. 실행 월드는 `official`인데 평가용 벽은 `experimental`인 혼합 검사를 하지 않습니다.

```bash
python3 scripts/validate_basic_autonomy.py --laps 2
python3 scripts/validate_basic_autonomy.py --laps 1 --speed-profile brisk
python3 -m pytest -q tests src/arena_vehicle_interface/test
```

기본 출력은 `artifacts/tests/basic_autonomy_official/`의 `report.json`, `trace.json`, 차량 RGB 사진과 마지막 스캔입니다. 이전 `artifacts/tests/basic_autonomy/` 기록과 분리하며, 이후 반복 실행도 `--output`으로 새 폴더를 지정하여 보존할 기록을 덮어쓰지 마십시오. 진행 중에는 `progress.json`과 한 줄씩 저장되는 `trajectory.jsonl`에 중간 기록을 남깁니다. 세션이 끊겨도 이미 저장된 기록을 확인할 수 있지만, `completed: false` 또는 최종 보고서가 없는 실행을 통과로 간주하지 않습니다. 최종 검증 수치와 확인된 한계는 당일 활동 기록에 남깁니다.

직진·조향·센서·명령 중단 정지만 짧게 확인하려면 다음을 사용합니다. `smoke_simulation.py`의 기본값도 `official`이며, `original`과 `experimental` 선택을 유지합니다. 이 검사는 완주나 마커별 가시성 검사가 아닙니다.

```bash
python3 scripts/smoke_simulation.py --d435i-profile low_load_30
python3 scripts/smoke_simulation.py --track experimental --d435i-profile low_load_30
python3 scripts/smoke_simulation.py --track original --d435i-profile low_load_30
```

짧은 검사 보고서는 `artifacts/tests/`의 트랙·프로필별 이름으로 저장합니다. 같은 트랙·프로필을 반복 실행하면 같은 이름을 사용하므로, 보존이 필요한 결과는 재실행 전에 별도 기록으로 남깁니다.

### 2026-08-31 공식 트랙의 실제 실행 결과

현재 `official`에서 수행한 [짧은 실행 검사](../../artifacts/validation/2026-08-31/official_track/smoke_low_load_30.json)는 RGB·깊이 848×480 약 30.30 Hz, 여섯 ToF의 8×8 점군 약 15.15 Hz, 엔코더·IMU·직진·조향·명령 중단 정지와 정상 종료를 통과했습니다. 이 주기는 시뮬레이션 시간 기준이며, 해당 RGB·깊이의 실제 시간 처리율은 약 8.41 Hz였습니다.

별도의 [`brisk` 한 바퀴 보고서](../../artifacts/validation/2026-08-31/official_track/brisk_one_lap.json)는 단일 모터·ToF 보호층으로 **48.6927 m, 본선 1바퀴+약 2 m**를 통과했습니다.

- 최고 지면 속도 0.794307 m/s, 최대 중심선 거리 0.124205 m, 고정 20×15 cm 외형과 정적 벽의 겹침 표본 0개였습니다.
- 조기 출발은 없었고(`false_start: false`), 마커 0/20/30/45와 좌·우 벽 전환을 확인했습니다.
- 벽 추종은 RGB·`/scan`·`/clock`, 보호층은 명령·여섯 ToF·`/wheel_states`·시계만 구독했습니다. 지도·정답값을 차량 제어 입력으로 쓰지 않았습니다.
- 실제 지속 정지·정상 종료를 통과했습니다. 정지 대기 약 0.897초(시뮬레이션 시간), 부모 프로세스 종료 코드 0/0·오류 없음·`shutdown_clean: true`였습니다.
- 공식 반영 최종 소스에서 Python 검사 105개·C++ 차동 CTest 1개·ROS 패키지 5개 빌드, 공식/실험 해시 검사와 공식 실행 월드의 엄격한 SDF 검사도 통과했습니다.

실제 화면은 [전체 조감도](../../artifacts/screenshots/2026-08-31/official_update/official_track_overview.png), [출발 구역](../../artifacts/screenshots/2026-08-31/official_update/official_start_area.png), [보정 후 ID 30](../../artifacts/screenshots/2026-08-31/official_update/official_marker_id30_after.png)에 보존했습니다. 이 결과는 단일 차량 본선의 해당 조건 검증이며 노면 경계 준수·모든 출발 슬롯·지름길 진입/합류·다중 차량·추월·고속 완주·실차 안전성을 증명하지 않습니다.

### 2026-08-31 이전 실험 트랙의 최종 통합 결과

이 절은 `experimental` 트랙·속도 강제 구동 모델의 당시 결과입니다. `official`로 기본값을 바꾼 뒤 얻은 결과가 아닙니다.

[최종 통합 보고서](../../artifacts/screenshots/2026-08-31/basic_demo/final_integrated_report.json)는 95.2856 m·2바퀴+약 2 m, 기록 표본상 초록 전 움직임 없음, 고정 20×15 cm 외형과 정적 벽의 겹침 0회, 마커 4개와 좌·우 벽 선택, 지속 정지와 **정상 종료까지 한 실행에서 통과**했습니다. 정지 요청 뒤 약 0.592 s에 지속 정지했고 부모 프로세스 종료 코드 0/0·오류 0·`shutdown_clean: true`였습니다.

검증기의 조기 출발 판정은 토픽 수신 순서와 무관하게 허가/속도 명령을 확인하도록 보강했습니다. 최종 실행 도중 소스만 수정했으며 이미 실행 중이던 검사기에는 소급 적용되지 않았습니다. 생산 제어 입력은 바꾸지 않았고 이번 실행의 출발은 초록 전 107개 원시 표본의 별도 감사로 보강했습니다. 새 판정의 회귀 검사를 포함한 자동 검사 62개와 패키지 5개 빌드, 두 월드의 엄격한 SDF 검사를 통과했습니다.

앞선 [연속 주행 보고서](../../artifacts/screenshots/2026-08-31/basic_demo/continuous_laps_report.json)의 종료 실패와 [단기 종료 검사](../../artifacts/screenshots/2026-08-31/basic_demo/shutdown_check_report.json) 3회는 원인 분리의 과거 근거로 그대로 보존합니다. 과거 보고서를 성공으로 수정하지 않았습니다.

[출발 대기 화면](../../artifacts/screenshots/2026-08-31/basic_demo/start_grid_red.jpg) · [차량과 라이다](../../artifacts/screenshots/2026-08-31/basic_demo/car_lidar_oblique.jpg) · [실제 RGB 초록 신호](../../artifacts/screenshots/2026-08-31/basic_demo/signal_green.png)

[사선 조감도 한 바퀴 영상](../../artifacts/videos/2026-08-31/basic_autonomy_overhead_one_lap.mp4)은 같은 최종 실행을 기록했습니다. 녹화 중 두 체크포인트 사이에만 약 47.57 m를 진행해 한 바퀴 46.6329 m 이상을 포함합니다. Gazebo 3D 사용자 카메라만 녹화했으며 2548×1448·약 25 FPS·165.88 s입니다. 재생 길이는 시뮬레이션 시간 기준으로, 실제 PC 녹화 시간 약 13분 20초와 다릅니다. 저장 뒤 모든 검사·GUI 프로세스를 종료했습니다.

### 이전 실험 트랙의 단일 모터·ToF 브리스크 결과

후속 토크 구동 기준선에서 `--laps 1 --speed-profile brisk`는 48.6527 m를 진행해 한 바퀴를 통과했습니다. 최고 실속도 0.8005 m/s, 최대 중심선 거리 0.1301 m, 고정 20×15 cm 외형과 정적 벽 겹침 0회, 센서 입력만 사용한 벽 추종/ToF 보호, 실제 지면 정지와 정상 종료를 확인했습니다. [고정 보고서](../../artifacts/validation/2026-08-31/vehicle_dynamics/brisk_one_lap.json)를 참고하십시오.

이 실행은 `experimental`에서 수행한 토크 구동 검증이며, 이전 속도 강제 모델의 두 바퀴 보고서·영상과도 별개입니다. 새 `official`의 본선 재완주 결과는 위 공식 트랙 절과 별도 보고서에 기록했으며 이 과거 결과를 승계한 것이 아닙니다. ToF 제한 때문에 `brisk`보다 높은 프로필이 그대로 20 km/h를 내지는 않으며, 노면 경계 준수·상대차·추월·고속 완주는 아직 검증하지 않았습니다.
