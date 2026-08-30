# D435i 사양과 시뮬레이션 설정

확인·적용 날짜: 2026-08-31

현재 모델은 **공식 명목 사양을 반영한 이상적인 RGB·깊이 카메라**입니다. 실제 보유한 D435i의 보정값이나 측정 오차를 재현한 디지털 복제본은 아닙니다. 설정의 출처와 실제 구현 범위를 아래처럼 구분합니다.

## 기본값과 실행 선택

설정 원본은 `src/arena_description/config/vehicle.yaml`의 `sensors.d435i`입니다. RGB와 깊이를 서로 다른 Gazebo 센서로 분리해 해상도·주기·시야각·렌더링 거리를 각각 적용합니다.

| 프로필 | RGB | 깊이 | 용도 |
|---|---|---|---|
| `high_speed_async` — 기본 | 848×480, 60 FPS | 848×480, 90 FPS | 독립적인 고속 관측. 두 영상의 촬영 시각은 항상 같지 않음 |
| `synchronized_60` | 848×480, 60 FPS | 848×480, 60 FPS | 같은 주기의 RGB·깊이 결합 실험 |
| `low_load_30` | 848×480, 30 FPS | 848×480, 30 FPS | 처리 부하를 낮춘 개발·화면 확인 |

공식 USB 3.1 스트림 표에서 RGB 848×480의 최대 주기는 60 FPS, 깊이 Z16 848×480은 90 FPS입니다. RGB 1920×1080은 30 FPS까지이므로 해상도와 최대 FPS를 따로 조합하면 안 됩니다. 다른 주기의 스트림을 동시에 사용할 수 있지만, RGB·깊이 하드웨어 동기화는 동일 주기가 조건입니다. `hardware_sync_compatible`은 이 조건의 충족 여부이며 실제 장치에서 동기화를 활성화했다는 뜻은 아닙니다. [D400 데이터시트 Revision 021, 표 4-2·4-5](https://dev.realsenseai.com/download/42003/)

WSL에서 빌드한 환경을 불러온 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch arena_bringup simulation.launch.py
```

`d435i_profile`을 생략하면 `vehicle.yaml`의 `active_stream_profile`을 따릅니다. 일시적으로 바꾸려면 아래 중 하나를 사용합니다. 지도 선택과 독립적인 설정입니다.

```bash
ros2 launch arena_bringup simulation.launch.py d435i_profile:=synchronized_60
ros2 launch arena_bringup simulation.launch.py d435i_profile:=low_load_30
ros2 launch arena_bringup simulation.launch.py track:=original d435i_profile:=high_speed_async
```

## 시야각과 깊이 거리

| 항목 | 적용값 | 해석 |
|---|---|---|
| RGB 시야각 | 수평 69.4°·수직 42.5° | 상세 명목값. 제품 요약에서는 약 69°×42°로 표기 |
| 깊이 시야각 | 수평 87°·수직 58° | HD 기준 명목값을 현재 모델에 적용 |
| 깊이 최소 거리 | 0.195 m | 848×480 모드의 Min-Z |
| 깊이 기본 최대 거리 | 3.0 m | 시뮬레이션의 임의 절단 경계. 실제 센서의 절대 최대 거리 아님 |
| 확장 실험 후보 | 4.0 m | 별도 실험값이며 기본 실행에는 적용되지 않음 |
| RGB 렌더링 거리 | 0.02~20.0 m | 깊이 측정 범위와 무관한 가상 카메라의 표시 경계 |

RGB 상세 시야각은 [Intel 공식 D400 데이터시트의 D435 광학 사양](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D40-Series-Datasheet.pdf), 제품 요약값은 [D435i 공식 제품 페이지](https://www.realsenseai.com/products/depth-camera-d435i/)를 참고했습니다. 깊이 HD 시야각과 해상도별 Min-Z는 [Revision 021의 광학 사양·표 4-11](https://dev.realsenseai.com/download/42003/) 기준입니다. 해상도·화면 비율·개체별 렌즈 편차에 따라 실제 값이 달라지므로, 보유 장치의 스트림 프로필과 보정값을 읽어 교체해야 합니다.

0.25 m나 4 m를 모든 모드에 공통인 정확한 하한·상한으로 취급하지 않습니다. 예를 들어 1280×720의 Min-Z는 0.280 m입니다. 공식 자료의 범위는 환경에 따라 3 m를 넘을 수 있다는 의미이고, 깊이 품질 표의 오차 조건은 별도로 2 m 이하·HD·중앙 80% 영역 등 시험 조건을 갖습니다. 4 m까지 항상 유효하고 정밀한 깊이가 나온다는 보장은 아닙니다. [Revision 021, 표 4-11·4-14](https://dev.realsenseai.com/download/42003/)

4 m를 가정한 실험을 하려면 `depth.simulation_far_clip_m`을 명시적으로 변경합니다. `extended_test_far_clip_m`은 후보값을 기록한 메타데이터일 뿐 자동 적용되는 설정이 아닙니다. 마찬가지로 품질 기준 거리·셔터 방식·보정 상태 등의 메타데이터는 잡음 모델을 자동 생성하지 않습니다.

수평·수직 시야각으로부터 별도의 `fx`, `fy`를 계산해 Gazebo의 렌즈 내부 파라미터와 ROS `CameraInfo`에 함께 적용합니다. 주점은 영상 중앙으로 가정합니다. 화면 비율만으로 수직 시야각을 결정하지 않으며, 이 값들은 실제 카메라에서 추출한 보정값이 아닙니다.

## ROS 인터페이스와 주의점

| 토픽 | 형식 | 의미 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image`, `rgb8` | RGB 영상 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB 내부 파라미터 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image`, `32FC1` | 광학 축 방향 깊이, m 단위 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 깊이 전용 내부 파라미터 |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | 깊이로 생성한 점군. 이름은 기존 인터페이스 호환을 위해 유지 |
| `/camera/imu` | `sensor_msgs/msg/Imu` | 200 Hz로 설정한 이상적 결합 IMU |

- RGB·깊이 광학 좌표계는 각각 `camera_color_optical_frame`, `camera_depth_optical_frame`입니다. 같은 픽셀 좌표가 같은 방향을 뜻하지 않으므로 RGB 영상의 검출 좌표로 깊이 영상을 그대로 인덱싱하면 안 됩니다. 영상 정렬·투영과 시간 대응이 필요합니다.
- 두 카메라의 위치는 현재 동일하게 근사합니다. 실제 RGB·깊이 렌즈 사이의 위치·방향 차이, 렌즈 왜곡과 개체별 내부 파라미터는 미반영입니다.
- 실제 RealSense의 Z16/ROS `16UC1` 경로와 시뮬레이터의 `32FC1`은 표현·단위가 다를 수 있습니다. 어댑터에서 인코딩과 깊이 스케일을 확인해 m 단위로 통일해야 합니다. 유효 범위 밖의 비정상·무한 값을 장애물 거리로 그대로 사용하지 않습니다.
- 현재 점군의 `rgb` 필드는 실제 RGB 영상으로 색칠한 값이 아니라 Gazebo 깊이 시각화의 회색조입니다. `/color/points`라는 이름만 보고 RGB 정렬이 완료됐다고 판단하면 안 됩니다. [Gazebo DepthCameraSensor 구현](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/DepthCameraSensor.cc)
- 점군은 별도 브리지에서 ROS 구독자가 있을 때만 전송합니다. 점군의 지속 구독은 영상 처리량에 영향을 줄 수 있습니다.
- 200 Hz 결합 IMU는 실장된 BMI055/BMI085별 가속도·자이로 주기를 그대로 재현하지 않습니다. 실제 장치 리비전 확인이 필요하며, Gazebo가 제공하는 이상적 자세를 실제 D435i가 직접 측정한 절대 자세로 간주하면 안 됩니다.

RGB의 롤링 셔터, 깊이 카메라의 글로벌 셔터라는 사양은 기록했지만, 현재 모델은 움직임에 따른 롤링 셔터 왜곡·노출 시간·모션 블러·스테레오 결측·재질/조명 의존 오차를 재현하지 않습니다. 고속 주행에서 실물보다 유리한 이상적 영상입니다.

## 프레임률·부하 검증의 해석

`update_rate`는 시뮬레이션 시간 기준입니다. 실제 1초 동안 받는 영상 수와 반드시 같지는 않습니다. 현재 PC에서 60/90 설정으로 시뮬레이션이 실시간보다 느리게 진행되는 것을 확인했습니다. 알고리즘은 `/clock`과 메시지 시각을 사용해야 하며, 느린 재생으로 인한 여유를 Jetson의 실시간 처리 성능으로 해석하면 안 됩니다.

Gazebo Sensors 8의 현재 구현은 갱신 주기를 정수 ms로 변환합니다. 따라서 명목 60 Hz가 약 62.5 Hz, 90 Hz가 약 90.9 Hz로 생성될 수 있습니다. 이는 실제 D435i의 주기가 아니라 사용 중인 시뮬레이터의 시간 해상도 한계입니다. [Gazebo Sensor::Update 구현](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/Sensor.cc)

다중 MB 영상은 reliable 발행 큐 5를 사용하고, 카메라 정보·IMU·점군은 센서용 QoS를 사용합니다. 실행 파일은 Fast DDS의 `LARGE_DATA` 전송을 하위 프로세스에 기본 적용하되 기존 환경변수가 있으면 유지합니다. 시스템 네트워크나 사용자 셸 설정은 변경하지 않았습니다. 별도 터미널에서 영상 수신 노드를 실행할 때도 해당 환경을 맞춥니다.

```bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

이는 대용량 데이터를 위한 SHM/TCP 전송 선택이며 실시간 처리나 무누락 보장은 아닙니다. 다른 ROS 통신 구현을 사용하면 이 Fast DDS 환경변수는 적용 대상이 아닙니다. [Fast DDS 대용량 데이터 안내](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/use_cases/large_data/large_data.html)

검사 도구는 영상 해상도·인코딩·광학 좌표계·`CameraInfo`·깊이 유효 거리와 시뮬레이션/실제 시간 기준 프레임률을 따로 기록합니다. 점군 수신을 먼저 확인한 후 점군 구독을 중단하고 기본 영상 처리율을 측정하므로, 점군을 계속 사용하는 전체 파이프라인의 성능 시험은 아닙니다.

```bash
python3 scripts/smoke_simulation.py --track experimental
python3 scripts/smoke_simulation.py --track original --d435i-profile synchronized_60
python3 scripts/smoke_simulation.py --track experimental --d435i-profile low_load_30
```

실행별 결과는 `artifacts/tests/`의 JSON에 저장되며, 요약은 [당일 활동 기록](../activity/2026-08-31.md)에 남깁니다. 일부 진단 실행에서 영상 누락이 관측됐으므로, 설정값을 실측 성능 보장으로 표현하지 않습니다. 전체 한 바퀴나 다른 차량과의 경쟁을 검증하는 도구는 아닙니다.

## 최장 직선과 센서 범위의 관계

원본·실험 지도 모두에서 중심선 선분들의 방위각 변화 폭을 1° 이하로 제한한 최장 연속 구간은 **10.2989 m, 약 10.3 m**입니다. 센서 범위 실험에서는 약 11 m 구간을 참고할 수 있습니다. CSV 시작·끝이 같은 직선 중간에 있다는 점을 반영해 폐회로 이음부를 연결해서 계산했습니다.

이는 완전한 직선이나 장애물 없는 가시거리의 보장이 아닙니다. 더 엄격한 각도 조건의 결과와 재현 방법은 [트랙 감사 기록](../track/TRACK_AUDIT.md#최장-직선에-가까운-구간-2026-08-31)에 있습니다.

직선이 10.3 m라고 깊이 카메라의 유효 거리를 10.3 m로 늘리지는 않습니다. RGB의 렌더링 거리를 깊이 범위와 분리하고, 실제 장애물 회피 요구 거리는 `속도 × 총 지연 + 속도² / (2 × 유효 감속도) + 여유 거리`를 출발점으로 정합니다. 감속도·타이어·노면과 센서 오차는 실측이 필요합니다. 이후 기초 주행용으로 [C1 기준의 임시 라이다](RPLIDAR_C1_SIMULATION.md)를 추가했지만 실물 LiDAR·ToF 선정은 아직 확정하지 않았습니다.

RGB 20 m 렌더링 역시 20 m에서 마커·신호등을 인식한다는 뜻은 아닙니다. 영상 안에서 차지하는 픽셀 수, 카메라 높이와 다른 차량·벽에 의한 가림을 별도로 시험해야 합니다.

## 다음 실제 장치 확인

`demo.launch.py`의 기초 벽 추종은 RGB 30 Hz와 라이다를 사용하고 깊이 센서는 기본적으로 끕니다. `depth_camera:=true`로 함께 켤 수 있으며 일반 `simulation.launch.py`의 고속 기본값에는 영향을 주지 않습니다. [기초 데모 안내](../autonomy/BASIC_DEMO.md)를 참고하십시오.

2026-08-31에 이 문서의 카메라 위치·시야각·해상도를 유지한 채 실험 시설을 수정하고, 실제 RGB로 신호등 출발 위치 6곳·마커 4개의 세 접근 거리에서 총 18/18 조건을 확인했습니다. 단일 차량의 정지 영상 검사이며 고속 인식·가림·실물 카메라 성능 검증은 아닙니다. 영상 근거와 마커 크기 기준은 [시설 안내](../track/FACILITIES.md)에 있습니다.

1. 보유 D435i의 펌웨어·IMU 리비전·USB 연결 속도와 사용 가능한 동시 스트림을 조회합니다.
2. 선택한 해상도에서 RGB·깊이 내부 파라미터와 두 센서의 외부 파라미터를 저장합니다.
3. 실제 실내 조명·트랙 재질·노출 설정에서 거리별 결측·오차·지연·움직임 왜곡을 측정합니다.
4. 이 측정으로 시뮬레이션 오차·지연 모델과 실제 센서 어댑터를 보완합니다. 실물 연결 검증은 이번 작업에서 수행하지 않았습니다.
