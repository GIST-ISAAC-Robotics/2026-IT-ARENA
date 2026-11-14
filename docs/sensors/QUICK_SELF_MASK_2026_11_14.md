# 현장 임시 자체 반환 마스크 — 2026-11-14

> 경기 종료 후 분류: **현장 미사용 대체안/준비 이력**. 대회용 최종 기준은 [ESP32 소스 보존본](../releases/FINAL_ESP32_2026_11_14.md)이다. 아래 내용은 당시 오프라인 준비 기록이며 실차 적용 결과가 아니다.

사용자가 실측 보정 없이 차량 중심의 전후20cm × 좌우15cm 영역과 각 방향3cm 여유를 지정했다. 최종 범위는 **전후26cm × 좌우21cm**다. 이 영역의 유효 LiDAR 반환점을 자체 반사로 간주하는 임시 근사이며, 기둥·배선 실측이나 외부 물체 구분을 검증한 결과가 아니다.

## 좌표와 선택

- 프로필: [quick_self_mask_2026_11_14.json](../../config/sensors/quick_self_mask_2026_11_14.json).
- 현재 `base_link`는 축간 중점이다. [motion 설정](../../src/arena_autonomy/arena_autonomy/lidar_motion.py)의 `rear_axle_x_m`과 [launch](../../src/arena_bringup/launch/simulation.launch.py)의 `-wheelbase/2` 연결에서 확인했다. 축거128mm이므로 후륜축은 `x=-0.064m`이다.
- [현재 차량 프로필](../../src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml)의 차체 기하 중심은 `x=+0.00365m`다. 임시 box는 이 중심을 사용하여 `base_link` 기준 `x=[-0.12635,0.13365]m`, `y=[-0.105,0.105]m`다.
- 각 소비자의 `motion_lidar_x_m`, `motion_lidar_y_m`, `motion_lidar_yaw_rad`를 사용해 취득 순간 센서 좌표에서 box 좌표로 변환한다. 현재 CAD LiDAR 위치는 `(0.0754,0,0.1346)m`이며 실장 보정값은 아니다.
- ROS 문자열 파라미터 **`lidar_self_mask_json`에 프로필 JSON 본문**을 동일하게 전달한다. 파일 경로 자체를 넣는 파라미터가 아니다. 빈 문자열 또는 전체 프로필의 `enabled:false`로 비활성화한다. 기본값은 빈 문자열이다.

사용자 지정 box만을 자체 영역으로 사용하는 설치 overlay는 기존 각도 마스크도 대체해야 한다. `prepare_uno_q_quick_setup.py`는 `rear_blind_half_angle_deg=0.0`, `lidar_blind_sectors_json=''`, `known_structure_policy='off'`를 함께 적용한다. 기존 CAD 구조각에 의존하는 `g1_g2_rear_wall_witness`도 false로 맞춘다. quick layer만 추가하고 기존 후방 30° 또는 CAD blind sector를 남기면 box 밖 반환이 그 후단에서 다시 제거될 수 있다. 과거 프로필의 기본값은 수정하지 않는다.

## 데이터 경계

`native_lidar_input.subscribe_scan`의 공통 수신 경계에서 계획기와 독립 보호층이 같은 프로필을 독립 적용한다. 이는 `/drive → 보호층 → /drive/safe` 구조를 바꾸지 않는다. 계획/current 실행자로 전달하는 `snapshot_call`/JSON IPC/`restore_call`도 원천 인덱스 분류를 보존하므로 같은 scan 기반 차량 추적·점유 입력에서 자체 반환만 빠진다.

원시 `/scan`, `/scan/native` 메시지는 수정하지 않는다. 내부 복사본에서 **범위 안의 유효 반환만 NaN과 `self_masked_indices`로 표시**하며 LocalScene에서는 `masked`, 점유 끝점은 없음으로 남긴다. 원래 NaN/Inf·범위 밖 입력은 `unknown`을 유지한다. 마스크 뒤 공간에 점·최대거리·Inf 반환·자유 공간을 생성하지 않는다. 차량 충돌 외곽·속도·조향 설정은 유지한다. box 밖의 유효 반환까지 후단에서 제거하지 않도록, 이번 임시 overlay에서만 위의 기존 후방/CAD 마스크를 끈다. 과거 기본 프로필은 보존한다.

`/autonomy/status`와 `/safety/status`의 `self_return_mask`에 임시 상태, 좌표 경계, 센서 변환, 가장 최근 수신 입력의 광선 수와 제외 반환 수를 표시한다. 원시 입력은 소비자 마스크 객체에도 별도로 보존한다. 마스크 구역 안의 외부 물체도 제외될 수 있다는 사용자 지정 근사의 한계는 남는다.

## 검증 범위

[전용 PC 검사](../../tests/test_quick_self_mask.py) **22개**는 네 경계·모서리, 경계 바로 밖 근접 장애물, 센서 평행이동/회전, unknown 보존과 가림 뒤 free 거절, CAD mask 조합, 잘못된 프로필/프레임 거절, 비활성 경로, native/LaserScan 복사·직렬화·복원, 실제 `Lane` plan/current 자식 프로세스 IPC를 검사한다. 두 자식 프로세스의 정상 종료도 확인했다.

기존 native/관측/CAD/trace/guard 회귀를 포함해 중복 제외 **271개 통과**다. [첫 검사 XML](../../artifacts/validation/2026-11-14/quick_self_mask_v1/pytest_initial_environment.xml)은151개 중141통과·10실패를 그대로 보존한다. 실패10개는 ROS Python 경로 미설정으로 `xacro`를 import하지 못한 CAD 검사다. 설치/소스 변경 없이 `/opt/ros/jazzy/lib/python3.12/site-packages`와 ROS 라이브러리 경로를 명시한 [후속 XML](../../artifacts/validation/2026-11-14/quick_self_mask_v1/pytest_ros_python_paths.xml)은 CAD15개와 관련 회귀120개, 총135개 통과다. 첫 결과를 성공으로 덮어쓰지 않는다.

실차 접속·설치·센서 보정·주행·Gazebo/ROS 실행은 하지 않았다. PC 검사는 실물 마스크 정확성이나 주행 안전 증거가 아니다.
