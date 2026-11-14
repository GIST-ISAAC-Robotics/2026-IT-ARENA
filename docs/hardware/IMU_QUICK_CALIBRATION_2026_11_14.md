# WAIT 자동 IMU 임시 보정 — 2026-11-14

> 경기 종료 후 분류: **현장 미사용 대체안/준비 이력**. 대회용 최종 기준은 [ESP32 소스 보존본](../releases/FINAL_ESP32_2026_11_14.md)이다. 아래 내용은 당시 오프라인 준비 기록이며 실차 적용 결과가 아니다.

현장 센서 교정 시간을 줄이기 위한 사용자 요청의 **오프라인 구현**이다. 장치 접속·통전·구동·설치·실차 보정은 실행하지 않았다. 임시 수치와 CAD 각도를 실측 완료값으로 승격하지 않는다.

## 자동 실행

[`imu_quick_calibration.yaml`](../../config/hardware/imu_quick_calibration.yaml)을 현재 실물 실행 파라미터 위에 겹쳐 **제어기와 독립 보호층 양쪽**에 전달한다. 공통 [`MotionInput`](../../src/arena_autonomy/arena_autonomy/lidar_motion_ros.py)이 부팅 후 입력을 기다리며 `WAIT`, `WAIT_GREEN`, `DISABLED`, `RACE_LATCHED`, `ACTUATOR_WAIT` 진입 뒤 자동 수집한다. 별도 수동 보정 명령은 필요하지 않다. 과거 시뮬레이션의 `motion_quick_calibration=false` 기본값은 유지한다.

- 신선한 대기 상태와 `/drive/safe` 속도 명령0을 요구한다.
- 엔코더 속도 절댓값 ≤0.025 m/s를 요구한다. 정규 속도 변환이 없는 진단에서는 **독립 MCU 관측 순서·취득 시각이 전진하는 raw count 불변**도 정지 후보의 근거로 허용한다. raw count로 정규 속도0이나 이동 이력을 만들어 발행하지 않는다.
- accel 크기 `9.80665 ± 0.5 m/s²`, 각 축 표준편차 ≤0.12 m/s², gyro 크기 ≤0.12 rad/s, 각 축 표준편차 ≤0.008 rad/s를 요구한다.
- 연속1.5초·gyro 최소80표본·서로 다른 accel 시각 최소20개를 모은다. gyro 공백0.08초, 운동/명령/accel 신선도0.15초, 상태 신선도0.7초를 넘으면 후보 창을 버린다. 수치는 초기 보정용 임시 기준이다.

통과하면 gyro 벡터 평균 `b`와 `up = mean(specific_force)/|mean(specific_force)|`를 얻고 `yaw_rate = dot(up, gyro-b)`를 사용한다. 기존 scalar gyro bias는 새 투영값으로 **교체**하여 이중 차감하지 않는다. accel 중력을 영점으로 없애지 않는다. 적용 시 운동 이력을 비워 다른 축/부호의 표본을 섞지 않는다.

전후축·엔코더 부호가 미확정이어도 **up·bias 기반 2D yaw를 먼저 적용**한다. 추가 ARM·자동 주행·새 출발 대기 조건을 만들지 않는다. 주행 중 활성 보정은 고정하며 다음 대기의 새 안정 창에서만 갱신한다. 입력 경계·시각 되감김은 후보와 오래된 입력을 버리되 활성 임시 보정은 보존한다.

## 짧은 주행의 후보

**전후축:** 회전이 작은 직선 가감속에서 수평 specific force와 encoder `dv/dt`의 방향 일관성을 모은다. 최소12개·0.25초, 방향 일관성0.9 이상이다. 먼저 얻는 벡터는 *encoder 양수 방향*이다. 양의 명령만으로 물리 전방을 확정하지 않는다. 등속 직진만으로는 이 축을 알 수 없다.

**엔코더 부호:** 실제 읽은 서보 count의 CAD 등가각 `δ`와 별도로 확인된 count 증가=좌/우 근거가 있으면 `yaw_rate ≈ v × tan(δ) / wheelbase`를 비교한다. 좌·우 회전 각각0.25초 이상, 전체12개 이상, 각 방향의 동일 부호 비율0.9 이상, yaw 크기 비0.4–2.5에서 좌우 결과가 일치해야 후보가 된다. overlay 축거0.128 m는 CAD값이다. 조향 명령을 실제 위치로 사용하지 않는다.

실제 확인된 조향 부호가 독립 기준이다. 다만 낮은 슬립과 평면 운동을 가정한 **근사 후보**다. 슬립·기구 유격·CAD각 오차·들고 돌리기는 오판 원인이 될 수 있다. 좌우 불일치·자극 부족이면 후보로 남긴다. 검증된 기존 encoder 부호 프로필로 전후축 후보를 해석할 수도 있다.

정규 속도 프로필이 없으면 raw count 차분을 **보정 후보 내부에서만** 공칭28 count/rev·11.9118:1·반지름0.0325 m로 환산한다. WAIT 표기라도 센서에서 움직임을 관측하면 bias 창을 버리면서 동적 축 근거를 모은다. 실제 제어용 속도를 발행하지 않는다. 이 값들은 분해능·감속비·타이어 실측 보정 완료를 의미하지 않는다.

후보는 다음 안정 WAIT에서만 채택한다. `encoder_sign_candidate_basis`는 `motion_feedback`과 `raw_count_nominal`을 구분한다. 정규 피드백 근거의 sign은 평면 이동 이력에 적용하며, raw 근거의 sign은 `raw_encoder_sign`과 전후축 해석에만 적용한다. 이미 부호변환된 정규 속도를 이중 반전하지 않는다. **MCU 모터 극성·PI·설치 프로필은 바꾸지 않는다.** `encoder_sign_conflicts_configured`, `raw_sign_requires_installed_profile_reconciliation`, `motor_controller_polarity_updated=false`를 보존한다.

## 입력·진단 계약

| 입력 | 형식·의미 |
|---|---|
| 현재 `motion_imu_topic` | `Imu`, gyro rad/s·고유 취득 시각. `motion_imu_frame_id=lsm6dsox_sensor` 검사. quaternion 미사용 |
| `/hardware_runtime/accel` | `Imu`, 같은 frame의 specific force m/s²·중력 포함. 별도 FIFO stream 고유 stamp. accel covariance[0]=-1이면 사용 불가 |
| `/autonomy/status` | String JSON `state`, `sim_time_s`. 과거 호환키이며 실물에서는 ROS 실제 시각 |
| `/drive/safe` | `AckermannDriveStamped` stamp·speed. 명령0 확인용이며 구동 발행 없음 |
| 기존 encoder topic | 모터축1채널 또는 과거 rear pair. 기존 기어비/반지름/시간 검사를 통과한 속도 |
| `/hardware_runtime/encoder_observation` | String JSON `schema=uno_q_encoder_observation_v1`, `valid=true`, `boot`, `observation_seq`, `capture_us`, `motor_counts`, `receipt_realtime_ns`, `age_ns`. 순서/시각 미전진·boot 변경·수신 공백은 창 재시작 |
| `/hardware_runtime/steering_observation` | String JSON `schema=uno_q_steering_observation_v1`, `valid=true`, `central_angle_rad`, `receipt_realtime_ns`, `age_ns`, `steering_sign_verified`, `sign_reference`. 실제 부호 근거가 있어야 방향 추론에 사용 |

`receipt_realtime_ns`는 Jetson 수신 기준 시각이며 하드웨어 취득시각 검증을 뜻하지 않는다. raw encoder의 `stamp_ns`가 있으면 MCU capture를 변환한 그 추정 시각을 우선 검사하여 오래된 취득이 새 수신으로 신선해지는 것을 막는다. 원 accel timing/boot/seq 근거는 host `/hardware_runtime/accel_meta`에 보존한다. 서로 다른 gyro/accel 취득시각을 합치지 않는다. 실제 동일 취득의 별도 드라이버/합성 메시지만 `motion_calibration_accel_topic`을 gyro topic과 같게 선택할 수 있다.

각 노드 `~/imu_calibration/status`에0.5초마다, 채택 때 즉시 JSON을 발행한다. 주요 키는 `status`, `version`, `gyro_bias_rad_s`, `up_axis_sensor`, `forward_axis_status`, 전후축 후보/활성값, `encoder_sign_candidate`, `encoder_sign_source`, `turn_evidence_s`, `confidence`, `physical_calibration_verified=false`다. 스캔 운동 진단에도 요약이 들어간다. 재시작 간 영구 저장은 하지 않는다.

## 검증·한계

합성 입력과 실제 ROS 메시지 클래스를 사용한 fake node 검사다. `rclpy.init`, ROS 노드·Gazebo·SSH·GPIO·장치 포트는 실행하지 않았다. 최종 검사 결과는 [XML](../../artifacts/validation/2026-11-14/imu_quick_calibration_v1/pytest_v4.xml)에 보존한다. 임의 장착13방향·뒤집힌 장착·벡터 bias, stale/누락, 흔들림·수동 회전·명령 잔류, 재진입·입력 경계, 등속 무자극, 반대 encoder 부호, 좌우 불일치, raw count 변화/중복 seq/boot, raw 공칭 축추정, 어댑터 이력 교체·기본 경로를 확인했다.

초기 완료 진단의0.5초 지연은 채택 즉시 발행하도록 수정했다. 옛 정적 검사의 “어댑터에 accel 문자열이 없어야 한다”는 제한은 이번 사용자 범위에 맞게 수정했으며 지도/정답/quaternion 금지와 순수 운동 적분의 accel 미사용은 유지했다. raw 보정 시험의 최초 실패는20 Hz 입력40개가 최소80개 조건에 못 미친 fixture였으며 [실패 XML](../../artifacts/validation/2026-11-14/imu_quick_calibration_v1/pytest_v2.xml)을 보존하고 충분한 표본으로 재검사했다.

중력만으로 정지 시 yaw heading은 얻지 못한다. 경사에서는 중력 위쪽과 차체 z축이 달라 오차가 남는다. 엔코더0과 약1g만으로 들고 등속 이동하는 상태까지 배제하지 못한다. 절대 IMU 정확도·센서 시각 동기·모터 극성·조향각·전체 코스 주행·실차 안전의 증명이 아니다.

관련: [운동 보정](../sensors/LIDAR_MOTION_IMPLEMENTATION.md), [모터축 관측 한계](../autonomy/MOTOR_MOTION_OBSERVABILITY_2026_10_13.md), [ADR 0018](../decisions/0018-sensor-based-lidar-motion-compensation.md), [ADR 0026](../decisions/0026-motor-shaft-encoder-baseline.md). 이번 opt-in 보정은 사용자 후속 범위이며 과거 입력 계약을 소급 변경하지 않는다.
