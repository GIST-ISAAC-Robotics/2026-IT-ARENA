# r556 비대칭 조향 범위 검토

**현재 진행 중인 r553과 이번 완주 수정 묶음은 ±16.315°를 유지하는 것이 타당하다.** 오른쪽 추가 범위는 완주에 의미 있는 기하 이득이 있지만, 현행 single-max 계약에서 한두 군데 변경으로 정확히 사용할 수 없다. 새 제어 알고리즘은 필요 없으나 명령 생성→보호→host/MCU→시뮬레이션의 부호 있는 범위 계약을 함께 바꾸는 별도 묶음이 필요하다. 이번에는 소스·설정·장치 변경과 시험 실행을 하지 않았다.

## 가능한 이득과 근거 한계

- 사용자 선택 운용 목표 count2600–2980, zero2811, count 증가=좌회전이다. 전동 시험 실제 도달2606–2976 및 수동 끝2550–3031과 구분한다. 바퀴각은 실측하지 않았으며 아래 값은 CAD 환산이다.
- r541 파생값은 중앙각 right −.3588071936151835 rad(−20.5581°), left +.2847543845603947 rad(+16.3152°)이다. 현재 대칭 bound는 ±.28475436568260193 rad, 목표 count2643–2980으로 오른쪽43 count를 사용하지 않는다.
- wheelbase .128 m의 `R=L/tan(|δ|)`에 따르면 현재 최소 반경 .437295 m, 전체 오른쪽 범위에서는 .341295 m로 약96 mm/22.0% 감소한다. 오른쪽 곡률 여유는 약28.1% 증가하지만 왼쪽 최소 반경은 그대로다. 저속 코너 진입/탈출의 추종 오차를 줄일 가능성이 있으며 고속화만의 이득은 아니다.
- float32 안쪽 반올림의 오른쪽 중앙각 후보는 −.35880717635154724 rad다. 이것은 단순 수치 파생이며 아직 실제 portable 목표/시뮬 계약으로 검증한 새 선택값은 아니다.
- r550의 오른쪽 포화 명령은 실제 이득을 검토할 근거지만, 해당 정지의 직접 원인은 이미 분리된 운동 사각 팽창이었다. 추가 조향만으로 그 오류를 해결한다고 볼 수 없다. r553에서 남는 실패가 실제 오른쪽 곡률 부족인지 먼저 구분하는 편이 효율적이다.

## 현재 single-max 계약이 막는 지점

| 경계 | 확인한 현재 동작 | 비대칭 활용에 필요한 최소 변경 |
|---|---|---|
| 프로필/CAD | Python `steering_calibration._cad_contract`, portable `ResolveCadServoGoal`은 ±command_limit 두 끝 모두를 먼저 검사한다. `BoardProfile.valid`도 두 끝의 unclamped/실장 범위를 요구한다. | zero/count window/CAD 표는 유지하고 중앙각 하한·상한을 분리해 두 실제 끝을 검증한다. 단일 max를 .3588로 올리면 왼쪽 허용 범위까지 넓히려 하므로 준비 검증부터 거절된다. |
| 제어기 | `FollowConfig.max_steering_rad`, `pursue_path`의 대칭 clip, helper/MovingPass의 `abs(previous)>max` 거절, piecewise 경로의 단일 최소 반경이 연결되어 있다. | 최종 요청각을 범위 안에서 만든 뒤 slew와 실제 그 각도의 sweep를 검사한다. 직전의 유효한 큰 우회전도 정상 상태로 인정해야 한다. 최소 반경을 오른쪽 값으로 양쪽에 적용하면 안 된다. 초기에는 기존 큰 반경을 유지해도 되지만 경로 생성의 전체 이득은 제한된다. |
| 독립 guard/출력 | `lidar_safety`가 ±limit로 clip하며, AppliedGuard 및 virtual MCU Python receiver도 단일 abs bound다. | 동일 부호 범위로 입력/출력 검증하되 raw 점유·STOP·시간 계약은 유지한다. planner가 큰 우회전 sweep를 계산한 뒤 guard나 MCU가 더 작은 각도로 바꾸는 불일치를 없애야 한다. |
| host/MCU | `receiver_owned_actuation_host.prepare_drive`, portable `DriveLimits`/`ReceiverConfig`가 abs 범위다. RuntimeServoConfig·board profile·설치 profile/ROS 파생 도구도 command_limit 하나를 전달한다. | 두 끝을 source-selected profile에서 일관되게 전달하고 host, portable parser, receiver, servo 준비/목표 검사에서 동일하게 거절한다. DRIVE 메시지의 실제 rad 필드나 영점은 바꿀 필요 없다. |
| 시뮬레이션 | 차량 YAML의 단일 내륜 cap→`vehicle_geometry.equivalent_steering_limit`의 대칭 command_limit→ROS/virtualMCU/actuator로 파생된다. Gazebo CAD mapping도 현재 내륜 cap .306662에 제한된다. | 중앙각의 두 끝과 각 wheel CAD 목표를 분리 검증해야 한다. 오른쪽2600의 내륜 목표 크기는 .395460518 rad이므로 현재 관절/plugin 상한으로는 도달 불가다. 공통 관절 cap을 넓혀도 양의 중앙각 제한은 별도로 유지해야 한다. |

변경 경로 근거: `local_follow.py:430~443`, `current_wall_reacquire.py:34,50`, `lidar_safety.py:270`, `portable/cad_servo_goal.hpp:12~23`, `firmware_runtime_drive/main/board_profile.hpp:21~39`, `portable/actuation_command.hpp:645~662`, `scripts/runtime_installed_limits.py:55~93`, `vehicle_geometry.py:54~72`, `single_motor_drive.cpp:215~220`.

단순히 최종 count를 비대칭으로 clip하거나 중앙값을 옮기는 방법은 적합하지 않다. 계획/점유 검사에 쓰인 조향과 실제 구동 조향이 달라지며, 특히 우회전 포화 장면에서 원하는 회전 반경을 잃는다. 새 부호 범위를 기본 대칭값과 호환되게 추가하는 방향이 적절하다.

## 채택한다면 필요한 최소 검증

1. **기존 Python/portable 변환 검사 재사용:** 좌/우 끝·zero·각 끝의 안/밖 1 float32 ULP·범위 내 소수 점에서 두 구현의 count와 CAD wheel 목표를 비교한다. count2600–2980 밖 출력0, 방향/영점 불변, 양자화 보정 후 범위 보존을 확인한다. 기존 대칭 프로필의 출력도 유지한다.
2. **host/receiver/STOP 계약:** 유효 큰 우회전은 전달하고, 왼쪽+.284754보다 큰 값은 오른쪽 최대 크기보다 작아도 거절해야 한다. NaN·기한·세대·STOP 및 대칭 기본값 거절 의미를 유지한다. 기존 계약 시험의 새 부호 경계 사례만 추가한다.
3. **제어/guard 기하:** 오른쪽 포화 장면과 그 좌우 반사 장면에서 요청·이전·slew 후 각도와 swept에 사용된 각도가 실제 수신/적용 각도와 일치해야 한다. 각 방향의 실제 전방 장애물 정지를 유지한다. 경로 최소 반경도 부호별 유효 범위를 넘지 않아야 한다.
4. **선택 launch/SDF 전달:** 실제 node 파라미터, virtual MCU, AppliedGuard, 관절/plugin cap과 CAD 목표를 확인한다. 오른쪽 확대와 왼쪽 기존 제한이 동시에 성립해야 하며, 현재 YAML의 scalar만 바꾼 검사는 충분하지 않다.
5. **완주 효용:** 남은 실제 오른쪽 포화 저장 입력1~2개에서 원인 분리 후 같은 출발 단독 주행1회로 평가한다. 정지·접촉·진행·정상 종료 기준은 유지한다. CAD 기반 시뮬 결과를 실측 바퀴각/실차 검증으로 확대하지 않는다.

따라서 당장 대칭 상한을 버리기보다 현재 실제 완주 결과를 회수하고, 남는 실패에 오른쪽 곡률 부족이 확인되면 위 범위의 비대칭 계약 묶음을 우선하는 것을 권고한다. 시간상 작은 조향 tweak으로 취급할 변경은 아니다.
