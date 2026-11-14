# r542 측정 count 범위의 조향 제한 전달

선택 차량 YAML을 만들고 Windows 순수 검사 10개를 통과했다. 공통 코드·설정, WSL, Gazebo, Jetson은 변경/실행하지 않았다. 현재 주행 중인 r539 입력도 변경하지 않았다.

## 다음 단독 주행 입력

- 파일: `artifacts/validation/2026-11-14/steering_limit_propagation_r542_v1/vehicle.measured_steering_2026_11_14.yaml`
- SHA256: `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021`
- 기존 `src/arena_description/config/vehicle.final_cad_2026_11_13.yaml`에서 `vehicle.drivetrain.max_steering_angle_rad` 한 줄만 변경했다. 조향 CAD 표·형상·질량·속도·센서 설정은 같은 바이트로 유지했다.
- 다음 실행의 `arena_bringup simulation.launch.py` **`vehicle_config` launch argument**에 위 파일의 실행 환경 절대경로를 전달한다. 상위 course wrapper가 이 값을 고정하면 그 wrapper의 `IncludeLaunchDescription(... launch_arguments=...)`에서 `vehicle_config` 값을 바꿔 전달해야 한다. JSON 프로필 파일만 바꾸는 것으로는 시뮬레이션에 적용되지 않는다.
- 이번 검증 경로는 `autonomy_mode:=local_pursuit`, 기존 독립 보호층/guarded actuation 유지다. 나머지 단독 대표 조건은 root가 기존 대표에서 유지한다. 새 실제 주행 결과는 아직 없다.

## 선택값과 방향

근거 프로필은 r541 `selected_v2/steering_zero.selected.json`이며 SHA256은 `e3077ab027dd2b25e384d76d5be2e649c664aa44a1140393494f9551f54bae72`이다. 영점 2811, 운영 목표 count 2600–2980, count 증가=좌회전, CAD servo 상대각 양수=우회전이므로 `count_direction_per_cad_positive=-1`이다. 수동 끝점 2550–3031, 전동 실측 도달 2606–2976, 운영 목표 2600–2980은 서로 다른 값이다. 바퀴 각도는 실측하지 않았고 기존 CAD 변환을 사용한다.

| 값 | rad | 의미 |
|---|---:|---|
| count에서 유도한 대칭 중앙각 C | 0.2847543845603947 | 비대칭 좌측 끝점이 제한 |
| 공통 명령 상한 L | 0.28475436568260193 | float32 안쪽값, 약16.31522335° |
| 파생 YAML 내륜 joint cap | 0.30666234131856146 | ±C의 네 바퀴 CAD각 최대 절댓값 |
| launch 재계산 물리 중앙각 | 0.28475438456006436 | CAD 이분법 역변환 |
| launch 제어기/guard 송신 상한 | 0.28475436568260193 | 선택 L과 정확히 일치 |
| 수동 패널 상한 | 0.2 | `min(0.2,L)`로 기존값 유지 |

중앙각 L을 차량 YAML의 내륜각 칸에 직접 넣으면 이중 축소된다. 또한 L에서 직접 구한 내륜각을 다시 역변환하면 이분법 오차 약3.15e-13 rad 때문에 float32 한 ULP를 추가로 잃는다. 임의 여유를 더하지 않고 count에서 유도한 반올림 전 C를 사용해 해결했다. 실제 목표 ±L은 새 joint cap 안에 들어간다.

실차 count 목표는 우/직진/좌 2643/2811/2980이다. 단일 대칭 계약을 유지하므로 우측 43 count, 중앙 CAD각 약4.243°는 사용하지 않는다. Gazebo는 연속 CAD 목표각을 쓰며 MCU는 정수 count를 쓰므로 양쪽의 실제 바퀴 목표가 바이트/수치상 완전히 같지는 않다. 예를 들어 우측 −L 명령은 정수 count에서 중앙 CAD각 −0.2840975295 rad가 된다. 이는 기존 양자화 구조이며 실차 추종 정확도 증명이 아니다.

## 전달 경로와 누락

| 경계 | 실제 소스 위치 | 전달 내용 |
|---|---|---|
| 차량 YAML → 중앙각 | `vehicle_geometry.py:49`, `simulation.launch.py:878/884` | CAD `command_limit`로 내륜 cap을 중앙각으로 변환 |
| 제어기 `/drive` | `simulation.launch.py:1492/1624`, `local_pursuit.py:178` | local_pursuit는 float32 안쪽 L 사용 |
| 독립 guard `/drive/safe` | `simulation.launch.py:1503`, `lidar_safety.py:31/167/265` | 같은 L로 현재/수정 조향 제한 |
| virtual MCU | `simulation.launch.py:1381`, `actuation_ros.py:170` | 중앙 물리 상한을 strict 수신 계약에 전달 |
| 가상 출력 감시 | `simulation.launch.py:1311`, `actuation_ros.py:336` | 같은 물리 상한의 AppliedGuard; 초과 시 정지 |
| Gazebo | `simulation.launch.py:1018`, `model.sdf.xacro:596/609/654`, `single_motor_drive.cpp:59/215` | YAML 내륜 cap을 관절/플러그인에 전달, 중앙 명령→CAD 좌우 바퀴 변환 |
| 실제 ROS/host 선택 | `runtime_installed_limits.py:65/76/96` | 설치 bundle의 command_limit→두 ROS 노드와 `--max-steering-rad` 생성 |
| 실제 MCU | `board_profile.hpp:21`, `runtime_servo.hpp:203`, `cad_servo_goal.hpp:13` | board servo.command_limit strict 계약, 같은 CAD→영점/방향/정수 count 변환 |

**누락 1:** `simulation.launch.py`는 설치 servo JSON/bundle을 읽지 않는다. 위 파생 차량 YAML을 명시해야 다음 시뮬레이션에 적용된다. 현재 local_pursuit 경로는 모든 상한 연결이 있다. 다른 legacy autonomy 모드는 controller 초기값에 내륜 cap을 그대로 쓰는 분기가 남아 있으므로 이번 결과를 전체 모드 공통 검증으로 확대하지 않는다(`simulation.launch.py:1565/1624`).

**누락 2:** 실제 host CLI, 실제 ROS YAML, firmware는 각각 선택해야 한다. r541 `selected_v2/board_steering_patch.json`의 servo.command_limit=L과 ready steering JSON으로 `scripts/runtime_installed_profile.py prepare`를 사용해 bundle을 만들고 빌드 입력에 연결해야 한다. 이어 `scripts/runtime_installed_limits.py prepare --installed-profile <bundle> --runtime-params <기존 선택 ROS YAML> --output <새 selection>`로 `runtime_profile.yaml` 및 `runtime_host_limits.json`을 생성한다. 실제 실행은 생성된 ROS YAML과 host `--max-steering-rad 0.28475436568260193`을 함께 선택해야 한다. 이 작업에서 bundle 빌드/장치 설치는 하지 않았다.

**누락 3:** 패널은 host 상한을 자동 수신하지 않고 `race_ops.manual_steering_limit_rad`를 사용한다. 현재 0.2<L이므로 이번 제한에서 실제 변경은 필요 없다. `selection.json`에 `min(0.2,L)` 결과를 명시했다. UI 수정은 필요 없다.

## 검사와 한계

`python artifacts/validation/2026-11-14/steering_limit_propagation_r542_v1/prepare_and_check.py`: 10개 통과. 프로필 경계, YAML 한 줄 변경, float32 정확한 재변환, 중앙/내륜 혼동 반례, 좌우 count/바퀴 방향, 관절 상한, 실제/가상 strict receiver 양끝과 다음 float32 초과 거절, 가상 출력 감시, STOP/만료, 패널 하위 상한을 검사했다. 최초 STOP 시험의 `CommandProducer.control('STOP')` 호출은 API 사용 오류였으며 실제 STOP 프레임으로 고쳤다. 최초 오류 수치는 `initial_test_api_error.json`에 보존했다.

현재 Python 환경에 PyYAML/ROS가 없어 전체 YAML 로더·ROS CDR는 실행하지 않았다. 기존 YAML을 바이트 보존하고 숫자 한 줄만 교체한 결과다. C++/Gazebo/실차 실행, 실측 바퀴각, 축소 상한에서의 코너 완주는 미검증이다. root의 r475 저장 명령 분석상 새 상한을 넘는 코너 명령이 있었으므로 다음 단독 대표에서 영향 확인이 필요하다. 저장 명령분포는 새 상한 적용 후의 주행 결과가 아니다.

## root 공통 이관과 실제 launch 확인

동일SHA 차량 설정을 공통 `src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml`에 추가했다. r544 첫 시도의 launch 계측에서 실제 선택 실행기는 `g1_g2_handoff_trial`이며 제어기/guard 중앙L, virtual MCU/actuator 물리 중앙각, SDF 좌우관절/plugin 내륜각은 기대값과 일치했다. 첫 계측이 실행기 이름을 `local_pursuit`로 잘못 고정해 주행 전 실패했고, 이름 검사만 수정해 후속시도한다. 이 오류를 실제 제어 실패 또는 완주 증거로 해석하지 않는다.
