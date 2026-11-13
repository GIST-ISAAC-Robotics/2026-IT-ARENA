# r439 장착 조향 상한 전달 경로 확인

2026-11-13, GPT. Windows 읽기 전용 확인이다. 이 문서 외에 공통 소스/설정은 수정하지 않았고 프로필 생성·시험·WSL·ROS·펌웨어·장치를 실행하지 않았다. 아래0.2 rad는 질문의 가정값이며 실제 장착값으로 채택하지 않았다.

## 결론

**장착 `command_limit`를 planner·guard·host까지 자동 전달하는 연결은 없다.** r430으로 Python 영점 export와 MCU의 선택 범위는 일치했지만, 설치 bundle을 준비/선택해도 r401 ROS YAML 두 곳과 r415 host CLI의0.4578436017036438 rad는 바뀌지 않는다. 따라서 MCU만0.2 rad로 바꾸고 기존 세 명령을 그대로 실행할 여지가 실제로 남아 있다.

현재 연결은 다음처럼 둘로 나뉜다.

```text
장착 board.servo.command_limit
  → runtime_installed_profile.py → 선택한 firmware bundle
  → MCU receiver.max_steering_rad + RuntimeServo ResolveCadServoGoal

r401/r415 별도 선택
  → ROS YAML local_pursuit.max_steering_angle_rad
  → ROS YAML lidar_safety.max_steering_angle_rad
  → host CLI --max-steering-rad
```

## 확인한 진입점

| 소비자 | 실제 입력과 근거 |
|---|---|
| export·선택 bundle | [`runtime_installed_profile.py`](../../../../scripts/runtime_installed_profile.py) 24–25행의 출력은 board/zero JSON과 두 C++ header뿐이다. 72–79행에서 board의 command_limit/기하를 export와 생성 구조체에 함께 전달한다. 93–107행은 원 입력과 manifest를 보존하지만 ROS YAML·host 인자를 생성하지 않는다. |
| firmware 선택 | [`build_runtime_preview.sh`](../../../../scripts/build_runtime_preview.sh) 10–15, 64–70행에서 명시 `--installed-profile`을 검증해 빌드 사본으로 복사한다. [`board_profile.hpp`](../../../../src/arena_vehicle_interface/firmware_runtime_drive/main/board_profile.hpp) 50–56행은 명시 compile 선택에서만 생성 Profile을 소비한다. 이 경로가 Jetson ROS 프로필을 선택하지는 않는다. |
| MCU | `board_profile.hpp:15–19`가 `servo.command_limit`를 receiver 상한으로 사용한다. [`runtime_servo.hpp`](../../../../src/arena_vehicle_interface/portable/runtime_servo.hpp) 203–204행도 같은 `config_.command_limit`를 `ResolveCadServoGoal`에 전달한다. 이 내부 연결은 이미 구현되어 있다. |
| planner | [r401 runtime_profile.yaml](../runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml) 26행의 `local_pursuit.ros__parameters.max_steering_angle_rad=0.4578436017036438`. [`wall_follow.py`](../../../../src/arena_autonomy/arena_autonomy/wall_follow.py) 32–37행이 ROS parameter를 읽고 [`local_pursuit.py`](../../../../src/arena_autonomy/arena_autonomy/local_pursuit.py) 171–172행이 FollowConfig에 전달한다. |
| 독립 guard | 같은 r401 YAML150행의 `lidar_safety.ros__parameters.max_steering_angle_rad`도0.4578436017036438이다. [`lidar_safety.py`](../../../../src/arena_autonomy/arena_autonomy/lidar_safety.py) 31행이 별도로 읽고 97행의 경로 검사와148/233행의 출력 제한에 사용한다. |
| host | [`run_runtime_actuation_host.py`](../../../../scripts/run_runtime_actuation_host.py) 93–105행은 `--max-steering-rad`를 필수 CLI로 요구하며 설치 bundle/YAML 인자는 없다. 431행에서 이 값을 link로 넘긴다. [`receiver_owned_actuation_host.py`](../../../../scripts/receiver_owned_actuation_host.py) 81–89, 99–105행은 그 값을 저장하고 초과 DRIVE를 거절한다. [r415 명령3](../runtime_entry_audit_r415_v1/REVIEW.md)의 값은 여전히0.4578436017036438이다. |

r401에 값 전달의 일부는 있다. [`prepare_profile.py`](../runtime_launch_profile_r401_v1/prepare_profile.py) 84–91행은 r399의 고정 `owned_ros_parameters`를 복사하고,167–170행은 planner 상한을 [bindings.json](../runtime_launch_profile_r401_v1/prepared_v2/bindings.json)의 `host_arguments.max_steering_rad`에도 기록한다. 그러나 이것은 **설치 bundle 입력을 읽는 기능도, host를 해당 값으로 자동 시작하는 기능도 아니다.** host parser는 이 bindings 파일을 받지 않는다. r401 생성기를 그대로 다시 실행하는 것도 장착0.2 rad의 전달 방법이 아니다.

코드상 예상되는 불일치는 명확하다. host까지 기존 큰 상한이면 MCU가 자신의 작은 상한을 넘는 DRIVE를 `invalid_drive`로 거절/래치할 수 있다([`actuation_receiver.hpp`](../../../../src/arena_vehicle_interface/portable/actuation_receiver.hpp) 191–194행). host만 작게 바꾸면 host에서 `invalid_drive_values`로 송신을 거절한다. 실제 장치에서 발생시킨 결과가 아니라 현재 분기의 정적 확인이다. planner/guard도 함께 맞춰야 하며 MCU나 host의 상한만 넓혀 해결할 문제가 아니다.

## 현재 도구로 가능한 가장 짧은 절차

1. 조립 후 확인한 실제 안전상한을 원 기록에 보존하고, **같은 중앙등가각 운용값 `L` 하나**를 선택한다. 이를 board JSON의 `servo.command_limit`로 넣어 기존 prepare/verify 및 `build_runtime_drive.sh --installed-profile` 경로로 명시 선택한다. 원 영점/count 범위·assembly ID는 실제 확인한 값을 유지한다.
2. r401 원 보관본을 덮지 말고 선택할 ROS YAML 사본에서 **두 값** `local_pursuit.ros__parameters.max_steering_angle_rad`와 `lidar_safety.ros__parameters.max_steering_angle_rad`를 같은 `L`로 바꾼다. planner와 guard 둘 다 이 새 파일을 `--params-file`로 선택한다. 동등한 임시 방법은 두 기존 명령에 각각 같은 `-p max_steering_angle_rad:=L`을 명시하는 것이다.
3. r415 host 명령의 **한 인자** `--max-steering-rad L`도 맞춘다. 값은 별도로 다시 계산하지 말고 선택한 bundle의 `runtime_board.json → servo.command_limit`에서 가져온다. 기존 B3/clock/frame/토픽·`--enable-drive-wire`의 별도 의미는 그대로 유지한다.

즉 사람이 맞출 곳은 **board1 + ROS YAML2 + host CLI1**, 선택할 경로는 **firmware bundle + 두 ROS 노드의 params-file**이다. 기존 YAML/host 명령을 그대로 두고 firmware bundle만 바꾸는 것은 네 소비자의 일치를 만들지 않는다.

`L`은 기존 [`vehicle_geometry.float32_limit_not_above`](../../../../src/arena_autonomy/arena_autonomy/vehicle_geometry.py) 6–20행 방식으로 실제 안전상한을 넘지 않는 float32 값으로 정해야 한다. 표현 계산만 확인한 예로,0.2는 float32에서0.20000000298023224가 되고 아래쪽 인접값은 **0.19999998807907104**다. 네 운용 입력을 이 아래쪽 값으로 통일하면 MCU/host의 엄격한 비교와 ROS float32 송신의 의미가 일치한다. 이는 실측값/허용각을 늘리는 것이 아니며, 새 영점이나 장착값을 합성하는 절차도 아니다. 이미 MCU 상한0.2를 선택한 경우 planner/guard 송신값을 위 아래쪽 값으로 제한하고 host/MCU는0.2를 유지해도 초과는 없지만, 이 경우는 수치까지 네 곳이 같은 구성과 구분해야 한다.

## 필요한 최소 개선 한 가지

**설치 bundle을 단일 입력으로 받는 얇은 운영 설정 연결**을 추가하면 된다. 준비 단계에서 기존 float32 내림 운용값 `L`을 한 번 확정해 board/firmware bundle에 사용하고, 운영 진입은 그 bundle의 같은 `L`을 읽어 두 ROS parameter override와 host의 기존 CLI 인자를 함께 파생하도록 한다. 이미 생성한 bundle의 board값을 운영 진입에서 조용히 다시 쓰지는 않는다. 같은 assembly/bundle 경로와 파생값을 실행 설정에 남기면 사용자가 네 숫자를 독립적으로 맞출 필요가 없다. 현재 설치 export 또는 운영 진입 도구 옆에 이 연결을 두되, 원 r401 보관본·기하·제어기·보호층·펌웨어 계약은 다시 만들지 않는다.

이번 확인에서는 이 연결을 구현하거나 실제 운용 파일을 만들지 않았다. r430의 export/MCU 일치와 r436의 **시뮬** 출력기 parameter 연결은 이미 완료된 별도 범위이며 재개발할 필요가 없다. 남은 것은 실제 선택한 설치 한계를 운영 ROS 두 노드와 host에 같은 원천으로 전달하는 부분이다.
