# r541 실측 조향 범위 반영 후보 — ready 프로필 생성 완료

2026-11-14, GPT. **실제 영점2811·count 증가=좌회전·CAD 방향−1과 현재 선택한 전동 목표2600–2980 counts를 기존 ready 프로필/MCU 헤더에 반영했다.** [선택 프로필](selected_v2/steering_zero.selected.json), [MCU 영점 헤더](selected_v2/mounted_steering_zero.hpp), [파생 상한](selected_v2/derived_limits.json)을 생성했다. 공통 `config/hardware/steering_zero.assembled_2026_11_14.json`에 같은 SHA로 채택했다. 장치 접속·flash·실물 명령은0이다.

## 측정과 현재 선택

원본은 Git 제외 `build/mounted_servo_setup_20261114_v1/manual-capture.MMQBmq-verified/`의 `straight.json`, `summary.json`, `powered-sweep-summary.json`이다. 원본 SHA와 현재 선택 근거는 프로필의 `selection_basis`에 남겼으며 원파일은 변경하지 않았다.

| 구분 | 값 | 해석 |
|---|---|---|
| 수동 직진 | 2811 counts | 기계적으로 맞춘 실제 영점 |
| 수동 좌/우 끝 | 3031 /2550 | 영점 대비 서보축 +19.336°/−22.939°; wheel각 아님 |
| 후속 전동 목표 | 2600–2980 | 사용자의 실측 반영 요청에 따라 기존 전동시험 범위에서 GPT가 선택한 내부 운영 범위 |
| 전동 실제 도달 | 2606–2976 | 목표와 구분; 마지막2807 |
| 방향 | count 증가=차량 왼쪽 | 기존 CAD-positive 서보각 기준 `direction=-1` |
| wheel각 | 모두 null | CAD 표 유지, `hardware_angle_measured=false` |

수동 기록 당시 `operating_limits_confirmed=false`와 후속23스텝 전동시험·현재 운영 채택을 시간순으로 구분했다. 모든 값이 미측정이라는 이유로 프로필을 계속 pending에 두지 않는다. 동시에 수동 끝점을 그대로 밀거나, 과거60° 가정을 실측 조향각으로 바꾸지 않는다. 실제 수동 **서보축 전체 폭은42.275°**, 선택한 내부 목표 폭은33.398°이며, 현재 CAD 내륜30°·중앙등가각은 별도다.

ready flag는 기존 스키마 의미에 따라 다음 근거로 설정했다. 각 flag의 설명은 생성 JSON에도 포함한다.

- straight/direction: 수동 직진·양 끝 count와 후속 전동 이동 방향.
- endpoints: 기계적 절대 끝점이 아니라 **사용자의 반영 요청에 따라 GPT가 선택한 내부 명령 범위2600–2980**. 원 수동 판정false는 provenance에 그대로 남는다.
- position mode: [11/12 원 identity](../../2026-11-12/st3215_servo_bench_v1/summary.json)의 ID1/mode0, 11/14 배포 도구의 mode·ID 검사와 완료 근거. `manual-three-point-v1.tar.gz`의 도구 SHA `9cf5ec3d98baf379d69f1485d1444d1218acc05e065e6aff6356e0c10c66d264`는 원 배포 로그와 일치하며 `bounds()`가 ID1/1Mbps/mode0/resolution1을 검사한 뒤 측정에 진입한다. 11/14 UART 구성 원문 대신 이 완료 흐름을 대조한 부분은 코드 경로에 근거한 추론이다.
- scale: resolution1·0–4095의 **4096-count 단일회전 프로토콜 단위 확인**이다. 외부 각도계로 서보축/바퀴각 정확도를 검정했다는 뜻이 아니다.

## 실제 제어에 사용할 값

기존 대칭 명령 경로를 유지한다. `/drive`·MCU receiver·ROS planner/guard·host에 사용할 중앙등가 상한은 **0.28475436568260193 rad =16.31522335°**다. 기존 전체 CAD 상한 약0.4578436rad을 그대로 쓰면 새 실장 범위를 넘으므로 해당 영점의 설치 생성이 거절된다.

| 요청 | 목표count | CAD 바퀴각/범위 |
|---|---:|---|
| 오른쪽 최대 `−L` | 2643 | 음수 중앙각→count 감소 |
| 중립0 | 2811 | 기존3877/3879 등 무부하 영점은 사용하지 않음 |
| 왼쪽 최대 `+L` | 2980 | 양수 중앙각→count 증가 |
| 선택한 오른쪽count2600 | 실제 대칭 경로에서는 미사용 | CAD 중앙등가−20.55813785° |
| 선택한 왼쪽count2980 | 현재 대칭 상한을 결정 | CAD 중앙등가+16.31522443° |

대칭 경로에서는 오른쪽 **43counts·약4.2429145°(오른쪽 CAD 가용각의20.64%)**가 남는다. 이번에는 좌우 비대칭 제어를 새로 만들지 않는다. 중앙각을 vehicle YAML의 joint 내륜 상한에 직접 넣으면 이중 축소된다. r542에 전달한 내륜 상한은 원 count에서 유도한 `raw C=0.2847543845603947`의 네 wheel각 최대인 **0.30666234131856146rad**이며, 기존 변환 뒤 float32에서 정확히 위 L로 돌아온다.

## 공통 배치·기존 생성 명령 인계

root 검토 후 프로필의 공통 배치 경로는 `config/hardware/steering_zero.assembled_2026_11_14.json`이다. 파일 SHA는 **e3077ab027dd2b25e384d76d5be2e649c664aa44a1140393494f9551f54bae72**, canonical JSON SHA는 `0f2288064939bbf331e0539eb87bc9086e95c7d80083e1923077664ab4feb891`이다. source-selected metadata는 기존 `check_profile`/`cpp_zero_profile`과 호환된다.

[board_steering_patch.json](selected_v2/board_steering_patch.json)은 **전체 board 파일이 아닌 변경 지시**다. 실제로 검토한 board의 `servo.command_limit`만 L로 바꾸고 기존 CAD geometry .128/.1239/π÷6과 대조한다. motor PI/duty/encoder 방향/MCU 속도상한/포트는 이 작업에서 채우지 않았다.

```powershell
# root가 위 SHA 그대로 공통 배치한 뒤 선택한다. 미배치 상태에서는 selected_v2 원본 경로를 쓴다.
$selectedSteering = 'config/hardware/steering_zero.assembled_2026_11_14.json'
# $reviewedActualBoard: 실제 board 원본을 보존한 별도 사본이며,
# selected_v2/board_steering_patch.json의 servo.command_limit만 반영한 검토 완료 파일이다.
python -B scripts/runtime_installed_profile.py prepare --board-profile $reviewedActualBoard `
  --steering-profile $selectedSteering `
  --assembly-id arena-car-v1 --output $freshInstalledBundle
python -B scripts/runtime_installed_profile.py verify --profile-dir $freshInstalledBundle
python -B scripts/runtime_installed_limits.py prepare --installed-profile $freshInstalledBundle `
  --runtime-params $reviewedRuntimeYaml --output $freshSelection
python -B scripts/runtime_installed_limits.py verify --selection $freshSelection `
  --installed-profile $freshInstalledBundle --runtime-params $reviewedRuntimeYaml `
  --host-steering-rad 0.28475436568260193
python -B scripts/runtime_installed_limits.py host-arguments --selection $freshSelection `
  --installed-profile $freshInstalledBundle
```

firmware build는 기존 `build_runtime_drive.sh --installed-profile`을 사용해야 generated header가 실제 `ARENA_RUNTIME_INSTALLED_PROFILE` 경로에 들어간다. planner/guard에는 같은 `$freshSelection/runtime_profile.yaml`, host에는 같은 선택의 `host-arguments`를 전달한다. **파일을 배치하는 것만으로 native MCU/host가 자동 채택하지 않는다.** 기존 standalone `steering_zero_profile.py export-cpp`는 전체 CAD 범위가 기본이므로 이 좁은 프로필에는 위 installed generator 또는 제공 헤더를 사용한다. `goal`은 r504의 `--command-limit-rad`로 L을 명시하면 된다.

이번 공개 프로필에는 측정 count·선택 근거·파일 SHA만 있으며 인증정보·장치 접근 키는 없다. r533 source-only 수집은 새 config 파일을 자동 운영값으로 채택하지 않으므로 최종 handover에서 root가 이 선택 프로필을 명시적으로 전달해야 한다. r533 자체는 변경하지 않았다.

후속 설치 입력은 **선택 영점 JSON + L을 반영한 실제 board + 동일 installed bundle에서 파생한 planner/guard YAML·host arguments**를 한 세트로 넘긴다. source ZIP 교체 또는 r542 vehicle YAML만의 선택으로 MCU/host 반영이 끝났다고 판정하지 않는다. 실제 board의 나머지 미결 값은 그대로 미결이며, 원 r500 설치본은 rollback 기준으로 보존한다.

## 검사와 한계

[최종 stage 영수증](validation_receipt.json)은 선택 프로필의 ready 검사, 원자료6개 SHA 보존, 파생값/헤더 재현, 문서 링크와 Python 구문 확인을 기록한다. `selected_v1`은 중간 산출물로 보존하고 최종 인계에는 **selected_v2만** 사용한다.

[Python6검사](python_tests.log)는401개 명령의 단조성/범위·좌우 부호·영점, 기존 count 포화와 바깥 중앙명령 거절, float32/joint 재환산, 최신 CLI를 확인했다. 기존 설치 생성기→MCU 헤더·두 ROS 노드·host의 동일 상한도 **임시 합성 board**로 통과했다. 첫 실행의 해당1개는 임시 fixture 디렉터리 중복 때문에 실패했고, 경로만 고친 [1개 재실행](installed_test_retry.log)이 통과했다. 합성 motor 값은 운용 산출물에 남기지 않았다.

[portable MCU 검사](stage/test_measured_steering.cpp)는 기존 C++ `ResolveCadServoGoal`로401개 변환·영점·좌우·포화·범위 거절을 확인하도록 준비했다. 이 작업의 Windows에는 C++ compiler가 없어 **직접 실행하지 않았고 root에 실행을 인계**했다. Python 결과를 MCU 실행 통과로 표기하지 않는다. 접지 부하·실측 wheel각·최종 flash/실물 폐루프는 이번 검증이 아니다.

## 공통 이관 후 native 확인

root가 WSL의 기존 C++ portable `ResolveCadServoGoal`로401개 목표·단조성·영점·좌우 방향·포화·상한 초과 거절을 실제 실행해 통과했다. 실물 UART/펌웨어 실행은 아니다. 관련 조향/설치/독립 보호 회귀110개도21.09초에 통과했다(`native_verification/run.log`, `pytest.xml`). r542 파생 차량 설정도 `src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml`에 별도로 추가했으며 기존 파일은 보존했다. 새 범위에서 코너·완주 여부는r544 한 번의 실제 시뮬레이션으로 확인하며 아직 결과는 없다.
