# r430 선택 조향 범위와 실장 영점 export 일치

2026-11-13. **stage 사본의4파일을 수정했고 관련45개 검사의 요구를 충족했다. 공통 원본은 수정하지 않았다.** 실제 영점·실장 한계·PI·출력값을 만들거나 채택하지 않았고, 장치·Jetson·ROS·시뮬·flash·IDF 빌드는 실행하지 않았다. 원 r416 자료는 보존한다.

## 문제와 변경

r416의 board는 `servo.command_limit/wheelbase/track/wheel_limit`을 명시 전달하지만 Python 영점 export는 고정 CAD 전체 범위를 먼저 요구했다. 그래서 실제 사용할 조향 범위를 줄여도, 더 좁은 실장 count 구간으로는 설치 프로필을 만들지 못할 수 있었다.

`cad_servo_goal()`과 `cpp_zero_profile()`에 위4개 선택 입력을 keyword 인자로 추가했다. 인자를 생략하면 기존 CAD 기하/전체 명령 범위를 사용하며 생성되는 영점 header 형식과 내용은 그대로다. `runtime_installed_profile.headers()`는 board의 동일4개 값을 export에 전달한다. C++ `RuntimeBoardProfile.valid()`와 `ResolveCadServoGoal()` 원문은 바꾸지 않았다.

- 선택 중앙각 상한의 양 끝이 CAD 표/명시 바퀴각 상한에서 잘리지 않아야 한다. 범위 밖 값·NaN·0/음수/잘못된 타입은 거절한다.
- 선택 양 끝에 필요한 실제 count 구간을 원 `safe_min_count/safe_max_count`와 대조한다. count 범위 자동 확대·wrap·실장 확인값 생성은 없다.
- 중립과 선택 양 끝의 양자화 결과를 확인한다. 실제 count 끝단 밖의 소수 count를 반올림해 안쪽처럼 받아들이지 않는다. 기존 C++와 같은 CAD/바퀴각 끝단의1count 안쪽 보정은 유지하고, 보정 뒤에도 장착/CAD/바퀴각 범위를 벗어나면 거절한다.
- 기하의 단위·모델 의미는 기존과 같다. 중앙각 명령과 실제 양자화된 각도는 반환값에서 계속 구분한다. CAD 표 자체를 실물 측정값으로 바꾸지 않는다.

## 변경 파일과 적용

stage 아래 저장소 상대 경로를 유지한다.

1. `src/arena_vehicle_interface/arena_vehicle_interface/steering_calibration.py`
2. `scripts/runtime_installed_profile.py`
3. `tests/test_steering_calibration.py`
4. `tests/test_runtime_installed_profile.py`

[manifest.json](manifest.json)에35개 복사 원본의 SHA-256, 수정4파일의 전후 SHA-256, patch 해시를 기록했다. [apply.patch](apply.patch)는 공통 경로 기준이며 `git apply --check` exit0을 확인했다. [original](original/)에 수정4파일 원본을 보관한다. 원본35개가 처음 읽은 해시와 같은 것을 마지막 패키징에서 확인했다. **공통 이관은 주 담당 검토 후 별도 수행하며, 이 문서 작성으로 적용된 것은 아니다.**

## 검증과 최초 실패 보존

WSL 실행 자원을 명시 배정받아 아래 두 파일의 pytest와 그 내부 native g++ 검사만 실행했다. 두 파일의 총45개 검사는 기존31개와 추가14개다. 새 대표는 두 장착 방향의 좁은 count 구간을 사용하고, 각 방향에서 양 끝/중간/중립/half-count 근처7개 목표를 C++와 Python으로 비교한다. 총14목표의 count는 정확히 같고 실제 상대각/좌우각은1e-12 이내다. 합성값은 테스트 fixture에만 있다.

- 최초 실행: `tests/test_steering_calibration.py tests/test_runtime_installed_profile.py`, **44통과·1실패·skip0**,12.19초. 실패는 기존 launch 순수함수가 참조하는 `arena_autonomy.vehicle_geometry`를 stage에 복사하지 않은 준비 누락이었다. 신규 좁은 범위와 기존/신규 native 검사는 통과했다. [원 로그](pytest_v1.log)·[원 JUnit](junit_v1.xml)을 보존한다.
- 누락된 기존 순수 모듈2개를 stage에 복사하고 PYTHONPATH를 명시했다. 제품 수정 없이 실패한 `test_launch_limit_uses_cad_and_legacy_remains_identical` **1개만 재검사하여 통과**,0.14초·exit0이다. [wrapper](verify_missing_dependency_v2.sh)·[로그](pytest_v2.log)·[JUnit](junit_v2.xml)·[종료코드](pytest_v2.exitcode)를 보존한다. 전체45개를 단일 실행에서 모두 통과했다고 쓰지 않는다.
- 최초 Windows→WSL 명령의 shell 인용 때문에 `$PWD`와 종료코드 변수가 바깥 shell에서 먼저 확장됐다. pytest 결과는 위 로그에 남았으며 바깥 shell도 exit1이었다. 이 작업이 만든 JUnit/tmp의 경로와 생성시각을 확인하고 r430 아래로 이동해 보존했다. 재검사는 변수 확장이 없는 파일 경로로 LF wrapper를 실행했다.
- 종료 후 `pgrep -af installed_steering_limit_r430`는 출력없음·exit1이었고, 이 작업의 pytest/g++/wrapper 잔여 프로세스 없이 WSL 자원을 반환했다.

추가 테스트는 좁은 장착 구간/두 부호, 기본 export 호환, 선택 기하의 잘못된 값, CAD 범위 밖, 명령 범위 밖, 실제 count 끝단0.1count 초과의 반올림 은폐 거절, 입력/프로필 보존, 좁은 C++ 목표 일치와 범위 거절을 확인한다. 무변경 기본 IDF 빌드나 실물 수신/구동 시험을 반복하지 않았다.

## 남은 한계와 문서 반영 초안

실제 장착 영점·count방향·양 끝·중앙각 상한과 board값은 여전히 사람이 확인해야 한다. 이 수정은 측정·출력 허가·실물 STOP 확인이 아니다. 선택 범위를 받는 planner/guard/host/MCU의 최종 운용 설정 일치는 기존 설치 인계 작업으로 남는다.

운영 문서에 넣을 짧은 설명: “실장 영점 export는 운영 board의 `command_limit/wheelbase/track/wheel_limit`과 같은 범위로 검사한다. 확인된 장착 구간이 CAD 전체보다 좁으면 실제 사용할 중앙각 상한을 명시해 그 구간 안에서 생성할 수 있다. CAD 표·바퀴각·실장 count 한계를 넘는 값은 거절하며 count 범위를 자동으로 늘리지 않는다. 기본 독립 `export-cpp` 명령은 기존 CAD 전체 범위 검사다.”
