# CAD 조향·설계 한계·조립 후 영점

2026-11-13. [Fusion 측정](../../../../docs/hardware/STEERING_FUSION_2026_11_13.md)96행을 공통 Python/C++와 Gazebo 단일 모터 플러그인에 반영했다. **설계 최대각은 내륜 약30°·외륜 약23.3°, 서보 상대각은−23.6~+23.4°다.** 기존0.45rad 임시 내륜 상한을 새 CAD 한계로 재사용하지 않는다.

선택 설정은 [`vehicle.final_cad_2026_11_13.yaml`](../../../../src/arena_description/config/vehicle.final_cad_2026_11_13.yaml)이다. `simulation_steering_model: fusion_cad_2026_11_13`, 축거128mm·윤거123.9mm·관절 외곽±30°이며, 좌우 목표는 원 표의 끝점에서 멈추고 외삽하지 않는다. 다음 확정 기하 주행에 이 설정을 명시한다. 원r350와 기존 기본 설정은 과거 재현용으로 보존했다.

`/drive`는 서보 축 각도가 아닌 차량 중앙 등가각이다. 좌우 바퀴각에서 기존 odometry 정의 `atan(2 tan(left) tan(right)/(tan(left)+tan(right)))`를 사용한다. 양방향 공통 명령 상한 약26.2325°는 내륜30°를 임의로 줄인 값이 아니라 두 바퀴각을 하나의 제어값으로 나타낸 것이다. 제어기·보호층·시뮬 MCU가 같은 상한을 사용하며 float32 송신 경계를 유지한다. 부품 강성·유격·킹핀에 따른 바퀴 중심 이동·실물 전류까지 재현한 모델은 아니다.

## 영점 기록과 보정

[`steering_zero_profile.py`](../../../../scripts/steering_zero_profile.py)는 장치 연결/구동/EEPROM 변경 없이 파일과 목표 count만 계산한다.

1. 바퀴를 직진으로 맞추고 현재 count를 읽는다. `record --neutral-count <읽은 값> --assembly-id <장착 판본> --confirm-wheels-straight --evidence-note <측정 메모> --output <새 파일>`로 기록한다. 임의2048이나 이전 무부하3879를 중립으로 쓰지 않는다.
2. 설치 방향·허용 count 구간·4096count 위치 모드를 확인한 뒤 `confirm-mount <영점 파일> --direction <1 또는 -1> --safe-min-count <하한> --safe-max-count <상한> --confirm-position-mode-and-mounted-limits --output <새 확인 파일>`로 저장한다.
3. `goal <확인 파일> --steering-deg <차량 중앙각>`은 송신 없이 보정 목표 count와 양자화 후 좌우각을 표시한다. C++ `ResolveCadServoGoal`은 같은 offset을 기존 RAM 목표 패킷에 전달할 준비물이다. **실물 운용 펌웨어의 출력 HAL 연결은 후속**이다.

중립이0/4095 근처여서 요구 범위가 경계를 넘으면 임의 modulo/EEPROM 변경 대신 이유를 반환한다. 그 경우 실제 위치 모드/경계 이동을 확인한 뒤 별도 설정한다. 정상 끝점의 반 count 반올림은 한 count 안쪽으로 처리해 불필요한 목표 거절을 피한다. 기존 r319 실측표 경로는 보존했다.

## 검증

- 초기 관련 Python25개·Gazebo 포함5패키지 빌드·C++2개 통과. 영점/끝점 보완 후 관련65개와 C++ 엄격 컴파일/실행도 통과했다. [빌드](build.log)·[후속 회귀](final_regression.log). 같은 후속 명령의 Gazebo Python 바인딩 조회만 `ModuleNotFoundError`였고, 이후 기존 ROS–Gazebo 브리지를 사용했다.
- [Gazebo 부품 검사](component/report.json): 고정 차체·중력/접촉 없음·모터0명령, 중립→좌중간→좌최대→중립→우중간→우최대→중립7단계. 표와 플러그인 목표 차이 최대4.45e−13rad, 관절 정착 오차 최대0.000615rad(약0.0352°),513표본·두 프로세스 정상 종료. **코스 주행·타이어 하중·실차 조향 증명은 아니다.**

`staged/`와 `proposed.patch`는 인계 당시 초안으로 이후 최대각·중앙 명령 상한·영점 CLI 보완을 포함하지 않는다. 실제 기준은 공통 src/scripts/tests이며 초안 재적용으로 덮어쓰지 않는다. 첫 초안 생성은 삽입 anchor 오류로 실패했고 이를 수정한 뒤 적용했다.
