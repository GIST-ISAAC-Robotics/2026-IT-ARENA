# 단일 모터·기계식 디퍼렌셜 차량 동역학

갱신: 2026-08-31. 실차를 측정한 디지털 트윈이 아니라 **질량·토크·마찰·접촉을 갖춘 알고리즘/물성 민감도 시험 모델**입니다.

## 이번 구현의 범위

- 구동 BLDC 하나를 대신하는 평균 바퀴 속도 PI 제어와 모터 토크-속도 외피, 토크/제동 한계·1차 응답 지연.
- 강체·질량 없는 기어의 단일 입력과 좌우 출력 관계. 기본은 이상적 오픈 차동이며 손실과 점성식 차동 저항을 선택할 수 있습니다.
- 각 뒷바퀴에는 토크만 적용합니다. 원하는 바퀴 회전속도를 강제하지 않습니다. 접지·휠 관성·관절·차체 운동은 Gazebo 물리 엔진이 적분합니다.
- 앞바퀴 두 조향각은 한 중앙 명령에서 Ackermann 관계로 만들고, 토크·목표 변화율이 제한된 가상 서보가 제어합니다. 두 관절의 총 토크 예산을 공유하지만 실제 링크 기구의 레버비/유격까지 재현하는 것은 아닙니다.
- 좌우 뒷바퀴 엔코더 두 개: 2048틱/회전·100 Hz·추가 지연 2 ms. 입력 관절의 타임스탬프로 표본을 잡고 시간 되돌림 때 이전 표본을 폐기합니다.
- 마찰 한계와 force-dependent slip(FDS), 차체의 자유로운 3차원 병진·회전, 충돌과 바퀴 들림 가능성. 차량을 평면에 고정하거나 속도로 자세를 덮어쓰지 않습니다.

기존 속도 강제 구동은 `drive_mode:=legacy_velocity`로 명시적으로 선택할 수 있습니다. 이는 **기존 제어 방식 비교**이지 과거의 질량·타이어·센서까지 그대로 복원한 역사적 실행은 아닙니다.

## 질량과 기하

| 구성 | 모델 질량 |
|---|---:|
| 바퀴 네 개 | 0.160 kg |
| 조향 링크 두 개 | 0.020 kg |
| D435i 대체 형상 | 0.072 kg |
| C1 대체 형상 | 0.110 kg |
| ToF 캐리어 여섯 개 | 0.003 kg |
| 잔여 차체·배터리·컴퓨터·구동계 등 통합 질량 | 1.635 kg |
| 합계 | **2.000 kg** |

사용자가 전달한 약 2 kg 예상에 맞춘 합계이며 실제 BOM 합산/계량은 아닙니다. 통합 차체 질량은 특정 부품 위치를 묘사하지 않습니다. 기본 가정의 전체 무게중심은 명목 지면에서 약 5.68 cm입니다. 물리 시험에서 렌더링만 꺼도 센서 링크와 질량은 유지합니다.

외형 20×15 cm·축거 14.5 cm·윤거 13.5 cm·바퀴 지름 5 cm는 유지했습니다. 각 바퀴 조향 ±0.45 rad를 지키도록 중앙 조향 명령을 약 ±0.375 rad로 제한합니다. 뒤축 중앙 최소 반경은 약 0.368 m이며, 코스 중심선의 작은 곡률 반경과 구분해야 합니다.

## 기계식 차동 관계

전진 회전 부호가 같고 전체 감속비가 `N`일 때 다음 관계를 사용합니다.

```text
캐리어 속도 = (왼쪽 바퀴 속도 + 오른쪽 바퀴 속도) / 2
모터 속도   = N × 캐리어 속도
이상적 출력 토크: 왼쪽 = 오른쪽 = N × 모터 토크 / 2
```

이는 좌우 바퀴를 같은 속도로 묶는 방식이 아닙니다. 지면과 관절의 운동이 서로 다른 회전속도를 만들며 모터는 평균만 제어합니다. 따라서 한쪽 접지가 나쁘면 그쪽이 헛돌면서 평균 목표 속도를 만족할 수도 있습니다. [MathWorks의 이상적 차동 관계와 손실 구분](https://www.mathworks.com/help/sdl/ref/differential.html).

| 프로필 | 기어 효율 | 캐리어 점성 저항 | 좌우 차동 저항 |
|---|---:|---:|---|
| `ideal_open` — 기본 | 1.00 | 0 | 없음, 좌우 동일 토크 |
| `lossy_open` | 0.90 | 0.0001 N·m/(rad/s) | 없음 |
| `viscous_lsd` | 0.90 | 0.0001 N·m/(rad/s) | `clip(0.002 × (ωL−ωR), ±0.030 N·m)` |

점성 차동 저항은 빠른 쪽에서 토크를 빼 느린 쪽에 더하며 에너지를 소모합니다. 역구동에서도 손실이 에너지를 생성하지 않도록 효율 적용 방향을 구분했습니다. **특정 LSD 제품의 토크 바이어스 비, 클러치 예압·스틱슬립·잠금 장치를 재현한 것은 아닙니다.** [제조사 모델의 제한 차동 개념](https://www.mathworks.com/help/sdl/ref/limitedslipdifferential.html)과 구별한 소규모 수동 저항 근사입니다.

## 임시 모터·타이어 값

설정 기준은 `src/arena_description/config/vehicle.yaml`입니다.

- 모터→캐리어 감속비 8:1, 모터 축 토크 상한 0.060 N·m, 제동 상한 0.045 N·m.
- 무부하 외피 속도 2200 rad/s, 응답 시정수 40 ms, 평균 속도 기준 증속 램프 3 m/s².
- 20 km/h는 목표 명령 상한입니다. 물리 속도 클램프가 아니므로 과도 응답·외력으로 넘을 수 있습니다.
- 타이어 횡/종방향 마찰계수 각각 0.9, FDS compliance 각각 0.02, 기준 바퀴 수직하중 4.905 N. 실제 접촉 하중은 물리 엔진이 계산하며 이 값은 FDS 정규화 기준입니다.
- 바퀴 회전 관절 한계를 1500 rad/s로 두어 기존 200 rad/s가 지름 5 cm의 20 km/h 시험을 몰래 막지 않게 했습니다. 고회전 안정성 보장이나 실제 베어링 정격은 아닙니다.
- `WheelSlip` 규약에 맞춰 바퀴 로컬 회전축 Z를 횡방향 마찰 기준 `fdir1`으로 사용합니다. [Gazebo WheelSlip 공식 설명](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WheelSlip.html).

이 수치는 구매 사양이 아닙니다. 실측 없이는 모터 출력·제동거리·슬립 시작 속도·전복 임계속도를 확정할 수 없습니다. 실시간보다 느리게 실행돼도 물리 시간의 속도·가속도와 센서 주기를 유지하므로 고속과 저속의 결과는 다릅니다. 단, 실제 CPU 과부하/USB 지연까지 똑같이 재현된다는 뜻은 아닙니다.

## 실행

WSL 프로젝트 디렉터리에서 ROS 환경과 빌드를 준비합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch arena_bringup demo.launch.py speed_profile:=brisk
```

| 속도 프로필 | 직선 요청 상한 | 비고 |
|---|---:|---|
| `cautious` | 0.35 m/s | 기존 시연 속도 수준, 기본 |
| `brisk` | 0.70 m/s | 단계 상승 비교 |
| `exploratory` | 1.40 m/s | 추가 제한과 제동 검증 필요 |
| `hardware_target` | 5.56 m/s | 하드웨어팀 목표 입력용. 본선 완주/안전 검증 아님 |

모든 프로필은 벽 곡률·전방 거리·횡가속 가정과 [ToF 안전층](../autonomy/TOF_SAFETY.md)의 추가 제한을 받습니다. `hardware_target`을 선택해도 현재 ToF 검출 거리 가정 아래에서 20 km/h로 달리는 것은 아닙니다.

```bash
ros2 launch arena_bringup simulation.launch.py differential_profile:=lossy_open
ros2 launch arena_bringup simulation.launch.py differential_profile:=viscous_lsd
```

`/drive` 요청 → `/drive/safe` → 차량 어댑터 → `/sim/cmd_vel`·`/sim/steering_angle` → 모터/조향 토크 경로입니다. 일반 사용자는 `/drive`만 발행합니다. `/wheel_states`·`/wheel_encoder_ticks`는 좌우 바퀴 관측을 유지합니다.

`/odom`은 이상적인 바퀴/조향 관절을 적분한 오도메트리로 실제 지면 이동과 다를 수 있습니다. `/sim/drivetrain`의 `truth_*`는 **검사용 정답 진단**입니다. 대회용 벽 추종과 ToF 안전 노드 어느 쪽도 이 정보를 읽지 않습니다.

## 독립 물리 시험과 한계

```bash
ctest --test-dir build/arena_gazebo --output-on-failure
python3 -m pytest tests src/arena_vehicle_interface/test -q
python3 scripts/validate_vehicle_dynamics.py --case low_speed_curve
python3 scripts/validate_vehicle_dynamics.py --case straight_20kmh
python3 scripts/validate_vehicle_dynamics.py --case split_grip_open
python3 scripts/validate_vehicle_dynamics.py --case split_grip_lsd
python3 scripts/validate_vehicle_dynamics.py --case corner_5kmh
python3 scripts/validate_vehicle_dynamics.py --case corner_8kmh
python3 scripts/validate_vehicle_dynamics.py --case high_cg_high_grip
python3 scripts/validate_vehicle_dynamics.py --case curb_trip
python3 scripts/validate_vehicle_dynamics.py --case bump_20kmh
python3 scripts/validate_vehicle_dynamics.py --case tof_stop
python3 scripts/validate_vehicle_dynamics.py --case tof_reverse_stop
```

시험장은 대회 트랙과 별도의 120 m 평지입니다. 실행별 YAML·SDF·입력 해시·원시 궤적·보고서는 날짜가 다른 `artifacts/tests/vehicle_dynamics/` 폴더에 보존합니다. 광학 센서가 필요 없는 시험은 렌더링만 끄며 차체/센서 질량은 유지합니다. `--physics-step 0.0005`로 시간 간격 민감도를 비교할 수 있습니다.

`passed`는 보고서의 `scenario_expectation` 조건을 뜻합니다. 위험 민감도 시험에서 자료 수집이 정상적으로 끝났더라도 `stopped: false`, `rolled_over: true`, 표적 접촉이 있을 수 있습니다. 모든 물리 단위 시험은 `safe_racing_verified: false`를 유지합니다. 안전 주행 성공과 계측 성공을 혼동하지 않습니다.

### 아직 재현하지 않는 항목

- 전압·상전류·PWM/FOC·센서리스 정류·배터리/BMS·전기 병렬 배선·발열·자기 포화.
- 모터 회전자/기어의 반사 관성, 기어 톱니·백래시·축 비틀림·클러치 접촉 상세.
- 실차 서스펜션·타이어 변형/온도·Pacejka 결합 슬립 곡선·실측 노면 마찰.
- 실제 부품 CAD·무게중심·공력·충돌 변형. 카메라/라이다/ToF 대체 링크에는 별도 충돌체가 없고 보호 구조도 미정입니다.
- 실제 다중 차량 추적·추월·접촉 이후 복구와 실물 센서 결측/간섭 전체.

하드웨어팀과의 확인 질문은 [회의용 구동계 정리](../hardware/DRIVETRAIN_MEETING.md)에 모았습니다. 검증 수치는 최종 실행별 근거와 함께 아래에 추가하며, 이전 실패나 중간 실행을 덮어쓰지 않습니다.

## 2026-08-31 기준선 결과

대표 보고서와 수치 정의는 [고정 검증 자료](../../artifacts/validation/2026-08-31/vehicle_dynamics/README.md)에 있습니다.

- 현재 토크 구동·ToF 보호층·RGB/라이다 벽 추종을 함께 켠 `brisk` 프로필은 실험 지도 48.6527 m(한 바퀴 46.6329 m+약 2 m)를 진행했습니다. 최고 실속도 0.8005 m/s, 최대 중심선 거리 0.1301 m, 고정 20×15 cm 외형과 정적 벽 겹침 0회, 조기 출발 없음, 네 마커·양쪽 벽 전환, 정지와 정상 종료를 한 실행에서 확인했습니다.
- 평지 직선 목표 20 km/h에서 마지막 1초 중앙값 19.997 km/h, 순간 최고 20.559 km/h에 도달하고 실제 차체/바퀴가 정지했습니다. 이 목표 도달 시험은 ToF를 끈 독립 시험장 결과이며 대회 트랙의 20 km/h 완주나 안전을 증명하지 않습니다.
- 조향 명령 0.37 rad에서 5 km/h 시험의 실제 중앙 회전반경은 0.394 m, 8 km/h는 약 0.628 m였습니다. 8 km/h의 1.0 ms/0.5 ms 물리 시간 간격 결과가 거의 같았습니다. 기하 반경보다 커진 것은 현재 타이어/조향 모델의 속도 의존 언더스티어이며, 실차 곡선으로 보정한 값은 아닙니다.
- 편측 마찰을 기준의 4%로 둔 이상적 오픈 차동에서는 좌우 최대 속도차 149.51 rad/s가 생겼고 동일 출력 토크를 유지했습니다. 명령 종료 약 3초 뒤 바퀴 표면 속도는 0.012 m/s인데 차체는 0.606 m/s로 계속 움직여 `stopped: false`였습니다. 점성 LSD 근사는 같은 조건에서 정상상태 약 2.000 m/s와 실제 정지를 보였지만 특정 제품의 보증 성능이 아닙니다.
- ToF 보호층은 높이 5 cm 정적 표적에 전진 최고 1.090 m/s에서 0.195 m, 후진 최고 0.799 m/s에서 0.224 m의 최소 차체 여유를 두고 멈췄습니다. 0.50 m의 보수적 저상 표적 가시거리와 평지 마스크를 적용하므로 20 km/h용 안전층이 아닙니다.
- 20 km/h에서 길이 20 cm·높이 1 cm 코사인 요철은 차체 피치 25.79°와 기준점 높이 0.1177 m, 편측 4 cm 턱은 롤 4.88°·피치 13.02°와 큰 휠 슬립을 만들었습니다. 명목 CG를 약 10.18 cm로 올린 고마찰 시험까지 포함해 **이번 조건에서는 전복이 발생하지 않았습니다.** 따라서 모델이 고속과 저속을 다르게 계산하는 것은 확인했지만 실제 전복 임계속도를 검증했다고 말하지 않습니다.

다음 보정의 우선순위는 실제 총질량/각 부품 위치와 CG, 타이어 하중별 힘-슬립 곡선, 서스펜션/범프스톱, 모터·ESC 계단 응답과 제동거리입니다. 이 값이 들어오기 전에는 위험 시험의 수치를 차체 설계 허용치로 사용하지 않습니다.
