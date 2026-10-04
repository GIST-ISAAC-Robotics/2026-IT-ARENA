# 모터축 엔코더 1차 검증과 제한된 고장 대응

날짜: 2026-10-02. [확정 구성 반영](../hardware/DRIVETRAIN_GEARING_2026_10_02.md#9-시뮬레이션-반영-후속) 이후 사용자의 재개 지시로 실행 시험을 시작했다. A4 중 새 모터축 엔코더·입력 경계·정지 대응을 우선 다룬다. GPT가 분석·코드 수정을, GPT 6 Luna가 빌드·ROS/Gazebo 실행과 대기를 담당했다. Claude는 사용하지 않았다.

## 1. 구성과 시험 경계

- **팀 선정:** MB4266-24180 + PGE-213 하나, 전체 감속비 15:1, 바퀴 외경 65 mm. 7 PPR·4체배 = 28카운트/모터 회전 가정이며 납품품의 실제 PPR·전압·배선은 미확인이다.
- **수학적 환산:** 평균 바퀴 회전당 420카운트, 약 0.486199 mm/카운트. 100 Hz(10 ms) 차분 속도 한 단계는 0.048620 m/s = 0.175032 km/h다. 50 Hz(20 ms)에서는 절반이다. 기어비는 회전수 비이고 바퀴 둘레는 이후 이동량 환산에 별도로 사용한다.
- **임시값:** 엔코더 표본 100 Hz·기본 지연 2 ms, 축거 135 mm, 바퀴 폭 12 mm, 약 2 kg, 기존 토크·PI·마찰. MD20A의 PWM·전류·열·단락 제동 실측 모델이 아니다.
- **정답의 용도:** 가상 엔코더의 원천과 시험 평가에만 관절/차체 정답을 사용한다. 주행 제어기는 모터축 측정·IMU·C1·RGB를 사용하며 지도/정답 자세를 받지 않는다. `/drive` → 독립 보호층 → `/drive/safe`를 유지한다.
- **서로 다른 제어 루프:** Gazebo 내부 모터 PI는 여전히 이상적 관절 속도를 쓴다. 아래 별도 PI 시험만 양자화된 카운트 속도를 입력한다. 두 시험의 성공을 합쳐 실물 MCU 속도 제어를 검증했다고 하지 않는다.

## 2. 먼저 재현한 문제와 수정

### 2.1 유한한 속도 튐과 잘못된 카운트 격자

수정 전 ROS 메시지 경계 검사는 **8개 실패·34개 통과**였다. 매우 큰 유한 모터 속도를 LiDAR 운동 보정에 받아들이는 3조건, 속도 경계 설정을 검사하지 않는 4조건, MCU 어댑터가 0.25카운트를 반올림해 받는 1조건을 재현했다. [수정 전 로그](../../artifacts/validation/2026-10-02/motor_encoder_validation/boundary_before/pytest.log), [당시 소스](../../artifacts/validation/2026-10-02/motor_encoder_validation/before_sources/).

수정 사항:

1. `MotionInput.on_drive_motor`는 평균 바퀴 속도로 환산한 절댓값이 `motion_max_motor_feedback_mps`(기본 5 m/s)를 넘으면 거절한다. 기존 MCU의 비현실 속도 한도와 맞춘 **입력 무결성 경계**다. 목표 최고속도나 실차에서 안전한 속도가 아니다. 거절한 값을 0으로 바꾸거나 측정 시각을 새로 붙이지 않는다. 단발 거절은 기존 이력으로 처리할 수 있으나 공백이 길면 기존 운동 입력 신선도 보호가 작동한다.
2. 가상 MCU는 모터 위치가 28카운트/회전 격자에 맞는지 `MotorEncoderSpec.counts_from_position`으로 검사한다. 격자 밖 표본은 버린다. 계속 거절되면 기존 50 ms 신선도 감시가 정지시킨다. 첫 표본의 미관측 속도 `NaN`은 허용하지만 위치는 유효해야 한다. 최종 검토에서는 거대 유한 위치의 나눗셈 overflow와 64비트 카운터 범위 이탈도 `ValueError`로 거절하여 콜백이 중단되지 않게 했다.

### 2.2 새 시각으로 계속 발행되는 고착 카운트

수정 전에는 일정 1 m/s 주행 명령 중 카운트만 고정해도 `ACTIVE`·목표 1 m/s가 유지되고 `stationary_ready=true`가 됐다. 신선도 감시는 **메시지가 끊기는 고장**만 잡으므로, 이 경우에는 부족했다. [수정 전 숫자 시험](../../artifacts/validation/2026-10-02/motor_encoder_validation/count_before/report.json).

단일 모터축 모드의 `ActuationReceiver`에 제한된 무진행 감시를 추가했다.

| 조건/행동 | 현재 설정과 의미 |
|---|---|
| 감시 시작 | 0.1 m/s 이상 목표 속도를 수신하면 시작 |
| 진행 근거 | 유효한 모터축 누적 카운트가 바뀜. 수신 시각이 아니라 취득 시각 기준 |
| 정지 | 500 ms 동안 카운트 변화가 없으면 `motor_feedback_no_progress`로 고장 고정·목표 0 |
| 복구 | 입력 복구만으로 재출발하지 않음. 기존 저속 유지 조건·명시 `CLEAR`·별도 `ARM` 필요 |
| 조정 위치 | `motor_no_progress_us`, `motor_progress_min_command_mps`; ROS 가상 MCU에도 같은 이름으로 노출 |
| 적용 범위 | **선택형 `virtual_mcu` 구동 경로**, 모터축 피드백 모드. 기본 `actuation_mode:=legacy`를 몰래 변경하지 않음. 실물 펌웨어에는 아직 이식되지 않음 |

500 ms·0.1 m/s는 **실측 전 임시 개발값**이다. 센서 고착·막힌 모터·구동계 고장을 구별하는 진단이 아니며, 과전류 차단이나 긴급 장애물 회피를 대신하지 않는다. 2.5 m/s라면 500 ms에 1.25 m를 이동할 수 있다. 기존 50 ms 피드백 제한·100 ms 명령 임대 제한을 완화하지 않았다.

사용자 확인상 하드웨어팀에서 이 지연을 별도 측정하거나 정한 적은 없다. 여기서 기동 지연은 제품의 정해진 대기 시간이 아니라 **명령 이후 첫 엔코더 카운트가 관측되기까지**를 뜻한다. 통신/드라이버 응답·정지 마찰·하중을 포함하므로 부품 도착 후 측정한다. 500 ms를 모터의 실제 지연이나 사양으로 인용하지 않는다.

## 3. 카운트·지연 시험 결과

[`validate_motor_encoder.py`](../../scripts/validate_motor_encoder.py)는 정수 카운트와 취득/도착 시각을 생성하고 실제 `ActuationReceiver`를 호출한다. 모의 실제 이동은 평가에만 쓴다. [최종 결과와 소스 해시](../../artifacts/validation/2026-10-02/motor_encoder_validation/count_final/report.json).

### 양자화

0~4.08 m/s의 13개 속도 × 10/20 ms의 26조건에서 일정 속도 차분 오차가 한 카운트/표본 간격 이내임을 확인했다. 이것은 계수 모델의 성질이지 실측 정확도 보증이 아니다.

- 10 ms·0.01 m/s에서는 약 80%의 속도 표본이 0이다. 나머지 카운트 변화가 200 ms 정지 유지 구간을 끊어 정지 준비는 되지 않았다.
- 10 ms·0.001 m/s에서는 약 56%의 평가 시점에서 정지 준비가 됐다. 이는 기존 저속 기준 0.03 m/s 이내이며 **완전 정지 증명은 아니다**.
- 0.005~0.03 m/s에서도 주기적인 한 카운트 변화가 저속 유지 조건을 자주 끊었다. 현재 판정은 저속에서 보수적으로 재허가를 늦출 수 있다. 이를 해결하려고 정지 조건을 느슨하게 바꾸지는 않았다.

### 고장/전달 조건

| 입력 조건 | 관찰/판정 |
|---|---|
| 기본·2 ms·20 ms 고정 지연 | 일정 1 m/s를 평균 약 1.0001 m/s로 환산. 도착 지연을 차분 간격으로 오인하지 않음 |
| 지터·배송 순서 역전 | 오래된 표본을 재사용해 신선도를 늘리지 않고, 후속 유효 누적 카운트로 환산 |
| 메시지 한 개 누락 | 누적 카운트 보존으로 다음 표본에서 평균 속도 복원 |
| 60 ms 연속 메시지 누락 | 마지막 유효 표본 0.790 s 후 50 ms인 0.840 s에 `encoder_timeout`, 목표 0·고장 유지 |
| 50 ms 지연이 처음부터 지속 | 유효 피드백이 없어 ARM 불가. **주행 중 50 ms 지연 전환 시험과는 다름** |
| 큰 카운터 초기화·부호 있는 32비트 래핑 | `encoder_implausible`로 정지. 래핑을 자동 복원한 것이 아님 |
| 카운트 고착 | 마지막 진행 0.800 s에서 500 ms 후인 1.300 s에 무진행 고장·목표 0 |
| 10% 펄스 손실 | 실제 1 m/s를 약 0.9002 m/s로 측정. 그럴듯한 측정값이므로 검출되지 않음 |
| ±한 카운트 수준의 변화 | 평균은 유지됐으나 개별 속도값은 튐. 정확성/고장 검출 보증은 아님 |

시험의 `passed`는 **정해 둔 기대 동작을 재현했다**는 뜻이다. 펄스 손실 사례의 통과는 손실을 보정했거나 주행이 안전하다는 뜻이 아니다.

## 4. 카운트 입력만 사용하는 별도 PI 시험

`SpeedPI.update_mean`을 추가했다. 단일 축에서 환산한 평균 바퀴 속도를 직접 받으며, 좌우 측정값 두 개를 꾸며낼 필요가 없다. 옛 `update(left, right)`는 호환용으로 유지한다.

임시 1차원 모형: 5 ms 제어, 10 ms 계수, 기본 2 ms 배송 지연, `가속도=3×effort−0.3×속도`, 정지/음의 effort에서는 2 m/s² 감속이다. 실제 모터의 응답·정지 마찰·전류·PWM 데드밴드·타이어 슬립을 재현하지 않는다. 목표 5종 × 이상적/양자화/20 ms 지연/10% 펄스 손실의 20조건을 비교했다.

| 목표 m/s | 양자화 피드백의 5~8초 실제 평균 m/s | 같은 구간 실제 속도 최대−최소 m/s |
|---:|---:|---:|
| 0.05 | 0.04996 | 0.02578 |
| 0.10 | 0.09989 | 0.02888 |
| 0.60 | 0.60045 | 0.00257 |
| 1.40 | 1.40138 | 0.00672 |
| 2.50 | 2.50178 | 0.00672 |

20조건 모두 출력 범위 ±1과 종료 시 모형 정지는 확인했다. **이 통과 기준에 정밀 추종 오차는 포함하지 않는다.** 특히 10% 펄스 손실·목표 2.5 m/s에서는 실제 속도가 약 **2.7801 m/s**가 됐다. 제어기가 부족하게 측정되는 속도를 보상하려고 더 구동하기 때문이다. 계수 손실은 smoothing 필터로 해결되지 않는다.

현재 경주 속도 대역의 양자화만을 이유로 긴 이동평균 필터를 기본 경로에 넣지는 않았다. 저속 정밀 제어는 필요해질 때 짧은 창·펄스 주기 측정을 비교하되, 지연·정지 응답·원시 카운트를 함께 평가한다. 위 숫자는 단순 모형의 결과이며 PI 실차 조정값의 적합성을 입증하지 않는다.

## 5. ROS 연결과 Gazebo 실행

ROS 독립 프로세스 시험에 `--feedback-mode drive_motor_shaft`를 추가하여 실제 모터축 토픽·28카운트·15:1 환산으로 호스트/가상 MCU/출력 보호 경로를 검사했다. 기존 legacy 실행은 그대로 재현할 수 있다.

[ROS 결과](../../artifacts/validation/2026-10-02/motor_encoder_validation/ros_motor_v1/report.json): 기존 14조건과 고착 정지/복구 후 재출발 방지 2조건 통과. 시작 시 자동 허가 금지, 명시 허가, 명령/엔코더/프로세스 소실, STOP, 시계 정지/역행, 명시 복구를 포함한다. 모든 자식 프로세스 종료 코드 0, 강제 종료/잔존 프로세스 없음. 합성 종방향 모형이므로 Gazebo나 실물 제동 시험은 아니다.

경계·구동 회귀 117개 통과 로그는 [after_v1](../../artifacts/validation/2026-10-02/motor_encoder_validation/after_v1/pytest.log)에 있다. 추가 계수·기하 검사 70개도 [주행 전 검사](../../artifacts/validation/2026-10-02/motor_encoder_validation/pre_drive_checks/pytest.log)에서 통과했다. 겹치는 검사이므로 합산하지 않는다.

최종 확장 회귀는 **579개 통과**했다([로그](../../artifacts/validation/2026-10-02/motor_encoder_validation/final_checks/pytest_broad.log)). 독립 프로세스를 여는 검사 파일 5개를 제외한 기존 확장 집합이며, 이번 별도 ROS/Gazebo 실행과 구분한다. 제외 목록·실행 래퍼의 종료 코드 표기 한계·최종 직접 검사 근거는 [검토 메모](../../artifacts/validation/2026-10-02/motor_encoder_validation/final_checks/REVIEW.md)에 남겼다.

첫 한 바퀴 시도는 **실패**했다. `system`·실제 시간 감시 3 s로 실행하자 900초 동안 시뮬레이션 시간이 약 18.7초만 진행했다. 10 Hz 스캔의 실제 시간 간격이 약 4.8초가 되어 `stale_input`/`SENSOR_STOP`을 반복했고 진행은 0 m였다. MCU는 명시 ARM 후 `ARMED`/`ACTIVE`를 오갔지만 고장 고정은 0건이었다. 즉 이번 무진행 고장이 발동한 결과가 아니다. [실패 보고서](../../artifacts/validation/2026-10-02/motor_encoder_validation/gazebo_lap_v1/report.json), [MCU 상태](../../artifacts/validation/2026-10-02/motor_encoder_validation/gazebo_lap_v1/actuation_trace.json). 종료 서비스 응답과 14개 자식의 정상 종료·잔존 프로세스 없음은 확인했다. **실제 정지 절차/완주는 미통과이며 정상 종료와 혼동하지 않는다.**

재시험은 과거 성공한 `capture_v3`의 명시적 오프라인 실행 조건인 `wsl_nvidia`·실제 시간 감시 30 s를 사용하고 목표를 12 m로 제한했다. GPU/실제 시간 한도를 함께 바꾸므로 GPU 단독 효과의 비교 실험은 아니다. 시뮬레이션 취득 시각에 대한 50 ms 피드백·100 ms 명령 임대·운동 보정 제한은 바꾸지 않았으며 기본 `system`/3 s도 변경하지 않았다. 이 30 s는 실물의 통신 두절 정지 한도로 가져가면 안 된다. 첫 시험의 실제 렌더러는 [llvmpipe 로그](../../artifacts/validation/2026-10-02/motor_encoder_validation/final_checks/system-renderer.txt)로 확인했다.

[12 m 재시험 결과](../../artifacts/validation/2026-10-02/motor_encoder_validation/gazebo_12m_v2/report.json)는 **주행·명시적 정지·정상 종료 통과**다.

실제 렌더러는 [Ogre2 원문](../../artifacts/validation/2026-10-02/motor_encoder_validation/final_checks/nvidia-ogre2.log)의 `D3D12 (NVIDIA GeForce RTX 5070 Laptop GPU)`로 확인했다. 초기 원시 스캔 10프레임에서는 전방/측방 각 417개의 유한 반환과 12 cm 미만 반환 0개를 확인했다. 정지 출발 자세에서 가까운 차체 가림 증거가 없다는 뜻이며, 모든 자세·실물 마운트의 가림을 부정하는 결과는 아니다.

| 항목 | 결과 |
|---|---|
| 조건 | 공식 코스, 슬롯 3, 순차 C1 10 Hz, 결합 운동 보정, local Pure Pursuit 50 Hz, `local_fast`, 선택형 가상 MCU, 방지턱 비활성 |
| 진행/속도 | 12.01849 m; 최고 2.25034 m/s ≈ **8.10 km/h**, 중앙값 약 4.82 km/h |
| 평가용 코너 속도 | 중앙값 1.32408 m/s ≈ 4.77 km/h. 곡률 ≥0.5/m·속도 >0.3 m/s로 분류한 28표본 |
| 경로·관측 | 평가 중심선 RMSE 약 2.31 cm, 주행 중 상태 표본의 `SENSOR_STOP` 0건, 운동 보정 상태 거절 0/42개. 모든 제어 주기/센서 수신 무손실 보증은 아님 |
| LiDAR | 500점, 실제 시뮬레이션 시각 기준 10 Hz·스캔 0.1 s·점 간격 0.0002 s, 순차 취득 확인. 순간 스냅샷 아님 |
| 충돌 | 주행 및 정지 평가에서 정적 벽 충돌 표본 0. 연속 충돌 검출 증명이나 다중 차량 시험 아님 |
| STOP | 명시 `software_stop`, MCU `ESTOP_LATCHED`, 출력 차단/0명령, 평가용 차체 정지 안정 확인 |
| 정지 요청 이후 | **약 0.7924 m 이동**, 정지 검사 대기 1.332 sim s. 모형의 제동 응답이며 실차 정지거리·최대 안전거리 보증이 아님 |
| 종료 | Gazebo 서비스 종료 응답 확인, 자식 14/14 정상 종료, 잔존/강제 종료 없음 |
| 범위 | **12 m 부분 주행 1회**. 첫 전체 랩 실패를 성공으로 바꾸거나 완주로 표현하지 않음 |

`actuation_observation.latched_fault_samples`는 고장뿐 아니라 **요청한 정상 STOP 이후의 `ESTOP_LATCHED`도 합산**하는 원시 진단 필드다. 이 수치를 주행 중 고장 횟수로 해석하지 않는다. 최종 이유와 `actuation_passed`를 함께 본다.

주행 전 기하 확인에서는 C1 스캔면 95 mm와 생성된 차체 상단 92.5 mm의 **2.5 mm 여유**를 검사한다. 단단히 고정된 차체와 센서가 함께 기울 때 이 상대 여유가 곧바로 줄어드는 것은 아니다. 타이어/지면/외부 물체, 실제 지지대·배선·공차에 의한 가림은 별도 문제다. 현재 평평한 상자 모델만으로 실물 자기 가림을 보증하지 않는다. Gazebo 초기 원시 스캔 10프레임의 전방/측방 근거리 반환도 평가용으로 남긴다.

## 6. 미해결 사항과 다음 범위

1. **단일 축 관측 한계:** 헛돎·차체 미끄러짐·좌우 반대 회전·구동계 이탈·잘못된 환산비는 모터축 하나로 해결되지 않는다. 정지 후에도 새 시각의 0카운트만 보내는 센서가 정상인지 확정할 수 없다.
2. **제한된 고착 감시:** 0.1 m/s 미만 명령, 카운트가 계속 조금씩 떨리는 고착, 500 ms보다 짧은 정체는 이번 감시의 검출 보장 밖이다. 정상 명령 전환으로 0목표가 반복되면 감시 구간도 다시 시작된다. 실물 기동·제동·제어 주기를 확인한 뒤 임계값을 조정한다.
3. **실물 카운터 계약:** PCNT 원시 래핑은 MCU에서 확장 누적 카운트로 처리해야 한다. 부팅/재연결/카운터 초기화를 명시적으로 알리고 자동 재허가를 막는다. 작은 초기화·계속된 펄스 누락은 plausible 범위 안에 들어오면 감지하지 못한다. 28카운트/PPR 옵션과 신호 레벨·카운터 필터·최대 입력률부터 실측해야 한다.
4. **A4 미완료:** IMU 유한 이상치·고착과 C1 부분 무응답 `+Inf`의 자유 공간 오인 문제는 이번 엔코더 보완으로 해결되지 않는다. [기존 C1 위험/재현](../reports/REPO_REVIEW_2026_10_01.md#no-return-decision)을 다음 입력·보호 계약 작업으로 우선 이어간다. 이를 건너뛰고 다중 차량/실물 C1 안전성을 선언하지 않는다.
5. **실물/Jetson:** 새 구성의 Jetson 재생·실제 ESP32 속도 루프·MD20A 전기/제동·장착 시험은 하지 않았다. 이전 A3 성능 결과를 새 코드의 실물 검증으로 승계하지 않는다.

## 7. 재현과 근거 보존

출력 디렉터리는 매 실행 새 이름을 사용한다. 아래는 저장소 루트에서 실행한다. ROS/Gazebo는 Ubuntu 24.04·ROS 2 Jazzy와 해당 작업 소스의 colcon 설치본을 먼저 source한다. `colcon build`에는 `--base-paths src`를 사용해 무관한 `node_modules`를 검색하지 않는다.

```bash
python3 scripts/validate_motor_encoder.py --output artifacts/validation/NEW/counts
python3 scripts/validate_actuation_ros.py --feedback-mode drive_motor_shaft --output artifacts/validation/NEW/ros
python3 scripts/validate_basic_autonomy.py --autonomy-mode local_pursuit \
  --lidar-acquisition sequential --lidar-compensation both --lidar-rate-hz 10 \
  --control-rate-hz 50 --speed-profile local_fast --disable-speed-bump \
  --actuation-mode virtual_mcu --actuation-stop software_stop --shutdown-mode service \
  --render-backend wsl_nvidia --sensor-wall-timeout-s 30 --stop-wall-timeout-s 120 \
  --grid-slot 3 --target-progress-m 12 --max-sim-seconds 60 --wall-timeout-s 900 \
  --output artifacts/validation/NEW/gazebo
```

수정 전 실패와 소스는 `before_sources/`, `boundary_before/`, `count_before/`에 보존했다. 최초 빌드의 검색 범위 문제와 수정 후 빌드 결과는 `preflight/`에 남겼다. 일부 실행 래퍼 로그의 빈 `EXIT_CODE=` 대신 최종 `ACTUAL_EXIT_CODE=0`과 pytest/검사기 JSON을 대조했다. [전체 근거 디렉터리](../../artifacts/validation/2026-10-02/motor_encoder_validation/).
