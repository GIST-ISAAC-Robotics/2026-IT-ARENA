# 현재 현황과 다음 작업

갱신: **2026-10-27 (KST), 통합 재점검 반영**. 구현·시험 기준은 **`26ea47bfbf376490fd193fa56eb2a7fa6ae0c98e`**다. 문서 정리 `0e6c28a` 이후의 선택형 관측·교통 후보가 반영된 상태이며, 아래 계획 수정 자체가 새 주행·속도 상향·실물 검증을 뜻하지 않는다.

**저속 단독 1랩과 제한 교통/코너 주행을 확보했다. 다음 목표는 같은 후보의 교통 1랩·공식 출발/M5·방지턱/다랩·실물 연결이다.** 무반환 작업은 부분 성공이지 종료가 아니다. 통합 뒤 실제 최고속도 **2 m/s 이상을 가능하면 시험**하되 어렵거나 일정 부담이 크면 낮은 속도를 택한다. 이는 완주보다 낮은 우선순위의 사용자 목표이며 기존 15 km/h 목표와 실제 설정값을 구분한다.

읽기 순서: **이 문서 → [현재 결정 ADR 0027](decisions/0027-completion-first-policy-and-placement.md) → [현행 계획](DEVELOPMENT_PLAN.md) → [필요한 기술 근거](README.md)**. 제작팀 전달은 [하드웨어 요구사항](HARDWARE_AND_RULE_QUESTIONS.md)에 모은다. 상세 이력은 [이번 정리 직전 현황](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/blob/26ea47bfbf376490fd193fa56eb2a7fa6ae0c98e/docs/PROJECT_CONTEXT.md)과 연결된 원본 결과에 보존한다. 아래는 현재 요약이며 실행별 연혁을 누적하지 않는다.

## 현재 구성

| 영역 | 사실과 적용 범위 |
|---|---|
| 구매 센서 | B0183 RGB + RPLIDAR C1 + Adafruit 4438 LSM6DSOX. 구매 완료는 사용자 확인이며 수령·장착·실측은 품목별 미확인. 기보유품·깊이/ToF를 새 후보로 다시 넣지 않는다. [구매 구성](hardware/ESP32_BUDGET_REVIEW_2026_10_02.md) |
| 구동 | MB4266-24180 후면축 브러시 DC + MD20A, 단일 모터·기계식 차동·PGE-213 모터축 엔코더 1개. 65 mm·15:1, 7 PPR×4=28카운트는 납품 대조 전 판매 사양. [ADR 0026](decisions/0026-motor-shaft-encoder-baseline.md) |
| MCU·조향 | VND018 ESP32-S3-WROOM-1 N16R8, ST3215-HS와 Bus Servo Adapter (A). 실제 출력·배선·제동·정지는 미검증 |
| 기구·센서 배치 | 외형 목표 20×15 cm. 축거/윤거 135 mm·바퀴 폭 12 mm·약 2 kg·CG/마찰은 임시값. C1 x=60 mm/z=95 mm·후방 60° 마스크, 카메라 z=145 mm 수평도 임시값. **배치 변경 가능** |
| 기본 주행 | C1 순차 10 Hz·모터축/gyro 운동 보정·관측 벽 지역 경로 + Pure Pursuit·별도 영상 프로세스. `/drive` → 독립 LiDAR 보호층 → `/drive/safe`. 제어기에 지도·Gazebo 정답 자세/신호 상태를 주지 않음 |
| 최신 통합 비교의 출발점 | G1·`bounded_local_v4`·`corner_distance_candidate`·`per_ray_candidate`·`corridor_fallback`·평행/통과 선택기·GUARD 곡선 후보. [고정 실행](../artifacts/validation/2026-10-27/g1_guard_curve_v1/run2_trial.sh). 기본 strict·후보 OFF이며 경기 기본값 채택은 아님 |
| 기존 선택형 경로 | 구형 M3·최근 관측·R0+·M5·공식 4-red/B3는 각각 연결/상호 배제 조건이 있음. 새 G1 선택기와 모두 동시에 켠 가상 조합을 검증 완료로 쓰지 않음 |
| 실제 기본/속도 | `connected`/`oldest`, `actuation_mode=legacy`, 임시 3등, 방지턱 OFF. `local_fast` 2.5 m/s(9 km/h), G1 1 m/s, 측방/GUARD 0.35 m/s 및 관측 후보의 더 낮은 제한. 2 m/s 시험 목표·15 km/h 목표와 다름 |
| 환경 | PC WSL2 Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic. Jetson Ubuntu 22.04.5 / L4T 36.4.4, Humble·빌드·기준 재생 확인. 최신 후보의 전체 처리 지연·실물 센서/MCU 폐루프는 미확인. [A3](simulation/JETSON_REPLAY_2026_09_28.md) |

`vehicle.yaml` 높이 상한 `null`, `b0183_c1.yaml`의 IMU 후보 표기는 공식 40 cm 상한·확정 구매와 다른 오래된 메타데이터다. 후속 설정 정리 대상이며 이번 문서 작업은 설정을 바꾸지 않는다.

## 검증 상태

| 기능 | 확인한 성과 | 남은 한계와 근거 |
|---|---|---|
| M1 단독 | R0 마커 관측 48.693 m·전체 가림 48.653 m, G1 v4 마커 관측 48.673 m에서 각 1랩·접촉0·STOP·정상 종료 | 상한1 m/s·임시3등·방지턱OFF. 가림 실행은 약4.6초 정체·최대 두 바퀴 중심 이탈. 정합 승인0, G1 v4 전용 현폭/정합 호출0으로 기법 효과/반복성 미입증. [R0](../artifacts/validation/2026-10-27/partial_observation_v1/README.md), [가림](../artifacts/validation/2026-10-27/marker_visibility_v1/README.md), [G1](../artifacts/validation/2026-10-27/g1_partial_integration_v1/README.md) |
| M2 추종·정지·재개 | 기존 복합 모터축/IMU 오차·부분 합류·코너 재개, 80 ms IMU 지연 후 복구의 제한 시험. 새 G1 v4의 정지/재개도 HOLD3.240 s·재개.547 s·재개후1.840 m·789판단 동일 재생 | 새 G1 앞차 시험은 결측0. 부분 관측과 교통이 동시에 생긴 검증은 아님. Gazebo 관절 피드백은 실물 MCU PI 검증과 다름. [기존 오차 시험](../artifacts/validation/2026-10-26/m2_motion_stress_v1/README.md), [새 연결](../artifacts/validation/2026-10-27/g1_partial_traffic_v1/README.md) |
| M3 측방·복귀 | 평행 후보 좌우 각3.160 m 뒤 통과 선택기 run4/5 좌우 약3.880 m·접촉0·STOP/종료. GUARD 후속 run2는13.018 m·첫 U자 코너·해당40초14관문 통과 | 구형 최소약69.8 mm·R0+접촉·선택기 run1/2 교착은 보존. 최신 전체 교통 랩·움직이는 후방 소실/재접근·곡선 병렬·실물 미완료. [좌우](../artifacts/validation/2026-10-27/g1_pass_release_integration_v1/README.md), [최신 코너](../artifacts/validation/2026-10-27/g1_guard_curve_v1/README.md), [기존 E3](autonomy/MULTI_ACTOR_E3_2026_10_24.md) |
| M4 경기 운영 | 두 번째 픽셀 신호 전 정지·초록 후 재출발·3.146 m·STOP/종료 | 공식4등/실물 MCU 검증 아님. B3 기본 미채택. [운영](autonomy/RACE_START_CONTRACT_2026_10_13.md) |
| M5 밀집 출발 | 일부 이상 운동 시험에서 옆줄 선출발 후 진행, 앞차 정지/재개 Gazebo 연결 | 독립 자율차5대 출발 아님. 초기약3 cm·후방 가림·반복 HOLD·계산 마감, 최신 G1 결합이 남음. [재개점](../artifacts/validation/2026-10-26/m5_start_constraints_v1/README.md) |
| G2·긴급 잔디 회피 | 두 바퀴 잔디의 일부 가장자리 정지 표적에 기하/이상 추종 후보 | 운영 우회/복귀·중앙/낮은 표적·접지 미검증. 세 바퀴 이상 긴급 회피는 전략만 기록, 구현/검증 미완료. [G2](autonomy/G2_PREPARATION_2026_10_13.md) |
| 실물 센서·MCU | 입력 계약·원시 기록·C++ 코어·출력 비활성 UART 벤치 준비/빌드 | 실제 생산자·시각·단위·핀·전기STOP·모터/서보·속도PI 미검증. [센서](hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md), [구동](hardware/ACTUATION_CONTRACT_2026_09_22.md) |
| 방지턱·5랩·속도 시험 | 공개 형상·정적 기하·제한 LiDAR 비교 | 동적 통과·센서복구·5랩·2 m/s 통합 시험/실차 미입증. [방지턱](simulation/BUMP_GEOMETRY_2026_10_23.md) |

**회귀 범위:** 최신 GUARD 후반부 후보는 관련178검사·빌드·run2·2,047판단 동일 재생을 확인했다. 전체5,911통과/19건 건너뜀은 이전 run1 소스다. 최신 전체 회귀는 보충 대상이며 둘을 합산하지 않는다. 과거 strict R0 43.613 m·G1 전체 랩·가림 초기 실패와 당시 증거를 새 정책의 성공으로 소급하지 않는다.

## 주요 미해결 문제

1. **무반환의 결합 공백:** 단독1랩과 GUARD 후반부 1 mm 후보는 서로 다른 제한 근거다. 평행 후보는 `fully_observed`가 아니면 거절하고 GUARD 대안은 직선 적합 실패 등을 대상으로 하므로, 허용된 부분 관측에서도 측방 모드가 정지할 수 있다. 코드상 분기 확인이며 지속 교착 재현은 아직 별도다. [평행 후보](../src/arena_autonomy/arena_autonomy/parallel_lane_trial.py), [선택기](../src/arena_autonomy/arena_autonomy/pass_release_trial.py). 소실 장애물 래치·제동 중 모터축/차체 차이·센서 오차·일반화도 남는다.
2. **후방 소실·평가 정합:** `rear`/`unknown`은 확장 측방 관측창의 마스크 외 관측으로 제한 해제하는 경로가 이미 있다. 사각 내 뒤차 부재 증명은 아니며 재접근/코너 재확인이 남는다. 좌우 시험의 원래 앞차 거리 실패는 경로에 들어온 옆차를 추종한 결과와 구별한다. 기존150/97 mm는 과거 비교선이지 사용자 고정 규정이 아니다. [최신 상태와 원인](../artifacts/validation/2026-10-27/g1_pass_release_operation_v1/README.md).
3. **공식 출발·가림:** 후방 x=-.075 m·높이 .20/.30 m 후보는 화면 포함을 개선했다. 정지 렌더링에서는 빨강4개가 세로 연결 성분으로 붙어 기존 검출기 출발0/4. 순차 증가 시제품은 정지 이미지 기반이며 초록 단독/중간 누락을 거절한다. 빨강 가림 뒤 출발은 미해결, 최종 장착값도 미정이다. [기하·픽셀·시제품](../artifacts/validation/2026-10-27/camera_mount_preparation_v1/README.md).
4. **실물·처리율·납기:** C1 정렬/복제 전 점 순서·host 기반 시각, B0183 CSI/PTS, IMU 축/단위/시계, UART0 콘솔, 실제 제동/STOP을 벤치에서 확인해야 한다. 최신 전체 파이프라인의 Jetson 지연과 담당/첫 연결일/조립일 확정 근거는 미확인. [C1](sensors/C1_DRIVER_TIME_CONTRACT_2026_10_23.md), [영상](sensors/B0183_INPUT_HANDOFF_2026_10_23.md), [설계 전달](HARDWARE_AND_RULE_QUESTIONS.md).

## 공식 입력과 팀 임시 요소

- 저장소에 기록된 공식 확인은 **2026-10-27**, main `d8107de6ccff14ae17db7fb08732b55fed82b5b4`·`v2026.10.24`다. 이번 문서 편집에서 주최 측을 새로 조회한 것은 아니다. 팀 입력은 `config/tracks/official_v2026.10.24.yaml`. 도로 백색·벽 회색, 공개450×50×10 mm 방지턱 시각·기존20구간 충돌 근사, 100/150 mm 완화안 미확정. [경계](track/OFFICIAL_V2026_10_24_UPDATE.md)
- 본선45 cm·양측 잔디 각20 cm, 차량20×15×40 cm, 프리랩1·본경기5랩, 두 바퀴 잔디 무벌점. 세 바퀴 이상·장시간은 페널티 대상이며 충돌 임박 시 추가 이탈 선호는 팀 결정. [공식 고정본](https://github.com/MOSW626/istech-it-arena/blob/d8107de6ccff14ae17db7fb08732b55fed82b5b4/MANUAL.md)
- 코스 ID0/20/30/45·10 cm 판·중심40 cm. 후방 식별5 cm·ID49의 중심9 cm는 임시값으로 상향 가능하다. 빨강4세트 순차 점등 뒤 초록의 직접 영상 인식과 HW/SW 정지가 요구되며, 빨강 네 열 사전 관측·프레임 수는 개발 정책이다.
- 차체 물리 버튼·PC SW 정지·통신 단절·검차 시연은 실물 확인 전이다. 공식10초 정지 후 개입 허용을 자율 교착 해결로 간주하지 않는다. 예산/품목은 [확정 구매안](hardware/ESP32_BUDGET_REVIEW_2026_10_02.md), 기보유품 미사용은 사용자 확인을 따른다.

## 다음 작업과 진행 범위

**재개 기준은 [최종 GUARD run2](../artifacts/validation/2026-10-27/g1_guard_curve_v1/README.md)와 [현행 계획의 재개점](DEVELOPMENT_PLAN.md#현재-재개점--무반환-작업을-덮어쓰지-않는다)이다.** 당시 시험은 종료 기록이 있으며 새 실행·로컬 미게시 상태는 재개 전에 대조한다. 과거 보고서의 “커밋·푸시 없음/다음/승인 대기”는 당시 경위이며 현재 게시 상태나 영구 금지가 아니다.

- **이어갈 주행:** 관측–GUARD 결합·옆→뒤 소실/재접근 대표 시험과 같은 후보의 교통1랩. 이미 한 코너를 조건 변화 없이 반복하지 않는다.
- **기존 다음 작업 유지:** M5 2/3/5대 입력 대조와 공식4등·빨강 가림/초록 노출 출발 연결을 병행한다.
- **바로 병행:** 제작팀 담당·납기/첫 벤치 확인, 센서/MCU/STOP, 최신 후보 처리 시간. 준비 문서를 실물 완료로 쓰지 않는다.
- **통합 뒤 확장:** 방지턱·5랩·긴급 잔디 회피를 누락시키지 않고, 여건에 맞게 실제2 m/s 이상 시험을 시도한다. 어렵다면 낮춰 완주 우선으로 간다.

세부 후보·수치는 개발자가 구체화하고 기존 합의 밖의 새 구매/전략/위험 수용만 결과와 함께 사용자에게 올린다. 역할/기록은 [AGENTS](../AGENTS.md), 이관은 [통합 인수인계](reports/RECORD_REPLAY_JETSON_HANDOFF_2026_09_29.md)를 따른다.
