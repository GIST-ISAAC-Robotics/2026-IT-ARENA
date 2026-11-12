# 문서 안내

작업 시작점은 [현재 현황](PROJECT_CONTEXT.md)입니다. 최신 공식 코스는 [11/6 610 mm 직각 코스 원본 감사](track/OFFICIAL_V2026_11_06_AUDIT.md), 규정·신호 결정은 [11/2 회의·모델 검토](reports/OFFICIAL_UPDATE_2026_11_02.md), 주행 목표는 [11/1 중간 점검](reports/MIDPROJECT_REVIEW_2026_11_01.md)·[ADR 0028](decisions/0028-speed-continuity-and-final-gearing.md), 기능 완성도에 따른 우선순위는 [11/12 전체 재점검](reports/READINESS_REVIEW_2026_11_12.md)·[현행 개발 계획](DEVELOPMENT_PLAN.md), 기구팀 전달은 [하드웨어 요구사항](HARDWARE_AND_RULE_QUESTIONS.md)에 모읍니다. [모니터·세이브포인트 재개·사건 기록](autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md)에는 설계와 기본 OFF 시뮬 연결 상태를 구분해 정리했습니다.

**현재 기준·계획 → 필요한 기술 근거 → 과거 이력** 순서로 읽습니다. 날짜가 있는 보고서의 “현재/다음/승인 대기”는 당시 상태이며, 후속 사용자 결정을 덮어쓰는 지시가 아닙니다. 시험 본문·실패 판정·원시 산출물은 보존합니다. 이 색인은 시험 수치를 반복하지 않습니다.

## 작업별 읽기 경로

| 작업 | 먼저 볼 문서 | 필요할 때 확인할 근거 |
|---|---|---|
| 11/13 확정 STEP·배선 움직임·실장 가림 | [확정 기구와 보정 준비](hardware/PACKAGING_FINAL_2026_11_13.md) | 같은 이름의 새 원본 해시/지주, 거리 한정 자기 반환 후보, 잔디 +3mm 비교와 평탄 주행 기준 |
| 11/12 패키징 STEP·기둥 배선·XT30·안정성 | [1차 기구 종합 검토](hardware/PACKAGING_STEP_REVIEW_2026_11_12.md) | [B-rep 치수·단면·배선 두께·질량 가정·렌더](../outputs/cad_review_2026_11_12/README.md). 기존 전체 CAD와 현행 JR 시뮬을 구분. 실물 조립/주행 검증은 아님 |
| 전체 계획·다중 차량 | [현행 개발 계획](DEVELOPMENT_PLAN.md), [현행 목표](decisions/0028-speed-continuity-and-final-gearing.md), [다중 차량 전략·설계](autonomy/MULTI_VEHICLE_DESIGN_2026_10_04.md), [G0 관측·기하 시험](autonomy/MULTI_VEHICLE_G0_2026_10_05.md), [G2 기하·추종 준비](autonomy/G2_PREPARATION_2026_10_13.md) | [10/23 이전 계획·독립 교차 검토](reports/DEVELOPMENT_PLAN_2026_10_23.md#9-토의와-독립-검토에서-바뀐-점), [10/4 이전 계획](reports/DEVELOPMENT_PLAN_2026_10_04.md), [알고리즘 비교 이력](autonomy/ALGORITHM_OPTIONS.md), [9/22 이전 계획](reports/DEVELOPMENT_PLAN_2026_09_22.md) |
| 10/24–26 실행 근거 | [E1 R0 결과](autonomy/R0_BASELINE_E1_2026_10_24.md), [D·E2 관측/간격 결정](autonomy/OBSERVATION_DECISION_BRIEF_2026_10_24.md), [E3 복수 actor](autonomy/MULTI_ACTOR_E3_2026_10_24.md) | 시험 당시 보호·운영 기본값과 실패/STOP 미수행을 보존. 최신 방향은 ADR 0028, 구현 상태는 현황 |
| 주행·지역 경로·보호층 | [지역 Pure Pursuit](autonomy/LOCAL_PURE_PURSUIT_2026_09_21.md), [G1-a 선택형 추종·대기](autonomy/LOCAL_FOLLOW_G1_2026_10_05.md), [G1-b Gazebo 앞차 폐루프](autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md), [가림 경계 조각 후보](autonomy/G1_FRAGMENTED_BOUNDARY_2026_10_13.md), [급커브 후보·전체 랩 실패](autonomy/G1_CORNER_BOUNDARY_2026_10_15.md) | [ADR 0022](decisions/0022-b0183-c1-local-pursuit-prototype.md), [알고리즘 선택](autonomy/ALGORITHM_OPTIONS.md) |
| C1 실물 입력·RPM·시뮬 차이 | [11/10 실측과 후속 명세](sensors/C1_HARDWARE_2026_11_10.md), [시각 계약 감사](sensors/C1_DRIVER_TIME_CONTRACT_2026_10_23.md) | 원시 가변 점/취득 순서·ROS 정렬/칸 재사용·호스트 시각·8/10/12Hz 진단을 구분. 운영 시간 관문은 미승인 |
| LiDAR 운동/시간 보정 | [구현과 입력 계약](sensors/LIDAR_MOTION_IMPLEMENTATION.md), [제조사 C1 드라이버 시각 감사](sensors/C1_DRIVER_TIME_CONTRACT_2026_10_23.md), [단일 모터축 관측·G1/보정 감사](autonomy/MOTOR_MOTION_OBSERVABILITY_2026_10_13.md) | [ADR 0018](decisions/0018-sensor-based-lidar-motion-compensation.md), [순차 취득 시험](sensors/C1_SEQUENTIAL_SPEED_EXPERIMENT.md) |
| 측방 무응답·관측 보완 후보 | [10/21 관측 계약·비용 검증](autonomy/G1_OBSERVATION_CONTRACT_2026_10_21.md), [10/16 필요한 이동 외곽과 현재/최근 광선](autonomy/G1_OBSERVATION_ENVELOPE_2026_10_16.md) | 기본 비활성·독립 보호 검사·저장 입력과 폐루프 구분·계산 비용·후방 사각·재개 순서 |
| 센서 잡음·누락·지연 | [C1 무응답·IMU 이상치 보호](autonomy/SENSOR_INPUT_GUARDS_2026_10_04.md), [모터축 엔코더·고착 1차 검증](autonomy/MOTOR_ENCODER_VALIDATION_2026_10_02.md), [LiDAR 내성](autonomy/LIDAR_ROBUSTNESS_2026_09_22.md), [기존 IMU·엔코더 내성](autonomy/MOTION_SENSOR_ROBUSTNESS_2026_09_22.md) | 각 보고서의 실행 설정·실패·JSON |
| Jetson–MCU 구동·정지 | [구동 계약](hardware/ACTUATION_CONTRACT_2026_09_22.md), [ROS 연결](hardware/ACTUATION_ROS_2026_09_22.md), [RGB/프로세스 중단·정지/복구](autonomy/PIPELINE_FAULT_VALIDATION_2026_10_04.md) | [ADR 0002](decisions/0002-jetson-esp32-actuation-boundary.md), [ADR 0023](decisions/0023-mcu-actuation-contract-reference.md), MCU 이관 코드 [`portable`](../src/arena_vehicle_interface/portable/README.md)·[UART 검사 앱](../src/arena_vehicle_interface/firmware_bench/README.md) |
| **사용자 승인 전장 배치·실물 이관** | [승인 전장 연결 기준](hardware/APPROVED_ELECTRICAL_LAYOUT.md), [실물 이관 절차](hardware/PHYSICAL_HANDOVER_APPROVED_LAYOUT.md) | Jetson 전용 19 V, M01 CH1 모터, M01 단일 12 V/2 A 서보, Jetson USB로 ESP32 전원/통신, ESP32 3.3 V IMU, **J01 CH4 5 V/0.5 A(XT30) 엔코더 전원**, YF08E/TXS0108E 마킹 식별된 CF-0108, **B0390 22↔22 FPC(11/10 사용자 실물 확인)**. 기타 물리 핀 매칭은 미완료 |
| ESP32 주변 전체 배선·GPIO | [통합 배선 설계 v1](hardware/ESP32_GPIO_WIRING_2026_11_11.md), [신호·전원 한 장 배선도·연결표](hardware/schematics/esp32_gpio_wiring_v1.html), [STEP 층별 케이블 배치](hardware/WIRING_LAYERS_2026_11_13.md) | 3V3 2핀·GND 4핀 배정, T3V3/T5V/TGND, CN1 입력·CH1~6↔CN2~7 정격. 서보 CH5(CN6)·3선 전용 케이블. STOP GPIO8↔스위치↔TGND(5접점). 전체 조립 검증은 별도 |
| ESP32·IMU 조립과 최초 시험 | [11/11 실측·형식·수신 결과](sensors/ESP32_LSM6DSOX_HARDWARE_2026_11_11.md), [배선·후속 계획](hardware/ESP32_IMU_BRINGUP_PLAN_2026_11_11.md) | N16R8 메모리·SPI·약199 Hz, 첫 실패/GC 변경 재시험·포트 재개방 리셋·예비 영점·I1/시각/장착 미검증을 구분 |
| 엔코더 손 회전·신호 점검 | [PGE-213·CF 최초 실측과 재개점](sensors/PGE213_ENCODER_HARDWARE_2026_11_12.md) | 초기 입력 미검출 보존. 공통 GND 연결·CF 교체 후 손 회전 −30/+34계수·방향 확인. 후속 저출력 모터 계수 확인. 분해능·고속 품질은 미검증 |
| 모터 단독 구동·PWM0 정지 | [MD20A·MB4266 실측](sensors/MD20A_MOTOR_HARDWARE_2026_11_12.md), [집계·독립 감사](../artifacts/validation/2026-11-12/md20a_motor_bench_v1/README.md) | 최초 엔코더0 보존. CF 교체 후 약5%×4회 계수·정지 확인, 해체 구간 잡신호 별도 기록. CH1 분리·원 IMU 앱 복구 완료 |
| 서보 단독 실물 시험 | [ST3215-HS·Adapter A 최초 실측](sensors/ST3215_HS_HARDWARE_2026_11_12.md) | 점퍼 두 개 A·ID1/1 Mbps·약±10° 무부하 왕복/복귀·명령 공백 후 토크 OFF. 실장 교정/부하 전류는 후속, 기존 IMU 앱 복구·IMU 현재 분리 |
| 실물 입력 기록 검사 준비 | [모터축·IMU 수동 감사](hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md), [센서 ROS 기록 휴대 묶음](hardware/SENSOR_PROBE_RUNBOOK.md) | [LSM6DSOX 실물 단위·시각·frame](hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#9-1023--lsm6dsox-입력을-연결하기-전)·[독립 가속도 v2](hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#8-1023--독립-가속도-표본을-보존하는-v2)·[명령 없는 원시 바이트 기록](hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#7-1023--명령-없는-pc-원시-바이트-기록). 실물 드라이버 아님 |
| B0390 실물 영상 연결 | [실물 취득 결과](sensors/B0390_HARDWARE_2026_11_10.md), [교체 사양·시뮬 가정](sensors/B0390_SIM_2026_11_07.md), [공통 RGB·촬영 시각 계약](sensors/B0183_INPUT_HANDOFF_2026_10_23.md) | 22↔22 FPC·Jetson-IO·Argus 기본 취득과 ROS/노출 시각·교정·인식 검증을 구분 |
| 경기 출발·재개·모니터 | [현재 CLI](autonomy/RACE_OPERATOR_RUNBOOK.md), [세이브포인트·웹 모니터·사건 기록 계획](autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md), [최신 신호·재개 규정](reports/OFFICIAL_UPDATE_2026_11_02.md) | [기존 B3 계약](autonomy/RACE_START_CONTRACT_2026_10_13.md), [M5 정지 시야](autonomy/GRID_START_M5_2026_10_24.md), [고장/복구](autonomy/PIPELINE_FAULT_VALIDATION_2026_10_04.md) |
| 실물 입력·현재 실행 지연 | [11/12 실행·센서 통합과 재개점](reports/RUNTIME_SENSOR_INTEGRATION_2026_11_12.md), [Jetson 실행 주기 계획](reports/JETSON_RUNTIME_PLAN_2026_11_12.md) | 전담 UART 수신·두 단계 계산/완료 알림·선택형 수치 커널. 원 실패/실시간 미달과 실물 구동 미완료 구분 |
| 기록·재생·처리량 | [기록·재생/Jetson 통합 인수인계](reports/RECORD_REPLAY_JETSON_HANDOFF_2026_09_29.md), [G1 판단 비용·최소 이관 묶음](simulation/G1_DECISION_PROFILE_2026_10_13.md) | [A2 PC 상세](simulation/RECORD_REPLAY_2026_09_24.md), [A3 Jetson 상세](simulation/JETSON_REPLAY_2026_09_28.md), [Jetson 접속](hardware/JETSON_ACCESS.md), [기록·재생 ADR 0024](decisions/0024-isolated-sensor-record-replay.md), [영상 분리 ADR 0025](decisions/0025-isolated-vision-processing.md), [10/4 계획 이력](reports/DEVELOPMENT_PLAN_2026_10_04.md) |
| 환경·GPU·종료 | [환경 ADR](decisions/0001-simulation-platform.md), [GPU 설정](simulation/GPU_RENDERING.md) | [GPU 부하 비교](simulation/GPU_FINAL_DIAGNOSTIC_2026_09_08.md), [종료 진단](simulation/SHUTDOWN_DIAGNOSIS_2026_09_15.md) |
| 공식 트랙·시설 변경 | [11/6 610 mm 직각 코스 원본 감사·이관 항목](track/OFFICIAL_V2026_11_06_AUDIT.md), [11/1 배포·방지턱 제거·공식 2×5 신호/0·45](reports/OFFICIAL_UPDATE_2026_11_02.md), [새 월드 전방 교통](../artifacts/validation/2026-11-02/official_forward_v1/README.md), [대표 교통 세 장면](../artifacts/validation/2026-11-02/official_traffic_v1/README.md), [실행 연결·점등·단독 1랩](../artifacts/validation/2026-11-02/official_facility_v1/README.md), [선두 신호·검출 영역·상향 후보](../artifacts/validation/2026-11-02/official_start_roi_v1/README.md), [원본·정적 대조](../artifacts/validation/2026-11-02/official_update/README.md) | [이전 10/24 적용](track/OFFICIAL_V2026_10_24_UPDATE.md). [과거 턱 기하](simulation/BUMP_GEOMETRY_2026_10_23.md), [9/14 재감사](track/OFFICIAL_V2026_09_14_REAUDIT.md), [ADR 0019](decisions/0019-official-v2026-09-14-gantry-markers.md), [초기 감사](track/TRACK_AUDIT.md) |
| 차량·구동 물리 | [현 감속비11.9118:1·81T 전달](../assets/hardware/procurement/2026-11-11/README.md), [현재 구동·검증](../artifacts/validation/2026-11-11/gearing_81t_r73_v1/README.md), [차량 동역학](simulation/VEHICLE_DYNAMICS.md) | [과거 65 mm/15:1 모델과 그때의 검증](hardware/DRIVETRAIN_GEARING_2026_10_02.md#9-시뮬레이션-반영-후속), [ADR 0026](decisions/0026-motor-shaft-encoder-baseline.md), [기존 ADR 0008](decisions/0008-single-motor-mechanical-differential.md), `src/arena_description/config/vehicle.yaml`. 이전 감속비·전류 검토는 [보류 메모와 PR 원본](archive/2026-10-01-drivetrain/README.md) |
| 구매품 인수·배치 | [11/8 구매 구성 변경·케이블/부속 대조](hardware/PROCUREMENT_AND_CABLES_2026_11_08.md), [현행 설계 전달 목록](HARDWARE_AND_RULE_QUESTIONS.md), [임시 차량 JR60P20](decisions/0029-temporary-jr60p20-vehicle.md), [확정 구매안·ESP32·인터페이스](hardware/ESP32_BUDGET_REVIEW_2026_10_02.md), [IMU 검토](hardware/IMU_SELECTION_2026_09_19.md) | [B0183/C1 선택](decisions/0020-b0183-c1-procurement-baseline.md), [과거 배치 비교](sensors/SENSOR_PLACEMENT_GEOMETRY.md), `hardware/`의 날짜별 가격 조사 |
| 규정·지원품·외부 질문 | [11/2 최신 공식 자료·최종 회의](reports/OFFICIAL_UPDATE_2026_11_02.md), [확인 질문·설계 전달](HARDWARE_AND_RULE_QUESTIONS.md) | [10/12 이력](reports/OFFICIAL_UPDATE_2026_10_12.md), [10/6 2차 회의](reports/OFFICIAL_UPDATE_2026_10_06.md), [이전 주행 규정 검토](autonomy/MULTI_VEHICLE_DESIGN_2026_10_04.md#2-규칙과-공간이-먼저-제한하는-것), [원격 STOP 과거 검토](hardware/EMERGENCY_STOP_AND_TELEMETRY_REVIEW_2026_09_18.md) |
| 기존 비교·경위 확인 | [기존 데모](autonomy/BASIC_DEMO.md), [스테레오/ToF 검토](sensors/STEREO_CAMERA_AND_MINIMAL_TOF.md) | [활동 기록](activity/), [정리 이전 보관본](archive/2026-09-27/README.md) |

## 기록 위치와 갱신 시점

- **현재 현황:** 실제 구성·완료 범위·문제만 교체하며 상세 시험표/중단 경위를 누적하지 않습니다. 사용자 결정의 이유·범위는 ADR, 앞으로의 순서는 `DEVELOPMENT_PLAN.md` 한 곳에서 갱신합니다.
- **이전 계획:** 날짜 있는 계획은 당시 전제·일정·실행 이력입니다. 현행 계획으로 연결하고 새 진행 상황을 계속 덧붙이지 않습니다.
- **기술 문서·실험 결과:** 조건·소스 버전·판정·실패·한계의 기준 문서 한 곳을 정합니다. 기존 주제 문서를 우선 갱신하고, 별도 주제나 재현 단위가 필요할 때만 새 문서를 만듭니다. 원시 로그/수치는 도구 산출물을 연결합니다. 다른 문서는 결론과 링크로 참조합니다.
- **ADR:** 되돌리는 비용이 큰 선택의 이유·대안·영향을 기록합니다. 단순 재시험·게시 확인에는 새 ADR을 만들지 않습니다. 대체 시에는 이전 결정을 지우지 않고 후속 결정 링크로 표시합니다.
- **README·AGENTS:** 실행 진입점/구조 또는 공통 규칙이 바뀔 때만 갱신합니다. 시험 개수·속도·매번의 게시 여부를 복사하지 않습니다.
- **Git/PR:** 커밋·변경 범위·게시 상태는 Git/PR을 기준으로 확인합니다. 확인 사실만을 기록하려고 추가 문서 커밋을 반복하지 않습니다.

## 활동 기록

활동한 날의 `activity/YYYY-MM-DD.md`에 아래 네 항목으로 마무리합니다. 보통 300~600자 정도를 목표로 하되 여러 중요한 이정표가 있으면 필요한 만큼 씁니다. 같은 날은 한 파일을 정리하고, 활동 없는 날에는 만들지 않습니다. 작업 중단 시 재개에 필요한 내용은 현황에 먼저 남깁니다.

- **활동:** 무엇을 구현·검토·결정했는가.
- **결과:** 실제로 확인한 성과. 실험하지 않았으면 문서/조사 작업이라고 구분.
- **한계·후속:** 실패·미검증·다음 확인 사항을 짧게 기록.
- **근거:** 관련 보고서·산출물·결정 또는 보관 원문 링크.

대기·승인 서비스 오류·중간 검사 개수·상세 명령은 재현이나 결과 해석에 꼭 필요한 경우에만 기준 보고서/원시 로그에 남깁니다. 실제 시도·실패를 숨기거나 결과를 성공으로 바꾸지 않습니다. 과거 일지는 당시 결과를 요약하며 후속 성공을 소급하지 않습니다.

## 보관 자료와 검증

[2026-09-27 보관본](archive/2026-09-27/README.md)은 정리 이전의 지침·현황·README·활동 원문을 보존합니다. 모든 과거 자료는 기본 읽기 목록에서 제외하고 관련 경위가 필요할 때만 확인합니다. 보관된 `AGENTS.before.md`는 현재 작업 지침이 아닙니다.

문서 변경에서는 링크/앵커, 사실·시점·성공/실패 구분, 원문 보존과 변경 범위를 대조합니다. 실행 코드·설정·원본·시험 증거가 바뀌지 않은 문서 정리 때문에 시뮬레이션을 재실행하지 않습니다. 실제 코드/모델을 바꾼 작업은 영향 범위에 필요한 회귀와 실행 검증을 별도로 수행합니다.
