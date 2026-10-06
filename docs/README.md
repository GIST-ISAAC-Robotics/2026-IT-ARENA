# 문서 안내

작업 시작점은 [현재 현황](PROJECT_CONTEXT.md)입니다. 필요한 행의 문서만 열고, 과거 날짜의 “현재/다음” 설명은 당시 이력으로 읽습니다. 이 색인은 작업 경로만 관리하며 시험 수치를 반복해서 옮기지 않습니다.

## 작업별 읽기 경로

| 작업 | 먼저 볼 문서 | 필요할 때 확인할 근거 |
|---|---|---|
| 전체 계획·다중 차량 | [10/4 현행 개발 계획](reports/DEVELOPMENT_PLAN_2026_10_04.md), [다중 차량 전략·설계](autonomy/MULTI_VEHICLE_DESIGN_2026_10_04.md), [G0 관측·기하 시험](autonomy/MULTI_VEHICLE_G0_2026_10_05.md) | [알고리즘 비교 이력](autonomy/ALGORITHM_OPTIONS.md), [9/22 이전 계획](reports/DEVELOPMENT_PLAN_2026_09_22.md) |
| 주행·지역 경로·보호층 | [지역 Pure Pursuit](autonomy/LOCAL_PURE_PURSUIT_2026_09_21.md) | [ADR 0022](decisions/0022-b0183-c1-local-pursuit-prototype.md), [알고리즘 선택](autonomy/ALGORITHM_OPTIONS.md) |
| LiDAR 운동/시간 보정 | [구현과 입력 계약](sensors/LIDAR_MOTION_IMPLEMENTATION.md) | [ADR 0018](decisions/0018-sensor-based-lidar-motion-compensation.md), [순차 취득 시험](sensors/C1_SEQUENTIAL_SPEED_EXPERIMENT.md) |
| 센서 잡음·누락·지연 | [C1 무응답·IMU 이상치 보호](autonomy/SENSOR_INPUT_GUARDS_2026_10_04.md), [모터축 엔코더·고착 1차 검증](autonomy/MOTOR_ENCODER_VALIDATION_2026_10_02.md), [LiDAR 내성](autonomy/LIDAR_ROBUSTNESS_2026_09_22.md), [기존 IMU·엔코더 내성](autonomy/MOTION_SENSOR_ROBUSTNESS_2026_09_22.md) | 각 보고서의 실행 설정·실패·JSON |
| Jetson–MCU 구동·정지 | [구동 계약](hardware/ACTUATION_CONTRACT_2026_09_22.md), [ROS 연결](hardware/ACTUATION_ROS_2026_09_22.md), [RGB/프로세스 중단·정지/복구](autonomy/PIPELINE_FAULT_VALIDATION_2026_10_04.md) | [ADR 0002](decisions/0002-jetson-esp32-actuation-boundary.md), [ADR 0023](decisions/0023-mcu-actuation-contract-reference.md) |
| 기록·재생·처리량 | [기록·재생/Jetson 통합 인수인계](reports/RECORD_REPLAY_JETSON_HANDOFF_2026_09_29.md) | [A2 PC 상세](simulation/RECORD_REPLAY_2026_09_24.md), [A3 Jetson 상세](simulation/JETSON_REPLAY_2026_09_28.md), [Jetson 접속](hardware/JETSON_ACCESS.md), [기록·재생 ADR 0024](decisions/0024-isolated-sensor-record-replay.md), [영상 분리 ADR 0025](decisions/0025-isolated-vision-processing.md), [개발 계획](reports/DEVELOPMENT_PLAN_2026_10_04.md) |
| 환경·GPU·종료 | [환경 ADR](decisions/0001-simulation-platform.md), [GPU 설정](simulation/GPU_RENDERING.md) | [GPU 부하 비교](simulation/GPU_FINAL_DIAGNOSTIC_2026_09_08.md), [종료 진단](simulation/SHUTDOWN_DIAGNOSIS_2026_09_15.md) |
| 공식 트랙·시설 변경 | [9/15 문서 적용](track/OFFICIAL_V2026_09_15_DOC_UPDATE.md), [9/14 재감사](track/OFFICIAL_V2026_09_14_REAUDIT.md) | [ADR 0019](decisions/0019-official-v2026-09-14-gantry-markers.md), `config/tracks/official_v2026.09.14.yaml`. 초기판 재현에는 [초기 감사](track/TRACK_AUDIT.md) |
| 차량·구동 물리 | [현행 모터축 엔코더·65 mm/15:1 반영](hardware/DRIVETRAIN_GEARING_2026_10_02.md#9-시뮬레이션-반영-후속), [차량 동역학](simulation/VEHICLE_DYNAMICS.md) | [ADR 0026](decisions/0026-motor-shaft-encoder-baseline.md), [기존 ADR 0008](decisions/0008-single-motor-mechanical-differential.md), `src/arena_description/config/vehicle.yaml`. 이전 감속비·전류 검토는 [보류 메모와 PR 원본](archive/2026-10-01-drivetrain/README.md) |
| 부품 구매·배치 | [확정 구매안·ESP32·인터페이스](hardware/ESP32_BUDGET_REVIEW_2026_10_02.md), [IMU 검토](hardware/IMU_SELECTION_2026_09_19.md) | [B0183/C1 선택](decisions/0020-b0183-c1-procurement-baseline.md), [배치 기하](sensors/SENSOR_PLACEMENT_GEOMETRY.md), `hardware/`의 날짜별 가격 조사 |
| 규정·지원품·외부 질문 | [10/6 2차 회의·공식 업데이트](reports/OFFICIAL_UPDATE_2026_10_06.md), [규정/지원품 기록](reports/OFFICIAL_UPDATE_2026_09_15.md), [10/4 주행 규정 재검토](autonomy/MULTI_VEHICLE_DESIGN_2026_10_04.md#2-규칙과-공간이-먼저-제한하는-것) | [확인 질문](HARDWARE_AND_RULE_QUESTIONS.md), [원격 STOP](hardware/EMERGENCY_STOP_AND_TELEMETRY_REVIEW_2026_09_18.md). 새 판정은 공식 최신 원문 대조 |
| 기존 비교·경위 확인 | [기존 데모](autonomy/BASIC_DEMO.md), [스테레오/ToF 검토](sensors/STEREO_CAMERA_AND_MINIMAL_TOF.md) | [활동 기록](activity/), [정리 이전 보관본](archive/2026-09-27/README.md) |

## 기록 위치와 갱신 시점

- **현재 현황:** 채택한 상태·문제·다음 작업이 바뀔 때 해당 항목을 교체합니다. 자세한 시험표/중단 경위를 누적하지 않습니다.
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
