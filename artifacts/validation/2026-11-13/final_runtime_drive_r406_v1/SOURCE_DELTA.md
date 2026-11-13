# r377 이후 공통 변경 검토 범위

기준은 r377 `registered.json.input_sha256`와 `sources.zip`이다. 현재 공통 source와 r405/r406 staged의 파일별 hash는 `source_review_draft_v1.json`에 남긴다. 이 문서는 설명이며 그 JSON의 개별 경로·hash 검토를 대신하는 허용 목록이 아니다. 실제 이관 뒤 inventory가 다르면 새 판본으로 다시 비교한다. 모든 옛 보고서/정책 계보를 재실행하지 않는다.

| 변경 범위 | 의미와 이번 실행 관계 |
|---|---|
| `pass_release_trial.py`, `test_shared_ray_classification.py` | r379 depth/masked 공유. 동일성 근거가 있는 공통 최적화이며 source가 r377과 바이트 동일하다고 표시하지 않음 |
| `local_follow.py`, `local_pursuit.py`, `m3_execution_trial.py`, `m3_observed_follow.py`, `pass_release_trial.py`, `vehicle_geometry.py`, `simulation.launch.py`, 기하 검사 | r388/r399 최종 윤거·타이어/float32 조향 상한 전달 및 공통 geometry helper. 이번 기하 경로에 직접 연결됨 |
| `execution_environment.py`, `wall_follow.py`, `lidar_safety.py`, `lidar_motion_ros.py`, hardware preview scripts와 관련 검사 | r395 실제시각 운영 환경 분리. 이번은 기존 verified simulation을 명시하며 물리 입력 경로로 바꾸지 않음 |
| `race_runtime_policy/`, `runtime_node_backend.py`, `runtime_dual_process.py`, `g1_g2_node.py`, 관련 검사 | r398/r400 정책 패키지·처음부터 두 자식·부모 발행 확인·종료 연결. r377의 동적 artifact 정책 진입점을 대체 |
| `lidar_source_frame.py`, `lidar_motion.py`, `lidar_motion_ros.py`, `scene_motion.py`, `corridor_observer.py`, `linear_observation.py`, `observation_support.py`, frame 검사 | r403 원 C1 frame 명시 계약. 이번 frame은 `laser_frame` 기본을 유지하며 시각 verified 승격 없음 |
| r405의 `g1_g2_node.py`, `runtime_node_backend.py`, `follow_trace.py`, evaluator와 corner/component/profile 모듈 | 명시 simulation 전용 `r396_v1` 후보, header/receipt. root 저장223 비교와 공통 이관 완료를 선행 조건으로 둠 |
| r406 `simulation.launch.py` | 두 launch argument와 controller parameter 연결만. 기본 둘 다 off |
| `actuation_contract.py`, `mcu_speed_loop.py`, portable PI/receiver/endpoint, runtime servo/MD20A/HAL 파일과 검사 | r381/r384/r387의 출력 의미·준비/STOP/만료 연결. 시뮬의 기존 PI 기본범위/receiver 동작 보존은 기존 회귀 근거를 사용하되 이 파일들의 변경을 단순 삭제/무시하지 않음. 실제 HAL 앱은 이 launch에서 실행하지 않음 |
| `actuation_host_session.py`, receiver-owned host/offer/clock/link/runner, `motor_feedback_runtime.py`, 관련 검사 | r391~r397 운영 host·실제 count 입력. 이번은 가상 MCU이고 이 실제 serial host를 시작하지 않음 |
| C1/camera bridge, `sensor_stream_selection.py`, 관련 검사 | r402 명시 domain/topic. 이번 simulator producer 경로에는 실제 장치 bridge를 시작하지 않음 |

새 snapshot에 `.so`가 포함되면 실제 native extension 바이트도 동결한다. 원 r377 source 목록에 없던 추가 파일은 `added_to_snapshot`으로 별도 표시한다. 명칭만으로 새 구현 시점이나 실행 여부를 단정하지 않는다. 기하/정책/계산 연결 변경과 모터·host 비실행 준비 변경을 구분하되 모두 목록에 남긴다.

판정 코어 `validate_g1_traffic.py` 및 기존 평가 함수/설정은 별도의 판정 완화 대상이 아니다. source 목록에서 이 파일들의 새 차이가 발견되면 해당 diff를 확인하기 전 동결하지 않는다. 마커90mm/4가림, 최종 기하 전달, runtime 연결, r405 후보의 복합 결과이므로 주행 개선을 한 파일의 단독 효과라고 해석하지 않는다.
