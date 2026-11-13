# r443 추정 시각 실물 입력으로 crop/corner 보완을 옮기는 최소 경계

2026-11-13, GPT. [r433 사전 검토](../real_crop_transfer_r433_v1/REVIEW.md)의 후속 **Windows 읽기·설계 검토**다. r441 실행 중 공통 소스와 r439 stage는 수정하지 않았다. WSL/ROS/Jetson/장치·주행·새 재생 검사는 수행하지 않았다. 아래 선택 이름은 제안이며 현재 구현·선택·실물 활성화된 값이 아니다. r441 결과를 받기 전 후보의 실물 채택을 결정하지 않는다.

## 결론

실물 시간 입력 경로를 새로 만들 필요는 없다. 현재 코드에는 `hardware_runtime_candidate_v1` → `native_estimated_v1` → `host_estimated_v1`/`timing_verified=false` → 운동 보정·기본 현재 재검사가 이미 연결돼 있다. **추가 crop/corner의 시뮬 전용 선택·증거 검사만 분리하여 명시 estimated 선택을 통과시켜야 한다.** 추정 입력을 `simulation_acquisition`이나 `verified=true`로 바꾸는 우회는 필요하지도, 적절하지도 않다.

실물 후보는 **기존 현재 판단이0이고 관측 보완 분기에 들어온 경우에만0.1 m/s 이하의 단일 추가 목표를 전체 검사 전에 생성**하고, 같은 원 scan·epoch·현재 운동·유한 반환 장애물·독립 guard를 그대로 통과시키는 범위로 준비한다. 기존 정상 양수 판단 전체의 상한을0.1로 바꾸는 설계가 아니다. 현재 시뮬 crop는 먼저0.2를 검사한 뒤 특정 후측방 가림에서만0.1을 다시 검사하므로, 마지막 발행값만0.1로 clip하는 것으로는 이 요구를 충족하지 않는다. 이 이관은 시각 정확도·실차 진행 성공·사각 내부 물체 부재의 증명이 아니다.

## 1. 실물에서 이미 연결된 부분과 확인 범위

- [execution_environment.py](../../../../src/arena_autonomy/arena_autonomy/execution_environment.py):6–41은 실물 환경에서 실제 ROS clock, `both`, 추정 정책, false 표기를 함께 요구한다. [native_lidar_input.py](../../../../src/arena_autonomy/arena_autonomy/native_lidar_input.py):8–28,37–63은 선택한 basis와 메시지가 일치해야 하며, 원 ranges/angles/점별 시각/frame/source epoch를 `MeasurementScan`으로 받는다. 메시지 자체가 검증 승인을 만들지 않는다.
- [scene_motion.py](../../../../src/arena_autonomy/arena_autonomy/scene_motion.py):32–64는 추정 basis를 유지하며 실제 모터/gyro 이력으로 점별 운동을 보정한다. 미래·원천 나이0.3초·frame·운동 epoch·이력 조건은 유지된다. [retained_recheck/_geometry.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/retained_recheck/_geometry.py):37–75도 명시 추정 정책을 이미 받으며, 경로/센서/운동 epoch와 현재 실제 반환·제동을 검사한다. [runtime_node_backend.py](../../../../src/arena_autonomy/arena_autonomy/runtime_node_backend.py):80 이후는 모터 표본이 없을 때0속도를 합성하지 않는다.
- [r401 선택 YAML](../runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml):4–9,128–133은 **planner와 guard 모두** 위 추정 조합이다. 122/40개 매개변수와 실제 두 ROS 생성자 대조가 완료됐지만 센서/양수 명령을 생성한 시험은 아니다([r401 결과](../runtime_launch_profile_r401_v1/RESULT.md)). 새 crop/corner 선택은 이 기존 결과에 포함되지 않는다.
- [r404 실제 비구동 결과](../runtime_live_r404_v1/RESULT.md)는 C1 183개/93,224점, 양쪽 gyro 3,795개, `/drive` 1,314개와 `/drive/safe` 1,313개의 **0명령**을 확인했다. 원 [node summary](../runtime_live_r404_v1/remote_result/extracted/actual/node/summary.json)의 wheels/drive_motor/motor_samples는 모두0이다. 따라서 실제 시간 정책 수신은 확인됐지만 양수 전체 계산·실제 차량 운동 보정의 정확도·구동 처리율은 확인되지 않았다. 당시 frame은 `c1_unmounted_preview`다.
- [lidar_safety.py](../../../../src/arena_autonomy/arena_autonomy/lidar_safety.py):178–203,233 이후는 독립 부분관측 검사·속도 제한·반환점 swept 검사를 수행하고 `/drive/safe`를 발행한다. 이 경로를 유지한다. [occupancy_timing.py](../../../../src/arena_autonomy/arena_autonomy/occupancy_timing.py):6–21의 고정 여유+점별 나이×물체 이동 상한은 이미 존재하며, 취득 추정 오차/회전 오차의 별도 보증항은 아니다.

## 2. 현재 시뮬 전용이라 막힌 필수 변경 5묶음

| 경계 | 현재 거절/전달 조건 | 최소 변경 대상 |
|---|---|---|
| ① 양쪽 ROS crop 선택 | [crop_edge_selection.py](../../../../src/arena_autonomy/arena_autonomy/crop_edge_selection.py):11–20이 sim clock·nonhardware·verified·verified_only·native simulation만 허용. 호출은 [local_pursuit.py](../../../../src/arena_autonomy/arena_autonomy/local_pursuit.py):92–97와 [lidar_safety.py](../../../../src/arena_autonomy/arena_autonomy/lidar_safety.py):65–68 | 두 노드에 동일한 **명시 estimated scope**를 연결한다. 기존 bool을 모든 실물 입력에 무조건 허용하는 식으로 풀지 않는다. |
| ② current corner/runtime의 선택과 header | [runtime_node_backend.py](../../../../src/arena_autonomy/arena_autonomy/runtime_node_backend.py):8–29는 verified simulation만 허용하고 header에 `use_sim_time=True`, `motion_timing_verified=True`를 직접 넣는다. [evaluator.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/evaluator.py):24–29,66–67에서 다시 검사. [g1_g2_node.py](../../../../src/arena_autonomy/arena_autonomy/g1_g2_node.py):95–97이 진입점 | 같은 scope와 실제 실행 정보로 header를 만든다. worker가 독립적으로 정확한 조합을 재검사한다. `current_corner_budget='r396_v1'` 자체는 유지할 수 있다. 새 corner enum을 추가한다면 node:251, [follow_trace.py](../../../../src/arena_autonomy/arena_autonomy/follow_trace.py):102, [runtime_dual_state.py](../../../../src/arena_autonomy/arena_autonomy/runtime_dual_state.py):72–73도 함께 고쳐야 하므로 아래 설계는 그 확산을 피한다. |
| ③ plan/guard의 crop 성분 생성 | [partial_observation.py](../../../../src/arena_autonomy/arena_autonomy/partial_observation.py):273–301이 `simulation_acquisition`+`verified_only`만 허용하고 진단 scope도 simulation으로 고정 | 이미 선택 검사를 거친 scope를 `PartialObservationPolicy`까지 전달한다. 기존 전체 성분/양 flank/금지 장착 구간/6도·5%/중앙 제외/근접 반환 조건을 그대로 적용하고 estimated 진단 scope를 붙인다. 이 진입 검사 변경을 정상 양수 출력의 새 전역0.1 제한으로 사용하지 않는다. |
| ④ 같은 원 scan의 현재 crop proof | [crop_current_scope.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/crop_current_scope.py):4,30–48은 simulation scope·submitted/current verified true를 요구한다. [crop_current_proof.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/crop_current_proof.py):34는 raw simulation basis만 허용. [crop_current_candidate.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/crop_current_candidate.py):12–38,64는0.2→조건부0.1과 simulation 진단 | 선택 scope에 맞는 **false/host_estimated/unverified_estimate** 조합을 추가한다. 부모의 frozen 원 배열 전체 비교를 재사용하고 원 epoch·frame·각도·점시각·source index 검사를 삭제하지 않는다. 실물 후보는 첫 target부터0.1로 전체 core/region/proof를 검사한다. |
| ⑤ corner 부분 장착 범위의 증거 | [profile_evidence.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/profile_evidence.py):9–15가 false 또는 estimated raw를 거절한다. [range_mask_component.py](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/range_mask_component.py):64–70에서 호출 | 선택 scope/시간 정책을 명시 전달하여 원 profile·거리 구간·mask/return 상태·epoch·hit 재구성 일치를 그대로 검사한다. 시간의 검증 여부와 배열/장착 profile의 내부 일치를 별개로 기록한다. |

`source_valid()` 자체는 시뮬 전용 문턱이 아니다. [observation_support.py](../../../../src/arena_autonomy/arena_autonomy/observation_support.py):142–164의 등록 frame·0.3초 나이·CW/CCW 순서·각도 간격·0.08~0.12초 원 회전 폭은 그대로 사용할 수 있다. 이 조건을 통과했다고 추정 시각의 절대 오차가 검증되는 것은 아니다.

## 3. 권고하는 최소 설계

**새 timing framework 대신 명시 scope 값 하나**를 양쪽 노드에 둔다. 예: `g1_observation_input_scope=host_estimated_crop_v1`(제안 이름). 기존 crop/current가 꺼져 있으면 현행 동작을 유지하며, 기존 시뮬 scope가 기본이다. 예상 조합은 다음과 같다.

| 선택 | 환경/clock | 원 표현·basis | 운동 시각 계약 |
|---|---|---|---|
| 기존 시뮬 | 기존 simulation 구성·`use_sim_time=true` | `native_simulation_v1` / `simulation_acquisition` | `both`, `verified_only`, true |
| 명시 실물 후보 | `hardware_runtime_candidate_v1`·`use_sim_time=false` | `native_estimated_v1` / `unverified_estimate` | `both`, `host_estimated_v1`, **false** |

`hardware_shadow_v1`을 출력 후보 승인으로 취급하거나 basis만 보고 자동 선택하지 않는다. 위 작은 조합 검사를 기존 선택 모듈에서 재사용하고, node → FollowConfig/부분관측 정책 → runtime header → evaluator/proof로 전달한다. 기존 `r396_v1`과 기본 OFF는 보존한다. trace/header에 scope·실제 환경·원 basis·false를 기록하고 저장 비교에서도 원 시뮬 결과를 실물 결과로 소급하지 않는다. r401/r415 기본 파일을 조용히 바꾸지 말고 후속 선택 profile에서 planner/guard 두 노드에 명시한다.

실물 scope에서 **원 현재 결과가0인 관측 보완 분기의 추가 후보만 최대0.1**로 제한한다. crop는0.2 시도 없이 최초이자 단일 target에0.1을 넣고, corner 거리 제안은 `min(기존 제안, 0.1)`을 최종 현재 검사 **전에** 넣는다. 현재 측정속도·직전 실제 요청각·원 경로는 바꾸지 않으며 core/제동/전체 현재 이동영역/반환 장애물을 다시 계산한다. 기존 정상 양수 판단과 그 속도 상한을 그대로 보존하고, planner/guard 전체를0.1로 제한하지 않는다. 실제 첫 운영의 개발 속도 상한을 별도로 선택하는 절차와도 구분한다.

부모가 이미 만드는 [current_request](../../../../src/arena_autonomy/arena_autonomy/runtime_dual_state.py):12–24의 전체 frozen scan equality를 유지한다. source epoch뿐 아니라 motion epoch도 기존 core의 제출/현재 검사에서 일치해야 한다. `same_scan=true` 표식만 외부에서 만들거나, 과거 proof·점유 허가·timestamp를 갱신해 재사용하지 않는다. partial/profile 진단의 `scope`만 실물 선택을 정직하게 나타내도록 바꾸고 `unknown_filled=false`, `residual_hidden_object_risk=true`, `continuous_sweep_certified=false`는 그대로 둔다.

[rear_fringe_trial._bounded_nonreturn](../../../../src/arena_autonomy/arena_autonomy/rear_fringe_trial.py):51 이후의 모든 유한 반환·padding·near return·raw/scene 대응과 positive-inf/NaN 구분, 현재 crop의 전체6도/5cm 합산, 독립 guard의 자체 현재 scan 검사는 보존한다. 실물용 전역 proof 교체나 `/drive/safe` 직접 발행은 필요하지 않다. MCU/host·ARM/STOP·기존 만료 시간에도 이번 이관이 부여하는 새 권한은 없다.

**범위 밖 잔여:** 현재 crop/corner 진입은 측정속도 음수를 제외한다(`crop_current_candidate.py`:54, `corner_candidate.py`:23). 시간 정책 이관만으로 음수 축속도 복구가 새 후보에서도 해결됐다고 쓰면 안 된다. 음수 표본을0 또는 절댓값으로 바꾸지 않으며, 명시된 새 후보가 적용되지 않을 때 기존 판단을 보존하는지를 반례로 남긴다. 이 문서는 별도 signed-motion 정책 변경을 제안하지 않는다.

## 4. 지금 저장 입력으로 준비 가능한 비교와 실물에 남는 것

다음은 **후속 최소 검증 설계이며 이번에 실행한 시험이 아니다.** 원 자료는 불변으로 두고 파생 입력/결과에 변경 가정과 SHA를 남긴다.

| 대표 | 저장 입력과 대조 | 주장할 수 있는 범위 |
|---|---|---|
| 시각 선택만의 대조 | [r438 inputs](../crop_masked_recovery_r438_v1/inputs/)의 r437 96개와 r426 313개는 `plan_input/plan_output/current`를 보존한다. 같은 수치·같은0.1 검사에 대해 simulation 계약과 파생 estimated 계약을 비교하고 false/basis/scope만 달라지는지 확인 | **시뮬 숫자에 대한 계약/로직 비교**. 실제 센서나 시각 정확도 증거 아님. r440은 원 조건409개에서72개 개선/337개 기존 출력 보존을 이미 확인했다([원 보고서](../crop_current_crawl_r440_v1/verify_v1/saved_result/report.json)); 새 실물 선택에서도 동일하다는 결과는 아직 없다. |
| 실제 C1 수신 형식·나이 | r404 [converted.jsonl](../runtime_live_r404_v1/remote_result/extracted/actual/c1/converted.jsonl)에 payload 전체와 receipt/ready/publication 시각·diagnostic이 있고 [scans.json](../runtime_live_r404_v1/remote_result/extracted/actual/node/scans.json)에 노드 수신 요약이 있다. [IMU frames](../runtime_live_r404_v1/remote_result/extracted/actual/imu/frames.jsonl)도 보존됨 | 원 배열/단조 시각/frame/epoch/유실·수신 나이 검사 가능. **엔코더가0개이므로 실제 전체 motion replay는 불가**. 없는 운동을0으로 합성해 실제 주행 비교라고 쓰지 않는다. |
| 제한 시각/회전 민감도 | 위 저장 current의 완전한 운동 이력이 있는 경우만, 파생 사본에 scan 대 운동 이력의 상대 offset ±5/10/20ms, 작은 yaw/gyro 가정 오차를 한 축씩 적용. 이후 필요할 때 한 조합만 비교. current 시각을 뒤로 미는 지연 비교는 실제 이력 coverage 안에서만 시행 | 숫자는 **가정한 민감도**, 실측 허용오차가 아니다. motion transform·나이 padding·폭·최종0/양수·거절 이유 차이를 기록한다. 미래/만료/이력부족은 원 거절을 유지한다. 실측 보정을 덧붙이는 작업은 아니다. |
| 필수 반례 | 선택 혼합(sim clock+estimated, verified true+basis estimate), 같은 첫 시각/epoch지만 ranges 또는 각도 1개가 다른 scan, sensor/motion epoch 변경, 근접 유한 반환 추가, NaN을 positive-inf로 오인하는 경우, 다른 전체 성분·중앙/넓은 성분, 음수 측정속도 | 선택·동일성 거절, 실제 장애물/전체 성분 검사 유지, 기존 출력 보존을 확인. 새로운 큰 반례 탐색이나 전역 안전층은 만들지 않는다. 기존 [estimated 시간 검사](../../../../tests/test_estimated_motion_time.py), [current corner 선택 검사](../../../../tests/test_current_corner_runtime.py), [r440 단일 저속 검사](../crop_current_crawl_r440_v1/tests/test_crawl_evaluator.py)를 확장하는 정도면 된다. |

실제로 사람이 해야 하는 일은 기존 조립 뒤 최소 목록에 남긴다: 장착 C1 축/위치·자체 가림 범위, IMU 축과 실제 엔코더 부호·속도, 실제 조향 중립/상한, 제한 공간에서0.1 후보의 진행·STOP·guard/MCU 수신을 확인한다. r404의 미장착 배열만으로 최종 배선/고정물 그림자와 진동 여유를 검증할 수 없다. 센서를 움직여야 구분되는 시각/축 오차도 저장 자료로 절대 정확도 보증을 만들지 않는다. 이를 위해 새 정밀 시각 계측 장치나 광범위 센서 재시험을 선행조건으로 추가하지 않는다.

[C1SampleClock](../../../../src/arena_vehicle_interface/arena_vehicle_interface/c1_sample_clock.py):20–21,123–125의10ms anchor residual/5ms publication wait는 내부 일관성·대기 예산이다. r404 첫/마지막 점 나이 p95 129.83/29.87ms도 추정 clock상의 나이일 뿐이다. 이 수치를 위 민감도 범위의 실측 보증으로 바꿔 적지 않는다.

## 인계

현재 산출물은 이 설계 검토 한 파일이다. 권고 다음 순서는 r441의 실제 대표 결과 검토 → 필요 범위만 위5묶음의 독립 stage 구현·저장 비교 → 실제 노드의 선택/미구동 경계 확인 → 조립 뒤 기존 최소 저속 확인이다. r441이 실패하면 실패 원인과 무관한 timing gate 완화로 실물 후보를 먼저 켜지 않는다. 현황/활동일지·공통 이관·실행 자원 배정은 주 담당이 관리한다.
