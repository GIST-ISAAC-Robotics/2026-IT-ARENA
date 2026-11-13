# r482 실물 추정시각 경로의 코드 대조

2026-11-14, GPT. 아래는 실제 파일의 연결 관계를 확인한 결과이며 실물 입력·장치·ROS 실행 시험이 아니다.

**실물 명시 진입도 같은 `CurrentLaneSelector`와 `fresh_parallel_speed_trial.proposal`을 사용한다.** 다만 이 후보가 비용을 줄이는 조건은 동일 proposal에서 `measured_region`이 이미 계산된 경우뿐이다. 해당 생산부는 `speed > config.max_speed_mps`일 때 실행되므로 모든 실물 판단이 빨라진다는 뜻이 아니다.

| 확인한 연결 | 원문 근거 |
|---|---|
| r444/r446 실물 진입은 G1G2Node 및 native 두 계산 runtime을 거친다. 일반 LocalPursuit 직접 estimated 선택은 거절한다. | `src/arena_autonomy/arena_autonomy/local_pursuit.py:97`, `runtime_node_backend.py:15`, r444 `verify_nodes.py:23` 및 r446 RESULT |
| 이 진입에서 사용하는 r401/r407 profile은 current-lane 선택을 true로 설정하고, `host_estimated_v1` / `motion_scan_timing_verified: false`를 유지한다. | `artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml:8,39`, `runtime_ops_r407_v1/prepared/runtime_profile.yaml:8,39`; r439는 생성 params-file 사용, r453은 estimated scope override를 별도로 연결 |
| G1G2Node가 같은 FollowConfig로 전달한다. | `g1_g2_node.py:306`의 current_lane 파라미터 → config replace → BypassHandoffFollow |
| runtime evaluator도 같은 제어기를 만든다. plan 호출의 시간 정책 일치 검사는 앞에 유지된다. | `runtime_node_backend.py:42`, `race_runtime_policy/evaluator.py:18,90`, `decision_process.py:15` |
| 정상 주행 제어기는 같은 selector를 선택하고 호출한다. | `local_follow.py:535`의 CurrentLaneSelector 선택, `:601`의 pass_selector.select |
| 실제/시뮬 원시 시간 계약은 scene 생성에서 구분한다. 추정시각을 true로 바꾸지 않는다. | `scene_motion.py:31`의 `_measurement_transforms`; `unverified_estimate`와 `host_estimated_v1`의 일치 검증 |
| 중복 제거 대상 함수에는 시뮬/실물별 별도 구현이 없다. | `current_lane_preference.py:91` → `fresh_parallel_speed_trial.py:104–117`의 measured-region 생산 → 동일 호출의 최종 target/steering 재사용 |

원 profile의 `max_speed_mps=1.5`는 `local_pursuit.py:179`에서 FollowConfig의 `min(1., settings max_speed)`를 거쳐 1.0이 된다. 따라서 이번 r476에서 이득을 본 세 행의 측정속도 1.5306/1.1633/1.0408m/s는 이 분기를 밟았다. 실물 운영 예시의 개발 상한 0.3m/s는 별도 출력 상한이며, 실제 측정속도가 1.0m/s 이하인 상태에서는 이 재사용이 활성화되지 않는다.

실물 부하에서 같은 약20%의 개선을 측정한 것은 아니다. 현재 r482 입력은 전부 r476의 원 시뮬 저장 자료이며, hardware timing 표식을 바꾸거나 실물 합성 입력을 만들지 않았다. r444/r446 계약의 실제 장착·시각 정확도·센서 동시 부하·주행은 이 결과의 검증 범위에 포함되지 않는다.
