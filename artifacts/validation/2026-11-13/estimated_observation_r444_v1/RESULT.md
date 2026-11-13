# r444 실물 추정 입력의 명시 observation scope — stage 결과

2026-11-13, GPT. [r443 설계](../real_observation_transfer_r443_v1/REVIEW.md)를 독립 stage에 구현하고 Windows/WSL의 제한 검증을 마쳤다. stage 작성·검증 당시에는 **공통 미이관·실물 기본 비활성**이었다. 이후 주 담당이 전체 diff와 검증 근거를 검토해17파일을 공통 이관했다([검토](root_review.json), [이관·SHA](integration.json)). 실물 기본 활성화·실제 실행 profile 선택은 하지 않았다. stage 검증 중에는 공통 소스, 기존 실행 입력, r439 stage를 변경하지 않았다. Jetson/실물·flash·Gazebo 실행은 없다.

## 변경과 선택 계약

새 명시 선택은 `g1_observation_input_scope=host_estimated_crop_v1`다. 기존 기본값은 `verified_native_simulation_v1`이며, crop/current 보완의 기본 OFF는 유지한다. 새 값은 `hardware_runtime_candidate_v1` + 실제 clock + `native_estimated_v1` + `both` + `host_estimated_v1` + **timing_verified=false** 조합에서만 허용된다. 두 계산 runtime와 기존 `g1_current_corner_budget=r396_v1` 선택도 필요하다. `hardware_shadow_v1`이나 `verified=true`, 시뮬 basis 혼합은 새 선택의 유효 조합이 아니다.

planner와 독립 guard 양쪽에 같은 scope를 명시하고 기존 `g1_crop_edge_component_trial`을 선택하는 진입점만 준비했다. 실제 선택 profile을 배포하거나 기존 r401/r415 값을 자동으로 바꾸지 않았다. FollowConfig 필드를 추가하지 않았으며, 기존 시뮬 header의 구조·현재 crop의0.2→조건부0.1 동작을 보존했다. estimated 선택에만 optional header/진단을 추가하고 실제 false/시간 정책·원 basis를 남긴다.

추가된 실물 현재 후보는 **원 현재 판단0·기존 관측 보완 분기·실측0..0.15 m/s**에 한정한다. crop는0.2 시도 없이 단일0.1 목표를 core/현재 전체 영역/proof **전에** 넣는다. corner 거리 제안도 `min(기존 제안, 0.1)`을 최종 전체 검사 전에 적용한다. 기존 정상 양수 결과와 일반 속도 상한은 바꾸지 않는다. 음수 측정속도를0이나 절댓값으로 바꾸지 않는다.

같은 원 scan 전체 frozen 배열의 부모 비교, source/motion epoch, frame·각도·점시각·source index, 유한 반환 장애물, positive-inf/NaN 구분, 현재1cm·6도·5cm 조건과 `/drive` → 독립 guard → `/drive/safe`를 유지한다. 임의의 시각 정확도·연속 충돌 부재·사각 내부 자유 공간 보증은 추가하지 않았다. MCU/host/STOP/기존 만료·장착값도 변경하지 않았다.

## 파일 경계

[변경 목록·원본/최종 SHA](changes.json)와 [적용 diff](apply.patch)가 이관 기준이다. **소스15개+시험2개=17개**이며, 공통의 기존16개 원본 SHA가 전후 동일하고 신규 시험1개는 아직 공통에 없다. `git apply --check`를 통과했다.

| 묶음 | stage 수정 |
|---|---|
| 명시 선택과 작은 공통 검사 | `crop_edge_selection.py` |
| 두 ROS 노드·header 전달 | `local_pursuit.py`, `lidar_safety.py`, `g1_g2_node.py`, `runtime_node_backend.py`, `follow_trace.py` |
| 계획기/guard 부분관측 선택 보존 | `decision_process.py`, `partial_observation.py` |
| 현재 crop/corner·proof 전달 | `race_runtime_policy/evaluator.py`, `crop_current_scope.py`, `crop_current_proof.py`, `crop_current_candidate.py`, `corner_candidate.py`, `profile_evidence.py`, `range_mask_component.py` |
| 시험 | 신규 `tests/test_estimated_observation_scope.py`, 기존 `tests/test_current_corner_runtime.py`의 double에 `core_policy` 한 줄 보완 |

위 소스는 모두 `stage/src/arena_autonomy/arena_autonomy/` 아래에 있다. `runtime_dual_state.py`의 부모 same-scan 전달과 기존 r396 enum은 수정하지 않았다. 공통 문서·현황·활동일지 통합은 주 담당이 맡는다.

## 검증 결과

| 확인 | 결과·근거 |
|---|---|
| Windows 순수 검사 | 최종 **14개 통과**, 실패/skip0. [최종 로그](verify_windows_v2/unittest.log). 선택 조합, 원천 비교, 실제 합성 crop/profile geometry, 유한 장애물/NaN/-inf, 단일0.1의 검사 전 전달, 정상 양수·범위 밖 실측 보존. Windows evaluator 생성자 검사는 native configure/controller를 mock한 계약 확인이며 native 실행으로 취급하지 않는다. |
| WSL 관련 회귀 | **69개 통과**, 실패/skip0. 신규14 + 기존 corner/estimated 시간 검사. [로그](verify_native_v2/units.log), [JUnit](verify_native_v2/units.xml). Linux native binary를 기존 그대로 사용했고 새 빌드는 하지 않았다. |
| 기존 시뮬 전체 Evaluator | r437 96개+r426 313개=**409개가 r440 보존 결과와 전체 exact**. 원 입력 변이0, 외부 project import0. [보고서](verify_native_v2/saved_result/report.json), [건별 hash 대조](verify_native_v2/saved_result/simulation.jsonl). |
| estimated 제한 대표 | **13개 완료**: 단일0.1 수락, 핵심 반례5개, verified 위장 거절, 기존 양수0.15 exact, 새 plan 두 시간 계약의 수치 동일, 가정 민감도4개. 아래 표 참조. |
| 실제 ROS 생성자 | r401 실제 YAML을 읽고 신규 scope/crop/r396/B3를 명시한 **G1G2Node+LidarSafety** 생성 확인. 양쪽 false/host_estimated, planner·guard 부분관측 scope, native 두 자식 준비 확인. spin/센서입력/장치0. 생성·종료에서 관찰된 `/drive`는0명령3개, 양수0. [보고서](verify_nodes_v1/report.json). |
| 종료·무결성 | 두 계산 자식+vision **3개 모두 exit0**, forced0·소유잔존0. native 검증238개 및 노드 추가 입력3개 불변. [native receipt](verify_native_v2/receipt.json), [노드 receipt](verify_nodes_v1/receipt.json). WSL 소유권은 주 담당에게 반환했다. |

최종 native 묶음은 `validation_sources_v2.zip`, SHA256 `9deac9fdff3e7a3e9332108132eecb2feefe01c8f7df17252629ff9b480279fa`이며 [238개 manifest](manifest_v2.json)를 함께 보존한다. 실행은 전용 `/home/jinhyeong/it-arena-r444-verify-b`에서 수행했다. [execution.json](verify_native_v2/execution.json)에 명령·PID·exit·reap이 있다. unit1.62초/저장 비교19.62초는 이 오프라인 실행의 소요시간이며 Jetson/ROS 주행 마감 성능 주장이 아니다.

첫 `verify_native_v1`은 **62통과/7실패**로 종료됐다. 여섯 건은 묶음에 기존 `test_current_path_recheck.py` 및 r168 fixture를 누락한 것이며, 한 건은 기존 corner 시험 double이 r426 이후 evaluator의 `core_policy`를 제공하지 않은 문제였다. 공통 원 evaluator에도 같은 참조가 있다. fixture와 직접 관련 double을 stage에 보완했으며 [원 실패](verify_native_v1/units.log), [원 회수](verify_native_v1/receipt.json), 원 v1 zip/manifest를 보존했다. 첫 실행은 저장 비교를 시작하지 않았다.

## estimated 대표의 정확한 범위

동일 수치의 **저장 시뮬 입력 사본**에 명시 estimated 계약을 부여한 비교다. 실제 r404 C1/IMU의 없는 wheel값을 합성하거나 실물 관측이라고 표기하지 않았다. 원 저장 파일은 변경하지 않았다.

| 파생 입력 | 전체 native Evaluator 결과 |
|---|---|
| 원 r437 seq643 수치·false/estimated 계약 |0.1 m/s, 목표 시도1회/현재 전체 검사1회, crop proof 수락 |
| 전방 유한 반환0.12m |0, `rebase_current_gap_hold` |
| crop 밖 측방 유한 반환0.12m |0, `rebase_current_gap_hold` |
| 원 crop flank를+Inf로 바꿔 성분 확대 |0, `wide_nonreturn` |
| current motion epoch 불일치 |0, `rebase_epoch_or_side_changed` |
| current 원 scan만 변경 |0, `rebase_current_gap_hold`; 부모 full-scan equality도 false |
| false를true로 위장 |계산 전 `ValueError`, 추가 crop 호출0 |
| 기존 현재 baseline 양수 대표 |0.15 m/s 및 전체 결과 exact,0.1로 감속하지 않음 |
| 깨끗한 plan 인스턴스 두 계약 |component scope 표기 외 전체 수치 결과 동일 |
| scan 상대 시각±10ms, LiDAR yaw±0.5° 각1개 |4개 모두0.1 단일 현재 검사 수락. raw wheel/gyro값은 변경하지 않음 |

마지막 네 수치는 **가정 민감도**다. C1/IMU 절대 시각 오차나 실제 장착 오차의 허용 상한이 아니며, 다른 장면/동적 상대/회전속도에서도 성립한다는 보증은 없다. 상세 [13대표](verify_native_v2/saved_result/cases.json)는 로컬 보존한다. 실제 corner 거리 보완의0.1 전달은 관련 함수 계약·geometry/profile 검사를 포함했으며, 이번 저장 positive 대표의 새 수락은 crop 경로였다. 실차 corner 통과를 확인한 결과로 확대하지 않는다.

## 현재 이관 판단에 남긴 한계

- r441은20.09m까지 진행했지만 crop/crawl 호출·발행0, 기존 corner 거리 보완2수락/2발행이었다는 주 담당의 후속 감사를 따른다. live의0.1→재가속을 r440의 실제 효과로 확정하지 않는다. r444는 **입력 계약 연결 검증**이며 그 주행 효과를 증명한 이관이 아니다.
- 원 r404는 실제 C1/IMU 수신과0명령을 확인했지만 wheel0이다. 이번 저장 비교·무입력 ROS 생성자를 실제 차량의 운동 보정/양수 출력 검증으로 합산하지 않는다.
- 실물 scope의 명시 선택·공통 이관은20.09m 정체 분석과 stage 검토 후 주 담당이 정한다. 새 사용자 반복 승인을 전제로 작업을 중단한 상태가 아니다. 실장 profile/영점·축/자체 가림·실제 출력과 제한 저속 확인은 기존 조립 뒤 순서에 남는다.
