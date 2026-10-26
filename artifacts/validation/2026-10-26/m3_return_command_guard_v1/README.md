# 10/26 복귀 출력 보호와 원문 순환 계약

GPT가 설계·구현·분석을 직접 맡고 GPT-6 Luna가 고정 시험 실행·대기·회수만 맡는다. 현재 작업은 **운영 미연결 진단·선택형 순수 제어 시제품**이며 새 Gazebo 주행·센서 배치 변경·보호 예산 완화는 없다.

## 질문과 수용 조건

1. 이전 시제품의 원문 순환 계약을 공통 함수에 명시 `cycle=None` 선택형으로 옮겼을 때 기본 출력은 그대로인가? 기존 배열 끝/첫점이라는 이유만의 거절만 없애며 시간·반환·각도·거리 여유를 유지한다. 생성 시 원문 증거를 보존하며 노드는 증거를 자동 생성하지 않는다.
2. 기존 M3가 기본 G1으로 넘기는 출력도 실제 목표 속도/조향·직전 조향·한 주기+반응+제동 외곽을 검사했을 때 지지되는가? 원래 경로를 다시 추종해 다른 출력으로 바꿔 검사하지 않는다. 출력이 0이어도 측정 속도가 남으면 제동 구간을 보존하며 STOP 자체는 이 진단 때문에 지연시키지 않는다.
3. 복귀 후에도 진단 범위는 유지하되, 현재 저장 주행에서 복귀가 발생하지 않으면 그 사실을 명시한다. 가상의 기본 경로 검사를 실제 복귀 후 주행으로 쓰지 않는다. 이전 관측 기록 해제와 매 명령 허용은 별개다.

## 구현

- `src/arena_autonomy/arena_autonomy/scan_topology.py`: 원문 한 회전 metadata와 동일 장면만의 순환 증거. 현재 지지 함수의 최적화된 계산에 명시 증거를 전달할 수 있지만 노드 기본값은 그대로다.
- `src/arena_autonomy/arena_autonomy/m3_return_guard.py`: 출력 수정 없이 `sampled_sweep_ok`와 실패 단계를 반환한다. 연속 안전·숨은 물체 진입·실차 제동 인증이 아니다.
- [저장 입력 비교](replay_return_audit.py): 다섯 저장 입력 전체 출력은 이전 run7과 비교하고, 첫 측방 촉발 뒤 매 5호출 및 모드/선택/사유 전이에서 기본/실제 출력의 외곽을 진단한다. 평가 actor나 정답 위치는 읽지 않는다.
- [순환 계약 이관 비교](verify_topology_transfer.py): 기존 16합성 OFF/ON 결과와 대조한다. 현재 마스크에서 같은 출력이면 같은 Gazebo 주행을 반복하지 않는다.

## 실행 이력

| 실행 | 결과·범위 |
|---|---|
| run1_topology_regression | **수집 실패(exit 2)**: PYTHONPATH 누락. 테스트 미실행. 해시 수집에 pyc 포함/신규 검사 일부 누락이 있어 완전한 소스 불변 근거로 사용하지 않는다. |
| run2_topology_environmentfix | **시작 실패(exit 4)**: bash 마지막 줄 CRLF로 마지막 테스트 경로에 CR이 들어갔다. 실제 파일은 존재한다. 환경 import는 성공·테스트 미실행. bytes 진단 보존. |
| run3_topology_lf | **319통과(exit 0)**, 12파일. LF 고정·모든 경로 사전 확인·지정 소스/검사 전후 해시 동일. |
| [run4_saved_return_audit](run4_saved_return_audit/data/summary.json) | exit 0. 5입력 6,165판단+955 reset 전체 이전 결과 동일. 1,310개 시각의 기본/실제 명령 진단. 실제 복귀 0회라 기본 출력은 가정상 비교다. summary의 `calls`는 reset 포함 이벤트 수다. |
| [run5_topology_transfer](run5_topology_transfer/data/summary.json) | exit 0. 16합성 계약 이관 대조 전부 기존 결과와 동일. |
| [run6_release_sequence](run6_release_sequence/data/result.json) | **exit 1 보존**. 왼쪽 실제 해제 뒤 전진 9호출 전부 외곽 미관측. 오른쪽 해제도 있을 것이라는 기대는 틀려 0회. 검사/원본 결과를 성공으로 바꾸지 않는다. |
| run7_guarded_return_regression | exit 0, 정지형 복귀 출력 관문 관련 25회귀. [당시 소스](sources_after_run7.zip)는 후속 후보 재선택 전이다. |
| run8_guarded_release_selection | exit 0, 81회귀. 복귀 명령 거절 시 장부·활성 상태를 비우지 않고 기존 후보를 검사하는 후속 판본. |
| [run9_guarded_saved_comparison](run9_guarded_saved_comparison/data/summary.json) | exit 0. OFF/ON 각각 저장 6,165판단+955 reset 이전 결과 동일. 현 장부가 해제에 이르지 않아 실제 관문 개입 0회인 한계. |
| [run10_guarded_synthetic_selection](run10_guarded_synthetic_selection/data/result.json) | exit 0. 좌우 OFF/원래 출력 동일. 왼쪽 미지지 복귀 1회 거절 후 같은 HOLD_TRIAL 경로의 0.35 m/s 목표를 유지하며 해당 9호출 외곽 검사 통과. 오른쪽 해제 0 유지. 고정 자차 합성이므로 실제 진행 증가 아님. |
| [run11_encounter_window](run11_encounter_window/data/summary.json) | exit 0. 과거 좌우 피추월 초기 3초의 평가 전용 기하 분해. 실제 최소 간격 69.783/69.818 mm, 같은 종위치에서 초기 횡위치를 고정한 비교 69.974/69.816 mm. 측방 위협 최초 관측에서 97 mm 미달까지 0.640/0.561 s. 정답 궤적은 평가에만 사용하며 대안의 주행 가능성 증명이 아니다. |
| [run12_yield_observation](run12_yield_observation/data/summary.json) | exit 0. 좌우 각각 61판단 출력 동일·12 fresh 스캔 계측. 최종 미지지 질의는 모두 뒤 모서리의 마스크만 원인인 띠였다. 16개 거절 묶음의 점별/일괄 검사 일치. |
| [run13_yield_transition](run13_yield_transition/data/summary.json) | exit 0. 최초 위협 장면에서 0.20/0.30/0.40/0.60 m 전이 비교. 0.20 m는 조향 상한 초과, 나머지도 현 마스크에서 계획·명령이 모두 미지지. 기본값·허용 기준 변경 없음. |

최종 상세·선택 이유·다음 작업은 [E3 §8.29](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#829-1026-직접-재개--복귀-명령의-관측간격-검사-유지)에 모았다. 새 `GuardedReturnTrial`은 기본 OFF이며 노드/launch/CLI 연결은 없다. `reset()`은 복귀 후 보호 필요성을 유지하고 `reset_session()`만 이를 지운다. 실제 운영 이관 때 호출 경계를 확인해야 한다.

[시작 소스](baseline_sources.zip)는 이관 전 64개 모듈과 SHA-256을 담는다. 이전 작업의 실패·원문·시제품은 [앞선 묶음](../m3_rear_residual_v1/README.md)에 보존한다. 운영 출력 변경·폐루프 성과는 이 묶음만으로 주장하지 않는다.

[후속 선택형 소스·회귀/fixture 보존](sources_guarded_selection_v2.zip)은 원래 모듈·검사·분석 스크립트와 파일별 해시를 담는다. 원래 노드 연결은 없다.
