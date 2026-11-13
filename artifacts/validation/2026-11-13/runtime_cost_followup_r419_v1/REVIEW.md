# r419 current 거리 후보의 계산 비용 후속 검토

2026-11-13. r411 최신 결과·차이 분류와 current index137/trace646/sequence296의 OFF/ON 두 행, 해당 공통 호출 경로만 읽었다. **168.26ms는 후보 내부 한 함수의 비용이 아니라 기존 현재 검사와 추가 거리 후보를 모두 포함한 Evaluator 총시간이다.** 다음에 좁은 계산 개선을 한다면, 거리 제안의 반복 자세 검사에서 같은 scene의 미반환 광선 준비 자료를 재사용하는 한 가지를 제안한다. 이번에는 코드 변경·재생·프로파일·WSL/Jetson 실행을 하지 않았다. r418 준비물도 수정하지 않았다.

## 이미 측정된 범위

근거는 [r411 RESULT](../runtime_cost_r411_v1/RESULT.md), [차이 분류](../runtime_cost_r411_v1/difference_review.json), [원 timing 행](../runtime_cost_r411_v1/revision_env_v2/remote_result/extracted/full/timing_rows.jsonl)이다. 380호출 완료·동일 ARM의 불변 검사는 통과했고, x86 과거 출력 exact는 실패했다. region count70개 차이를 제외하거나 성공으로 바꾸지 않는다. 경로/조향의 매우 작은 차이와 반올림 없는 중복 제거가 연결될 가능성은 있지만 query 생성 과정을 계측한 확정 원인은 아니다.

| 측정 항목 | 값 / 해석 |
|---|---|
| 전체 current OFF / ON | 각153개. 중앙45.32 / 45.52ms, p95 89.63 / 91.94ms. ON100ms 초과4개 |
| 해당 입력 | index137, trace646, generation361, sequence296, SOLO, reference21.522s. OFF/ON 입력 SHA256 모두 `6cc2d04d723ce31dcceb6accf412364b3466f47eaba627479d3cab4ff1d238e7` |
| 해당 OFF | Evaluator69.535465ms, 최종0m/s, `route_current_large_motion_region_unobserved` |
| 해당 ON | Evaluator168.258916ms, 최종0.09509513866308608m/s, `rebase_current_follow`, 후보 적용true |
| 두 측정값의 산술 차이 | 98.723451ms. 서로 다른 arm의 단회 직렬 호출 차이이며 후보 함수의 분리 계측값/절감 가능 시간으로 해석하지 않음 |
| 계측 범위 | 입력 `restore_call`·`motion_scene`·현재 정책·최종 `freeze`를 포함한다. 입력 해시/상태 감사·Lane 직렬화·파일 기록은 timer 밖이다. 해당 ON의 `lane_with_audit_ms=174.538058`도 운영 IPC 비용의 별도 실측으로 사용하지 않는다 |

해당 행은 arm의 첫 호출이 아니다(`first_call=false`). 다만 거리 후보가 실제 적용된 최초·유일한 행이므로 해당 분기의 초기 실행 효과까지 배제한 반복 평균은 아니다. `corner_budget`의 선택적 `costs` 계측 항목은 이번 Evaluator에서 전달하지 않아 `proposal_geometry_ms`, `final_core_ms` 등의 개별 수치는 없다.

## 해당 호출에서 실제로 추가되는 계산

[Evaluator](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/evaluator.py)의 59–105행은 현재 scan/history를 복원하고 **scene을 한 번 만든 뒤 같은 객체를 전달**한다. ON에서도 baseline을 다시 호출하는 wrapper를 얹지 않는다. baseline 결과가 eligible일 때만 [corner_candidate](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/corner_candidate.py)의 단일 제안/최종 검사를 추가한다.

1. **OFF에도 있는 현재 검사.** 기록된 원 목표0.45m/s 영역은 거리0.192375m, 질의3,465개, unobserved780개로 거절됐다. SOLO0.15 재검사는1,244질의/297unknown·측면 외곽14.2688mm,0.10 재검사는771질의/186unknown·10.0616mm로 기존10mm 조건을 넘었다. `solo_gap`/`solo_relative`는 각각 core 경로·현재 점유·영역을 재검사한다. 이 행에서는 mask-only용 `solo_low`/`solo_crawl`의 별도 실행 조건은 맞지 않는다. RELEASED 전용 후속도 SOLO이므로 적용되지 않는다.
2. **ON에서 거리 제안1회.** [corner_budget.propose](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/corner_budget.py)에서 현재 조향0.362731rad/직전0.375083rad 두 방향, 고정 거리·극점7개를 사용한다. 실제 기록은14자세, 중복 제거 전/후922질의, 얇은 교차204개다. 지지 분류는 free294/unknown222/masked406이다. D(0.10)=0.034m 범위에서 첫 나쁜 거리 앞0.0322157400m를 선택했고, measured0과 τ0.315s·반응0.1s·감속2m/s²를 그대로 둔 역산으로0.0950951387m/s를 제안했다.
3. **새 목표의 core 검사1회.** 같은 원 경로를 현재 좌표로 옮기고 추종·점유 swept clearance를 다시 계산한다. 속도·조향·경로 identity를 기존 결과와 대조하며 proposal 자체로 허가하지 않는다. 이 항목은 baseline core 계산과 겹치는 준비가 있지만 목표가 다른 호출을 단순 생략할 근거는 없다.
4. **최종 영역/공백 검사1회.** 최종694질의/169unknown/306masked-left를 새로 만든다. proposal의922질의를 잘라 쓰지 않는다. 최종 외곽9.61535mm와 기존 공백 조건을 통과했다. proof policy는 `guard_low_speed_body_nonreturn_r136`이다. [range_mask_component](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/range_mask_component.py)는 첫 기존 `bounded_body_nonreturn` 성공 시 즉시 반환하므로, **이 행에서는 뒤의 혼합 mask 성분 확장 루프까지 실행된 것으로 볼 수 없다.**

위 질의 수는 저장 진단이며, 호출 순서는 같은 바이트의 코드와 해당 분기 결과로 확인한 것이다. 질의별/native kernel별 소요시간은 계측하지 않았다. core/영역/제안 전체를 한 함수의 병목으로 단정하지 않는다. 이 경로에서 읽은 관련17파일은 r411 revision_env_v2의 동결 manifest와 바이트가 같음을 확인했다. r417의 새로운 crop-edge 선택이나 이후 실제 주행 비용을 이번 수치로 대신하지 않는다.

## 제안 한 가지: 거리 제안 안에서 scene 준비 자료만 재사용

[propose](../../../../src/arena_autonomy/arena_autonomy/race_runtime_policy/corner_budget.py)는 14자세마다 [body_queries](../../../../src/arena_autonomy/arena_autonomy/m3_execution_trial.py)를 호출한다. 그 내부 [path_observation_queries](../../../../src/arena_autonomy/arena_autonomy/observed_lateral_trial.py):22–69는 매번 다음을 새로 만든다.

- 같은 cfg 길이/폭으로 만든 차체 기본 border.
- 같은 `scene.rays`에서 고른 unknown/masked 목록과 origins/directions.
- 같은 원천 각도의 [angular_cell_widths/unknown_cell_directions](../../../../src/arena_autonomy/arena_autonomy/measurement_scan.py)가 계산한 lower/upper 경계. signed 간격 검증·각도 폭·삼각함수 계산도 반복된다.

**한 번의 `propose(scene, …)` 안에서 이 입력 준비 결과만 한 번 만들고 14회에 명시 인자로 전달**할 수 있다. 범위는 `propose → body_queries → path_observation_queries`에 한정한다. pose별 corners/교차/clip/바퀴 외곽, 질의 순서, `first_distance`, 각 호출의 exact `unique_xy`, 최종 motion_region과 공백 검사는 유지한다. 같은 scene와 cfg를 사용하는 함수 안의 재사용이므로 과거 허가·다른 frame/epoch·새 시각을 가져올 필요가 없다. 전역/시간 TTL 캐시나 새로운 관측 기억을 만드는 제안이 아니다.

이는 코드상 확실한 반복 준비를 **14회에서1회로 줄일 가능성**이다. 922질의나204교차 자체를 줄이는 것은 아니며, 절감 ms는 모른다. 자세들을 한꺼번에 합쳐 provenance/순서를 바꾸거나 final linspace가 proposal의 부분집합이라고 가정하지 않는다. r411의 region count 차이가 이미 있으므로 `unique_xy` 반올림/질의 생략을 섞으면 행동 불변 제안의 범위를 벗어난다.

## 우선순위 판단

이 입력에서 후보는 실제0명령을 작은 양수로 회복했지만 적용은153중1회다. 이 꼬리 비용을 줄였다고 전체 current p95/10Hz가 해결되는 것은 아니다. 기본 current도 p95 약90ms이며, 같은 Jetson의 계획·영상·guard 경쟁 비용과 실제 명령 만료는 이번 직렬 측정 밖이다. 이미 scene은 한 번 생성하고 native depth/masked 분류도 공유하므로 ‘scene 재생성 제거’나 ‘두 분류 합치기’를 새 개선으로 중복 제안하지 않는다.

따라서 **r418의 실제 진행/정체 결과와 실물 연결을 우선 회수하고**, 이후 해당 후보의 계산 경로를 다듬는다면 위 같은-call 준비 재사용부터 좁게 검토하는 편이 타당하다. 함수별 비용을 모르는 상태에서 wrapper 전면 재설계나 더 낮은 고정 속도 재시도를 먼저 추가할 근거는 없다. 이번 검토로 새 실행·필수 시험 묶음을 만들지 않았다.

자료 고정: timing rows SHA256 `bdf447df065cc9f54f6f832541f8e830ce4a84ff87b728b933d3e8ae18720b24`, difference review SHA256 `f791adc9719206bd16bc4319c2aff57a79ce0b5b373a6b4f1a8157279a5086fc`. 새 코드/원본 결과 수정 없이 이 문서만 작성했다.
