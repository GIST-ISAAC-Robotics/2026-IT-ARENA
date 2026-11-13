# r426 동일 crop 성분의 현재 이동영역 제한 재검사

**저장 입력 시험 준비본·공통 미이관·제어기 미실행.** r418 원 실패, r423의 대표 0 출력과 동결 자료를 보존한다. 기존 공통/r417을 baseline으로 사용하며, r423의 세 SOLO 게이트 확장은 포함하지 않는다. 이 후보는 검증된 native simulation 환경의 명시 선택에만 연결한다. 실물 적용, 실제 주행 성공, 연속 충돌 안전을 주장하지 않는다.

## 선택 범위와 추가 위험 가정

기존 `released_relative`와 기존 corner 후속 결과를 먼저 그대로 계산한다. 양수 결과에는 개입하지 않는다. 현재 결과가 `0 / route_current_large_motion_region_unobserved`이고 다음 조건이 모두 맞을 때만 별도 재검사를 한 번 한다.

- 기존 `crop_edge_component_trial=true`, `current_corner_budget=r396_v1`, native2slot/native numeric, simulation 시간, verified-only 입력 환경.
- 원 계획은 FOLLOW/SOLO/baseline/local_follow이며 allowed가 유지되고 known-obstacle latch나 memory boundary가 없다.
- 부모 소유 frozen plan/current scan 전체 사전이 exact 동일하다. 내부 compact 증거와 submitted 시각 계약을 전달하며 scan 전체를 IPC에 중복하지 않는다.
- 실제 측정속도는 0..0.6 m/s, dt는 기존 0..0.1 s 범위이고 현재·직전 조향은 기존 한계 안이다.

목표는 `min(original_plan, 0.2 m/s)` 하나다. 0.2는 이번 current 시험 상한이며 속도 검색이나 실물 기본값이 아니다. 기존 core를 그 목표로 한 번 호출해 현재 history, 원 경로, 제동·점유·양쪽 조향과 source/TTL을 다시 검사한다. 성공한 core의 현재 motion_region 전체를 한 번 생성하고, 그중 unknown 질의에만 아래 proof를 수행한다. 실패는 원 0 결과에 별도 진단만 덧붙인다. 원 목표를 성공까지 낮추는 반복이나 unknown 자유 공간 채우기는 없다.

새 proof는 **차체 옆 10 mm 상자 대신 같은 호출의 전체 이동영역**을 대상으로 하므로 허용 영역을 실제로 확대하는 별도 위험 가정이다. 기존 전역 100°/10 mm/body proof, fully 분기, guard, 임계값은 변경하지 않는다. 원 crop 계획은 `not metric_gap` 분기로 진입하므로 기존 50 mm/0.6 m/s metric 검사를 이미 통과했다고 해석하면 안 된다. 이번 수치는 다른 기존 metric 시험 규칙에서 가져온 새 current 위험 범위이다. 숨은 작은 비반사 물체의 잔여 위험은 남으며 같은 LiDAR의 독립 guard가 반드시 발견한다고 보장하지 않는다.

## 현재 proof 계약

원 `motion_region`의 q, ray 원점과 방향은 모두 현재 차체 중심 좌표다. 후륜축 이동이나 센서 offset, 차체 팽창을 다시 더하지 않는다. 원 현재 scene의 유한 원점/unit direction 계약을 유지하고 직접 합성 helper 입력도 비유한·비정상 방향을 거절한다. unit 허용오차는 기존 SensorRay의 1e-10을 사용한다.

1. 현재 raw/scene의 source frame, epoch, 전체 원 각도, 원 취득시각과 index 대응을 확인한다. 기존 `_bounded_nonreturn`의 실제 근접 점유, raw +Inf 전체 성분, finite 양 flank, mask 경계, 원 raw 거리 일치, 6° 합계를 그대로 수행한다.
2. 수락할 전체 raw 연속 성분의 index 집합과 양 flank가 원 crop의 **전체 성분**과 같아야 한다. query에 걸린 일부만 비교하지 않는다. CW의 내림차순 index도 같은 성분이면 허용한다. 다른 unknown 성분이나 structure mask로 확장하지 않는다.
3. 거리 R은 **전체 motion_region 질의와 모든 현재 ray 원점 사이 거리의 최댓값**, P는 기존 per-ray padding의 최댓값이다. 양 flank의 실제 반환거리는 R+P 이상이어야 한다. 원점 차이는 현재 좌표로 계산하며 P는 한 번만 더한다.
4. 같은 전체 성분과 양 flank의 방향을 취득 index 순서로 unwrap하여 `Δ=max−min`을 구한다. 각 성분과 합계는 6° 이내이고, 어느 성분 ray/flank라도 현재 중앙 ±30°에 들어오면 거절한다.
5. 해당 성분과 flank 원점의 최대 쌍거리 d를 구해 `W=d+2*(R+P)*sin(Δ/2)`를 사용한다. 각 W와 합계가 50 mm 이하여야 한다. 같은 원점에서는 기존 angular chord 식과 같고, 평행 ray라도 원점이 벌어지면 W를 0으로 오판하지 않는다.

W는 위 유한 원점/unit 방향/6° 구간과 R+P 조건에서 방향 구간 이등분선에 **수직인 표본 ray segment 횡폭 상계**이다. 임의 두 점 거리, 숨은 물체 최대 크기, 종방향 침투 길이 또는 연속 안전의 상계가 아니다. 전면 mask 0과 기존 tail 허용, independent guard/STOP 요구도 유지한다.

원 crop 내부 셀 폭 2.16°와 전체 셀 폭 4.32°의 metric 분류 차이는 별도 후속 사항이다. 이번에는 `partial_observation` 공통 판정이나 plan 속도를 변경하지 않는다.

## 구현 및 입력 계보

staged는 5파일이다. base/staged SHA와 새 파일 여부는 [source_ledger.json](source_ledger.json), 부모 검토용 diff는 [source_changes.patch](source_changes.patch)에 남긴다.

- `runtime_dual_state.py`, `race_runtime_policy/crop_current_scope.py`: r423의 부모 compact evidence와 후보 분류를 재사용한다. 외부 ROS에서 evidence bool을 받지 않는다. OFF request는 기존 사전 그대로다. 사용처가 사라진 `partial_retry_scope`와 그 전용 재사용 검사는 제거했다.
- `race_runtime_policy/evaluator.py`: 기존 baseline/corner 이후 단일 후보를 명시 함수 인자로 호출한다. global monkeypatch, 새 header/default 필드는 없다.
- `crop_current_candidate.py`, `crop_current_proof.py`: 단일 목표 core·motion_region·raw proof와 별도 진단이다.

r423 초기 설계의 child 전체 submitted scan 비교 대신 부모가 이미 보유한 두 frozen scan을 비교한다. 이는 축약 IPC 최적화를 유지하기 위한 변경이다. 입력 동일성은 후보 분류일 뿐 과거 점유 허가가 아니다. 현재 reference/history/측정속도/직전 조향을 옛 값으로 바꾸지 않는다.

r418 `follow_decisions.jsonl` SHA256 `67bd68142f4d0596ed60bd89fe280fd5b9a3e60e79802d3d7d0d1d6da6ba30da`와 r421 동결 감사의 nested plan/current 동일성을 확인해 current 313개를 원 순서로 추출한다. same-scan은 305개다. 원 기록의 양수 current는 127개이며, 대표는 trace seq1003/index312이다. plan 전체 상태 이력 재생이나 실제 시간 명령 만료 시험이 아니다.

source는 r418 보존 archive의 필요한 import 폐포를 사용하며 준비 시 현행 common과 exact 동일한지 확인한다. r423 gate 수정이나 다른 후속 후보를 섞지 않는다. 기존 `_gaps`, `rear_fringe_trial`, `partial_observation`, SOLO/RELEASED/corner/core geometry 파일의 baseline/candidate bytes 동일성을 manifest에 기록한다.

## 좁은 검사 및 종료 조건

새 합성 경계 17개와 r423 중 필요한 12개 사례, 총 29개를 준비한다. 17개는 동일 원점 식/내림차순 성분, 가까운 flank·실제 점유·NaN·under-range·열린/넓은 gap·현재 중앙 회전, 전체 성분 불일치, 원점 spread/전체 방향 span, 잘못된 직접 기하, 원 대표에 finite 전방/측방 표적을 넣은 두 음성 입력, 실제 측정속도/단일 cap, 기존 양수 불변이다. 변경한 센서 음성 입력은 합성이며 원 재생 성공으로 세지 않는다. 재사용 12개는 실제 `offer_next_current` 부모 경로의 True/False/OFF, compact metadata와 시각 불변, false allowed/known-lost, 원 scan 차이다. r423 원본은 보존하며 bundle의 복사본에서 제거된 helper import와 그 전용 검사 함수만 제외한다. 두 판본 SHA를 manifest에 기록한다. fully 경로는 기존 공통 source 그대로이며 실제 Evaluator 저장 비교의 비대상 exact로 확인한다. r423의 30개 전체나 r417 plan184 검사를 반복하지 않는다.

단위검사 후 대표 seq1003 baseline raw evaluator 출력과 원 checked target을 exact 대조한다. 부모 cap 후 원 기록은 별도로 비교한다. baseline 실패 시 candidate를 호출하지 않는다. 이어 candidate를 한 번 호출하며, 두 arm의 원 출력·진단·입력/소스 불변·실제 import 경로·Lane 회수를 보존한다. baseline 계산 횟수는 1회여야 한다.

대표가 양수로 회복하고 정확성/회수가 통과한 경우에만 나머지 312개를 각 arm 1회 비교한다. 대표는 재사용해 고유 313개를 맞춘다. 비대상은 전체 출력 exact, 후보 거절은 새 진단을 제외한 원 필드 exact, 회복은 경로/조향/source/시각과 독립 guard 계약을 확인한다. 대표가 0이면 proof 원인을 남기고 종료하며 자동 확대하지 않는다. 원 거절, 후보 회복, 계산 완료, 정상 종료는 별도 판정이다.

## 실행 담당 고정 명령

준비 단계에서는 제어기/WSL/ROS/Jetson/주행을 실행하지 않는다. 아래는 root 검토 후 실행 담당이 WSL x86_64에서 한 번 수행할 명령이다. 별도 ROS 노드나 센서/장치는 시작하지 않는다. 동일 root/receipt 재사용으로 결과를 덮어쓰지 않는다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
python3 -B artifacts/validation/2026-11-13/crop_current_reach_r426_v1/execute_isolated.py --stage units --root "$HOME/it-arena-r426-crop-reach-a" --receipt artifacts/validation/2026-11-13/crop_current_reach_r426_v1/verify_v1
python3 -B artifacts/validation/2026-11-13/crop_current_reach_r426_v1/execute_isolated.py --stage representative --root "$HOME/it-arena-r426-crop-reach-a" --receipt artifacts/validation/2026-11-13/crop_current_reach_r426_v1/verify_v1
python3 -B artifacts/validation/2026-11-13/crop_current_reach_r426_v1/execute_isolated.py --stage conditional-full --root "$HOME/it-arena-r426-crop-reach-a" --receipt artifacts/validation/2026-11-13/crop_current_reach_r426_v1/verify_v1
```

각 exit0 및 `verification_passed=true` 확인 뒤 다음 단계로 간다. units는 native build 1회/최대120초, 동일 binary 양 arm 복사, 선택 검사/최대120초다. 대표는 baseline/candidate 각 1개이고 arm 최대240초, 내부 Lane timeout20초/phase 총180초를 사용한다. 이는 저장 계산 종료 예산이며 제어 TTL을 늘리는 설정이 아니다. inherited PYTHONPATH는 isolated 경로 뒤에 유지한다.

`representative_recovered=false`이면 원 0과 진단을 회수하고 종료한다. 조건부 명령을 호출해도 이 경우 추가 제어 호출 없이 skipped를 기록한다. 양수인 경우만 나머지 baseline312 exact 후 candidate312를 검사한다. `verify_v1/{units,representative,conditional-full}.{json,zip}`에 로그/원 행/비교/예외/회수를 보존한다. 자식은 정상 close, exit0, forced=false, alive=false를 요구하며 timeout/신호 시 소유 process group을 회수한다. manifest 전후 변경이나 외부 project import는 실패이다. 실패 판본·기존 결과·source를 고쳐 같은 결과 폴더에 덮어쓰지 않는다.

원시 입력·상세 행·ZIP은 기존 Git 제외 정책을 유지한다. 정적 준비 결과와 실제 실행 결과는 구분하며, 주행/공통 통합은 root의 별도 검토 대상이다.
