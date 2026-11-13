# r468 RELEASED 구조 가림 — 독립 검토

2026-11-14. GPT 보조 담당이 Windows에서 base/stage의 변경3파일 및 직접 호출 계약을 읽었다. 정책 계산·새 시험·WSL/Jetson 사용·코드 수정은 하지 않았다. 저장된 base/stage 재생 결과는 주 담당이 실행한 근거로 읽었으며 독립 재실행으로 합산하지 않는다.

**의견: 설계는 수용 가능하며, RELEASED current의 선택 gate1곳을 plan과 맞추는 최소 수정을 권고한다.** 실제 점유를 면제하거나 측방 상태를 지워 통과시키는 critical 경로는 발견하지 못했다. 1 cm 꼬리 기준을 무조건 유지하는 것 대신 같은 선언 구조의 숨은 폭을 따로 검사하는 목적은 이번 원 실패와 일치한다.

## 확인한 보존 경계

- `pass_release_trial.py:705–746`의 음수 속도·비정상 baseline·SOLO·GUARD·원 baseline0 반환이 새 검사보다 먼저다. 새 코드는 RELEASED 양수 경로에만 도달한다. 측방 관측/통과 기록·guard anchor를 수정하지 않는다.
- `pass_release_trial.py:747–754`에서 retained이면 먼저 `min(b_speed, guard_speed_cap_mps)`를 적용한다. 새 helper는 이 목표/조향으로 이미 계산한 region/context를 사용한다. first-stop의 .35 m/s retained cap을 우회하거나 낮은 목표로 실제 측정 속도를 바꾸지 않는다. 구조 proof가 수용되면 같은 목표를 다시 낮춰 전수검사하는 `lower_masked_motion`은 생략하므로 불필요한 추가 감속을 피한다.
- `structure_region.py:13–23`은 명시 구조 정책/traffic recovery, 현재 partial allowed와 구조 evaluated, 일반 unknown0, 전면 mask0, 실제 masked context를 요구한다. 실제 다른 반환은 baseline 점유 검사 및 `current_mask_shadow`의 질의-반환 패딩/양옆 깊이 검사에 그대로 남는다. 숨은 기억은 기존 structure-aware baseline/독립 보호층의 책임이며 이 helper는 그것을 새 자유 공간 근거로 바꾸지 않는다.
- `pass_release_trial.py:831–833`은 구조 proof가 수용됐더라도 최종 거절 이유가 왼쪽/오른쪽 **mask**일 때만 해제한다. 일반 unknown을 이 proof로 해제하지 않는다. 기존 fringe/body-gap 판단 자체의 허용 범위를 바꾸지는 않는다.
- current는 먼저 기존 `rebase_current_released`를 통과해야 한다. retained `_released.py:35–51`의 현재 경로 점유·전방 관측·새 측방 반환 거절 뒤에만 새 구조 판단에 들어가며, `current_side_presence`가 양쪽 false여야 한다. 원 plan의 구조 허가를 복사하지 않고 current scene/context와 `profile_evidence`를 다시 사용한다.

## 최소 수정 의견

**[P2] plan/current의 명시 선택이 비대칭이다.** 새 plan helper는 `cfg.traffic_recovery_trial`이 false이면 거절하지만(`structure_region.py:15–16`), `current_region.py:36–39`의 새 RELEASED `structure_mode`에는 같은 flag가 없다. 따라서 structure policy만 켜고 traffic recovery를 끈 구성이 기존에 허용한 작은-mask RELEASED 계획을 갖고 있을 때, 다음 current의 큰-mask 거절은 새 구조 경로로 해제될 수 있다. runtime 선택 검사는 native2slot/retain 등을 확인하며 이 flag를 항상 true로 강제하지 않는다.

이번 r465 header는 true여서 저장207건의 원인이나 결과를 바꾸는 발견은 아니다. 다만 “명시 선택을 끄면 새 RELEASED 판단도 꺼진다”는 계약에는 실제 차이가 있다. **RELEASED 조건에 `cfg.traffic_recovery_trial`만 추가하고 기존 SOLO 조건은 그대로 두는 수정**을 권고한다. raw/scene 재구성이나 새 보호층은 필요하지 않다.

plan helper의 source 대조는 frame/양끝 시각/angles/count이며 독립 `profile_evidence`의 전체 epoch/광선별 상태 검증과 같지는 않다. 다만 실제 plan 호출은 `follow_trace.replay_call`이 같은 scan에서 scene을 만든 뒤 같은 호출의 baseline/partial/context를 전달하고, `_partial_command`도 반환 index/scan 일치를 확인한다. 현재 연결에서 과거 partial이나 다른 raw/context를 섞는 경로는 확인하지 못했다. 이 내부 helper를 독립 외부 입력 검증기로 설명하지 않는 것으로 충분하며, 성능 비용이 큰 전체 광선 대조를 또 추가할 근거는 없다.

## 비용·현재 근거의 해석

주 담당 저장 결과는 base207/207 exact, stage의113 exact/원0→양수94, 기존 양수→0·감소0, 측방 상태/하위 상태 변경0이다. 원 first-stop에서는 일반 unknown0·front mask0·left tail11.62 mm를 5 cm 숨은 폭 기하로 검사하여 기존 retained .35 m/s를 회복했다. current 대표3개도 .35를 반환하지만, `replay.py`가 같은 저장 plan 입력을 current로 사용한 **국소 계약 검사**다. 실제 후속 current/부모 발행·진행을 증명하지 않는다.

새로 양수가 된94개 저장 plan의 계산 중간값은 base 약25 ms→stage 약67 ms, stage 최대79.65 ms다. 같은 저장 WSL 결과이며 ARM 또는 ROS 전체 주기로 확대하지 않는다. 새 helper의 query/ray 계산이 plan에도 추가되고 current에서 별도로 필요하므로 비용은 실제 후속 항목이다. 반면 현재 context를 재사용하며 구조 수용 뒤 같은 목표의 저속 재시도를 생략하는 흐름은 적절하다. r467 동일 판정 최적화와 결합할 때 근거를 재사용할 수 있으며, 비용 때문에 현재 점유/현재 재검사를 삭제하라는 의미는 아니다.

## 핵심 반례7개

새 범용 검사 묶음 대신 아래 분기별 대표만 권고한다. 이미 주 담당 검증에 포함된 항목은 반복할 필요가 없다. 원 실패 입력을 바꾼 반례는 수정된 scan으로 scene/partial을 다시 만들어 양쪽의 실제 일관성을 유지해야 한다.

1. **retained 상한·현재 연결:** first-stop의 같은 경로/입력에서 원0→.35, 속도 상향 없음, 측방/retained 기록 불변을 확인한다. 후속 저장 입력을 current로 연결할 경우 실제 source/epoch/시각을 유지하고 “새 주행 궤적”으로 표현하지 않는다.
2. **유한 실제 점유:** 경로 sweep 안의 전방 반환 또는 질의 패딩 안의 성분 옆 반환을 coherent하게 넣는다. 새 구조 proof가 점유 정지를 해제하면 결함이다. 선언 각도 내부라도 range band 밖의 유효 return은 mask로 지우지 않아야 한다.
3. **새 측방 물체/모드:** 같은 current에서 유효 측방 반환이 임계 개수에 도달하면 `_released`의 side-present 거절이 남아야 한다. plan에서는 실제 모드가 GUARD로 돌아가며 새 RELEASED helper가 이를 우회하지 않아야 한다.
4. **일반 unknown·front mask:** 각각1개 이상의 해당 질의를 가진 대표를 사용한다. 새 구조 proof는 수용하지 않아야 하며, 기존 별도 unknown 정책의 결과와 구분한다. 모든 unknown 사례를 일괄0으로 만드는 새 조건은 요구하지 않는다.
5. **숨은 폭·양옆 깊이:** component의 숨은 폭이 .05 m를 넘거나 실제 padding을 뺀 flank 깊이가 질의에 닿는 사례는 수용하지 않아야 한다. declared 내부 NaN/Inf가 component를 확장하는 것은 곧바로 “일반 unknown” 반례가 아니므로, 누락 flank는 선언 밖 실제 경계를 사용한다.
6. **명시 선택 OFF:** traffic recovery false에서 plan/current 모두 기존 판단을 유지해야 한다. 위 P2 수정의 대표다. 구조 policy OFF/partial not allowed/evaluated false도 새 허가를 주지 않아야 하며 각 경우를 대량 조합할 필요는 없다.
7. **현재 출처·숨은 기억:** 현재 epoch/scan 불일치는 기존 runtime 입력 계약으로 거절되고, verified false/host-estimated 입력은 실제 선택 계약을 유지해야 한다. 이전에 본 물체가 그림자로 들어간 대표는 기존 baseline/독립 guard의 기억 제한을 보존한다. 새 helper가 기억을 지웠다거나 현재 관측으로 승격했다고 쓰면 안 된다.

5 cm보다 얇고 처음부터 가려진 물체, 가림 안의 동적 접근, 실제 장착/센서 오차 위험은 남는다. 이번 수용은 동일한 구조 위험 예산을 적용하는 운용 판단이며 벽/타차 충돌 불가능이나 실차 안전의 보증이 아니다.

## 검토한 stage SHA-256

기준: `stage/src/arena_autonomy/arena_autonomy/`.

| 파일 | SHA-256 |
|---|---|
| `structure_region.py` | `91abf0bd017b6bcf448b1e1a42ca1fe692b75f645097fdfb403437aa96f867c8` |
| `pass_release_trial.py` | `cc40f2985457c75c8a6bb1b232ed6801a5795e5c7758c2314184ff96ce8f1958` |
| `race_runtime_policy/current_region.py` | `8753c37b5ee534d9b1060bd9ec84a6fedc3c79db1ab88edfe800573d995b9d97` |
