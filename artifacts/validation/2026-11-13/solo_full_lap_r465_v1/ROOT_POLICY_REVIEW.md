# r465 준비 — r463 정책 독립 정적 검토

검토일: 2026-11-13. GPT 보조 담당이 Windows에서 r463 stage의 4파일과 직접 호출 계약만 읽었다. 공통 코드·r457 입력 수정, WSL/Jetson 사용, 빌드·정책 계산·주행 실행은 하지 않았다. 이 문서는 주 담당의 전체 저장 비교·반례 검사·최종 이관 판단을 대신하지 않는다.

**결론:** 검토 범위에서 추가 critical 구현 결함이나 GUARD/RELEASED 우회를 발견하지 못했다. 자기 가림 때문에 같은 목표를 반복 감속하던 경로를 선언된 가림의 실제 폭·반환 깊이 검사로 바꾼 것은 현재 문제와 일치한다. 코드 수정 요구 없이 주 담당의 진행 중인 반례 확인과 통합 검토로 넘긴다.

## 근거

- **순서와 모드 경계:** `race_runtime_policy/current_region.py:12`는 먼저 `observation_policy.rebase` 결과를 받으며 0/미지원은 즉시 반환한다. retained `scope`가 원 모드를 선택한 뒤 해당 GUARD/RELEASED 검사를 수행한다. SOLO의 retained `_geometry.py:35–81`도 epoch/원 경로 나이/현재 입력과 경로 이동, 실제 반환의 현재·직전 조향 swept 점유와 제동 간격을 먼저 검사한다. 새 가림 허용(`current_region.py:35–40`)은 바깥 **원 output**의 `FOLLOW`, `SOLO`, `baseline`을 요구한다. GUARD/RELEASED 내부 기하 검사에 쓰는 일시적 SOLO surrogate는 이 바깥 gate의 입력이 아니다. 기존 모드의 bounded body-gap 경로(`current_region.py:59–82`)는 유지된다.
- **관측과 위험 수용 구분:** 일반 `unobserved == 0`, 전면 mask 표본 0, 실제 `motion_region` context가 있어야 새 helper를 호출한다. `profile_evidence`는 현재 raw/scene의 source frame·시각·epoch·각도·광선 상태·return 위치와 명시 거리 한정 장착 프로필을 대조한다. 시뮬 또는 명시 `host_estimated_v1`을 분리하며 verified 값을 새로 만들지 않는다. helper 수용 뒤에도 region의 mask count와 원 시각을 지우지 않고 `unknown_filled=False`, `source_timestamp_renewed=False`, `old_occupancy_permission_reused=False`를 기록한다.
- **폭/깊이의 실제 경계:** `known_structure.py:60–130`은 유효 질의/패딩, 선언된 단일 구조 성분, 성분 양옆 유효 return, 질의의 인접 광선 bracket을 요구한다. 실제 반환이 질의의 패딩 안이면 거절하고, 양옆 반환 깊이에서도 패딩을 뺀다. 숨은 폭은 실제 취득 방향의 최대 각폭과 원점 이동 폭을 포함한 `2 * reach * sin(angle / 2) + origin_span`으로 계산하여 기존 0.05 m를 넘으면 거절한다. 질의 거리 패딩을 그림자 폭에 다시 더하지 않으며, 일반 unknown이나 선언 밖 mask를 free로 바꾸지 않는다.
- **제거한 재시도와 연결:** `solo_low.py`는 `current_region`, `solo_crawl.py`는 `solo_gap`으로 전달하는 호환 함수를 남겼다. `solo_gap`이 참조하는 `prior.base` 연결도 유지된다. 따라서 같은 mask 실패에 .15→.10 m/s로 전체 current 검사를 반복하는 분기는 제거되지만, 일반 미관측용 `solo_gap`/`solo_relative`와 GUARD/RELEASED는 제거되지 않는다. 새 helper는 해당 mask 실패에서만 호출된다. 성능 개선의 크기나 실시간 충족 여부는 이 정적 검토로 보증하지 않는다.

## 기존 실패 기록의 해석 및 한계

검토 중 읽은 `r463/verification_v1/summary.json`에는 원 4개 실패가 그대로 있었다. 이후 주 담당은 다음 원인을 전달했다. 이는 주 담당의 분석이며 이 검토에서 새 계산으로 재현하지 않았다.

- `non_mask_outputs_exact`: 46건은 최종 속도/경로/시각이 같고 제거된 retry 또는 proof 진단이 달라 전체 exact가 아니었다.
- seq1986의 실제 mask 성분은 430..440, 선언 범위는 425..445였다. 429의 Inf/NaN은 선언 내부의 알려진 mask를 확장하며 일반 unknown 반례가 아니다. 보고된 폭 17.75 mm의 수용은 현 코드와 모순되지 않는다.
- 429의 .20 m 반환은 보고된 패딩 약 .065 m와 질의 거리 약 .112 m보다 멀다. 그 반환이 무조건 정지되어야 한다는 기존 반례 전제가 맞지 않는다. 주 담당이 선언 밖 424/446 결측과 실제 패딩 안 .14 m로 좁은 반례를 정정하여 확인 중이며, 원 실패와 판정선은 보존한다.

실제 점유 sweep은 선행 기하 검사에, 이전에 본 반환의 숨은 기억은 독립 보호층에 남아 있다. 이 helper 자체는 기억을 재검사하지 않으며 `independent_guard_required=True`를 유지한다. 5 cm보다 얇고 처음부터 가려진 물체, 가림 안의 동적 접근, 장착/센서 오차 위험은 여전히 남는다. 이번 변경을 자유 공간 증명이나 실차 안전 보증으로 해석하면 안 된다. 또한 선언 구조 정책이 꺼지거나 profile 검증에 실패하면 제거한 .15/.10 fallback으로 돌아가지 않는다. 이는 현재 명시 구조 정책을 사용하는 실행 범위에서 검토한 변경이다.

## 검토한 stage 바이트

기준 경로: `../solo_current_mask_r463_v1/stage/src/arena_autonomy/arena_autonomy/`

| 파일 | SHA-256 |
|---|---|
| `known_structure.py` | `e20e6a1506080530efcd9308308182e1110c225655592df5db4b785047448ecc` |
| `race_runtime_policy/current_region.py` | `fc2c007cd42d576a3f09f3b08f79b7102ec8b90b854fd2bde1dce0420032630b` |
| `race_runtime_policy/solo_low.py` | `8a25b8391472df21958b5f6a44340b469f7ffd83630be518f9925b95d9a4a6c9` |
| `race_runtime_policy/solo_crawl.py` | `412ba2bf1829e4ad56200fe7197df8e285ee08d6ffbc6d4861c6fa870fba53a5` |

r465는 여전히 Windows recipe 준비 상태다. r463/r464 공통 통합 뒤 실제 source delta 전체 검토, 빌드·입력 동결, 실행 배정 순서를 유지한다. 이 정적 검토로 r465 동결이나 주행 시작을 대신하지 않는다.
