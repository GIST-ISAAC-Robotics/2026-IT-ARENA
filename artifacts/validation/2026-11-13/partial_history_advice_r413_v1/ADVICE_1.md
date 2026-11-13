# Fable 5.2/high 읽기 전용 자문 1

파일 수정이나 정책·pytest·ROS 실행은 하지 않았고, 소스 열람과 저장 JSONL의 읽기 전용 stdlib 집계만 수행했습니다.

## 결론

**분리 방향은 타당하지만, 제안하신 형태는 필요 이상으로 크고 이 정체를 푸는 데는 부족합니다.** 저장 입력의 약 2/3는 수정 후에도 `unbounded_missing`으로 거절되고, 재출발에는 연속 2회 허가가 필요하기 때문입니다. 남는 문제는 기억 갱신으로는 풀 수 없는 허가 범위 문제이므로, 아래 (4)의 판단이 필요합니다.

## (1) 가장 작은 건전한 형태

- **기존 패턴 재사용:** `partial_observation.py:381-388`이 이미 위협 계산과 `_remember`를 마친 뒤 `forward_gap_*`로 거절합니다. 구조 기억도 base 거절과 무관하게 fresh 스캔마다 갱신됩니다(`known_structure.py:212-218`, `286-304`). `unbounded_missing`을 같은 지연 거절로 만들면 되고, 별도 경로는 필요 없습니다.
- **깨끗할 때만 갱신:** 거절 경로에서는 재관측 제거 후 위협이 0일 때만 `_remember`를 호출하고, 그 외에는 모든 상태를 그대로 둡니다. latch와 `_capture_latched_threat`는 호출하지 않습니다. 위협이 있으면 지금처럼 기억이 동결되어 다음 bounded 스캔에서 기존 latch가 걸립니다. 거짓 만료만 사라지고 새 정지 종류는 생기지 않습니다.
- **crop 밖 연속 결측은 제외를 권합니다:**
  - `fully_observed` 경로(159행)는 ±100° 밖으로 사라진 점을 검사 없이 잊으므로, 기존 기억 범위 밖입니다.
  - in-crop 끝 ray의 기존 여유(0.03 m + 2 m/s × 약 0.1 s ≈ 0.23 m)가 4–5° 연장분을 대부분 덮습니다. 이것은 계산하지 않은 기하 추론이며, 아래 T2b로 판정할 수 있습니다.
- **fresh 스캔에서만 수행:** 보호층은 같은 스캔을 명령 주기로 재평가하므로, 비-fresh 호출은 즉시 원래 거절을 반환해야 합니다.

## (2) 포함·제외, hook, 사유 우선순위

- **포함:** 241행의 `unbounded_missing` 한 곳(staged helper의 `None` 분기 포함)만입니다. 기준 254개 입력의 partial 사유는 `fully_observed` 156 / `unbounded_missing` 98뿐입니다.
- **제외:** `partial_invalid`, `partial_time_reset`, `partial_speed_outside_trial`, `angular_metadata_hole`, `invalid_near_return`, `wide_or_many_missing`, `lateral_metric_outside_trial`, `central_missing`. 루프 뒤의 `central_missing_after_motion`, `lateral_metric_observation_unconfirmed`는 실제 차단이 관측되면 그때 추가합니다.
- **우선순위:** 해당 run에서 `denied = {**report,'reason':'unbounded_missing'}`를 복사해 두고 `break`합니다. 이후 어떤 분기도 반환 사유·허가·속도 상한을 바꾸지 않습니다.
  - `central_after` 반환, metric 확인, `central` 보고는 건너뜁니다. `boundary_ranges`가 비면 `min([])`가 `ValueError`를 냅니다.
  - warmup과 0.3 s 만료는 latch 없이 `denied`를 반환합니다. 실제 센서 공백은 지금처럼 다음 bounded 스캔에서 만료 latch가 됩니다.
- **hook:** `_refine_threat` 호출 지점은 하나로 유지합니다. 구조 mixin의 후방 기억 이전은 그대로 동작하므로 `known_structure` 카운터만 달라질 수 있습니다.
- **예외:** 거절 경로 안에서 `ValueError/TypeError/MotionUnavailable/FloatingPointError`를 잡아 상태 변경 없이 `denied`를 반환합니다. 그러지 않으면 사유가 `traffic_recovery_invalid`로 바뀝니다.
- **이름 충돌:** `RecoveryPolicy._evaluate`가 이미 있으므로 base에 같은 이름의 helper를 만들면 안 됩니다.

## (3) 최소 반례와 계산 비용

| 반례 | 입력 순서 | 기대 결과 |
| --- | --- | --- |
| T1 거짓 만료 | ok → 0.1 s 간격 unbounded 3회(정적) → flank-only | 매 unbounded에서 기억 시각 전진, 마지막은 `bounded_side_missing` 허가. trace 684–688을 fixture로 사용 |
| T2 기억 보존 | ok(근접점 P) → unbounded(P가 in-crop 결측으로 소실) → bounded | 둘째 스캔에서 상태 불변, 셋째에서 `previous_point_in_missing_sweep` latch와 capture 1회. 무조건 `_remember`하는 변형은 통과하지 못해야 함 |
| T2b crop 밖 소실 | T2와 같되 P가 101–103°에서 소실 | 같은 결과. 실패할 때만 crop 밖 성분을 추가 |
| T3 실제 공백 | ok → 0.4 s 무입력 → unbounded → bounded | 기억 갱신 없음, 이후 `partial_history_expired` |
| T4 격리 | 운동·취득 메타 불가 / 같은 스캔 재호출 | 사유 `unbounded_missing`, 상태 7필드 동일, 재호출 시 `_refine_threat` 0회 |
| T5 중립성 | helper 없이 accounting만 적용한 254개 재생 | 출력 전부 동일(진단 키 제외), latch 변화 0, 기억 시각만 전진 |

계산 비용에서 주의할 점은 다음과 같습니다.

- `timed_missing_sweep`은 ray마다 `joint_missing_sweep`을 1개씩 호출하므로(`recovery_geometry.py:117-123`) 16 ray 한도는 이 경로에서 걸리지 않습니다. 실제 한도는 점 1024개, 속도 1.2 m/s, age 0.3 s 미만, 조향 0.6 rad입니다.
- 비용은 ray 수 × (조향 2 × horizon/2 cm) × 점 수이며, 이 지점의 ray 수는 6°와 5% 예산으로 이미 묶여 있습니다. crop 밖 성분을 넣으면 이 상한이 사라집니다.
- 거절 출력에 `latch_evidence` 같은 대형 목록을 싣지 말고, 재생에서 unbounded 행의 처리시간 p99를 기록하십시오.
- 0.1 s 격자에서는 갱신 1회 누락(age 0.2 s)까지만 안전합니다. 2회 누락은 0.3 s 부동소수 경계에 걸립니다.

## (4) 남는 차단과 필요한 판단

저장 자료에서 확인한 사실은 다음과 같습니다.

- **정지의 시작:** 22.62 s의 `follow_partial_transformed_gap_uncovered`(`local_follow.py:687-690`)이며, 그 스캔의 partial 판정은 `fully_observed`였습니다. 이후 측정 속도 0에서 in-crop 결측은 0.68–0.76°의 1 ray로 +100° 경계에 고정됩니다.
- **재출발 조건:** `_hold`가 `release_count`를 0으로 만들고(`:569-573`), 재출발에는 fresh 허가 2회 연속이 필요합니다(`confirmations=2`, `:1189-1196`).
- **허가 가능한 구간:** 25 s 이후 crop 밖 연속 결측이 아닌 행은 1–4개씩만 이어집니다.
- **기억이 신선할 때:** 유일한 후보 행(22.92 s)은 `bounded_side_missing`이었고, 450점·1 ray에서 possible 42개가 모두 재관측되었습니다.

여기서 나오는 가설은, accounting과 helper를 함께 적용하면 첫 양수 계획이 24.32 s 부근에 나오되 0.1–0.3 s 길이의 펄스로만 나온다는 것입니다. 21.62–21.82 s에 명령 0.45에도 측정 속도가 0이었으므로 차가 실제로 약 1 cm를 움직여 결측을 crop 밖으로 내보낼지는 불확실합니다. 보호층도 crop 밖 연속 결측 스캔은 따로 거절합니다.

**판단이 필요한 대안(허가 범위 변경):** crop 경계에 닿는 run을 안쪽 flank만으로 기존 side-gap 절차에 넘기는 방법입니다.

- **조건:** 한쪽 끝만 경계에 닿고, in-crop 폭이 기존 3° 이하이며, 안쪽 이웃이 실제 반환이고 `near_boundary` 이상입니다. 소실 위협·latch·0.5 m/s 상한은 그대로이고 새 임계값은 없습니다.
- **근거:** 끝 ray가 100.1°에서 시작하면 같은 5° 결측이 지금도 상한 없이 `fully_observed`로 통과합니다. 99.9°일 때만 정지하는 것은 crop 경계가 만든 불연속입니다.
- **효과:** 77행 전체가 일반 경로로 흘러 기억도 자연히 갱신됩니다. 이 정체에는 accounting 분리가 필요 없어지고, flank helper도 이 규칙에 포함됩니다.
- **한계:** 안전성은 실행으로 검증하지 않은 제 설계 판단입니다.

**권고:** 현 허가 범위를 유지하신다면 "깨끗할 때만 갱신" accounting(T5로 중립 확인)과 helper를 적용하고 주행 1회로 확인하십시오. 다만 펄스식 재출발이 예상되므로, 꾸준한 주행이 목표라면 대안의 채택 여부를 먼저 정하시는 편이 반복이 적습니다.

## 남는 불확실성

- 보호층 쪽의 스캔별 판정과 재평가 빈도는 저장 자료에 없습니다.
- 첫 양수 명령 이후의 재생은 open-loop이므로 궤적을 예측하지 못합니다.
- 22.62 s의 변환 방향 hold는 어느 안에서도 남습니다. 대안을 채택하면 약 0.2 s의 일시 정지로 줄어들 것으로 보이나 이것도 가설입니다.
- warmup, 속도 초과 등 제외한 거절 종류의 기억 동결은 관측 근거가 없어 그대로 두었습니다.
