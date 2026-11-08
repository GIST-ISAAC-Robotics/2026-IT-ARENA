# 새 코스 g1g2 C1 거리오차·지연 A/B 쌍 — 사전 계획 (AWAITING_RUN)

상태: **AWAITING_RUN**. 도구만 준비했다. 소스 등록(`register.py`), Gazebo 실행, colcon은 아직 하지 않았다.

## 기준과 변경

- 기준은 [r27](../traffic_faster_reacquire_live_v1/PLAN.md) `two_actor_course_lap`의 등록 인자다. 조건은 0.8 m/s 두 앞차, 60초, 한 랩 44.64 m, v2026.11.06 공식 신호 코스, paced g1g2, seed 0, 기록 모드다. 설정과 overlay는 바이트가 같은 사본(`new_track_course.json`, `actor_overlay.json`)을 쓴다. 원래 이름 `additional_actors.json`은 Git 제외 규칙에 걸리므로 이름만 바꿨다. 두 run이 공유하는 인자는 [pair_args.json](pair_args.json)에 있다.
- **A:** 공통 인자만 쓴다(무주입).
- **B:** 공통 인자에 `--lidar-test-profile config/tests/lidar_uniform30_delay50_jitter20.json` 하나만 더한다. 값은 uniform ±0.03 m, 지연 0.05±0.02 s, seed 23이다.
- 평가 CLI 변경은 `config/tests/g1g2_lidar_stress_r34_sha256.json`이다(부모 r33). 제어기·보호층·가드·기어·센서 코드는 바꾸지 않았다.
- 현재 소스는 r27 등록 이후 여러 파일이 바뀌었다. 그래서 A를 다시 실행해 같은 스냅숏의 무주입 기준으로 삼는다. r27 원 결과는 이력으로 보존하며 소급하지 않는다.

## 순서

1. `python3 -B register.py --check` → `python3 -B register.py`. 두 run이 하나의 `registered.json`·`source_snapshot.zip`을 공유하고, `A_no_injection/`·`B_c1_injection/` 폴더를 만든다. 이미 등록돼 있으면 재실행을 거부한다.
2. `bash run.sh A`가 끝나고 프로세스가 정리된 뒤 `bash run.sh B`를 실행한다. 동시 실행은 하지 않는다(preflight pgrep). 각 run은 다음을 거쳐 실행된다.
   - 등록 해시와 스냅숏을 대조한다.
   - `check_runtime.py`로 실제 import·launch 경로가 등록 src 바이트인지 확인한다(`runtime_check.json`).
   - 기존 출력이 있으면 거부한다.
3. 각 run은 자기 폴더에 다음을 남긴다: `run1/`(원 report), `run1.exit`(HARNESS·SURFACE·INTEGRITY·LEFTOVER·ASSESSMENT 종료 코드와 벽시각), `surface_run1.json`, `postflight_integrity_run1.json`, `assessment_run1.json`.
4. 두 run이 모두 끝나면 `python3 -B compare.py`를 실행한다(`pair_comparison.json`, 기록 전용).

## 판정

- **원 판정:** 각 `report.passed`와 `HARNESS_EXIT`를 그대로 보존한다. r27처럼 최종 간격 실패가 나올 수 있으며, 그 경우 실패로 둔다.
- **보충 판정**(`assess.py`, run별 독립)은 r27 12조건을 같은 한계값으로 적용한다. 조건은 유효, 한 랩 44.64 m, 접촉 0, 표본·스윕 간격 ≥ 0.08 m, 추가 actor 1대 유효, 겹침 0, 노면 이탈 ≤ 2바퀴·벽 0, STOP/정상 종료, 소스 불변(등록 수 전부·스냅숏), 잔여 프로세스 없음, 최장 저속 ≤ 2.5 s다. 여기에 다음 조건을 더한다.
  - A: `lidar_test_profile` null, 런타임 프로필 파일 없음, 취득 행에 `impairment`·`injected_delay_s` 없음, 취득 수 = 수신 수.
  - B: 등록 경로·값 일치, `integrity.lidar_stress_verified`, evidence `verified`와 6개 checks 모두 참, 런타임 프로필 불변, 추가지연 ∈ [0.03, 0.07] s, 0 < 최대 오차 ≤ 0.03 m(수치 허용 1e-8), 취득 = 수신 = evidence 수, unmatched·invalid·source_errors 없음.
  - 공통: 판단 호출의 스캔 시각이 모두 취득 기록에 있고, 판단 기록 footer의 dropped·failed·capture_errors가 0이다.
- 판단 시점 스캔 나이 분포는 기록만 한다. r27 A는 0.118/0.158/0.198/0.198 s였고, B는 약 +0.03–0.07 s 이동을 기대한다(판정선 아님).
- 판정선은 완화하지 않는다. 실패하면 증거를 보존하고, 변경 없이 재시도하지 않는다.

## 한계

합성 반경 오차와 발행 지연이며 C1 실측 잡음·USB·Jetson 처리 시간을 보증하지 않는다. 각본 두 앞차 한 장면이고 독립 5대 경기나 실차 안전 증명이 아니다. 보호층은 `/scan` 구독까지만 확인한다.
