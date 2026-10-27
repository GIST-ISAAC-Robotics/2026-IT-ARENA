# 통과 선택기의 정상 코너·앞차 정지/재개 연결

2026-10-27. 기본 `g1_pass_release_trial=false`를 유지한다. [이전 묶음](../g1_pass_release_integration_v1/README.md)의 run4/run5 소스(`pass_release_trial.py` `96c3f592…`)에서 출발했고, 독립 보호층·차량 크기·물성·센서 배치·원래 판정선은 바꾸지 않았다. 평가 정답(actor/자차 자세)은 아래 모든 분석에서 사후 분류에만 썼다.

## 1. run4/run5의 양수 속도와 원래 앞차 간격 실패

[`end_limit_audit.py`](end_limit_audit.py)는 기록 입력으로 장면을 복원하고, 기록된 `swept_clearance_m`와 목표 속도를 같은 함수로 다시 계산해 **불일치 0**을 확인한 뒤 `swept_limit`과 같은 진행 격자의 첫 간섭 반환을 모았다. 반환은 그 광선 취득 시각의 정답 상자(3 cm 허용)로만 분류했다. 요약은 [`end_limit_summary.json`](end_limit_summary.json), 행 단위 값은 로컬 `end_limit_audit.json`(Git 제외)이다.

| 항목(RELEASED 뒤 G1 출력) | run4 왼쪽 | run5 오른쪽 |
|---|---:|---:|
| 판단 수(전체 / 종료 전 3초) | 473 / 198 | 476 / 196 |
| 첫 간섭 반환의 주 분류 | 옆차 468·벽 5 / 옆차 198 | 옆차 471·벽 5 / 옆차 196 |
| 원래 앞차 반환이 보인 판단 | 473(4–7개) | 476(3–7개) |
| 속도 제한 항 | 전부 차간시간 `available/0.8 s` | 같음 |
| 종료 전 3초 clearance(팽창 포함 / 팽창 0) | 0.36–0.60 / 0.52–0.68 m | 같음 |
| 종료 전 3초 첫 간섭 반환의 팽창량 | 0.087–0.129 m | 0.090–0.132 m |
| 종료 전 3초 명령 속도 | 0.25–0.55 m/s | 같음 |
| 옆차 중심의 자차 기준 횡/종 위치 | +0.130 / 0.72–0.92 m | −0.130 / 0.73–0.93 m |

- 옆차(폭 0.15 m)의 중심 횡간격 0.13 m는 자차 폭 0.15 m와 **약 2 cm 겹친다**. 통과한 옆차가 자차 경로 앞에 들어와 있었고, 원래 앞차의 반환은 계속 보였지만 첫 간섭은 한 번도 아니었다.
- 끝까지 양수였던 속도는 0.45 m/s로 움직이는 옆차를 차간시간 항으로 따라간 결과다. 원래 앞차 최종 간격 1.720/1.714 m 실패는 그 사이에 옆차가 끼어 있었기 때문이며, 정체가 원인이었던 run1/2와 다르다.
- 설명용 반사실(옆차로 분류된 반환만 빼고 같은 계산)에서는 종료 전 3초의 목표가 모두 상한 1.0 m/s(`cap`)였다. 제어 출력도, 그렇게 움직였을 것이라는 증거도 아니다.
- **기존 1.2 m·150 mm 판정은 바꾸거나 재실행해 맞추지 않았다.** 이 오버레이는 등록 당시부터 "명목 측방 여유가 기준보다 작아 전체 통과를 기대하지 않는" 진단 장면이었다.

## 2. 저장 입력에 선택기 ON 비교(개루프, 동일 재생 아님)

[`compare_saved_selector.py`](compare_saved_selector.py)는 선택기 OFF로 기록된 G1 입력에 같은 구성 + 평행/선택기 ON을 적용한다. [`compare_cross_inputs.py`](compare_cross_inputs.py)는 M3로 기록된 센서 입력에 run4 운영 구성의 G1 OFF/ON을 나란히 적용한다. 입력 궤적은 기록 당시 제어기의 것이므로 첫 명령 분기 뒤는 참고값이다.

| 입력 | 판단 | 측방 위협 | 수정 전(run4 소스) | 수정 뒤(`ff310084…`) |
|---|---:|---|---|---|
| [단독 1랩 run11](../g1_partial_integration_v1/README.md) | 2,951 | 0 | 전부 SOLO, 명령 2,930 동일 | 2,951 동일·세부 판단 2,951 동일 |
| [직선 앞차 정지·재개](../g1_partial_traffic_v1/README.md) | 789 | 0 | 전부 SOLO, 787 동일 | 789 동일 |
| [코너 내부 앞차 정지 run51](../../2026-10-25/m3_execution_review_v1/run51_curve_stop_restart/README.md)(M3 입력) | 2,046 | 0 | 전부 SOLO, 2,043 동일 | 2,046 동일 |
| [잔디 옆차 뒤 첫 코너 run32](../../2026-10-25/m3_execution_review_v1/run32_corner_after_threat/README.md)(M3 입력) | 2,036 | 24판단(11.82–20.92 s) | 11.82 s부터 끝까지 GUARD, OFF 양수·ON 정지 753판단, 최장 12.34 s(32.20–44.54 s), 평행 후보 `reference_not_straight` 732 | 24.02 s RELEASED, OFF 양수·ON 정지 46판단(전부 해제 전), 최장 0.48 s |

정상 벽·U자 코너·코너 안 정지 앞차를 옆차로 묶은 판단은 세 입력 모두 **0회**였다. 수정 전 차이는 두 종류였다.

1. **SOLO의 역행 정지 조향:** 제동 중 음수 모터축 속도에서 기존 G1은 속도0·조향0을 내지만 선택기는 SOLO에서도 직전 조향을 유지했다(21/2/3판단, 속도 차이 0, 조향 차이 최대 0.30 rad). SOLO가 "기존 G1 출력 그대로"라는 계약과 달라 폐루프에서는 이후 조향 이력이 갈라질 수 있다.
2. **`rear` 영구 GUARD와 코너 교착:** run32 입력에서 벽 3 cm 앞 잔디 옆차의 반환이 벽 반환과 한 묶음이 되며 `compact` 조건이 꺼졌다. 옆차가 실제로는 옆에 있는데도(사후 정답 +0.25 m, −0.10 m) 위협 소실로 처리됐고, 그때 최후단 ds 0.099 m가 앞면+0.10 m(0.20 m)에 못 미쳐 `rear`로 기록됐다. 정지한 옆차를 지나친 뒤에도 같은 `rear`였다. `rear`는 해제 경로가 없으므로 직선 전용 평행 후보만 남아 코너에서 계속 정지했다. [장면별 조건 분해](vanish_probe.json).

## 3. 최소 수정과 반례

`src/arena_autonomy/arena_autonomy/pass_release_trial.py`만 바꿨다.

- **SOLO 역행:** 위협을 본 적이 없고 기존 G1이 이미 정지(속도 ≤ 0)를 냈으면 그 출력과 조향을 그대로 쓴다(`solo_reverse_baseline_stop`). 기준 출력이 비정상적으로 양수면 종전처럼 정지·직전 조향을 유지한다. GUARD/RELEASED의 역행 처리는 그대로다.
- **`rear`/`unknown` 해제:** 기록은 지우지 않는다. 기존 15 cm 옆 띠를 측방 위협 창 전체(자차 원점 뒤 0.60 m–앞 0.35 m, 기존 `band_behind_m`/`band_ahead_m`)로 늘린 영역이 같은 규칙(마스크 외 전부 양의 관측, 양의 관측 1개 이상)으로 서로 다른 fresh 스캔 2회 연속 확인될 때만 통과로 본다. 띠만으로는 여전히 해제하지 않는다. 벽과 묶여 compact가 꺼졌어도 아직 옆에 있는 물체는 뒤 표본을 가리므로 승인되지 않는다. 해제 뒤 G1 출력은 종전 1 cm 횡변위·차체 이동 영역 검사를 그대로 받는다. 새 수치는 없다.
- **남는 위험:** 자차 뒤 마스크 원뿔에 숨은 물체의 부재는 증명하지 않는다. run32 입력의 해제 시점에 옆차 중심은 사후 정답상 자차 기준 x −0.552 m·횡 0.335 m로 띠 밖이었고 앞부분이 마스크 안에 있었다([`run32_release_position_eval.json`](run32_release_position_eval.json)). 이는 ADR 0027의 "순수 후방 미관측만으로 정상 전진을 무기한 막지 않는다"는 개발 권고 범위의 잔여 위험이며, `unknown_filled=false`는 유지된다.
- 반례 회귀: SOLO 역행 기준 유지·비정상 양수 차단, GUARD 역행 조향 유지, 띠만으로 `rear` 미해제, 창 전체 2회 확인 뒤 해제와 중간 가림 시 초기화, 해제 뒤 재접근 시 GUARD·`rear` 유지, 센서 재설정 뒤 `unknown` 해제, 창 기하가 기존 띠의 종방향 확장인지. `tests/test_pass_release_trial.py` 58개 통과(run1 뒤 6절의 실제 광선 반례를 더해 67개).

[새 소스 검증](new_source_v1/exit_codes.txt): run4/run5 기록 **657/657·648/648 동일 재생**(옆차 쪽 `rear`·SOLO 역행이 없던 기록), 선택 OFF 기록(평행 왼쪽 648·앞차 789) 동일 재생, 위 표의 수정 뒤 비교. run32 입력의 RELEASED 1,027판단은 작은 이동 650·관측된 큰 이동 262·작은 가림 꼬리 67(최대 0.0001 m)이었고 미관측 거절은 0이었다.

## 4. 출처 계약

- `config/tests/partial_observation_r0_sha256.json`: `dependencies`의 `pass_release_trial.py`만 `ff310084…`로 갱신했다. before·approved 기록은 그대로다.
- 관련 회귀 중 출처 사슬 1건이 **이번 수정 전부터** 실패하고 있었다. run4 이전에 바뀐 `scripts/profile_follow_trace.py`·`scripts/prepare_jetson_bundle.py`(현재 바이트 = run4 manifest)가 최신 계약에 기록되지 않아 m5 계약의 `after`(= HEAD `0e6c28a`)와 이어지지 않았다. 이전 계약의 before zip에는 두 파일이 없어 같은 계약에 행을 추가할 수 없으므로, 새 후속 계약 `config/tests/profile_bundle_selector_r0_sha256.json`(제어기 변경 없음, 두 평가 스크립트만)을 [`prepare_profile_contract.py`](prepare_profile_contract.py)로 만들고 HEAD의 before 바이트를 [`profile_contract_before.zip`](profile_contract_before.zip)에 보존했다. 사슬 목록 4개 시험 파일에 이름만 추가했다. 이 기록은 Jetson 실행 검증이 아니다.
- 실행 기록: 첫 회귀 시도는 실행 스크립트가 ROS `PYTHONPATH`를 덮어써 수집 오류만 났다(`related_tests_attempt1_pythonpath_error.log`, 시험 본문 미실행). 두 번째는 위 출처 사슬 1건 실패(`related_tests_attempt2_provenance_fail.log`). 세 번째 [`related_tests.log`](related_tests.log)는 [목록](related_tests.txt) 20개 파일 **571 통과**, [2패키지 빌드](build.log) 성공. 이 상태를 run1 소스로 고정했다.
- run1 뒤 첫 전체 회귀는 5,883 통과·19 건너뜀·**2 실패**였다([기록](full_regression_attempt1_manifest_fail.log)). 이것도 run4 이전부터 있던 누락이다: `local_follow.py`가 지연 import하는 `parallel_lane_trial`·`pass_release_trial`이 `scripts/validate_pipeline_faults_ros.py`의 ROS 고장 시험 보존 목록 `MODULES`에 없었다. 두 이름만 추가하고, `partial_observation_r0_sha256.json`의 이 스크립트 `after`와 사유만 갱신했다(`before`·zip 유지). 이에 따라 새 후속 계약의 `parent_contract_sha256`도 갱신했다. 대상 242개 통과 뒤 최종 [전체 회귀](full_regression.log)는 **5,885 통과·19 건너뜀·실패 0**이다.
- run1 뒤 바뀐 파일은 위 계약 2개·보존 목록 스크립트·선택기 시험 파일뿐이며 런타임 선택기(`ff310084…`)는 run1과 같다([대조](post_run_source_diff.log)).

## 5. 대표 폐루프 1회 (run1): 정지·재개는 통과, 옆차 뒤 첫 코너는 실패

앞차 정지·재개와 첫 코너, 실제 옆차→코너 전환을 한 번에 보기 위해 10/25 run32와 같은 등록 조건 `config/tests/g1_corner_after_threat_v1.json`(`stop_restart_first_curve`, 앞차 0–4 s 0.35 m/s·4–9 s 정지·이후 첫 코너)과 `config/tests/e3_stop_restart_corner_v1.json`(잔디 옆차 +0.32 m, 0–15 s 0.35 m/s 뒤 종점 정지)을 썼다. 차이는 제어 구성뿐이다: G1 v4 + corner-distance + 점별 점유 + corridor + 평행 후보 + 통과 선택기 ON. 정적 고정 소스 960파일([`run1_snapshot.py`](run1_snapshot.py)), [`run1_trial.sh`](run1_trial.sh) 1회, 실행·회수는 Claude Haiku 5.5 보조(별칭 `haiku`, 자체 보고 `claude-haiku-5-5`)가 맡았다. 재시도는 없다.

| 항목 | 결과 |
|---|---|
| 원래 40 s 판정 | **실패**(`passed=false`, `run_valid=true`, `harness_valid=true`). `final_gap`·`curve_exit_reached`·`curve_heading_changed` 거짓, 나머지 11관문 참 |
| 15 s 창 정지·재개(사후 분리, run27 재현 일치) | HOLD 3.336 s, 재개 0.650 s, 재개 뒤 1.820 m, 창 끝 간격 0.627 m — 네 관문 통과 |
| 코너 | 최종 s 53.492 m < 요구 56.433 m, 방향 변화 49.30°. 15 s 뒤 최장 정체 **15.846 s** |
| 간격·접촉 | 앞차 최소 표본/swept 0.3613 m, 옆차 0.1638/0.1638 m(150 mm 기준 통과), 접촉 이벤트 0, 차체 도로 밖 0, 바퀴 중심 22,041표본 전부 도로 안 |
| STOP·종료 | 명시 STOP·MCU software stop·밀집 정지 246구간(최대 0.0066 m/s), 14/14 정상 종료, 잔존 0 |
| 재현성 | 2,052/2,052 판단 동일 재생, 실행 전후 소스 960파일 불변 |

[원본 보고서](run1_corner/report.json), [창 분리·선택기 전이](run1_window_eval.json), [바퀴 중심](run1_wheel_centers.json). 같은 등록 조건의 run32(M3)는 진행 7.279 m·방향 1.544°·정체 11.572 s였다. 제어기가 달라 개선으로 주장하지 않는다.

### 정체 원인

선택기 전이: SOLO → 11.92 s GUARD(앞차 정지 중 잔디 옆차 통과) → 12.62 s 옆차가 아직 옆(사후 x +0.21 m)에 있는 채 위협 판정이 꺼져 `rear` → 19.52 s 정지 옆차 재관측 → 20.32 s 다시 `rear` → **39.32 s RELEASED** → 42.82–43.22 s 짧은 GUARD → RELEASED.

1. 20.5 s에 자차는 첫 코너 약 2.4 m 전(s 2.90 m)에서 **종점에 정지한 잔디 옆차 바로 옆**에 멈췄다(사후: 옆차 중심 자차 기준 x −0.145 m·횡 +0.314 m, 내측 모서리와 자차 옆면 사이 약 0.17 m).
2. 이 동안 기존 G1은 정상 `local_follow`였고 목표 대부분 1.0 m/s, swept clearance 1.04–1.08 m였다. GUARD의 직선 전용 평행 후보만 직선 적합 잔차 1.0–2.65 cm(> 1 cm)로 `reference_not_straight` HOLD를 냈다(정체 900판단 중 807).
3. 옆차가 옆에 남아 있으니 옆 띠 자체가 패딩 안에서 미관측이었다. [계측 동일 재생](run1_window_counterfactual.json): 20.3 s 뒤 확장 창이 2회 연속 빈 첫 시각 39.32 s, 뒤 범퍼~앞 0.35 m로 줄인 반사실 창도 38.62 s였다. **창의 뒤쪽 범위를 줄여도 정체는 거의 그대로였다.** 36.6 s 무렵 잔차가 1 cm 아래로 내려가 평행 후보가 조금 전진했고, 옆차가 창 뒤로 빠지자 해제됐다.
4. [설명용 반사실](run1_guard_counterfactual.json): GUARD 평행 정지 805판단에서 기존 G1 출력에 RELEASED 이동 검사를 적용하면 작은 이동 5·관측된 큰 이동 800, 미관측/가림 초과 0이었다. 즉 이번 병목은 해제 창이 아니라 **곡선에서 쓸 수 있는 GUARD 후보가 없다는 구조**다. 이 반사실은 채택안이 아니며, 옆에 물체가 남은 GUARD에서 G1을 허용하는 것은 설계 변경이므로 이번에 구현하지 않았다.
5. 42.82–43.22 s의 짧은 GUARD는 옆차가 3 m 이상 뒤에 있을 때 코너 안에서 옆 띠·창이 미관측 3–9표본으로 한때 비지 않아 생겼다. 코너 벽이 패딩 안에 들어온 것으로 **추정**하며 원인은 분리하지 않았다. 해제 상태도 매 fresh 스캔 다시 계산하는 기존 구조 때문에 코너에서 짧게 되돌아갈 수 있다.

## 6. rear/unknown 해제 조건의 검토 항목

- **넓은 창이 배제하는 것:** 측방 위협 창(−0.60…+0.35 m) 안, 옆면 밖 15 cm 띠의 **관측 가능한 부분**에 있는 물체. compact 판정이 꺼져 `present`가 아닌 물체(벽 병합·긴 반환열)도 그 뒤 표본을 가려 해제를 막는다. 기존 짧은 띠(−0.10…+0.10 m)만으로는 자차 옆뒤(x < −0.10 m)에 남은 물체를 놓친다. 실제 광선 반례: 벽에 붙은 옆뒤 물체는 `present=false`, 짧은 띠 통과, 확장 창 미관측 > 0으로 거절된다. run1에서는 실제로 정지 옆차가 옆에 남은 약 19 s(20.32–39.22 s) 동안 해제하지 않았다.
- **마스크 표본이 남은 해제의 잔여 위험:** 후방 ±30° 원뿔(라이다 x=+0.06 m 기준; 차체 뒤 범퍼에서 반폭 약 0.092 m, x −0.35 m에서 0.237 m, x −0.60 m에서 0.381 m) 안 물체는 배제하지 못한다. 원뿔에서 옆으로 나오는 물체는 다음 스캔(10 Hz)에서 관측되어야 다시 GUARD가 되며, 그 사이 이동은 점유 팽창의 0.5 m/s 물체 이동 상한 가정에만 기댄다. 15 cm 띠보다 바깥(+패딩)의 물체도 배제하지 않으며, 이는 benign 띠와 같다. `selector_unknown_filled=false`, `selector_hidden_rear_risk_unresolved=true`를 유지한다.
- **무제한 해제가 아님:** `rear`/`unknown` 기록은 지워지지 않고, 띠만으로는 계속 해제되지 않는다. 해제 뒤에도 매 fresh 스캔 창을 다시 확인하며 한 번이라도 비지 않으면 GUARD로 돌아간다. 해제 뒤 G1은 (a) 차체 모서리 횡변위 1 cm 이하(GUARD 평행 후보와 같은 한계)이거나 (b) 현재/직전 조향의 반응+제동 외곽이 관측된 경우만 허용된다. 그 방향 큰 횡이동은 미관측 1개, 1 cm 초과 가림 꼬리, 전면 가림 1개 중 하나라도 있으면 평행 후보로 거절된다(반례 3종). 큰 횡이동 자체를 일률 금지하지는 않는다.
- **뒤로 숨은 차 + 다른 물체 통과:** 실제 광선 반례에서 첫 물체가 벽에 붙어 옆뒤에 남은 채 두 번째 물체가 옆을 지나 최후단 0.2 m 이상 앞으로 빠져도 `vanish='rear'`와 GUARD가 유지된다. 첫 물체가 원뿔 안으로 완전히 숨으면 두 fresh 뒤 해제되며, 이 경우가 위 잔여 위험이다.
- **보존한 거절:** 해제 상태에서 시각 역행·세대 변경·장면 나이 0.3 s·비정상 속도는 이번 판단 정지·양쪽 `unknown`·GUARD로 돌아간다(반례 4종). 명시 STOP·독립 보호층·입력 만료 경로는 선택기 밖이며 바꾸지 않았다.
- 시험: `tests/test_pass_release_trial.py` 67개(초안 9개는 [`draft_rear_window_tests.py`](draft_rear_window_tests.py)에서 먼저 통과 후 이관, [`selector_tests_final.log`](selector_tests_final.log)).

## 7. 결론과 다음 권고

- 정상 단독 랩·직선/코너 앞차 정지 입력에서 선택기는 측방 오촉발 0회이며, 수정 뒤 저장 입력 판단은 OFF와 같다(개루프 비교). 폐루프 run1의 앞차 정지·재개 창도 통과했다. 단 **옆차를 지난 뒤 첫 코너 연결은 실패**했고 원래 판정·150 mm/1.2 m 기준은 그대로다.
- 다음 설계 후보(사용자/GPT 검토 필요): GUARD에서 곡선을 따라갈 후보. 예를 들어 기존 G1 경로의 경로 좌표 횡위치를 유지하는 곡선 평행 후보, 또는 옆 물체가 관측되는 동안 G1 출력을 같은 이동 영역 검사로 허용하는 방안. 후자는 run1 반사실에서 805판단 모두 검사를 통과했지만 GUARD 의미를 바꾸므로 반례(옆 물체 쪽 큰 횡이동, 가림 뒤 접근)와 함께 결정해야 한다.
- 해제된 상태의 코너 안 재확인 흔들림(42.82 s)은 원인 분리가 남았다. 기본 OFF·기본 strict를 유지하며 이번 결과를 실차·전체 랩·연속 충돌 없음으로 확대하지 않는다.
