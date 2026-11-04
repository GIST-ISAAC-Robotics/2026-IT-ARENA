# C1 장착 후보의 실제 주행 비교: R·JR·C — 2026-11-04

**결론(각 1회·이 시뮬 조건·현행 제어기 관측 계약 기준):** 지주와 뒤 마커 각도를 *알려진 구조 = unknown*으로 두는 시험 전용 선택만 추가했을 때, **JR은 R과 비슷한 랩으로 완주했다**(33.853 s 대 34.253 s). 같은 빌드 반복이 없어 이득·비열등을 확정하지 못한다. **C(앞 지주가 센서보다 앞)는 출발하지 못했다.** 앞 지주 unknown이 현행 전방 ±100° 관측 게이트 안에 들어가기 때문이다. 시험한 위치·지름·여유각 격자에서는 현행 관측 계약으로 고속 허가를 얻는 조합이 없었다. C가 물리적으로 불가능하다는 뜻은 아니다. 관측·자기반환 필터 정책을 다시 설계하면 가능할 수 있으나, 그것은 좌표 YAML 변경이 아니며 비용과 달성 성능은 측정하지 않았다. 지주 반환을 **그대로 처리하면 JR·C 모두 출발하지 못했다.** 현행 계약에서는 실측 장착에 맞춘 구조 각도 마스크(또는 동등한 자기반환 처리)가 필요하다.

등록 계획: [PLAN.md](PLAN.md). 각 1회 실행이며 자동 재실행하지 않았다. 실행·대기·회수는 Claude Haiku 5.5 보조(`haiku` 별칭, 자체 보고 `claude-haiku-5-5`)가 맡았고, 진단·문서는 Claude Opus 5.5가 맡았다. 구형 STEP 판독은 [CAD 참고](../lidar_mount_cad_reference/README.md)(비최종 참고 전용)에 따로 있다.

## 같은 빌드 비교(11/4, 같은 월드·속도 프로필·장면, 센서 장착 YAML만 다름)

| 실행 | 장착 | 출발 / 진행 | 랩 · 최고 속도 | 0명령 / 보호층 사유 | 접촉·노면 / STOP·종료 |
|---|---|---|---|---|---|
| [`r_lap1`](r_lap1_eval.json) | **R 대조** +60/95 mm, 후방 ±30° 마스크 | 0.508 s / 1랩 48.693 m | **34.253 s · 2.439 m/s** | 0.32 s / clear 34.25 s, 전방 미관측 1.79 s | 0·0 / 안정·15개 정상 종료 |
| [`jr_unk_lap1`](jr_unk_lap1_eval.json) | **JR unknown** +50/75 mm, Ø7 (+10,±62)(−80,±52), 정후방 마스크 없음 | 0.553 s / 1랩 48.653 m | **33.853 s · 2.409 m/s** | 0.56 s / clear 33.81 s, 전방 미관측 0.18 s | 0·0 / 안정·정상 종료 |
| [`c_unk_lap1`](c_unk_lap1_eval.json) | **C unknown** +30/95 mm, Ø7 (±80,±62) | **출발 실패** / 0 m | — | 7.04 s 전부 `wide_or_many_missing`(추종기·보호층 모두), 전방 미관측 7.19 s | 0 / 검사기 "주행 정지 상태 지속" |
| [`jr_raw_s1`](jr_raw_s1_eval.json) | JR, 지주 반환 그대로 | **출발 실패** / 0 m (25 s 제한) | — | 7.06 s 0명령(`follow_steer_in_place`), 보호층 스윕 간격 0 m | `report.json` 없음(아래 결함) |
| [`c_raw_s1`](c_raw_s1_eval.json) | C, 지주 반환 그대로 | **출발 실패** / 0 m (25 s 제한) | — | 7.02 s `follow_boundary_unconfirmed`, 스윕 간격 0 m | 0 / "주행 정지 상태 지속" |
| [`jr40_unk_lap1`](jr40_unk_lap1_eval.json) | JR 변형: 센서만 +40 mm(앞 지주와 x 간격 30 mm) | 0.556 s / 1랩 48.673 m | **35.806 s · 2.370 m/s** | **1.36 s**(`release_wait` 1.00, `partial_transformed_gap_uncovered` 0.28) / clear 35.81 s, 전방 미관측 0.18 s | 0·0 / 안정·정상 종료 |

**교통(11/3 오른쪽 후방 재접근 각본, 현행 교통 옵션):**

| 실행 | 진행 · 최고 | 최소 간격(표본/스윕) | 접촉·겹침·노면 | 상대 1 m 안 반환 3점 미만 스캔 | 원본 판정 |
|---|---|---|---|---|---|
| [`r_rr1`](r_rr1_eval.json) R | 3.020 m · 0.974 m/s | 84.89 / 84.88 mm | 0·0·0 | **36개 중 7개(연속 7)** — 후방 마스크 사각 | final_gap·옛150mm·뒤 유지 실패(11/3 v3와 같음), STOP·종료 통과 |
| [`jr_rr1`](jr_rr1_eval.json) JR | 3.020 m · 0.974 m/s | 84.98 / 84.98 mm | 0·0·0 | **36개 중 0개**, 전체 57스캔 중 3점 이상 56(R 36) | 같음 |

C는 이번 단독 시험에서 출발하지 못해 교통 장면을 실행하지 않았다. 원본 평가기의 옛 판정 실패는 성공으로 바꾸지 않는다.

### 해석

- **JR과 R의 랩 차이(−0.40 s)를 장착 효과로 해석하지 않는다.** [구간 비교](comparison.json)에서 차이 대부분(−0.97 s)은 40–44 m에서 R이 알려진 43 m 부근 정체를 겪은 결과이고, 나머지 구간 합은 JR이 +0.3 s 느렸다. 같은 빌드 반복 실행이 없으므로 반복 변동 폭은 측정하지 않았다(11/3의 다른 빌드 R 34.763 s는 반복 분포의 근거가 아니다). 관측된 것은 **비슷한 랩**이며, 이득이나 비열등은 확정하지 못한다.
- **정지·접촉:** JR은 영구 정체·접촉·노면 이탈이 없었다. 다만 짧은 0명령은 R 0.32 s → JR 0.56 s였고, R에 없던 일시 사유 `follow_forward_gap_too_wide` 0.06 s·`follow_partial_transformed_gap_uncovered` 0.04 s가 나타났다. 보호층에도 `forward_gap_too_wide` 0.046 s가 있었다. 1회 실행이라 이 일시 차이의 원인은 분리하지 않았다.
- JR은 정후방 마스크를 없애고 지주(±123°, ±158°)·뒤 마커(180°±9.9°)만 unknown으로 둔다. 따라서 JR에도 구조 사각은 남는다. 이 시험의 각도 마스크 합계는 JR 약 40.47°, C 약 43.12°이며, 지주·뒤 마커·각 구간 여유를 포함한 설정값이다(실물 사각 실측값 아님). 이 교통 각본에서는 **시험한 상대가 1 m 안에 있을 때 '반환 3점 미만' 스캔이 R 7개에서 JR 0개로 줄었다.** 이는 이 상대·이 기준에 한한 결과이며 후방 사각 전체가 없어졌다는 뜻이 아니다. 제어 결과(진행·간격)는 같았고, 후방 관측 개선이 주행 이득으로 이어지는 장면은 확인하지 않았다.
- **C는 현행 관측 계약에서 출발·고속 허가를 얻지 못한다.** `bounded_local_v4` 부분 관측은 전방 ±100° 결측 합계 6°·연속 3°(측방 거리 검사 전) 이하만 허용하고, 결측이 있으면 측정 속도 1.2 m/s 초과를 불허한다(`partial_observation.py:125, 149-154`). C 앞 지주는 ±51°에 각 6.0°다. [기하 예산](front_gate_budget.json): C는 ±10 mm·Ø3.6–10·여유 0–1° 모든 변동에서 전방 unknown이 생기고, 6°를 넘지 않는 것은 Ø≤4 mm·여유 0°뿐이다(그래도 매 스캔 1.2 m/s 상한 경로). C45(+45)도 앞 지주 중심이 약 61°라 같은 계약에 걸린다(계산만, 미실행). 판정선을 바꾸지 않았다. C를 쓰려면 알려진 구조 각도를 전방 결측 예산·속도 상한에서 어떻게 다룰지 관측/보호 정책을 재설계해야 하며, 이번 시험 범위 밖이다.
- **raw는 현행 보호 계약에서 출발 불가:** 지주 반환(JR 앞 70–73 mm·뒤 137 mm, 마커 150 mm, C 앞 76–78 mm)은 차체 외곽 안 점이다. 추종기는 장애물/경계 오류로, 보호층은 스윕 간격 0 m로 처리한다. 현행 코드에서 필터 없이 원시 스캔을 쓰는 구성은 출발하지 못할 것으로 본다(시뮬 확인, 실차 미검증).

## 미세 위치 민감도(개루프 shadow·기하, 폐루프 결과 아님)

[`shadow_jr_unk_lap1.json`](shadow_jr_unk_lap1.json): JR 1랩 저장 입력 1,803판단을 그대로 재생(기록과 완전 일치)한 뒤 변동만 바꿨다. 기록 스캔은 명목 구조 각도가 이미 NaN이라 **명목∪변동** 마스크만 표현할 수 있다. Ø4(구형 CAD 외경 3.6/4.0 mm 축척)는 명목 Ø7 안에 포함되므로 명목이 보수 상계다. 내부 상태 때문에 작은 변화도 첫 판단부터 갈라져 "바뀐 판단 수"는 의미가 작다. 평균 명령 속도·0명령 호출 수만 본다.

| 변동 | 평균 명령(m/s) | 0명령 호출 | 해석 |
|---|---:|---:|---|
| 기록 재생 | 1.588 | 28 | 기준 |
| 보정 오차 lidar_x ±5/±10 mm | 1.577–1.587 | 31–38 | 작음 |
| 지름 8/10 mm, 여유각 1.0/1.5° | 1.581–1.584 | 31–36 | 작음 |
| 뒤 지주 x ±5/10, 반간격 ±5 mm | 1.588 | 28 | 변화 없음(28.9 s 이후 20–26판단만 미세 변화) |
| 센서 +5/+10 또는 앞 지주 −5/−10 mm(간격 45–50 mm) | 1.580–1.584 | 31–33 | 작음 |
| **센서 −5 / 앞 지주 +5 mm(간격 35 mm)** | 1.565 | 57 | 악화 시작 |
| **센서 −10 / 앞 지주 +10 mm(간격 30 mm)** | **1.510** | **139** | 12–18 s에 `release_wait`·`partial_transformed_gap_uncovered` 증가([사유](shadow_hold_reasons.log)) |
| 지주 반환 그대로 Ø7 / Ø4 | 0 | 1,803 | 전 구간 정지, 스윕 간격 0 m. **지름을 줄여도 해결되지 않음** |

같은 스캔의 보호층 전방 판정은 모든 마스크 변동에서 기록과 같았다(ok 1,793 / 미관측 10). [기하 예산](front_gate_budget.json)에서도 JR은 모든 변동에서 전방 unknown 0°다. 간격 30 mm 쪽의 개루프 악화가 실제인지 `jr40_unk_lap1` 폐루프로 확인했다. **같은 사유(`release_wait`·`partial_transformed_gap_uncovered`)로 0명령 0.56→1.36 s, 랩 33.853→35.806 s**였고 손실은 16–32 m 구간(+0.5~0.9 s씩)에 모였다. 1회 실행이므로 이 차이는 개루프 결과가 뒷받침하는 **연관**이며, 약 2 s라는 크기를 정밀한 인과 추정으로 쓰지 않는다. 앞 지주 unknown이 ±116°(가장자리 ±112°)로 들어오면 측방 변환 공백 검사에 걸리는 것으로 보인다(추정, 코드 경로 미분리).

## 형상·미세 위치 판단과 날짜별 권고

**기구 요구(이 결과에서 나오는 것만):**
1. 현행 관측 계약을 유지한다면 스캔면 높이 고정 구조의 **가림 범위 전체**(중심이 아니라 지름·여유각을 포함한 각도 폭)를 센서 기준 **전방 ±100° 밖**에 둔다. C·C45처럼 앞 지주가 센서보다 앞이면 이 조건을 어긴다. 어기는 배치를 택하면 정책 재설계가 먼저다.
2. JR 형태의 **잠정 참고값:** 앞 지주를 센서 회전축보다 x로 **40 mm 뒤**(|y|=62 mm, Ø7, 여유 0.5°)는 폐루프 1회에서 작동했다. 30 mm 변형은 1회 폐루프에서 나빠졌고 개루프도 같은 방향이었다. 45–50 mm는 개루프 차이가 작았다. 40 mm는 검증된 최소값도, 모든 조건에서 충분한 공차도 아니다. 다른 |y|·지름·여유각에서는 다시 확인해야 한다.
3. 뒤 지주의 x ±10 mm·반간격 ±5 mm, 지름 Ø4–10 mm, 여유각 0.5–1.5°, 보정 오차 ±10 mm는 이 시험 범위에서 성능에 작은 영향만 보였다(개루프). 이 범위 안에서는 수 mm 정밀도보다 **실제 구조 각도를 정확히 unknown 처리하는 것**이 더 중요해 보인다(개루프 근거).
4. 광축 75 mm와 95 mm의 차이는 이 시뮬에서 분리하지 않았다. JR은 75 mm, C는 95 mm다. 낮출수록 실제 차체·바퀴·배선 가림의 위험이 커지며, 이는 최종 CAD 단면으로 확인해야 한다.

**개발 권고:**
- **지금(11/4–11/5):** 장착과 무관한 공통 속도·교통 개발은 계속할 수 있다. 현행 R 프로필로 진행하고 JR 시험 프로필은 opt-in으로 유지한다. 측방·후방 관측이나 결측 처리에 민감한 변경은 R/JR 교차 확인을 검토한다.
- **배치 결정(빠르면 11/5) 뒤:** 결정 형태가 "가림 범위 전체가 전방 ±100° 밖, 앞 지주는 잠정 참고값 40 mm 뒤 근처"이면 **설계 좌표로 바꾼 YAML로 단독 1랩·후방 교통 1장면을 다시 확인**한다(실행 약 15–20분). 이를 벗어나면 정책 재설계 여부를 먼저 결정하며, 판정선을 조용히 완화하지 않는다.
- **세부 설계(11/7):** 최종 CAD의 치수는 앞으로 정해질 **설계 좌표**다. 그 좌표로 시뮬 YAML을 갱신해 위 확인을 반복한다. 11/7에 실물이 있다고 가정하지 않는다.
- **실물 제작 뒤:** 빈 장착대 정지 스캔으로 실제 자기반환 각도를 측정해 구조 각도를 확정하고, 두 노드에 같은 값을 넘기는 실차 launch를 작성한다. 임시 R을 JR로 바꾸는 기본값 변경은 실측 전에는 하지 않는다.

## 코드 변경과 실제 옵션 연결 범위

기본 동작은 그대로다. 시험 전용 키가 없는 YAML·인자 미지정에서는 기존 실수 마스크와 모델을 그대로 쓴다.

| 파일 | 변경 |
|---|---|
| `src/arena_autonomy/arena_autonomy/local_scene.py` | `blind_spec`/`blind_parameter`: 후방 반각 실수 외에 `sectors_deg`([중심, 반폭])를 masked로 처리 |
| `src/arena_autonomy/arena_autonomy/local_path.py` | `mask_scan`이 같은 각도를 NaN(unknown)으로 둔다 |
| `src/arena_autonomy/arena_autonomy/local_pursuit.py`, `lidar_safety.py` | 파라미터 `lidar_blind_sectors_json`(기본 `''`) |
| `src/arena_bringup/launch/simulation.launch.py` | `_mount_trial`: YAML의 `scan_plane_posts`·`chassis_visual_top_m`·`known_structure_mask`·`known_structure_margin_deg`가 있을 때만 지주 표시 형상(질량·충돌 불변), 차체 표시 상자 윗면, 지주·뒤 마커 각도 계산·전달, `lidar_blind_sectors.json` 기록 |
| `scripts/validate_basic_autonomy.py`, `scripts/validate_g1_traffic.py` | `--local-sensor-config`(기본 미지정) |
| `tests/test_lidar_mount_trial.py`(신규), `tests/test_source_provenance_r0_plus_b3.py` | 기본 불변·각도 계산·raw/unknown 분리 6건, 후속 계약 사슬 1줄 |
| `config/tests/lidar_mount_trial_r1_sha256.json`(신규) | 부모 `current_lane_preference_r1`, 수정 전 7파일 [스냅샷](runtime_before.zip)과 [재구성 근거](reconstruct_before.py)(11/3 마지막 실행 스냅샷과 바이트 일치) |

실행 도구: [`run_solo.sh`](run_solo.sh)/[`run_solo.py`](run_solo.py)(11/3 선정 1랩 등록 인자 재사용), [`run_traffic.sh`](run_traffic.sh), [`evaluate.py`](evaluate.py), [`compare.py`](compare.py), [`shadow_replay.py`](shadow_replay.py), [`front_gate_budget.py`](front_gate_budget.py). 시험 프로필: [`profiles/`](profiles/)의 `jr_unknown`·`jr_raw`·`c_unknown`·`c_raw`·`jr40_unknown.yaml`. 기본 `b0183_c1.yaml`은 바꾸지 않았다.

**실차·실행 구성으로 옮길 때 최소 범위:** (1) 확정 기구의 실측 회전축 x/y/z를 YAML `xyz_m`에 넣는다(TF·운동 보정 오프셋은 이미 이 값 하나에서 두 노드로 전달됨). (2) `masked_rear_half_angle_deg: 0`과 구조 각도 목록을 둔다. 각도는 도면값에 맡기지 말고 **빈 장착 상태 정지 스캔의 자기반환 각도**로 확인한다. (3) 실차용 launch는 아직 없으며, 모든 실행이 `simulation.launch.py`를 거친다. 실차 launch가 생기면 `rear_blind_half_angle_deg`와 `lidar_blind_sectors_json`을 `local_pursuit`·`lidar_safety` **두 노드 모두**에 같은 값으로 넘겨야 한다. 한쪽만 넘기면 보호층과 계획기의 관측이 달라진다. JR에서는 제어 파라미터·속도 프로필·보호 판정선의 변경이 필요 없었다(C는 정책 재설계가 필요하며 범위 미측정).

**비용 질문에 대한 코드 규모:** 수정 전 [스냅샷](runtime_before.zip) 대비 7파일 **+134/−3줄**이다. 그중 실행 경로(관측·마스크·노드)는 `local_scene.py` +37/−2, `local_path.py` +7/−1, `local_pursuit.py` +4, `lidar_safety.py` +4로 **+52/−3줄**이다. 시뮬 모델·실험 배선은 `simulation.launch.py` +72, 검사기 2개 +10이다. 그 밖에 신규 테스트 1개, 계약 JSON 1개, 사슬 목록 1줄이 있다. 배치가 정해진 뒤 JR 계열 좌표로 옮기는 일은 코드 변경 없이 YAML 값과 확인 실행(단독 1랩+교통 1장면)이면 된다. 실차 launch 작성은 별도 작업이다.

## 확인한 검사

- 관련 회귀 [`tests.sh`](tests.sh) → [로그](tests.log): **285 통과, 2 실패**(exit 1). 두 실패는 기존 것이며 **미해결로 남긴다**(이번 범위 밖, 수정하지 않음). `test_r0_safety_contract`는 이번에 건드리지 않은 `drivetrain.hpp` 해시(HEAD d8f47117 값과 계약값이 다름)에서, `test_bypass_corridor_trial`은 수정 전 [스냅샷](runtime_before.zip)에도 있던 `g1_g2_corridor_policy` 문자열에서 실패한다. 출처 사슬 `test_source_provenance_r0_plus_b3.py` 81건은 후속 계약 추가 뒤 모두 통과했다.
- [등록 소스 대조](check_sources.log): `jr_raw_s1`만 수정한 검사기·계약·평가기 3파일이 다르고(위 결함), 나머지 실행은 등록 이후 변경 0이다. 기본 `b0183_c1.yaml`은 HEAD와 같다.
- 주행한 R/JR/JR40 단독과 교통 2개는 모두 STOP 안정, 하위 프로세스 정상 종료, 잔여 PID 없음이다. 노면 평가에서 바퀴 이탈·벽 간섭 0이다.

## 결함·한계

- **결함(수정함):** 처음 추가한 `--local-sensor-config`가 `Path` 형이라 `report.json` 직렬화가 실패했다(`jr_raw_s1`만 해당, 실행 기록은 보존). 문자열형으로 고치고 계약을 다시 생성했다. 이후 실행은 `source_unchanged: true`다.
- 각 1회. 랩 변동 폭을 재지 않았다. 교통은 한 각본(후방 재접근)뿐이며 전방 끼어들기 등은 실행하지 않았다.
- JR 75 mm는 단순 차체 **표시** 상자 윗면을 65 mm로 낮춘 경우다. 실제 차체·배선·범퍼·펜더·광학창 상하 크기·피치 진동은 모델 밖이다. 이상 타이어 윗면은 65 mm(구형 CAD 타이어 외경 64.4 mm·폭 26.1 mm)다. 높이 여유는 최종 CAD 단면으로 확인해야 한다.
- 뒤 ID49 마커(z 65–115 mm)가 두 스캔면을 모두 지나므로 정후방 약 20°는 JR에서도 unknown이다. 마커를 스캔면 밖으로 옮기면 더 열린다(미검증).
- 지주는 Gazebo의 이상 원통이다. 반사율·광학창 유한 크기·C1 거리 잡음·혼합 화소는 없다. 여유각 0.5°는 시뮬 점광선 기준이다.
- shadow는 개루프 반사실이고 보호층 전체를 재생하지 않는다.

## 재현

저장소 루트(WSL, Jazzy). 실행 하나씩, 동시 실행 금지.

```text
bash artifacts/validation/2026-11-04/lidar_mount_performance_v1/tests.sh
bash artifacts/validation/2026-11-04/lidar_mount_performance_v1/run_solo.sh r_lap1 -
bash artifacts/validation/2026-11-04/lidar_mount_performance_v1/run_solo.sh jr_unk_lap1 jr_unknown.yaml
bash artifacts/validation/2026-11-04/lidar_mount_performance_v1/run_solo.sh jr_raw_s1 jr_raw.yaml 25
bash artifacts/validation/2026-11-04/lidar_mount_performance_v1/run_traffic.sh jr_rr1 right_reapproach jr_unknown.yaml
python3 -B artifacts/validation/2026-11-04/lidar_mount_performance_v1/shadow_replay.py jr_unk_lap1 jr 16
python3 -B artifacts/validation/2026-11-04/lidar_mount_performance_v1/front_gate_budget.py
python3 -B artifacts/validation/2026-11-04/lidar_mount_performance_v1/compare.py
```

원시 실행 폴더(`*/`)는 기존 정책대로 로컬 보존이다.
