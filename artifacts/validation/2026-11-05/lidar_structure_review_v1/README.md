# 11/4 C1 구조 적응 변경(r1–r6) 검토 v1

- 날짜: 2026-11-05 (KST). 검토·재현·수정은 Claude Opus 5.5가 했다.
- 대상: JR60P20+Ø20 배선 설정으로 채택된 구조 적응 변경이다. 범위는 `known_structure.py`, `partial_observation.py`, `local_follow.py`(증속·마스킹 연결), `local_scene.py`/`local_path.py`(마스크), `lidar_safety.py`, `simulation.launch.py` 전달이다. 전역 감사는 하지 않았다.
- **조사 시점 상태(00:56 KST):** 실제 결함 1건을 합성 입력으로 재현했고 수정은 하지 않았다. 01:15 KST 도구 만료 전에 계약 사슬과 회귀 기대값까지 바꾸는 수정은 서두르지 않기로 했다.
- **현재 상태(01:04 KST, 후속 지시로 수정 완료):** `local_follow.py` 최소 수정, 회귀, r7 소스 계약, 출처 사슬을 갱신했다. 결과는 [수정 결과](#수정-결과-r7)에 있다. 그 아래 점검 내용은 조사 시점 기록이다.

## 수정 결과 (r7)

- **변경:** `local_follow.py` 한 파일(+19/−5, CRLF 유지)만 바꿨다.
  - `_partial_command`가 경로 후보를 만들기 전에 같은 원시 스캔에 순수 함수 `known_structure.classify`를 적용한다. 이렇게 얻은 **이번 스캔의 설명된 구조 광선**을 그 호출 동안만 넘긴다(`structure_explained_now`, `finally`에서 None).
  - 구조 경로의 증속 후보 전제는 "±100° 전방 광선이 모두 반환이거나 설명된 구조 광선"일 때로 좁혔다.
  - 일반 결측이 있으면 증속 후보를 내지 않는다. 기존 기본 목표가 v4 예산·감속 판단을 받는다.
  - 정책 `evaluate`는 호출당 1회 그대로다. 거리 합성·자유 공간 가정·보호층 변경은 없다. `known_structure.py`·`partial_observation.py`·`lidar_safety.py`는 바꾸지 않았다.
- **보존:** 수정 전 원본은 `structure_gate_before.zip`(`local_follow.py` r4 after `6c43b470…`, 회귀 2파일)에 있다. 수정 전 재현 결과 `repro_candidate_gate.json`도 덮어쓰지 않았다.
- **재현 수정 후**(`repro_candidate_gate_after.json`): 정면 0° 일반 결측 1광선은 0.0 → **0.2 m/s**(`bounded_forward_missing`), 측면 79.9°는 0.0 → **0.5 m/s**(`bounded_side_missing`)가 되었다. 온전한 관측의 증속(2.77 m/s)은 그대로다.
- **회귀**(`tests/test_structure_route_speed.py`, 10개):
  - 기존 `[missing]` 정지 기대값을 다음 사례로 바꿨다: 정면·측면 일반 결측은 증속 후보 없이 0 < v ≤ v4 상한.
  - 추가: 양옆 결측으로 설명할 수 없는 지주 각도는 면제하지 않고 증속 전제도 되지 않음.
  - 추가: 측정 속도 자체가 1.3 m/s면 기존 v4 불허(정지)가 남음 — **이번에 바꾸지 않은 기존 제한**.
  - 추가: 정책 evaluate는 호출당 1회.
  - 유지: 지주만 가려졌을 때 >1.2 m/s 증속, 근접 반환 정지, 구조 정책 off 계약.
  - 새 4개는 수정 전 코드 사본에서 실패하고 나머지 6개는 통과했다(`tests_new_on_before.log`). 수정 후에는 10개 모두 통과한다.
- **관련 회귀 묶음**(`tests.sh` → `tests.log`): 312 passed, 2 failed. 두 실패는 모두 이번 변경과 무관한 기존 실패다.
  - `test_r0_safety_contract::test_current_g1b_r0_contract_allows_only_the_registered_launch_change`: 11/4 `lidar_jr60_adoption_v1/tests.log`에서도 같은 실패가 있었다.
  - `test_measured_lane_preference_provenance`: 바꾸지 않은 `local_pursuit.py`의 과거 스냅샷 바이트 비교다. 그 파일은 11/4 r1/r2에서 바뀌었다.
- **계약:** `config/tests/lidar_mount_structure_gate_r7_sha256.json`(parent r6, `register_contract.py`)을 추가하고, `test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS` 끝에 r7을 넣었다(CRLF 유지). 이전 계약은 수정하지 않았다.
- **저장 주행 replay**(`replay_representative.json`): 11/4 `lidar_jr_forward_v1/jr60p20_1`(JR60P20 단독 구조 고속 주행) 판단 1,776회를 재생했다.
  - 1,766회는 완전히 일치했다.
  - 나머지 10회(seq 1956–1965)는 기록 당시 일반 측면 결측(`bounded_side_missing`, 상한 0.5)이 있던 바로 이번 수정 대상 호출이다. **최종 속도·조향은 모두 같았다.** 달라진 것은 증속 후보 상세 필드(`open_route_*`, `observed_length_speed_cap_mps`, `constant_arc_target_mps`, `turn_preview`)뿐이다.
  - 그 당시 후보가 1.2 m/s 이하여서 결과가 같았다.
- **Gazebo:** 새 위험이 없으므로 주행은 하지 않았다. 시뮬 벽에서는 해당 경로가 드물다.

## 조사한 계약

| 계약 | 런타임 변경 | 현재 파일 |
|---|---|---|
| `lidar_mount_trial_r1` | `local_scene.py`·`local_path.py` 구조 각도 마스크, `local_pursuit.py`·`lidar_safety.py`의 `lidar_blind_sectors_json`, launch 각도 계산 | 이후 계약의 after와 일치 |
| `lidar_mount_adapted_r2` | `partial_observation.py`(설명된 구조 광선 면제), `local_follow.py`(구조 정책 연결), `lidar_safety.py`(같은 정책 자체 계산), 신규 `known_structure.py` | 일치(`local_follow.py`는 r4 after) |
| `lidar_mount_pilot_r3` / `launch_record_r5` | launch의 배선 단면·마커 높이 전달, 기록 보완 | 일치 |
| `lidar_mount_speed_r4` | `local_follow.py`: 구조 경로에서 전방 전광선 반환을 증속 전제에서 제외 | 일치 |
| `lidar_mount_traffic_marker_r6` | 교통 검사기 `--pilot-marker-z`(평가 스크립트만) | 일치 |

현재 파일은 모두 가장 최근 계약의 `after` 해시와 같다. 기록되지 않은 변경은 없다. before 스냅샷(`runtime_before.zip` r1/r2/r4)과 diff로 의도와 실제 변경을 대조했다.

## 점검 결과

### (1) 구조 각도·좌표·전달 — 문제 없음

- **각도:** JR60P20 프리셋 좌표로 센서 원점 기준 각도를 다시 계산했다. 결과는 런타임 `lidar_blind_sectors.json`과 **0.0° 차이**였다(`checks.json` `sector_check`). 기준 주행에서 측정한 개방각 32.53°와도 맞는다.
- **좌표:** 마스크는 원시 광선의 센서축 각도에 적용된다(`mask_scan`, `rear_masked`). 구조물은 차체에 고정되어 있으므로 운동 보정 전 각도가 맞는 좌표다. 그림자 기하는 광선별 실제 취득 시각의 센서 원점·방향(`relative_poses`)으로 계산하고, 현재 기준 차체 좌표로 옮긴다(`known_structure._shadow`).
- **전달:** 계획기와 보호층은 launch에서 같은 JSON과 `structure_aware_v1`을 받는다.
  - 계획기: 기준 주행 판단 740회 모두 입력 각도가 런타임 파일과 같았다(런타임 증거).
  - 보호층: 소스 경로로만 확인했다.

### (2) "구조 가림만 제외"의 범위 — 다른 결측은 묵살되지 않음

- 면제는 구조 각도 광선 가운데 **양옆 광선이 모두 유효 반환인 경우에만** 적용된다(`classify`). 구조 밖 결측은 그대로 v4의 각도 예산·1.2 m/s 상한·래치 판정을 받는다. `_partial_command`의 변환 공백 검사도 설명된 구조 광선만 뺀다.
- 재현 입력에서도 정면 0°와 측면 80°의 일반 결측 1광선은 각각 `bounded_forward_missing`(상한 0.2)과 `bounded_side_missing`(상한 0.5)으로 판정되었다(`repro_candidate_gate.json`).
- **진단 표시 주의:** 뒤 지주 각도(반폭 1.84°)는 배선 각도(반폭 4.11°) 안에 들어 있다(`nested`). 그래서 지주 행의 양옆은 항상 마스크된 배선 광선이고, `unexplained_sector_deg`에 늘 지주 2개가 표시된다. 기준 주행에서도 `explained_sectors=4/6`이었다.
  - 실제 광선은 배선 행을 통해 면제되므로 동작에는 영향이 없다.
  - 배선 행의 옆이 끊기면 지주 광선도 일반 결측이 된다. 보수적인 방향이다.

### (3) 증속 후보와 결측 판정의 결합 — **실제 결함(잠재)**

- **원인:** r4는 구조 경로에서 `open_route`(최대 `open_route_speed_mps`) 후보를 전광선 반환 없이도 허용한다. 반면 `_partial_command`는 `max(후보 속도, 측정 속도)`로 v4를 평가한다. v4는 일반 결측이 하나라도 있으면 속도 1.2 m/s 초과를 **불허**한다(`partial_speed_outside_trial`). 그 결과 `_hold` → **0 m/s**가 된다.
- **r4 이전과의 차이:** r4 이전에는 `front_ready`(±100° 전광선 반환)일 때만 증속 후보가 나왔다. v4 감사도 같은 ±100°이므로 이때는 `fully_observed`로 먼저 끝나 이 문턱에 닿지 않았다.
- **재현**(`repro_candidate_gate.py`, 기존 회귀 `test_structure_route_speed.py`의 합성 직선 통로 사용, 마지막 스캔에만 일반 결측 1광선):

| 결측 광선 | 증속 후보 있음(현재 r4) | 증속 후보 없음(같은 관측) |
|---|---|---|
| 없음 | 2.77 m/s `fully_observed` | 1.0 m/s |
| 0°(정면) | **0.0 m/s** `follow_partial_speed_outside_trial` | 0.2 m/s `bounded_forward_missing` |
| 79.9°(측면) | **0.0 m/s** `follow_partial_speed_outside_trial` | 0.5 m/s `bounded_side_missing` |

- **의미:** 같은 관측이라도 v4 예산 안이면 감속 주행이 가능하다. 그런데 증속 후보가 문턱을 넘는 바람에 **정지**가 된다. r4의 의도(후보 속도로 최종 검사해 제한)와 ADR 0028의 속도 연속 원칙에 어긋난다. 안전 쪽(정지)으로 틀리므로 위험 증가는 아니다.
- **실제 발생:** 11/4–11/5 저장 주행 23개 모두에서 `partial_speed_outside_trial`은 0건이었다. 시뮬레이션 벽은 거의 모든 광선이 반환되기 때문이다. 실차에서 검은 면·낮은 반사율로 광선 탈락이 생기면 직선 고속 구간에서 급정지로 나타날 수 있다(추정).
- **기존 회귀의 기대값:** `test_new_cruise_candidate_is_not_final_permission[missing]`은 이 0 m/s를 기대값으로 고정하고 있다. 그래서 현재 회귀 24개가 모두 통과한다(`tests_current.log`). 수정하면 이 기대값이 "0이 아니되 v4 상한 이하"로 바뀌어야 한다.

### (4) 이동 차량의 구조 그림자 진입·이탈 — 기존 한계(변경 없음)

- 숨은 기억은 직전 스캔의 반환이 그림자로 들어갈 때 **정지 물체로 가정**하고 자차 운동으로만 옮긴다. 자차와 같이 움직이는 차는 실제로는 그림자에 남는다. 그런데 기억 점은 뒤로 흘러 그림자를 벗어나고 해제되므로, 그 뒤 현재 스캔에는 보이지 않을 수 있다. 문서화된 잔여 위험(`known_structure.py` 머리말)과 같다.
- 먼 그림자 제한은 그림자 표본이 **현재/직전 조향의 이동 외곽과 제동 지평 안에 들어올 때만** 작동한다. 측후방 배선 그림자(±159.6°)는 전진 경로와 거의 만나지 않는다. 폭 15 cm 차가 완전히 숨으려면 배선 그림자에서 센서로부터 약 1.05 m 이상 떨어져야 한다. 이 경우는 계획기에 표현되지 않는다. 추월 후 복귀 판단의 관측 한계이며(11/5 기준 주행 [기하 참고](../moving_pass_baseline_v1/README.md#기하-참고실행-증거-아님)), 이번 검토에서 보호 정책을 추가하지 않았다.
- 5 cm(`ACCEPTED_HIDDEN_WIDTH_M`)는 v4 측방 결측 선례이며 규정값이 아니다. 완화하지 않았다.

## 11/5 기준 주행 속도 톱니의 원인 분리

`checks.json` `baseline_sawtooth`(저장 판단 입력만 사용):

- 창 안 689회 판단에서 부분관측 판정은 **모두 `fully_observed`**였다. 구조·부분관측 속도 상한은 **0회**였다. 구조 각도(≥119.6°)는 `front_patch` 창(base x 0.20–1.60 m, |y|≤0.24 m)과 기하적으로 겹치지 않는다.
- `follow_entry_space`가 적용된 624회 모두에서 최종 속도가 그 상한과 같았다. 같은 624회에서 기본 추종 목표는 상한보다 높았다.
- **결론:** 톱니는 구조 적응 변경과 무관하다. 원인은 handoff 추종 예약(`observed_entry_distance_reserve` ↔ `advancing_patch_close`, `reserve_m=1.2`)이다. 출발 직후 1.5 m/s 명령은 `front_patch` 창 이탈(기준 주행 README)에서 나왔다. 추종 알고리즘은 고치지 않았다.

## 재개 계획

조사 시점의 제안이다. 이후 1–4번을 이행했고 결과는 [수정 결과](#수정-결과-r7)에 있다. 5번(Gazebo)은 필요하지 않아 수행하지 않았다.

1. **before 보존:** `local_follow.py`(현재 = r4 after `6c43b470…`)와 `tests/test_structure_route_speed.py`를 zip으로 보존한다. `repro_candidate_gate.json`이 실패 재현이다.
2. **최소 수정 후보:** 구조 경로의 `open_route` 허용 조건을 바꾼다. 지금은 "구조 정책이면 무조건"인데, 이를 "**설명된 구조 광선을 뺀** ±100° 전방 광선이 모두 반환일 때"로 좁힌다. 일반 결측이 있으면 r4 이전처럼 기본 속도 후보만 v4 예산 판정을 받는다.
   - 설명 여부는 현재 장면의 광선 상태로 `known_structure.classify`와 같은 양옆 규칙을 적용해 계산한다. `structure_excused_indices`는 직전 호출 값이라 이 판단에 쓰지 않는다.
   - 기본 경로(구조 정책 off)는 바뀌지 않는다.
   - 대안(1.2 초과 거절 시 기본 후보로 재평가)은 정책 상태를 같은 호출에서 두 번 갱신하므로 피한다.
3. **회귀:** `[missing]` 기대값을 "0 < v ≤ v4 상한"으로 바꾼다. 측면 결측 사례를 추가한다. 기존 증속 사례(C 지주만 결측 시 >1.2)는 유지한다. 구조 관련 3파일과 `test_local_follow`·provenance 사슬 회귀도 함께 돌린다.
4. **계약:** `lidar_mount_*_r7_sha256.json` 후속 계약(parent = r6)을 만들고 소스 출처 사슬 시험 목록을 갱신한다.
5. **Gazebo 주행:** 시뮬레이션 벽에서는 이 경로가 거의 발생하지 않으므로 긴 주행은 필수 근거가 아니다. 필요하면 기존 JR60P20 단독 1회로 회귀 여부만 확인한다.

## 남는 한계

- 실물: 실제 C1 결측 패턴, 배선 U채널 실제 형상과 높이, 반사율은 미측정이다. 각도는 Ø20 외접 예약 모델 기준이다.
- 보호층이 구조 파라미터를 실제로 받았다는 것은 소스로만 확인했다.
- 움직이는 상대: 그림자 속 이동 물체는 기억(정지 가정)으로 추적되지 않는다. 측후방 배선 그림자에 숨은 차는 경로 제한에 반영되지 않는다.
- 재현은 합성 직선 통로 1장면이다. 실제 Gazebo 주행에서는 이 결함이 관측되지 않았다.

## 파일

| 파일 | 내용 |
|---|---|
| `checks.py` → `checks.json` (`checks.exit`, `checks.log`) | 구조 각도 재계산 대조, 기준 주행 톱니 원인 분리. `python -I -B checks.py` |
| `repro_candidate_gate.py` → `repro_candidate_gate.json` (`.exit`, `.log`) | 결함 재현. WSL에서 `source /opt/ros/jazzy/setup.bash` 후 `python3 -B repro_candidate_gate.py` |
| `tests_current.log` / `.exit` | 수정 전 코드의 구조 관련 회귀 3파일(24 passed) |
| `structure_gate_before.zip` | 수정 전 `local_follow.py`·`test_structure_route_speed.py`·`test_source_provenance_r0_plus_b3.py` |
| `repro_candidate_gate_after.json` (`.exit`, `.log`) | 수정 후 같은 재현(`python3 -B repro_candidate_gate.py repro_candidate_gate_after`) |
| `tests_new_on_before.log` / `.exit` | 갱신 회귀를 수정 전 `local_follow.py` 사본에서 실행(4 failed, 6 passed) |
| `tests.sh` → `tests.log` / `.exit` | 관련 회귀 묶음(312 passed, 기존 2 failed) |
| `register_contract.py` (`.log`, `.exit`) | r7 계약 생성(기존 파일과 다르면 덮지 않음) |
| `replay_representative.py` → `replay_representative.json` (`.log`, `.exit`) | 저장 JR60P20 주행 판단 replay와 불일치 분류 |
