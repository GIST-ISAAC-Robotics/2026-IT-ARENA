# run15 관측 기반 해제 관문 — 사용량 경고 인계 (미완료)

2026-10-25, Claude Opus 5.5 → GPT. 실행 중인 시뮬레이션·pytest·보조 agent·WSL 프로세스는 없다(확인: `NO_PROCS`). Gazebo 새 주행 없음. 커밋·푸시 없음.

## 1. 현재 소스 상태 — **출처 계약 미연결, 정식 회귀 미실행**

| 파일 | 상태 | SHA-256 |
|---|---|---|
| `src/arena_autonomy/arena_autonomy/m3_execution_trial.py` | 관측 기반 해제 관문 구현(아래 2) | `35977c1b…ab4dec` |
| `tests/test_m3_baseline_release.py` | 새 의미로 재작성(12시험) | `75ca1ef4…5e35df` |

- 보존 이력: 반려된 run14 판본(`706a98…`)과 이전 시험은 `../run14_baseline_release/source_snapshot/sources.tar`에 있다. 원본 run13/run14 증거와 `config/tests/m3_baseline_release_r0_sha256.json`은 수정하지 않았다.
- **알려진 실패:** `m3_execution_trial.py`가 `m3_baseline_release_r0_sha256.json`의 `after`(706a98…)와 달라졌다. 따라서 `tests/test_source_provenance_r0_plus_b3.py`의 M3 사슬 시험이 실패할 것이다. corner_follow/dense_stop/physics_step 사슬 시험은 validator 미변경이라 영향이 없을 수 있으나 확인하지 않았다. 새 후속 계약이 필요하다(아래 4-①).
- 노드/launch/CLI는 바꾸지 않았다(옵션 이름 `release_to_baseline`, 기본 false 그대로).
- 기본 OFF 호출 형태는 유지했다. `M3ExecutionFollow`는 옵션 ON일 때만 `history`를 `lateral.step`에 넘긴다.

## 2. 구현한 계약 (`release_gate`, `ExecutionLateralTrial._release`)

- 위협이 present일 때 측면별 `ThreatRecord(side, scene, points, acquired)`를 갱신한다.
- 위협이 없는 **fresh 스캔**마다 다음을 확인한다.
  1. 기록 장면에서 반환점으로부터 `hypot(0.20, 0.15)` 이내이고, 그 장면에서 관측 자유가 아니며, 당시 자차 밖인 2 cm 칸을 가능 몸체로 둔다.
  2. `transport_points`로 현재 좌표에 옮기고 `0.5 m/s × (현재 − 가장 오래된 반환 취득) + 0.025 + 0.03`만큼 넓힌다. 도로 안·현재 자차 밖만 남긴다.
  3. 그 칸은 지금 `depth_support` free여야 한다. 예외는 양옆 광선이 모두 반환이면서 (관측 표면에 붙거나 ds > 측방 창 0.35 m)인 칸뿐이다. 그 외 하나라도 있으면 `threat_region_unobserved`다.
  4. 확인되면 영역 안 현재 반환(벽 제외)으로 기록을 갱신하고, 없으면 기록을 지운다.
  5. 관측 자유가 아닌 칸이 위협 쪽 복귀 회랑(자차 근측~중앙 + 반폭 + 97 mm)과 `ds ∈ [−0.10, 0.10 + 정지거리(1 m/s)]` 안에 있으면 `threat_possible_in_return_corridor`로 해제하지 않는다.
- 기록 나이가 1.0 s를 넘거나 epoch·시각이 맞지 않으면 `record_stale_or_inconsistent`로 해제하지 않는다(시험 유지).
- 해제 조건: 위협 없음, fresh, 운동 이력 있음, `clear_fresh ≥ 2`, 모든 기록 측면의 판단이 `release`.

## 3. 확인한 것과 하지 않은 것

- **수정 전 반례**(`pre_fix_scenarios.json`, 소스 706a98…): 광선 탈락·긴 판자 가림·2점 부분 관측이 1.2 s에 해제됐다(옆차 뒤면 실제 ds 0.29 m, 측방 창 안). 정상 통과는 1.5 s, 옆 소실은 해제 없음.
- **수정 후 개발 확인**(`dev_check_scenarios.json`, 같은 입력): 정상 통과 2.0 s 해제(뒤면 0.65 m, 회랑+정지거리 밖). 탈락·가림·2점·옆 소실은 해제 없음이며 1.0 s 뒤 기록 만료. 이 실행은 `history` 전달 조건 수정 전이지만 시험 단위 결과에는 영향이 없다. **정식 증거 실행(해시 전후·영수증)은 아니다.**
- **개발 pytest**(`dev_tests.sh`, 비공식): `test_m3_baseline_release.py`·`test_m3_execution_trial.py`·`test_m3_command_selection.py` 45 통과. 중간에 실패가 있었고 모두 이 시험 단계에서 고쳤다. (1) 기존 stub이 `history` 인자를 받지 않아 4건 → ON일 때만 전달. (2) 오른쪽 차를 벽 옆에 둔 고정 장면 오류 1건 → 벽 반환과 한 묶음이 되어 compact 아님 → 위치 수정.
- **미실행:**
  - 저장 run9/run13 OFF 동일성, 새 ON vs 이전 ON(`../run14_baseline_release/release_comparison/*_release_on.json`), 관문 비용 측정. 비교 스크립트는 아직 작성하지 않았다.
  - 통합·출처 회귀와 설치본 import 확인.
  - 문서 갱신. E3 §8.10·현황·`RELEASE_DRIVE_PLAN.md`는 아직 반려 전 판본을 설명한다.

## 4. 정확한 다음 순서

1. 새 후속 계약 `config/tests/m3_release_gate_r0_sha256.json`(부모 `m3_baseline_release_r0_sha256.json`, before 스냅샷 = run14 `sources.tar`, 변경 = `m3_execution_trial.py`, 필요 시 `validate_g1_traffic.py` SOURCE_FILES 추가)을 만든다. `tests/test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS`에 추가하고, `m3_revision_digest`가 `.tar` 스냅샷(tarfile)을 읽도록 한다. validator를 바꾸면 corner_follow/dense_stop/physics_step 사슬 튜플에도 추가한다.
2. `compare_release_gate.py`를 작성한다. run13/run9 저장 판단의 OFF 637/636 동일성, 새 ON 해제 시각·사유 분포·양수 수와 이전 ON(6.32/6.12 s) 대조, ON 판단 ms p50/p95를 기록한다.
3. 필요한 회귀(Haiku 별칭으로 실행)를 돌린다: `test_m3_baseline_release`, `test_m3_execution_trial`, `test_m3_command_selection`, `test_m3_execution_integration`, `test_source_provenance_r0_plus_b3` 및 사슬 시험 3개, 설치본 import 확인. 전체 201은 반복하지 않는다.
4. 문서를 갱신한다. E3에 §8.11(반려 사유·반례·새 계약·한계)을 추가하고, `RELEASE_DRIVE_PLAN.md`에 "반려" 표시, `PROJECT_CONTEXT` 요약을 바꾼다. 원래 실패·중단은 지우지 않는다.
5. 남는 한계(해결로 쓰지 말 것):
   - 해제 뒤 기본 G1은 후방 마스크·측방 예산을 검사하지 않는다.
   - 기록 갱신은 영역 안 재관측 반환을 같은 차로 보수 추적할 뿐 식별이 아니다.
   - 측방 창 앞의 가림 칸은 허용한다.
   - 2 cm 이산 표본이다.
   - 이동 상계 0.5 m/s 초과 물체, 등록 크기보다 큰 물체, 붙은 여러 차는 계약 밖이다.
   - 1 s 만료 뒤에는 시험이 끝나지 않는다(직진 전용 상태 지속).
   - 저장 run13에서도 해제가 늦어지거나 없을 수 있다(미측정).
