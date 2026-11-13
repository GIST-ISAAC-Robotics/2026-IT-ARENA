# r386 final 기하 비교: 중단 복구와 후보 미채택 근거

2026-11-13. 저장 JSONL/로그와 관련 함수의 정적 검토만 수행했다. replay·시뮬레이션·장치 실행 및 archived source/후보/판정선 변경은 하지 않았다.

**결론:** 시각 반례의 예외는 입력 장면 생성 단계에서 발생한 예상 거절을 하네스가 기록하지 못한 문제다. 이를 후보의 94회 적용 실패와 분리해야 한다. 저장 final 223개는 이미 모두 비교되었고, 기준/후보 양수 출력은 **119/119개, 추가 회복 0개**다. 후보 94회는 모두 최종 `unbounded_nonreturn`으로 거절되었다. 남은 반례 4개는 별도 기록으로 보완할 수 있지만, 현재 후보를 다음 실제 주행에 적용할 효과 근거는 없다. **미채택을 유지한다.**

## 1. 실패 위치와 최소 복구 범위

[final.log](remote_result/extracted/final.log)는 `completed_stored=223`, `completed_counterexamples=11`, `MotionUnavailable('scan_timing_unverified')`, `violations=[]`를 기록한다. [execution.json](remote_result/extracted/execution.json)은 final 프로세스 exit 1, 전체 실행 미완료, 확인한 소스 534개 변경 0개를 기록한다. `violations=[]`는 전체 성공을 뜻하지 않는다. 원 report의 `passed=false`와 원 로그를 보존해야 한다.

[compare.py](compare.py)의 `evaluate`는 다음 순서다.

1. `restore_call` → `motion_scene(... timing_verified=...)` (50–52행)
2. 기준 정책 `rebase` → 후보 `try_from_current_result` (54–62행)
3. 반례 결과 확인 → JSONL 기록/flush (94–97행)

원 [반례 생성기](../compare.py)의 0-based index 11은 `current_time_unverified`이며 `timing_verified=False`만 설정한다. 보존 [bundle_sources.zip](bundle_sources.zip)의 `geometry/src/arena_autonomy/arena_autonomy/scene_motion.py::_measurement_transforms` 48–49행은 이 입력을 `scan_timing_unverified`로 거절한다. 따라서 기준과 후보 함수에는 도달하지 않았고, 바깥 일괄 `except`(105행)가 나머지 반례까지 중단했다. 후보에 예외 catch를 넣어도 이 호출 순서에서는 해결되지 않는다.

**최소 수정 제안은 별도 복구 하네스의 scene 생성 경계에 한정한다.** 해당 반례에 한해 예상 예외의 정확한 클래스·reason을 확인하고 `input_rejected`, `candidate_evaluated=false`, `stage=motion_scene`로 기록한다. 성공한 후보 계산이나 실제 0속도 명령 송신으로 꾸미지 않는다. 다른 예외·일반 stored 입력의 예외를 무조건 정상 거절로 치환하지 않는다. 기존 입력 검증·후보·시각 정책에는 손댈 이유가 없다.

남은 범위는 다음 **4개뿐**이다. 원 223개 + 이미 통과한 11개(JSONL 총 234행)를 다시 계산할 필요는 없다.

| 반례 index | 이름 | 복구 시 확인할 내용 |
|---:|---|---|
| 11 | `current_time_unverified` | scene 단계의 정확한 예상 거절; 후보 미호출 |
| 12 | `previous_steering_outside_cad` | 원 하네스와 같은 기준/후보 비교 및 비적용·0속도 조건 |
| 13 | `dt_outside_contract` | 동일 조건 |
| 14 | `both_blind_opposite_previous_steering` | 동일 조건 |

원 생성기는 각 반례마다 마지막 r377 입력을 `deepcopy`하며, `evaluate`는 입력별 history/scene을 다시 만든다. 따라서 같은 archive·native backend·final geometry·마지막 원 입력·반례 이름/변형을 고정한 별도 보완 결과로 독립 실행할 수 있다. 단, 현재 CLI에는 부분 재개 기능이 없으므로 **보관 compare.py 자체를 덮어쓰지 않는 작은 별도 wrapper**가 필요하다. 복구 전후 소스 해시를 확인하고 원 결과의 해시를 연결한다. 합산 표현도 “원 11개 + 추가 계산 3개 + 입력 단계 거절 1개”로 구분한다. 현재 검토에서는 추가 4개를 실행하지 않았으며 원 실행 성공으로 소급하지 않는다.

## 2. final 223개의 실제 거절 분포

아래는 [final rows.jsonl](remote_result/extracted/results_native_v1/final/rows.jsonl)만 집계한 값이다. 다른 정책의 과거 기대 출력이나 795개 보존 집합을 섞지 않았다.

| 기준 `reason` | 개수 | 양수 출력 |
|---|---:|---:|
| `rebase_current_follow` | 107 | 107 |
| `rebase_current_present_guard` | 7 | 7 |
| `rebase_current_parallel` | 5 | 5 |
| `route_current_large_motion_region_unobserved` | 101 | 0 |
| `guard_current_new_side` | 3 | 0 |
| 합계 | 223 | 119 |

후보 진단은 `outside_scope` 129개와 `final_current_region_rejected` 94개다. `outside_scope`에는 이미 양수인 119개와 후보 조건 밖 정지 10개가 포함되므로 129개를 새 실패로 해석하면 안 된다. 94개 모두 거리 제안은 수락되고 core·최종 region 검사까지 진행했지만, 최종 proof가 `unbounded_nonreturn`이다. 후보 target 범위는 0.088221–0.094171 m/s, 최종 unobserved는 116–182개다. 223개 모두 후보 진단 키를 제외한 출력은 기준과 정확히 같았다.

이 **94 attempted / 0 applied는 r377 저장 입력에 대한 r386 후보 집계**다. 원 r377의 기존 `current_relative_body_retry`는 100회/0적용이고 그 이유는 `outside_small_body_side` 94개, `unbounded_nonreturn` 6개다. RELEASED 모드의 시도 수나 서로 다른 정책의 성공 수로 바꾸어 읽으면 안 된다.

## 3. 최초 지속 정체와 10 mm 조건

index는 저장 행의 0-based 값이다. 최초 일시 0출력은 index 39의 GUARD이므로 이후 지속 정체와 구분한다. 마지막 양수는 index 122(`reference_s=20.060`, 0.1 m/s)이며 **index 123부터 222까지 100개 모두 0출력**이다. 실제 측정속도가 0이 된 최초 후보 시도는 index 125다.

거리식은 기존 `D(v)=v·(dt+age+0.1)+v²/4`, `v=max(measured,target)`이다. **진행 거리 D**, 질의의 최소 진행 길이 10 mm, **차체 좌우 외곽 허용 폭 10 mm**는 서로 다른 값이다. 차체 옆 10 mm 이내라는 조건만으로 미관측 공간이 자유가 되는 것은 아니다.

| 시점 / 단계 | D (mm) | unobserved | 최대 측면 외곽 (mm) | 결과 |
|---|---:|---:|---:|---|
| index 123, 지속 정체 시작: 기존 0.10 재검사 | 37.790 | 42 | 8.878 | `unbounded_nonreturn`; 측정속도 0.12245 m/s의 제동거리 유지 |
| index 125, 첫 후보 전 기존 0.10 재검사 | 34.300 | 172 | 10.300 | `outside_small_body_side` |
| index 125, 후보 0.092788 m/s | 31.659 | 155 | 9.635 | `unbounded_nonreturn` |
| index 222, 마지막 기존 0.10 재검사 | 34.500 | 162 | 10.766 | `outside_small_body_side` |
| index 222, 후보 0.091639 m/s | 31.424 | 130 | 9.956 | `unbounded_nonreturn` |

첫 후보의 원 0.45 m/s 검사 D는 193.725 mm이고, 후보는 이를 충분히 줄였다. 첫/마지막 후보의 masked tail도 각각 8.824/9.029 mm로 10 mm 이하이다. 따라서 이 실패를 “D를 전혀 줄이지 못함” 또는 “측면 폭을 아직 초과함”이라고 설명하면 틀린다.

후보 [budget.py](budget.py)의 `observed_prefix`는 unknown/자체 mask의 **기하 한계만** 만족하는 접두 길이를 제안한다. [candidate.py](candidate.py)는 별도로 생성한 최종 질의에 `bounded_body_nonreturn`을 적용한다. 보관 소스 `current_path_recheck/_gaps.py` 29–32행의 측면 검사를 통과한 뒤, `rear_fringe_trial.py::_bounded_nonreturn` 105–107행은 해당 unknown 연속 구간 양끝에 실제 return이 없으면 `unbounded_nonreturn`을 반환한다. 94개 모두 이 단계에서 실패했다. 저장 proof에 실패 ray index/양끝 status는 없으므로 각 건이 배열 끝인지 mask 인접인지까지는 단정하지 않는다. 그 뒤의 gap 폭·이웃 거리·local chord 검사를 실패 원인으로 추정하지도 않는다.

## 4. 기하 불일치 해소의 효과와 적용 판단

final은 planning과 sender의 final YAML을 선택하고, 후보도 기준과 같은 `ps.lateral_config(cfg)` 및 wheel contact 생성 경로를 사용했다. 실제 변경값은 track 0.135→0.1239 m, wheel width 0.012→0.0261 m이며 wheel radius 0.0325 m는 같았다. 기준의 전체 결과 객체는 legacy 대비 220/223개 달라졌지만, **speed 값 변경 0개, geometry-only 양수 회복 0개**다. 마지막 0.10 재검사의 질의 수는 기존 792→802로 바뀌어도 unobserved 162개·최대 외곽 10.766 mm는 남았다.

이는 해당 저장 입력에서 **기하 정합성 수정만으로 정체가 풀리지 않았음**을 뜻한다. 올바른 기하 전달 자체가 불필요하다거나 다른 주행에서 효과가 없다는 일반 결론은 아니다. 또한 접두 예산 후보는 기하 조건을 충족시켜도 원 비반환 유계성 조건을 충족시키지 못한다. 후자의 조건을 완화하거나 새 옵션을 만드는 것은 이번 복구와 별개이며 제안하지 않는다.

새 후보 적용의 주행상 이득은 관측되지 않았다. 시각 반례 기록을 보완해 검증의 빈칸을 닫을 수는 있으나, 그것이 후보 채택 근거가 되지는 않는다. 기존 11반례/223기록과 원 실패를 보존하고, **r386 후보 미채택·기존 판정 유지**가 타당하다. 저장 계산 비교는 실제 주행·실물 관측·명령 만료 검증이 아니다.

## 확인한 근거 해시

- final `report.json`: `c8e0bd52c594767c79dbdeffd015331cd77aa46ed16b98aa2e2a5e94ab1061c6`
- final `rows.jsonl`: `32da245a37792319de7d9033568e5f9ee906c91c1691d36aa31686a54a6d8c26`
- `candidate.py`: `befa39afe1d8036059de6fccbdb86d0539332813bb472f793dd0ec9c9f28d791`
- `budget.py`: `590bfcbb72f52589c57a6aa4f7db22b51c5aa15676ca19fb7432c58f96ff74c2`
- `source_manifest.json`: `cf3e0806871de1345a113aaa3f2c1552e6d0ad82f0fb2b1744f777d2a35f8a3f`

반례 이름/순서는 원 compare의 생성기에서 확인했으며, scene/비반환 함수의 설명은 archive 안 `geometry/` 소스를 직접 읽어 대조했다. saved rows의 집계만 수행했으며 후보 함수를 import하거나 실행하지 않았다.
