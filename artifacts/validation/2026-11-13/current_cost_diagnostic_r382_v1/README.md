# r380 정확성 실패 보존과 Jetson 비용 진단 r382

**준비 단계다. 새 Jetson/WSL 실행은 수행하지 않았다.** 원 r380 실행의 `passed=false`를 유지하면서, x86 기대값 재현 실패가 ARM 내부 기준/후보 비교까지 중단시키지 않도록 진단만 분리한다. 현재 공통 소스나 r375 후보를 수정하지 않는다. 자동 채택은 허용하지 않는다.

## 원 실패의 전필드 감사

[`original_diff.json`](original_diff.json)은 회수된 r380 `timing_rows.jsonl`의 **10행 전부**를 원 r371 출력과 재귀 비교한 결과다. [`audit_original.py`](audit_original.py)는 저장 JSON만 읽으며 제어기를 import하거나 재실행하지 않는다. 원 보고서·입력·행·실행/import/build 로그 SHA256을 함께 보존한다.

- 원 r380 native 빌드와 import 사전 검사는 성공했다. aarch64, Python 3.10.12, GNU 11.4.0, native ABI1, 명시 동적 후보 7단계와 266개 소스/입력 무결성을 확인했다. 실행은 index9/reference8.76s의 원 출력 비교에서 중단했다. 정상 종료한 행은 9개, 계측이 기록된 행은 실패 행을 포함해 10개다.
- 기존 `follow_trace.equal_output`는 bitwise exact 비교가 아니다. 수치에 `math.isclose(rel_tol=1e-10, abs_tol=1e-10)`를 사용하고 구조·문자열·boolean을 재귀 비교한다. **이번 도구는 이 오차를 넓히지 않는다.** 기존 판정과 별도로 모든 정확한 트리 차이를 남긴다.
- 10행 전체에 정확한 값 차이 110개가 있다. 경로 좌표 106개(최대 절대차 `5.551115123125783e-17`), index2/8의 조향 각도 각 1 ULP, index9 영역 개수 2개다. 앞선 9행은 기존 비교 규칙 안에 있다.
- index9의 `current_route_proof.region.queries`는 **3966→3965**, `masked_left`는 **259→258**다. 기존 허용 범위를 벗어난 차이는 이 두 개다. 이 행의 나머지 13개 경로 좌표 차이, 속도·조향·reason은 중단 원인이 아니다. 다른 행의 미세 조향 차이까지 숨기지 않았다.
- 같은 ARM의 r370 기준과 r375 후보는 회수된 10행 모두 exact tree 동일하다. 모두 SOLO/미적용이며 RELEASED 적용 비용 표본은 **0개**다. 이 부분 표본의 p50/p95/max는 `28.018086/42.027932/42.143020ms`, 100/200ms 초과 0이다. r375 적용 비용이나 전체 비용을 대표하지 않는다.
- 소유 자식 3개는 각각 종료코드 0/0/1, 강제 종료 없음, 변경된 원본 0개다. 원 실패는 [`r380 회수본`](../current_cost_r380_v1/remote_result/extracted/jetson/report.json)에 남아 있다.

## 원인 근거와 아직 확인하지 못한 점

아키텍처/수학 라이브러리 부동소수점 차이는 **추정**이다. 동일 소스/입력에서 경로 좌표와 두 조향에 1 ULP 수준 차이가 있고 query와 masked_left만 각각 1개 줄었으므로, 미세 좌표 차이가 질의 생성의 중복/경계 판정으로 확대되었을 가능성이 있다. 아직 동일 x86/ARM 중간 질의를 짝지어 확보하지 못했으므로 확정하지 않는다.

동결 r380 소스에서 확인한 경로는 다음과 같다. 링크는 작업 트리의 해당 파일을 가리키며 실행본은 r380 archive의 바이트/해시로 고정한다.

1. [`point_set.unique_xy`](../../../../src/arena_autonomy/arena_autonomy/point_set.py)는 반올림·근접점 병합 없이 정렬한 이웃 좌표의 **정확한 불일치**로 중복을 제거한다. 아주 작은 수치 변화가 기존 중복점 하나를 합치거나 분리할 수 있다.
2. [`observed_lateral_trial.path_observation_queries`](../../../../src/arena_autonomy/arena_autonomy/observed_lateral_trial.py)는 `math.sin/cos` 회전 외곽, NumPy 차체 모서리, unknown 셀과의 엄격한 부호 비교/다각형 교차로 점을 만든다. [`m3_execution_trial.body_queries`](../../../../src/arena_autonomy/arena_autonomy/m3_execution_trial.py)는 바퀴 외곽을 더하고 다시 정확 중복을 제거한다.
3. [`pass_release_trial.motion_queries/motion_region`](../../../../src/arena_autonomy/arena_autonomy/pass_release_trial.py)는 현재/직전 조향 원호를 합친 뒤 정확 중복 제거와 기존 차체 밖 경계(`1e-9m`)를 적용하고 `queries`, 마스킹 수를 센다. 질의 생성 차이와 native 분류 차이를 아직 구분하지 못했다.
4. 원 native 빌드는 `-O3 -std=c++17 -ffp-contract=off -fno-fast-math`를 사용했다. 이 설정만으로 NumPy/libm/전체 Python 경로의 아키텍처 일치를 보장할 수 없다. 특정 컴파일러 최적화를 원인으로 단정하지 않는다.

[`query_probe.py`](query_probe.py)는 **전체 비용 계측 종료 후 별도 프로세스**에서 index9 하나만 관찰한다. `unique_xy` 입력/출력·제거 수, 최종 motion query 좌표/거리/두 조향, 실제 depth/masked 분류 배열을 NPZ로 보존한다. 근접점/반올림 후 중복 수는 별도 관찰 통계일 뿐이며, 원 함수가 반환한 좌표를 그대로 제어기에 전달한다. 관찰한 기준/후보 출력이 비계측 r382 출력과 exact tree 동일해야 한다. NumPy/libc/compiler 정보도 남긴다. 비용에는 포함하지 않으며, 이 ARM 관찰만으로 x86와의 차이 원인이 확정되었다고 선언하지 않는다.

## 판정과 중단 기준

[`run.py`](run.py)는 원 r375 입력 복원·반례를 재사용한다. 순서는 원 r380과 동일한 **r371 217 → r364 249 → r279 52 → r282 54 → 반례 17**이다. 현재 장면 복원→r370→r375 순서, 원 속도·조향·시각·센서 입력을 유지한다. 기준 계산 직후 최상위 r375 `rebase`만 재며 내부 중복 호출도 포함한다. 첫 표본을 버리지 않고 반복/ABBA/예열 실험을 추가하지 않는다.

| 판정 | 후속 동작 |
|---|---|
| 원 해시/후보 import 계보/입력 불변 실패, 예외 | 즉시 중단하고 부분 raw/실패 보존 |
| 같은 ARM에서 미적용 출력 변경, 355개 비대상 적용 | 기존 r375 comparator로 실패하면 즉시 중단. exact 차이도 별도 보존 |
| 적용 시 RELEASED/parallel/원 목표 상한·조향·시각·경로·region·10mm·미관측 비채움 조건 위반 | 기존 r375 불변 조건 그대로 즉시 중단 |
| 17개 반례의 적용/기준 불변/정지 실패 | 즉시 중단. 원 17개 모두 정지했던 사실 유지 |
| x86 원 기대값/기존 비교 행과 차이 | 전필드 diff를 남기고 나머지 입력 진단 계속. 재현 판정은 실패로 기록 |
| ARM 적용 수가 원 93과 다름 | 이식 차이로 보고하고 자동 채택 금지. ARM 자체 불변 위반과 구분 |

반례 gap 인덱스는 원 x86 비교 마지막 진단에서 고정하여 같은 17개 입력 변형을 만든다. 이 fixture는 반례 생성에만 쓰며 제어기 진행 허가로 사용하지 않는다. 마지막 ARM 적용 여부가 달라도 반례 전체를 진단할 수 있다.

`diagnostic_completed`, `same_arm_invariants_passed`, `x86_existing_comparator_reproduction_passed`, `x86_exact_tree_reproduction_passed`를 별도 필드로 기록한다. 단일 `passed=true`로 x86 재현 실패를 가리지 않는다. 실행자 bootstrap의 exit0은 진단 수집 완료를 뜻하며 x86 일치나 후보 채택을 뜻하지 않는다. 원 r380 실패를 새 실행 성공으로 소급하지 않는다.

[`diff_utils.py`](diff_utils.py)는 모든 dict 키/배열 원소/추가/삭제/길이/타입/정확 값 차이를 보존한다. `comparison_rows.jsonl`에 전체 diff, `timing_rows.jsonl`에 기준/후보 전체 출력을 남긴다. 명령/측정 상태, reason/selector, 시각/source, boolean/허가 차이는 보고서에서 따로 모으고 기존 오차 범위 내·밖을 구분한다. 분류를 통과 필터로 사용하지 않는다. 원 r371 출력에 ROS 부모의 감속 cap이 붙은 경우 원 방식으로 표시용 결과를 복원하되, 원 cap 입력의 checked target 일치 여부도 별도 비교해 덮어씌움으로 차이가 숨지 않도록 한다.

비용은 전체/입력집합×상태/적용·미적용별 p50/p95/max와 100/200ms 초과 수다. **100ms는 재검토 예산 참고이며 안전 허가 조건이 아니다.** 직렬 실제 경과시간으로 원 명령 시각을 재평가하거나 만료 통과를 선언하지 않는다. ROS/센서 동시부하·MCU 발행·실차 검증도 아니다. 의미 있는 출력 이식 차이가 있으면 비용만으로 동일 행동의 성능을 주장하지 않는다.

## 동결 번들과 실행 인계

[`build_bundle.py`](build_bundle.py)는 현재 `src/`를 수집하지 않는다. 원 r380 `bundle_sources.zip` SHA256 `122f13672bdac590285bc4bf4bc15be0341158dac0849d9f65a7c916c8530a44`에서 원 manifest의 **266파일 바이트 전부**를 읽고 대조한다. 원 manifest도 archive에 그대로 보존하고 r382 진단 도구/문서/원 diff만 추가한다. 11/12의 r281/r266을 포함한 후보 7단계가 들어 있다. r379의 공통 `band_check` 변경은 들어가지 않는다. 원 archive와 실행 결과는 수정하지 않는다. 현재 r380 README에 추가한 실패 기록과 달리, archive의 역사적 README는 시험 당시 원문을 유지한다.

```text
python artifacts/validation/2026-11-13/current_cost_diagnostic_r382_v1/test_diagnostic.py
python artifacts/validation/2026-11-13/current_cost_diagnostic_r382_v1/build_bundle.py
```

새 native 바이너리는 포함하지 않는다. 실행 담당자가 새 Jetson 사용자 홈 루트 `it-arena-r382-20261113`에서만 빌드한다. 실행은 빌드120초→import60초→단일 전체 진단900초→index9 관찰60초 상한이며, 각 자식 PID/종료/강제 종료/소스 무결성/실행 로그를 회수한다. 기존 r380 루트/결과를 재사용하지 않는다. 센서 포트·Gazebo·ROS·모터/서보·펌웨어·전력 설정을 열거나 바꾸지 않는다. 비용 계측 중 같은 Jetson 경쟁 부하는 금지한다.

고정 실행 명령과 archive/manifest/bootstrap 해시는 번들 확정 후 별도 `HANDOFF.md`에 기록한다. 이 파일은 생성된 archive 자신을 순환 참조하지 않도록 archive 외부의 인계 영수증이다. 실행 전에 archive와 업로드 bootstrap의 해시를 확인하며 bootstrap 자체도 archive 내부 원문과 일치해야 한다. 상세 JSONL/NPZ/압축본은 기존 raw Git 제외 정책을 유지한다.

## 준비 검증

Windows 로컬의 7개 진단 논리 검사와 AST 문법 검사를 통과했다. 개수 차이를 단순 부동소수점 잡음으로 통과시키지 않음, 1 ULP 조향 차이 보존, 배열 끝/구조/boolean 차이 보존, 감속 cap의 입력 차이 기록, ARM 적용 허가/미적용 불변을 검사했다. native 재현/전체589호출/Jetson 실행은 이 준비 검증에 포함되지 않는다.
