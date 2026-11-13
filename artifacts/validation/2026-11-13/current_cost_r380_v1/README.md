# r375 현재 검사 Jetson 비용 대표 r380 — 원 기대값 재현 실패

**Jetson 실행과 회수를 완료했으며 원 기대값 재현 실패로 중단했다.** native 빌드와 import 사전 검사(266파일·후보 계보7단계)는 통과했지만 r371 index9/reference8.76s에서 원 x86 출력과 비교가 실패했다. 원 `report.json`의 `passed=false`와 원 archive/소스를 그대로 보존한다. r375 호출이 130–200ms일 수 있다는 자문은 여전히 미계측 추정이다. 이번 회수 표본에는 r375 적용 사례가 없다.

[`회수 보고서`](remote_result/extracted/jetson/report.json)에는 SOLO/미적용 10행의 p50/p95/max `28.018086/42.027932/42.143020ms`, 100/200ms 초과0이 기록됐다. 완료 비교는9행이며 실패한10번째 호출의 시간도 raw에 남았다. 이를 RELEASED 적용/전체 비용으로 확대하지 않는다. index9의 `queries`3966→3965와 `masked_left`259→258이 기존 수치 비교 오차를 벗어났다. 같은 ARM의 기준/후보 출력10개는 정확히 동일하다. 소유 자식3개 종료코드0/0/1, 강제종료없음, 변경소스0으로 회수했다. 모든 출력의 상세 차이와 준비 중인 후속 진단은 [`r382`](../current_cost_diagnostic_r382_v1/README.md)에 별도로 기록한다. 후속 결과로 이 실패를 성공으로 소급하지 않는다.

## 계측 범위

[`run.py`](run.py)는 기존 [`r375 compare.py`](../released_current_r375_v1/compare.py)의 입력 복원·기존 출력 재현·반례 함수를 재사용한다. 순서는 원과 같은 **r371 217개 → r364 249개 → 과거 r279 52개/r282 54개 → 마지막 입력의 17반례**다. 원 reference/path/source 시각·속도·조향·입력 순서를 바꾸지 않는다.

각 입력에서 현재 장면을 복원한 뒤 r370 기준 결과를 먼저 계산하고, 바로 뒤 r375 최상위 `rebase` 한 번의 실제 경과시간만 `time.perf_counter_ns()`로 잰다. r375 내부의 기존 r370 호출·중복 재검사 비용은 모두 포함한다. 복원/외부 r370 비교/출력 freeze/JSON/파일 저장은 계측 구간 밖이다. 이 묶음에서 중복 계산을 제거하거나 캐시 구조를 새로 설계하지 않는다.

별도 ABBA·반복 배치·예열 표본 제거는 없다. 한 번의 순서 보존 직렬 계측이며 첫 호출도 포함한다. 직전 기준 계산에 따른 CPU/cache 영향이 있어 독립 냉시작 비용이나 실제 두 프로세스 동시부하와 같다고 해석하지 않는다. 기준 r370의 별도 비용 개선률은 보고하지 않는다.

기존 `results_native_v1/comparison.json`의 572행 전체/17반례 결과와 다시 대조하고, r371 원 217개 재현·93개 적용·124개 미적용·355개 비대상 불변을 유지해야 한다. `r371_all`, `r371_applied`, `r371_unapplied`, 비대상/반례, 입력집합×상태별로 **p50/p95/max, 100ms 초과 수, 200ms 초과 수**를 기록한다. 100ms는 재검토 예산 참고이며 안전 허가 조건이 아니다. 200ms도 계산 초과 수일 뿐 **명령 만료 통과를 판정하지 않는다.** 직렬 저장 재생은 명령 도착·발행·ROS·센서 동시부하·MCU·실차를 검증하지 않는다.

## 번들과 import 경로

[r369](../retained_schedule_r369_v1/README.md)처럼 새 사용자 소유 원격 폴더에 ZIP을 풀고 모든 입력/소스 해시를 대조한다. 기존 r369 원격 폴더나 설치 환경을 덮어쓰지 않는다. 후보 import가 `sys.modules`에 자동 등록되지 않는 문제를 고려해 r375→r370→r352→r346→r343→**11/12 r281→r266** 파일을 명시적으로 포함한다. r266 `candidate_recheck/` 패키지도 함께 보존한다.

`src/arena_autonomy`, `src/arena_vehicle_interface`의 계산 패키지 소스와 원 r375 입력/비교 결과, 동적 후보 계보, native C++ 소스·setup을 넣는다. x86 .so는 넣지 않는다. 각 Python 후보 원문은 `source_manifest.json`의 SHA256과 `bundle_sources.zip`에 보관한다. 추가 시스템 의존은 기존 Jetson의 Python3, numpy, setuptools/C++ 컴파일러이며 여기서 패키지를 설치하지 않는다.

[`preflight.py`](preflight.py)는 전체 해시, 동적 후보 7단계의 실제 `__file__`, 로드된 `arena_*`/`candidate_recheck`가 새 번들 루트 안에 있는지 확인한다. native 모드에서는 새 루트의 확장 ABI1과 `depth/masked/classes/polygon` 엔트리·바이너리 해시를 기록한다. **사전 검사에서는 rebase를 호출하지 않는다.** native import가 없으면 실패하며 Python으로 조용히 대체하지 않는다.

## 준비 명령과 실행 인계

로컬 번들 준비만 다음 명령으로 수행한다. 기존 archive/manifest는 덮어쓰지 않는다.

```text
python artifacts/validation/2026-11-13/current_cost_r380_v1/build_bundle.py
```

아래는 실행 당시 인계 절차다. 주 에이전트가 배정한 담당자가 archive와 `remote_bootstrap.py`를 사용자 Jetson의 새 위치로 복사하여 실행했으며 회수 결과는 `remote_result/extracted/`에 있다. 사용한 archive SHA256은 `122f13672bdac590285bc4bf4bc15be0341158dac0849d9f65a7c916c8530a44`, 루트는 `/home/q/it-arena-r380-20261113`이다. 이 루트를 재사용하지 않는다.

```bash
/usr/bin/python3 remote_bootstrap.py --archive /home/q/it-arena-r380-20261113.zip --sha256 <bundle_receipt.archive_sha256> --root /home/q/it-arena-r380-20261113 --build-native
```

`--build-native`는 새 독립 루트에서만 native 확장을 빌드하도록 명시하는 옵션이며, 현재 준비 작업이 원격 빌드를 수행했다는 뜻이 아니다. 실행 순서는 새 루트 빌드(120초 상한) → import 사전 검사(60초) → 단일 직렬 계산(900초)이다. 성공/실패/timeout 모두 원 로그·소스 불변성과 소유 자식 종료코드를 기록한다. 센서 포트·ROS·Gazebo·모터/서보·펌웨어·전력 설정을 사용하지 않는다. 비용 측정 중 같은 Jetson의 경쟁 작업을 추가하지 않는다.

결과는 원격 r380의 `jetson/report.json`, `jetson/timing_rows.jsonl`, `import_preflight.json`, `execution.json`, 단계별 로그와 `*-results.zip`으로 회수했다. 원시 입력과 상세 행은 기존 Git 제외 규칙을 따른다. 공개 요약에 원시 행/센서 데이터를 복사하지 않는다. 현재 부분 계측에서 관측하지 않은 r375 적용 비용을 확인된 성능으로 쓰지 않는다.
