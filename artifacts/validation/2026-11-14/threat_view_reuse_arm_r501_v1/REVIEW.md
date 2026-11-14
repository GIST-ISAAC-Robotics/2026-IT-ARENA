# r498 채택 검토와 r501 ARM 비교 인계

2026-11-14, GPT-6 Astra. **r498에서 채택을 막는 코드 결함을 발견하지 않았다. 후보 수정 0건이며 공통 적용은 주 담당이 결정한다.** 아래는 Windows 검토·준비 당시의 기록이다. 이후 주 담당의 Jetson 소유권 인계와 실행 승인으로 r501 ARM 한 쌍을 완료했으며 [실제 결과](RESULT.md)를 별도로 보존했다. 공통 소스·기존 설치·센서·ROS·구동은 변경하지 않았다. r491 설치 준비를 기다리게 하는 선행 조건으로 이 미세 최적화를 두지 않는다.

## 실제 원본과 검토 범위

- 기준 HEAD는 `b59efcab11df06a55d85620ef24433af9b8fd521`이다. [r498 원본 manifest](../threat_view_reuse_r498_v1/source_manifest.json)의 852개 원본 SHA와 현재 공통 852개 SHA를 각각 전수 확인했고 모두 같았다. [동결 영수증](prepared.json)에 현재 856개 Python/native 소스 해시와 바뀐 4개 파일 해시를 보존했다.
- r498 stage는 `traffic_solo_gap.py`, `local_follow.py`, `pass_release_trial.py`의 변경과 `tests/test_threat_view_transfer.py` 신설뿐이다. 원본/후보/입력의 내용은 이번 검토에서 바꾸지 않았다. [검토 diff](change.patch)는 줄끝을 정규화한 읽기용이며 원 `pass_release_trial.py`의 CRLF 때문에 기본 `git apply --check`는 실패했다. `git apply --check --ignore-whitespace`는 통과했다. 채택할 때는 manifest의 예상 baseline SHA 확인 뒤 후보 원바이트를 복사하는 편이 명확하다.
- r498의 [80회귀 기록](../threat_view_reuse_r498_v1/focused_tests_v2.log)과 [원 508 비교](../threat_view_reuse_r498_v1/comparison.json)를 확인했다. 해당 회귀/제어 재생을 중복 실행하지 않았다. 이번에는 5개 준비 스크립트의 문법과 patch 적용 가능 여부, 동결 무결성 및 기존 결과 재집계만 확인했다.

## 동일 호출 재사용 판단

| 검토 항목 | 실제 코드에 근거한 판단 |
|---|---|
| 예외·중첩·초기화 | `selection_view`는 token을 `finally`에서 reset한다. 모든 `select`가 자신의 `(selector, transfer)`를 설정하므로 안쪽의 transfer 없는 호출도 바깥 값을 숨긴다. `select` 이후 보정은 context 밖이다. `_forward_command`의 carrier 목록은 지역 변수이고 다음 호출에 보관하지 않는다. |
| 소비 조건 | owner 객체와 scene 객체가 동일하고, path 전체 배열 값과 lateral config가 동등할 때만 재사용한다. 경로가 바뀌면 새 `threat_view`를 계산한다. 서로 같은 값의 config 사본은 허용하며 다른 값은 거절한다. |
| scene·경로·설정 가변성 | 실제 `LocalScene`·`SensorRay`는 frozen/slots이고 광선/좌표는 tuple이다. lateral config와 budget도 frozen이다. path는 별도 배열 사본을 만들고 쓰기 금지하며, 원 경로의 in-place 변경도 대조한다. 실제 호출 계약의 근거이며 임의의 mutable scene 객체 전체에 대한 보장을 뜻하지 않는다. |
| view 가변성 | view dict/ExecutionFrame 자체를 깊게 동결하지는 않는다. 그러나 `eligible`는 요약을 읽기만 하고 caller에 노출되는 것은 이 호출의 carrier뿐이다. selector는 side 요약을 `dict(now_side)`로 복사해 자신의 기억을 갱신한다. frame의 가변 부분은 같은 query에 같은 투영값을 복사 반환하는 호출 내부 cache이다. 현재 동기 호출 경로에는 view를 외부에서 바꾸는 writer가 없다. |
| partial 기억·다른 policy | `requested_speed`·`evaluate`와 `oblique_metric_enabled` 복원은 기존대로 실행된다. 현재 경로 우선 분기도 기억 갱신을 생략하지 않는다. partial policy의 이전 점·surface cache는 `threat_view(scene,path,lat)` 입력이 아니며 scene/path를 변경하지 않는다. parallel 후보는 별도 detail/path를 만든다. partial 기억이 다른 두 controller 사이에 carrier를 공유하지 않는다. |
| 다음 scan·상속·runtime wrapper | carrier를 controller attribute에 보관하지 않는다. ContextVar owner 확인과 finally reset, 다른 scene의 identity 확인이 겹친다. 기존 `_select` 서명은 바뀌지 않아 실제 CurrentLaneSelector 상속과 runtime monkeypatch를 유지한다. 현재 경로는 동기 호출이며, 새로운 비동기 task/context 전파까지 검증한 것으로 확대하지 않는다. |

`threat_view`의 판단 조건, partial/selector 기억 갱신, TTL, 독립 보호층, 현재 경로 우선 정책은 변경하지 않았다. 기존 508건에서 재사용 37 hit/0 fallback, view 호출 545→508과 seq1304/1311의 2→1은 r498의 Windows 기록이다. 이 수치만으로 Jetson 비용 개선을 주장하지 않는다.

## 기존 Windows 기록의 추가 분리

[재집계 결과](windows_existing_reanalysis.json)는 원 `windows_run_v3`의 저장 출력만 다시 읽었다. 입력/제어 재생은 0회다.

| 비교 | 일치 |
|---|---:|
| 명령 exact | 508/508 |
| strict 전체 출력 | 507/508 |
| 기존 comparable_decision | 508/508 |
| strict 전체 상태 | 437/508 |
| 지정 cached processing_ms만 분리한 상태 | 508/508 |

출력 차이는 `normal.partial_observation.surface_correspondence.processing_ms` 1건, 상태 차이는 `normal.partial_policy.surface_cache[2].processing_ms` 71건이다. [state_codec.py](state_codec.py)는 정확히 이 cached fit 위치만 정규화한다. 같은 이름의 다른 필드를 재귀 삭제하지 않으며 원 strict 결과와 원값을 보존한다.

## r501 실행 자료와 정확한 인계

[bundle.zip](bundle.zip)은 현재 공통 baseline 856개 소스와 동일 기준에서 r498 변경만 적용한 candidate를 각각 동결했다. r493에서는 **508개 입력과 검증할 native binary의 출처만** 재사용한다. r493의 과거 정책 소스를 복사하지 않았다. 원 `numeric_execution=native_v1`, `current_corner_budget=r396_v1` header를 바꾸지 않는다. 최초 generation356 reset부터 원 sequence를 그대로 진행하며 과거 상태를 주입하지 않는다.

| 대상 | 기대 SHA-256 |
|---|---|
| bundle ZIP | `cb7a32f0d65a6646ecf3f779df227defcee21661dc0116979fad39b2dd95bc47` |
| 동결 source manifest | `517b39b26285227f51dddb5929691fd13d81c7754c576a79197ac4bebae3d5da` |
| 원 inputs.json | `909c107fe700cf751e0a8c09b154bec09bff99140cd78b6b449014f8dfa829ac` |
| 재사용 ARM native binary | `5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff` |

native 원본은 `/home/q/it-arena-r470-a/stage/src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-310-aarch64-linux-gnu.so`다. 실행기는 binary SHA와 원본/양쪽 후보의 setup 및 native 4소스 SHA를 모두 확인한 뒤 각각 사본으로 넣는다. 빌드·설치·원본 덮어쓰기는 없다. 기대 source SHA는 [source_manifest.json](source_manifest.json)의 `native_reuse.source_sha256` 및 `changed_files`에 있다.

1. root가 r500의 장치/Jetson 작업 정상 종료와 소유권 인계를 확인한다. `bundle.zip`을 `/home/q/it-arena-r501-a.zip`, [run_once.bash](run_once.bash)를 `/home/q/run-r501-once.bash`로 전송한다. 기존 SSH 연결 방법은 r500 담당자가 그대로 사용한다.
2. 정확한 원격 시작 명령은 `bash /home/q/run-r501-once.bash`다. 이 파일은 ZIP/manifest SHA와 새 결과 경로를 확인하고 `/home/q/it-arena-r501-a`에만 풀어 `python3 -B execute.py --resource-owner r501`을 실행한다. 자원 인계 전에 실행하지 않는다. preflight가 다른 관련 작업이나 센서 포트 점유를 발견하면 시작하지 않는다.
3. baseline508→candidate508→동일 ARM 비교를 순차로 한 번만 실행한다. 각각 새 Python process이며 warmup/반복/cProfile/ROS/센서/구동은 없다. 각 evaluator 자식은 240초, 비교 자식은 90초 한도다. `baseline.pid`, `candidate.pid`, `compare.pid`와 같은 이름의 로그, `execution_receipt.json`을 남긴다. 한도 초과 시 자기 child process group만 SIGINT→필요시 SIGKILL하고 강제 종료를 성공으로 취급하지 않는다.
4. `/home/q/it-arena-r501-a-results.zip`을 이 폴더의 `results.zip`으로 회수하고 표시된 ZIP SHA를 대조한다. `execution_receipt.json`의 completed/exit0/forced=false/residual_group=false, source_changes=[], manifest/native 불변, remaining 빈 목록을 확인한다. 해시·입력·순서/정상 종료가 모두 맞아야 결과를 유효하게 해석한다.
5. `results/comparison.json`의 command/decision/정규화상태 508 일치를 확인한다. raw strict 차이 경로와 값을 함께 읽고 새로운 차이를 진단값으로 임의 제외하지 않는다. 비용은 각 evaluator 호출만 측정하고 full state 직렬화·gzip은 타이머 밖에서 수행한다. baseline 후 candidate 한 쌍이므로 순서·온도·scheduler 영향은 남는다. seq717/1304/1311과 median/p95/max/100ms 초과 개수를 보고하며 전체 10Hz·실차·ROS 동시부하 증거로 사용하지 않는다.

실패/불일치면 원 결과를 보존하고 root에 인계한다. 새 디렉터리로 반복 실행하거나 정책/보호 조건을 바꿔 결과를 맞추지 않는다. 이 비교가 끝나면 장치 점유 없이 Jetson 소유권을 반환한다.
