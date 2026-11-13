# r411 현행 정책의 제한 Jetson 비용 — 동결 인계

2026-11-13. 승인된 [PLAN](PLAN.md)의 **plan prefix37 + current153, OFF/ON 각1회 = Jetson380호출**을 준비했다. 실제 제어 계산/native 빌드/WSL/Jetson/ROS/장치 실행은 아직0회다. Windows의 순수 하네스5검사와 AST·archive/입력/해시 검증만 통과했다. 공통 변경0, r410 후보 불포함이다. 실제 실행은 주 에이전트 또는 배정된 실행 담당자가 자원 소유권을 확보한 뒤 수행한다.

## 고정된 입력과 소스

원 r406 call265행은 고유 plan254개와 동일 input/output의 재사용11행이다. 두 Lane의 공유 sequence1..73에 원 입력이 모두 있는 처음37계획만 stateful 원순서 재생에 사용한다. 나머지217계획은 원 자식 이력 복원에서 제외한다. current153개는 각 원 plan input/output과 현재 scan/history를 포함해 독립 재검사 입력으로 복원한다. 상세 누락30개·discard5개·선택/제외 ID·원 source 시각·해시는 [lineage](lineage.json)에 보존했다.

current의 원 기록 분포는 양수145/0거절8, SOLO140/GUARD13, 거리 후보 적용1(index137)이다. 계획 prefix는 SOLO37개, 양수35/0출력2다. 둘 다 원 실패 주행에서 기록된 표본이며 전체 자식 호출/실시간 적용/실차 주행 비율이 아니다. RELEASED와 후반 계획 비용은 이번 범위에 없다. r382의 옛89–91ms나 r358 비용을 before로 쓰거나 개선율을 만들지 않는다.

`inputs/raw_header.json`은 원 r406 Follow/Execution/MotionConfig와4가림 JSON 문자열을 보존한다. evaluator용 header에 runtime policy/time policy/native 선택만 명시하며 OFF/ON의 차이는 등록된 거리 후보와 그 환경 표식뿐이다. 원 `laser_frame`, verified simulation 원천시각/epoch/dt, 원 wheels/gyro 값과 최종 기하를 바꾸지 않는다. 저장 시뮬레이션 값은 실제 encoder나 실센서 관측으로 승격하지 않는다. 현재153개에 전달할 원 plan output은 새 prefix 계산 결과로 대체하지 않는다.

**부모 감속 cap23개**는 계산 결과와 별개다. `runtime_dual_state._cap_ready`가 원 current 반환 뒤 붙인 `new_plan_deceleration_cap`과 `checked_target_mps`를 원문에 남겼다. runner는 실제 evaluator 속도와 checked_target를 먼저 상세 비교하고, 기록된 cap으로 부모 출력만 재구성하여 원 전체 결과와 별도 비교한다. 이 처리는 timer 밖이며 원 evaluator 반환도 그대로 보존한다. cap으로 속도가 같아졌다고 checked_target 불일치를 감추지 않는다. smoke의 current0/42/137에는 cap이 없다.

archive는157파일이며 현행 policy package23개(원 ledger17 + init/evaluator2 + r405 helper4)를 포함한다. package23개와 조사한 경계7파일은 승인 시점 r406 동결 해시와 같았다. 정적 import 폐포 및 C++ 원문만 포함하고 기존 .so/.pyc는 넣지 않았다. r410 후보나 artifact 동적 import는 없다. source freeze 뒤 공통이 바뀌어도 이 묶음은 아래 archive만 사용한다.

| 동결 대상 | SHA256 |
|---|---|
| `bundle_sources.zip` (4,655,202 B) | `cfdca33516c9ff638866e6224fe0c17be425b157979254bc91b3c8c06c02a06d` |
| `source_manifest.json` | `f3c35afe439ac8da262d9b393f436638e4a15955d0ae4e2e2ad0240aca8eff18` |
| `remote_bootstrap.py` | `b4c52985bdf659a1a96302ebad72042fa6a49afd18fa55ee4ce1fd644c9a3794` |
| 입력 계약 | `54bc6dd3c6f872f1cda12a569dc01ae4cf0121ba9f9e5dd7e2d3faf501c56a85` |

## 계측·판정 의미

기존 공통 `Evaluator`/`Lane`를 사용하며 새 전역 monkeypatch는 없다. 기존 plan evaluator가 자기 자식에서 설치하는 원 정책은 그대로다. `timed_evaluator.py`가 `Evaluator.__call__` 한 번의 시작~반환만 측정한다. 입력 복원/scene/기준 정책/선택 거리제안·최종검사/freeze를 포함한다. 원문 입력 hash, 상태 checkpoint, 상세 diff, 파일 쓰기는 계측 밖이다.

Lane에는 진단 wrapper의 입력 감사·선택기 상태/모듈 hash·진단 JSON 직렬화가 추가된다. 따라서 `lane_with_audit_ms`/`delivery_with_audit_ms`는 **진단 부하를 포함한 회수 지표**이며 운영 IPC 비용이라고 부르지 않는다. 주 결과는 `evaluator_ms`다. startup/첫 호출은 별도 표시하고 예열 반복/첫 표본 삭제는 하지 않는다. 순서는 plan OFF→ON, current OFF→ON이며 한 번에 자식1개다. 단일 순서·cache/온도 영향과 작은 표본의 한계를 남긴다.

plan/current·SOLO/GUARD·양수/0·후보 비대상/제안거절/최종검사거절/적용별 비용과 결과 분포를 기록한다. p50/p95/max·100/200ms 초과 개수는 계산 예산 참고이며 명령 만료나10Hz 운영 허가가 아니다. 원 적용1개는 `single_ms`로 보고한다. 같은 current의 ON−OFF는 paired difference만 기록하며 역사 버전 대비 개선율을 만들지 않는다.

원 r406 x86 기록 exact 비교와 같은 ARM의 OFF/ON 불변 검사를 분리한다. 기존 `comparable_decision`의 표면 정합 processing_ms 처리만 유지하고 tolerance/필드 삭제를 새로 추가하지 않았다. r382의 상세 diff 도구를 재사용하되 최상위 plan 속도/조향 차이도 command 범주로 분류한다. 작은 부동소수·region 개수·시각/상태/허가 차이를 모두 원값과 함께 보존한다.

- `passed`: 전체 계산·동일 기계 불변·원 기록 exact·소스/종료 검사가 모두 통과했다는 뜻이다.
- `cost_collection_completed`: 전체 계산·동일 기계 불변·소스/종료 감사가 완료됐다는 별도 값이다. ARM/x86 exact 실패와 동시에 true일 수 있다.
- `historical_exact_reproduction_passed`: 원 r406 출력 exact 여부다. 미세 수치 차이도 false를 유지한다.
- `applied_count_matches_recorded`: 원 적용 index137 한 개와 같은지 별도 기록한다. 다르면 이식/재현 차이이며 자동 채택하지 않는다.

WSL smoke는 exact 실패도 exit1로 중단한다. Jetson은 x86 차이를 상세 기록하며 전체 비용을 끝까지 수집할 수 있으나, 같은 ARM plan 출력/상태 불변·비대상/미적용 불변·적용 시 단일 최종검사/시각·경로 보존이 깨지면 중단한다. 해시·입력·native/import·예외·자식 회수 실패도 즉시 중단한다. full의 exit0은 비용 진단 완료일 수 있으며 `passed=true`를 대신하지 않는다.

## 1. WSL native smoke — 같은 runner,80호출

다른 WSL 실행/빌드 소유자가 반환한 뒤 실행한다. ROS setup/노드/센서는 사용하지 않는다. 계획 prefix37개는 OFF/ON 둘 다 실행하고 current는 **0/42/137 세 개만** 실행한다. 첫 양수·0거절·후보 적용을 포함하며, 나머지 current150개는 여기서 반복하지 않는다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
/usr/bin/python3 -B artifacts/validation/2026-11-13/runtime_cost_r411_v1/remote_bootstrap.py \
  --archive artifacts/validation/2026-11-13/runtime_cost_r411_v1/bundle_sources.zip \
  --sha256 cfdca33516c9ff638866e6224fe0c17be425b157979254bc91b3c8c06c02a06d \
  --root /home/jinhyeong/it-arena-r411-smoke-20261113-a \
  --subset smoke
```

결과: `/home/jinhyeong/it-arena-r411-smoke-20261113-a-results.zip`. 원문을 새 `verify_smoke_v1/`에 회수한다. zip 안 `smoke/report.json`의 `passed=true`,80호출, exact/불변 통과, execution의 source 변화0/강제·잔여0을 부모가 검토한다. 실패하면 해당 원결과를 보존하고 Jetson으로 진행하지 않는다. 전체 현재153 반복이나 성공까지 자동 재시도는 없다.

## 2. Jetson 본 계측 — smoke 통과 후380호출 한 번

소유권을 받은 실행 담당자가 아래 세 파일을 새 경로로 복사한다. 기존 접속/회수 도구를 그대로 쓰며 비밀번호·새 네트워크 설정·전력 변경은 필요 없다.

1. 동결 archive → `/home/q/it-arena-r411-20261113-a.zip`
2. 동결 bootstrap → `/home/q/r411-bootstrap.py`
3. 검토한 WSL 원 `smoke/report.json` → `/home/q/r411-smoke-report.json`

복사 후 archive/bootstrap hash를 위 표와 비교하고 smoke 보고서 hash도 WSL 회수 원문과 대조한다. 아래 hash 계산은 공개된 시험 보고서에만 사용한다.

```bash
r411_smoke_sha=$(sha256sum /home/q/r411-smoke-report.json | cut -d ' ' -f1)
/usr/bin/python3 -B /home/q/r411-bootstrap.py \
  --archive /home/q/it-arena-r411-20261113-a.zip \
  --sha256 cfdca33516c9ff638866e6224fe0c17be425b157979254bc91b3c8c06c02a06d \
  --root /home/q/it-arena-r411-20261113-a \
  --subset full \
  --smoke-report /home/q/r411-smoke-report.json \
  --smoke-sha256 "$r411_smoke_sha"
```

bootstrap은 ARM에서 smoke subset/x86/80호출/exact/불변 통과와 **동일 source_manifest SHA256**을 확인한다. 다른 source/input의 영수증은 재사용하지 않는다. 동일 archive가 유지되고 위 검토가 끝난 smoke 영수증만 본 계측의 전제다. source를 수정하면 새 판본과 새 smoke가 필요하며 이 파일을 덮어 재동결하지 않는다.

두 환경 모두 helper5검사10초→native build180초→import30초→runner180초다. runner 내부 전체 상한도180초이며 Lane별20초/대기25초는 시험 회수 상한이다. 운영 TTL과 관계없다. timeout은 소유 process group에 SIGINT 후10초, 필요하면 KILL 후3초로 회수한다. 잔여 group도 회수하여 실패로 기록한다. 새 root/result archive가 이미 있으면 거절하며 삭제·자동 재실행하지 않는다. 전력/클럭/BLAS 설정은 바꾸지 않고 읽을 수 있는 부하/메모리/CPU 주파수·온도와 환경값만 시작/끝에 기록한다.

결과 archive는 `/home/q/it-arena-r411-20261113-a-results.zip`이다. `execution.json`, `import_preflight.json`, 단계별 log, `full/report.json`, `full/timing_rows.jsonl`, 전체 `full/comparison_rows.jsonl`, 실패 시 traceback을 새 `remote_result/`에 회수한다. 원 r382/r406 자료와 기존 원격 root는 건드리지 않는다. 원시 입력·상세 행·archive는 기존 Git 제외 정책을 유지한다.

## 준비 검증과 동결

`windows_tests.json`: 순수5검사 통과. tiny/region/명령 diff 보존, 부모 cap이 잘못된 checked_target를 숨기지 않는 반례, 비대상/미적용 불변, 단일 표본 표시를 확인했다. `static_verification.json`:157개 archive 해시·전체 Python AST·native 원문·23개 정책·입력37/153 및 선택 해시 대조 통과. 정적 외부 런타임 의존은 numpy이며 native 빌드에는 기존 setuptools/C++ 도구가 필요하다. 실제 native import/계산은 아직 검증하지 않았다.

재사용 원문: r398 상태 checkpoint, r382 상세 diff/부모 cap 대조, r382/r407의 새 root·유한 실행·자식 회수 틀이다. `build_bundle.py`는 정적 준비용이며 동결 뒤 다시 실행하지 않는다. archive/source code는 부모에게 동결 사실을 전달했고, 최종 준비 manifest 인계 후에는 문서도 명시 반환 전까지 수정하지 않는다.
