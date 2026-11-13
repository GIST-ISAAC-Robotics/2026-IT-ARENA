# r382 보고서 오프라인 복구 r1

**제어 계산을 다시 실행하지 않고 저장된 589개 계측 행과 821개 비교 행에서 보고서를 복구했다.** 원 pipeline은 보고서 생성 오류로 실패했고 이 판정은 유지한다. 복구 감사 통과는 원 실행의 정상 종료나 x86 동일 재현 통과로 바꾸어 기록하지 않는다.

결과는 [복구 보고서](results/report.json), 근거는 [행·해시 감사](results/row_audit.json)다. 변경은 이 `recovery_r1/` 안에만 있다. 원 `run.py`, 후보/공통 소스, bundle/manifest, 원 로그/JSONL과 `execution.json`은 그대로 보존했다.

## 실패와 계산 완료 범위

원 r382는 native 빌드·import를 통과한 뒤 저장572개와 반례17개를 모두 계산하고 JSONL 두 개를 기록했다. 이후 `run.py:157`에서 다음 코드가 실패했다.

```python
dict(kind=r['kind'], index=r['index'], label=r['label'], **d)
```

외부의 `kind`는 비교 종류(`x86_original_current` 등), `d['kind']`는 개별 변경 종류(`value` 등)다. 같은 키를 두 번 전달한 보고 코드 오류이며 후보 계산 오류가 아니다. 복구본은 `comparison_kind`, `change_kind`, 원 `difference` 객체로 의미를 모두 보존한다. 원 파일은 수정하지 않았다. 초기 준비 검사는 개별 diff와 불변 조건을 검사했지만 실제 sensitive 목록 조립 경로를 빠뜨렸다. [회귀 검사](test_recovery.py)에 그 경로를 추가했다.

확인한 증거는 다음과 같다.

- 계측 **589행 = r371217 + r364249 + 과거106 + 반례17**의 완전한 JSON 행/마지막 개행/순서/인덱스/입력 메타데이터를 확인했다. 과거106개는 원 `selected_inputs.json`의 r279/r282 혼합 순서를 그대로 유지한다.
- 비교 **821행 = 원 현재217 + r375 요약572 + 부모 cap 입력15 + 반례17**을 동결 archive의 원 입력/기대값과 실제 ARM 출력으로 다시 만들었다. 821행의 전체 상세 diff까지 저장 행과 exact tree 동일하다. 저장된 성공 boolean만 읽은 복구가 아니다.
- 원 r375와 같은 ARM 불변 조건을 저장된 전체 before/after에서 재검사했다. 적용93개, 미적용124개, 비대상355개가 유지됐고17반례 모두 기준과 동일하며 정지했다. 전체 미적용496개(반례 포함)는 진단키를 제외한 기준/후보가 exact tree 동일하다.
- 원 archive278개 항목의 해시, 로컬 동결 도구 원문, 원격 preflight의 native ABI1/7계보, 종료 후 `changed_sources=[]`를 대조했다. 원 자식 종료코드는 **0/0/1**, 강제 종료는 없다.
- 원 로그에는 보고서157행의 traceback 하나만 있다. 모든 비교를 저장한 뒤 source 검증을 수행하도록 된 동결 제어 흐름, 전체821행, 앞선 예외 체인이 없다는 점과 종료 후 해시 검사가 계산 루프의 완료를 뒷받침한다. 원 입력 변경 검사는 당시 루프의 assertion으로 수행됐으며, raw에는 모든 입력의 별도 변경 전/후 복사본이 있는 것은 아니다.

복구 보고서의 `original_pipeline_passed=false`, `x86_existing_comparator_reproduction_passed=false`는 유지한다. `recovery_audit_passed=true`, `same_arm_invariants_reaudited_passed=true`, `computation_batch_completed_evidence=true`는 위 저장 증거에 대한 별도 판정이다. 원 `jetson/report.json`은 생성되지 않았고 `query_probe`도 실행되지 않았다. 원 `began_run` 시간이 저장되지 않아 계산 배치 전체 wall time은 복원하지 않았다. `execution.wall_s=92.542013s`에는 빌드/import가 포함되므로 계산 배치 시간으로 바꾸어 쓰지 않는다.

## 회수된 계산 시간

단위는 ms다. 원 top-level r375 호출만 측정한 저장값이며 r375 내부 재시도/중복 호출은 포함한다. 복구 작업에서 측정값을 보정하거나 제어기를 재실행하지 않았다.

| 범위 | n | p50 | p95 | max | >100ms | >200ms |
|---|---:|---:|---:|---:|---:|---:|
| 저장 입력 전체 | 572 | 57.395 | 90.577 | 118.021 | 2 | 0 |
| r371 전체 | 217 | 83.020 | 90.935 | 117.554 | 1 | 0 |
| r375 적용 | 93 | 89.648 | 91.296 | 91.768 | 0 | 0 |
| r371 미적용 | 124 | 47.404 | 87.486 | 117.554 | 1 | 0 |
| 비대상 | 355 | 47.083 | 87.776 | 118.021 | 1 | 0 |
| 반례 | 17 | 2.778 | 44.027 | 44.450 | 0 | 0 |

100ms 초과 두 표본은 r371 index36 GUARD `117.554329ms`, 전체 index255 r364 GUARD `118.020744ms`다. r371 RELEASED162개 전체의 p50/p95/max는 `88.609497/91.002442/95.462053ms`다. 원 자문의130–200ms 추정은 이번 적용93개 표본에서 관측되지 않았다. 다만 한 번의 직렬 저장 계산이며 바로 앞 r370 계산의 cache 영향이 남는다. **100ms는 검토 예산 참고이지 안전 허가선이 아니며, 명령 만료·10Hz 운영·ROS/센서 동시부하·실차 성공을 증명하지 않는다.**

## x86와 ARM 차이

기존 `1e-10` comparator를 벗어난 것은 원 현재 출력19개에서의 region 개수43개 차이다. 값 차이는 ±1~2개이며 `queries`, `masked_left`, `masked_right`에 한정된다. 해당 인덱스는 `9, 11, 17, 18, 19, 35, 36, 37, 40, 42, 61, 64, 96, 97, 99, 100, 119, 120, 121`이다. 상태 분포는 SOLO8/RELEASED9/GUARD2다. 전체 r375 요약572개와 부모 cap15개·반례17개는 기존 comparator 기준으로 같다.

정확한 트리 차이는 경로 좌표1903개, 명령 범주의 조향63개, region 개수43개, 관측 거리/꼬리/공백 기하 등 다른 수치120개다. 63개의 조향 차이(원 출력44, 후보 diagnostic19)는 기존 비교 허용 범위 안이며 원 값을 보고서에 남겼다. 속도·reason·시각·허가 boolean의 의미 있는 이식 차이는 이 저장 비교에서 발견되지 않았다. 모든 원 차이는 원 `comparison_rows.jsonl`에 유지된다. 개수 차이를 버리거나 전체 tolerance를 늘려 x86 재현을 통과시키지 않았다.

정확한 중복 제거/기하 경계에서 부동소수점 차이가 질의 개수로 확대되었다는 설명은 아직 **미확정 추정**이다. 이 비용 복구에는 query probe가 필수이지 않으므로 전체589호출을 재실행할 이유는 없다. 원인 관찰이 필요하면 주 에이전트가 별도 단일 입력 명령을 배정할 수 있다.

## 오프라인 재현과 선택형 query probe

```text
python artifacts/validation/2026-11-13/current_cost_diagnostic_r382_v1/recovery_r1/test_recovery.py
python artifacts/validation/2026-11-13/current_cost_diagnostic_r382_v1/recovery_r1/recover.py
```

6개 회귀 검사와 실제589/821행 감사가 통과했다. 이미 `results/`가 있으므로 두 번째 명령은 덮어쓰기를 거부한다. 재검증이 필요하면 `--output`으로 이 `recovery_r1/` 하위의 새 결과 경로를 명시한다. 제어기 import·제어 계산·Jetson/WSL 접속은 없다.

선택형 query probe는 이미 동결된 Jetson 루트의 `query_probe.py` SHA256 `6b8c753c033292266a3b3d13761be560731cfe0e8ffe341fbd58ae515cb87232`를 직접 실행한다. 원 `run.py`의 main은 실행하지 않는다(공통 보조 함수 import만 사용). 빌드/전체 계측 재실행이 필요하지 않다. 실행 담당자가 최대60초·소유 PID 종료를 관리하며 stdout/stderr를 새 별도 로그로 회수한다. `query_probe_r1` 경로가 존재하면 중단한다.

```bash
/usr/bin/python3 -B /home/q/it-arena-r382-20261113/artifacts/validation/2026-11-13/current_cost_diagnostic_r382_v1/query_probe.py --timing /home/q/it-arena-r382-20261113/artifacts/validation/2026-11-13/current_cost_diagnostic_r382_v1/jetson/timing_rows.jsonl --output /home/q/it-arena-r382-20261113/artifacts/validation/2026-11-13/current_cost_diagnostic_r382_v1/query_probe_r1
```

이 명령은 **아직 실행하지 않았다.** index9의 r370/r375를 계측 밖에서 관찰하며 반환 출력이 기존 비계측 출력과 exact 동일해야 한다. 결과 `query_probe_r1/report.json`, `intermediate_arrays.npz`, 새 실행 로그/종료코드만 별도로 회수한다. 기존 실패 bundle·execution·JSONL은 수정하지 않는다. ARM의 중간 배열만으로 x86와의 원인을 확정할 수 없다는 한계도 유지한다.
