# r547 선택형 guard status 실행 준비

**준비본이며 실행하지 않았습니다.** Jetson 재사용 허용·r543 공통 채택·선택한 최신 설치 source 검증 뒤 필요할 때만 기존 r536 단1회 재생에 연결합니다. 이 옵션은 주행·실측 설치의 필수 선행조건이 아닙니다. 현재 `selected_v2`에는 r543이 없으며 해당 ZIP을 수정하거나 candidate를 몰래 주입하지 않습니다.

## 선택본 생성

`stage/prepare.py`는 기존 r536 JSON profile을 받아 **guard의 `safety_status_detail`, `safety_status_evidence_path`만** 선택합니다. 옵션을 생략하면 `full`이며 경로는 빈 문자열입니다. planner의 명시 `g1_trace_detail=compact_v1`은 독립 설정으로 유지합니다. 제어·프레임·포트·보정·TTL 값은 변경하지 않습니다.

```powershell
python -B artifacts/validation/2026-11-14/guard_status_runtime_recipe_r547_v1/stage/prepare.py `
  --profile artifacts/validation/2026-11-14/current_delivery_audit_r536_v1/stage/profile.compact.json `
  --package-parent artifacts/validation/2026-11-14/guard_status_runtime_recipe_r547_v1/prepared `
  --target-results-root /home/q/it-arena-r547-runs `
  --guard-detail compact_v1
```

매번 시각+UUID의 새 `prepared/<run_id>/`에 `profile.json`, `selection.json`을 만듭니다. 상세 경로는 `/home/q/it-arena-r547-runs/<run_id>/guard_status_contexts.jsonl`입니다. 상대 경로·`..`는 거절합니다. PC 준비는 원격 경로의 존재/권한을 확인하지 않습니다.

[현재 준비한 선택본](prepared/r547_20261114T044513Z_e67a6c04bc624667ac859fbb734da581/selection.json)은 기존 r536 simulation profile을 그대로 사용한 연결 예시입니다. 실차 profile 또는 최신 보정 profile이라고 간주하지 않습니다. 실제 담당자가 다른 profile을 선택한다면 그 파일로 새 선택본을 만들며, 이 예시의 전체 설정을 최신 profile 위에 덮어쓰지 않습니다.

## 허용 후 기존 launcher 연결

r536 `stage/`와 원 입력, 이번 선택본을 별도 시험 경로에 복사하고, 설치된 `arena_autonomy/lidar_safety.py`가 두 guard 파라미터와 r543 encoder를 연결하며 `guard_status_payload.py`가 포함됐는지 선택 manifest와 대조합니다. **r536은 guard compact의 설치 여부를 자동 판정하지 않습니다.** 아직 없는 기능을 profile만 지정해 비교한 것으로 처리하면 안 됩니다.

대상 결과의 **부모 폴더만** 준비하고 `<new_result>` 자체는 만들지 않습니다. 아래 값은 해당 선택본의 `selection.json.target_result`와 정확히 같아야 합니다. 설치 activation은 기존 절차를 사용합니다.

```bash
ROS_DOMAIN_ID=158 ROS_LOCALHOST_ONLY=1 python3 -B /test/r536_stage/run_once.py \
  --installed-root /selected/latest-bundle \
  --source-manifest-sha256 SELECTED_MANIFEST_SHA256 \
  --profile /test/r547_selected_recipe/profile.json \
  --input /test/reconstructed_inputs.json.gz \
  --output /home/q/it-arena-r547-runs/<new_result> \
  --dds-library /selected/libarena_dds_probe_v1.so \
  --execute-once
```

명령은 경로 자리표시자를 포함한 후속 담당자용 안내이며 여기서 실행하지 않았습니다. r536이 JSON을 기존 namespace별 YAML로 변환하며 새 guard 값도 보존합니다. 기존 결과 폴더가 있으면 r536의 `mkdir(exist_ok=False)`에서 중단합니다. 상세 파일도 r543의 `open('x')`로 열므로 같은 파일을 덮지 않습니다. 재시험을 별도로 배정받더라도 `prepare.py`로 새 run을 선택하며 기존 파일 삭제로 재사용하지 않습니다.

r536 원 source 검증·입력/프레임 검사·20초 재생·scalar 관측·STOP·자식 회수·요약을 그대로 사용합니다. node wrapper/제어/drain 추가는 없습니다. 정상 종료 시 기존 `owner.destroy_node()`가 r543 writer 종료로 연결됩니다. 강제 종료나 writer 오류는 상세 보존 실패 가능성을 포함해 원 로그와 함께 판정합니다.

## 함께 회수·복원

실행 담당자는 `selection.json`, `profile.json`과 **결과 디렉터리 전체**를 기존 회수 방법으로 PC에 가져옵니다. status만 단독 회수하지 않습니다. 그 뒤 아래 로컬 도구를 사용합니다.

```powershell
python -B artifacts/validation/2026-11-14/guard_status_runtime_recipe_r547_v1/stage/collect.py `
  --selection <copied_recipe>/selection.json `
  --result-directory <copied_result> `
  --destination <new_collection_directory>
```

새 collection 폴더의 `runtime/`에 원 결과 전체, 최상단에 선택본과 `guard_status.restored.jsonl.gz`, 파일 SHA·판정 `collection.json`이 남습니다. 원 compact status/sidecar를 대체하지 않습니다. 원 수신 stamp와 모든 동적 상태 필드는 보존하며 r543 순수 복원 함수 2개를 원문 재사용합니다. profile SHA와 archive ID·record ID·상세 SHA/크기를 대조합니다. 누락·변조·다른 실행 상세는 실패 영수증을 남기며 원 회수 자료는 보존합니다. 같은 collection 경로 재실행은 거절합니다.

`compact_reduction_observed`는 실제 참조 행 유무일 뿐 주행 성공이나 성능 개선 판정이 아닙니다. `requested_compact_but_unobserved=true`이면 미지원/초기 full fallback 등을 조사하고 compact 비교 결과로 쓰지 않습니다. compact와 full이 섞이면 `compact_with_full_rows`와 `runtime/guard*.log` 등을 확인합니다. pending/no-context에는 상세 전체가 남거나 압축할 대상이 없어 축약량이 0일 수 있습니다. file flush는 전원 상실 내구성 보증이 아닙니다.

긴 양수 명령 공백·0목표 총시간/최장구간·원 명령 나이는 기존 `summary.json`, scalar와 STOP/실행 영수증으로 평가합니다. `/drive/safe`의 새 header를 원 명령 나이로 바꾸어 읽지 않습니다. 상세 복원 성공과 런타임 정상 종료는 별도 판정입니다.
