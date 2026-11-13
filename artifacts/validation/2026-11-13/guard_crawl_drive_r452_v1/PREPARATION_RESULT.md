# r452 준비·검증 인계

후속: 주 담당 WSL 복구 뒤 소유 잔여 0·wrapper 구문·동결 불변 확인을 마쳤습니다. 최신 상태는 [READY.md](READY.md)를 따르며 아래 오류 당시 회수 기록은 보존합니다.

**Gazebo 주행은 시작하지 않았습니다.** r441과 같은 config/actors/물성/기하/C1/속도 profile/80sim초/8sim초 무진행 STOP을 유지한 준비를 완료했습니다. root는 26파일 실제 delta를 검토하고 동결을 승인했습니다. 3패키지 build·actual import·native ABI 확인은 exit0이며, 기존 16개와 trace 연결 3개를 합친 사전검사 **19개 모두 통과·skip0**입니다. 이후 source 1,371개와 archive 불변을 다시 확인했습니다.

다만 정상 종료 뒤 사후 `/proc` 확인용 새 WSL 호출과 root가 허용한 단 한 번의 `/bin/true + 자기범위 조회` 재시도가 모두 **`Wsl/Service/E_UNEXPECTED`, exit -1**로 즉시 실패했습니다. build/import/preflight 전경 실행이 정상 회수됐다는 사실과 **사후 잔여 PID 미확인**을 구분합니다. 빌드나 시험은 반복하지 않았으며, WSL 소유권은 복구를 담당할 root에 반환했습니다. `[registered.json, run1, execution_v1/run.pid]` 모두 없는 상태입니다. 실행 승인은 여전히 별도입니다.

| 항목 | 결과 |
|---|---|
| r441 대비 실제 공통 delta | 26개: 기존 변경 19 + snapshot 신규 7. 미분류/이관 SHA 불일치 0 |
| 포함 이관군 | r439, r444, r446, r449, r451, trace launch 전달 |
| 선택 차이 | 기존 선택 유지 + `g1_complete_plan_trace:=true` 한 항목 |
| 공통 trace 기본 | false, controller bool 전달 확인, 독립 guard에 trace 옵션 추가 없음 |
| build/import/native | 3패키지 build exit0, Python 경로 11개 확인, native ABI1 및 masked/classes 확인 |
| preflight | 19 passed, failures/errors/skipped 0, 전후 입력 불변 |
| 동결 | 1,371 입력, root source 검토 receipt 포함, 변경 0 |
| 실제 주행/Jetson | 실행 0 |
| 사후 프로세스 조회 | WSL 서비스 오류로 미확인. 정상 회수 전경 작업과 구분 |

정확한 SHA는 다음과 같습니다.

- `prepared.json`: `03be19bc32e19b4f6b2982f875153316f7658f824e29c79f901730a40eac5477`
- `sources.zip`: `a97601678a3f2fbbe88b76cc37cb37fd637f104682fe80d340c917510adbf185`
- `source_scope.json`: `18f53a10dd927d6e09c989404af0d0a16c2d2b60002aeccfe082ed83c204efcd`
- `root_source_review.json`: `5eaee45ce41ff95f10a75afd97db08bc6b4a8bccbe567937452217648a77e38b`

[준비 영수증 원본](preparation_receipt.json), [후속 자원 인계](resource_handoff.json), [빌드 영수증](build_v1/receipt.json), [19개 원 로그](preflight_v1/pytest.log), [동결 직전 전체 검토](source_review_postbuild_v1.json), [전체 source diff](source_delta_postbuild_v1.patch)를 보존했습니다. colcon의 기존 self-underlay override 경고와 최초 inventory의 codec alias 오류도 원 로그/파일로 남겼습니다. codec만 `utf-8-sig`로 바로잡은 v2에서 전체 목록이 통과했으며 판정 조건은 완화하지 않았습니다.

root 최종 확인과 WSL 복구 뒤 사용할 명령은 `python3 artifacts/validation/2026-11-13/guard_crawl_drive_r452_v1/run.py`입니다. 준비된 `execution_v1/execute_once.sh`는 root 실행 receipt의 prepared/archive SHA, 시도 1회, 실패 반복 금지, preflight 19/skip0를 요구합니다. 실제 run argv에 `-B`를 붙이지 않고 `PYTHONDONTWRITEBYTECODE=1`을 사용합니다. 실행/STOP/관측 wrapper도 archive에 동결했습니다. 실행 wrapper의 별도 `bash -n` 확인은 사후 WSL 서비스 오류 때문에 수행하지 못했으며, 실행은 일절 하지 않았습니다.

후속 분석에는 후보 수락/발행뿐 아니라 **후보 시작 뒤 진행거리·연속 `.1` 구간 시간·이후 `.15` 및 `.35 m/s` 초과 가속 복귀**를 구분하도록 [준비 README](README.md)에 명시했습니다. 일시 탈출이 목적이며 영구 `.1` 주행을 성공으로 취급하지 않습니다. 원 성공 기준·8sim초 정체 STOP·r441 실패는 그대로 유지합니다.
