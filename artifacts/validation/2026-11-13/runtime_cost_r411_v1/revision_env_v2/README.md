# r411 환경 메타데이터 읽기 보완 — revision_env_v2

> 아래는 실행 전 동결 인계 기록이다. 이후 새 WSL smoke80과 Jetson380을 완료·회수했다. [결과와 정확 비교 실패 분류](../RESULT.md)를 우선하며, 아래 ‘미실행’ 문구를 현재 상태로 해석하지 않는다.

2026-11-13. 원 WSL smoke는80호출·exact/불변/소스·정상 회수를 통과했다. 그러나 첫 Jetson 실행은 `environment_snapshot()`의 `path.read_text()`에서 `TypeError: can't concat NoneType to bytes`로 종료했다. 첫 소유 subprocess와 native 빌드 이전이다. 원 `remote_execution.log`, archive/manifest, `/home/q/it-arena-r411-20261113-a`를 보존한다. 실패한 구체 경로는 로그에 없으며 sysfs 등의 일시적 읽기 상태 가능성은 **추정**이다. 원인 확정이나 광범위 환경 진단은 수행하지 않았다.

수정은 `remote_bootstrap.py` 한 파일이다. 각 경로에서 binary `read(4096)`을 요청하며 `OSError`/`TypeError` 또는 비바이트 반환을 `available=false,error`로 기록하고 다음 정상 경로를 계속 수집한다. 경로 열거 등 snapshot 전체의 부수 오류도 `safe_environment_snapshot`에서 unavailable로 반환한다. 시작/종료의 두 메타데이터 호출만 이 함수를 사용하므로 환경 수집 오류가 `finally`의 execution/ZIP 생성을 막지 않는다. 제어/native/import/계측 오류 처리에는 이 예외 포착을 적용하지 않는다.

`bootstrap.patch`가 전체 변경이다. 원 source157개 중 bootstrap1개만 다르고 **156개는 바이트 동일**하다. 입력 계약 SHA256도 동일하다. 제어기·wrapper/Lane·입력·시간·측정·판정·시간상한은 변경하지 않았다. main AST는 메타데이터 함수 호출 두 곳을 제외하면 동일하다. 새 테스트/문서/준비 도구는 별도 파일이며 실행 source157개에 추가하지 않았다.

순수 회귀4개가 통과했다(`tests.log`). OSError/TypeError 뒤 정상 메타데이터 보존,4096B 요청·None 처리, snapshot 전체 오류 뒤 JSON/메모리 ZIP 생성 가능성을 확인했다. 이는 실제 sysfs/native/Jetson 재실행 결과가 아니다. 새 판본의 WSL/Jetson 실행은 아직0회다.

| 파일 | SHA256 |
|---|---|
| 새 `bundle_sources.zip` (4,655,428 B) | `a9ff5eca335da1da023530f7ea10494a32d0dde901a5bf65e1fb4f89a3f4db67` |
| 새 `source_manifest.json` | `868e238e9dc7df9275c53a4e0f7062f0e399a9def977c3c93e7f1d3e8b1a25f7` |
| 새 `remote_bootstrap.py` | `fb8f5598f83eacbafcaf00de7bb5ca0f33ba04539a447311054a6a335230aed7` |
| 동일 입력 계약 | `54bc6dd3c6f872f1cda12a569dc01ae4cf0121ba9f9e5dd7e2d3faf501c56a85` |

## 새 WSL smoke80 — 반드시 먼저1회

기존 smoke 영수증을 새 manifest의 통과로 승격하지 않는다. 실행 담당자가 WSL 소유권을 받은 뒤 아래 새 root만 사용한다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
/usr/bin/python3 -B artifacts/validation/2026-11-13/runtime_cost_r411_v1/revision_env_v2/remote_bootstrap.py \
  --archive artifacts/validation/2026-11-13/runtime_cost_r411_v1/revision_env_v2/bundle_sources.zip \
  --sha256 a9ff5eca335da1da023530f7ea10494a32d0dde901a5bf65e1fb4f89a3f4db67 \
  --root /home/jinhyeong/it-arena-r411-smoke-20261113-b \
  --subset smoke
```

`/home/jinhyeong/it-arena-r411-smoke-20261113-b-results.zip`을 새 `revision_env_v2/verify_smoke_v1/`에 회수한다. 원과 같은 계획37 + 현재0/42/137, OFF/ON 총80호출이다. exact/불변/소스/정상 종료를 통과한 새 `smoke/report.json`만 아래 전제로 사용한다. 실패하면 첫 결과를 보존하고 멈춘다.

## 새 Jetson380 — 새 smoke 통과 후1회

동결 archive를 `/home/q/it-arena-r411-20261113-b.zip`, 새 bootstrap을 `/home/q/r411-bootstrap-b.py`, 새 smoke 원 보고서를 `/home/q/r411-smoke-report-b.json`으로 복사한다. archive/bootstrap hash와 smoke 보고서의 전송 전후 hash를 확인한다. 원 suffix a 파일/폴더는 재사용하거나 덮지 않는다.

```bash
r411_smoke_sha_b=$(sha256sum /home/q/r411-smoke-report-b.json | cut -d ' ' -f1)
/usr/bin/python3 -B /home/q/r411-bootstrap-b.py \
  --archive /home/q/it-arena-r411-20261113-b.zip \
  --sha256 a9ff5eca335da1da023530f7ea10494a32d0dde901a5bf65e1fb4f89a3f4db67 \
  --root /home/q/it-arena-r411-20261113-b \
  --subset full \
  --smoke-report /home/q/r411-smoke-report-b.json \
  --smoke-sha256 "$r411_smoke_sha_b"
```

결과는 `/home/q/it-arena-r411-20261113-b-results.zip`, 새 회수 폴더는 `revision_env_v2/remote_result/`다. source manifest가 바뀌었으므로 원 smoke는 기존 동일-manifest 관문에서 거절된다. helper10초/native180초/import30초/runner180초와 소유 프로세스 SIGINT10초→필요 시KILL3초 회수,380호출1회, 원문 diff·정확성/비용완료 구분은 [원 인계](../README.md) 그대로다. 메타데이터 unavailable은 비용 결과와 함께 남기며 성능 정책/전력 설정을 변경하지 않는다.

`build_revision.py`는 원 동결 archive에서만 파생했다. common을 다시 수집하지 않았으며 r410은 포함되지 않는다. 원 bootstrap/manifest/실패 log는 `reference/`에 바이트 사본·해시로도 남겼다. 새 준비 manifest 인계 후에는 문서 포함 명시 반환 전 수정하지 않는다.
