# r452 복구 뒤 최종 준비 확인

주 담당이 기존 허용 범위에서 Ubuntu-24.04를 복구한 뒤, **r452 소유 프로세스 잔여 0·실행 wrapper 구문 exit0·동결 1,371 입력 불변**을 확인했습니다. build/import/19preflight는 반복하지 않았습니다. 앞선 WSL 오류와 [준비 영수증 원본](preparation_receipt.json), [오류 당시 인계](resource_handoff.json)는 그대로 보존합니다. 최신 준비 상태는 [ready_after_recovery.json](ready_after_recovery.json)을 따릅니다.

**Gazebo 주행은 미실행**이며 `registered.json`, `run1`, `execution_v1/run.pid`도 아직 없습니다. 주 담당 최종 검토 뒤 한 번 실행할 준비만 끝난 상태입니다.

최종 `root_execution_review.json`에 필요한 필드는 다음과 같습니다. 이 예시는 실행 확인 자체가 아닙니다.

```json
{
  "root_reviewed": true,
  "execution_limit": 1,
  "repeat_on_failure": false,
  "prepared_sha256": "03be19bc32e19b4f6b2982f875153316f7658f824e29c79f901730a40eac5477",
  "source_archive_sha256": "a97601678a3f2fbbe88b76cc37cb37fd637f104682fe80d340c917510adbf185"
}
```

WSL 실행 담당이 사용할 고정 wrapper는 아래와 같습니다. wrapper가 root 확인 receipt·prepared/archive SHA·19검사/skip0·동결 불변을 확인한 뒤 실제 `python3 path/run.py`를 시작합니다. 실제 run argv에는 `-B`를 붙이지 않습니다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/guard_crawl_drive_r452_v1/execution_v1/execute_once.sh'
```

원 80sim초·RTF0.25·8sim초 무진행 STOP·실패 반복 금지·접촉/이탈/종료 기준을 유지합니다. `.1` 이후 진행·저속 지속시간·`.15/.35 m/s` 초과 정상 가속 복귀를 따로 집계하는 [분석 계획](README.md#후속-분석-항목)도 동결 자료에 포함돼 있습니다.
