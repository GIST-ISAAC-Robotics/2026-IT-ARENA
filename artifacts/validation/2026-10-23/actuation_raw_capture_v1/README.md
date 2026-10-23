# 수동 바이트 기록 v1 — 검사 경로 설정 실패

`run_checks.sh`가 `PYTHONPATH`를 명시하지 않아 `actuation_raw_capture`와 `actuation_host_observer` import에서 수집 오류 2건, exit 2로 끝났다. 테스트 본문은 실행되지 않았다. 소스 해시 차이 0 B, 실제 장치 접근 없음. [로그](related_tests.log)와 실행 당시 스냅샷을 보존한다. 다음 실행은 기존 로그를 덮어쓰지 않고 v2에서 경로를 명시한다.
