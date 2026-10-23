# 호스트 세션 검출력 첫 실행 — 실패 보존

- [기본 검사](tests.log) 52통과. [변이 보고](mutants/report.json)는 대조 사본 50통과, **12개 중 11개가 AssertionError로 검출**되어 exit 2다. 소스 해시 차이 0 B.
- `stop_does_not_preempt` 사본도 실패했지만 STOP envelope가 `None`인 상태를 확인하지 않고 인덱싱해서 TypeError로 끝났다. 의도한 계약 Assertion으로 판별하지 못했으므로 검출 성공으로 세지 않았다.
- 후속은 STOP 선점 결과 존재를 먼저 검증하도록 시험을 수정한다. 이 실패 로그/사본/스냅샷은 보존한다.
