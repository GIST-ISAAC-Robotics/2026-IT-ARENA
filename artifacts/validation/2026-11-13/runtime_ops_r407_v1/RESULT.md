# r407 운영 요청 연결 결과

새 runtime 정책·기존 RaceOps·독립 보호층·현행 host를 실제 ROS와 가짜 MCU PTY로 연결한 후속1회가 통과했다. 실물 포트·센서·모터/서보를 사용하지 않았다. **STOP→재개 준비 요청 연결까지의 결과이며 양수 주행·실차 정지 검증은 아니다.**

첫 실행은 `TimeoutError: resume input boundary`로 실패했다. STOP 수락은 intervention2였으나 HTTP에 아직 남은 intervention1로 resume을 요청해 제어기가 `intervention_mismatch`로 정상 거절했다. 원 실패·208파일 manifest·로그는 `verify_v1`에 보존했다. 공통 코드 결함으로 바꾸어 설명하지 않는다.

[revision_v2](revision_v2/README.md)는 시험 도구만 수정했다. 새 STOP 응답의 session/race_epoch/intervention·race_id와 화면의 explicit_stop가 일치한 뒤 그 상태로 resume을1회 요청한다. 순수8검사와 native build/preflight, 실제 ROS·PTY1회가 모두 통과했다. 상태·시간 제한·센서 조건·ARM 조건은 그대로다.

- STOP 버튼→경기 래치·host STOP송신·일치 ACK·가짜MCU 수신을 확인했다.
- RESET 없이 재개 준비가 수락됐으며 planner/guard의 새 입력 경계가 같았다. scan/RGB/gyro/encoder4개가 없으므로 입력 대기·정지 래치를 유지했고 ARM은 시험 전 명시1회에서 증가하지 않았다.
- 개발 상한0.3→0.2m/s의 정지 중 변경, WAIT_GREEN밖 수동 출발의 HTTP/수신노드 거절을 확인했다. 실제 WAIT_GREEN 영상 출발/양수 상한 작용은 이번 범위 밖이다.
- `/drive`와 `/drive/safe`는 모두0이며 보호층 원 command stamp가 host 송신까지 보존된2건을 확인했다. 보호층 우회 발행·합성 encoder 발행은 없다.
- 정상 종료 시 host의 SIGINT exit1은 기존 계약대로 별도 기록했다. STOP송신/ACK, ops/계획/현재검사/영상 자식 exit0, 강제회수0·잔여process group0·211동결소스/준비물 불변이다.

근거: `revision_v2/verify_v1/execution.json`, `revision_v2/verify_v1/extracted/results/report.json`, 원 host summary/trace 및 [원실패 감사](revision_v2/original_audit.json). 합성 과거 경기 기록의 복원은 실제 신호 출발 사실을 만들지 않는다. 동일 운영 요청 검사를 반복하지 않고 미완료 주행·현재 실행 비용/실물 의존 항목으로 이동한다.
