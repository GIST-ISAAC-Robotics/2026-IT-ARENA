# 같은 스캔 대기의 통과 이력 수정 — 실행 연결

2026-10-31. [저장 입력 후보](../g1_same_scan_history_v1/README.md)와 같은 두 조건을 `traffic_path_recovery_trial.py`에 반영했다. 기존 선택형 `traffic_path_recovery_trial`의 일시 대기 처리 수정이며 기본 활성화·속도·판정선·물성 변경은 없다.

- 변경 전 파일은 `before_connection.zip`, 부모/전후SHA는 `config/tests/g1_same_scan_history_r1_sha256.json`으로 보존했다.
- `implementation_equivalence.json`에서 후보와 운영 클래스의 AST가 이름 외 동일함을 확인했다. 이는 정적 이식 확인이며 실제 주행 증명은 아니다.
- `tests1.log`: 신규 같은 스캔 반례8개·출처검사·기존 복구/선택기/G2/기록 등을 포함해 **570개 통과**.
- `build1.log`: `arena_autonomy` 빌드 성공. 기존 overlay/byte-compiling 경고는 남았다. `installed1.log`: Python 소스80개 설치본 바이트 일치.

같은 스캔 대기 중0명령, 새 스캔 전 복원 금지, 현재 장애물/미관측 이동영역, 시간·epoch·입력 고장 취소를 유지한다. 이전 실패 기록은 당시 소스의 결과로 보존하며, 최신 코드의 진행 후보를 과거 실제 성공으로 쓰지 않는다.

다음은 [동일155초 실제 코스](../g1_same_scan_history_gazebo_v1/PLAN.md)1회다. 추가 저장 전체 비교를 반복하기보다 동일 구현의 실제 이동 후 영향을 확인한다.
