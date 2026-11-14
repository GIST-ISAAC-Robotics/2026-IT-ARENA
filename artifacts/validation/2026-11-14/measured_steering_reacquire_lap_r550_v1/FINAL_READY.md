# r550 최종 revision2 선택

root의 WSL 독점 인계·단독1회 승인을 받아 최종 revision2를 선택했다. 초기 [Windows 준비](READY.md)의 반대벽 fallback 후보는 채택하지 않는다. [최종 선택 영수증](revision2_selection_receipt.json)과 [source_selection.json](source_selection.json)이 이번 실행 입력의 기준이다.

- `current_wall_reacquire.py`: `598e1d70a61b5fb9757eacba6fd1f4c8bff42d375df2accb7ed9802171b75d1b`
- `local_path.py`: `9ee7d0e3761d96084b3c71c4ef3781c50484f7ea9e2c2f14a93cc3afe0c816ad`
- r543 `lidar_safety.py`/`guard_status_payload.py`: 기존 선택대로, 상태 출력은 기본 `full` 유지

최종 후보는 helper 속도 상한≤.1 m/s일 때 현재 선호벽의 최소 관측을 `min(.28, 차체길이 .207)`로 적용한다. 경로 자체 .25 m·원 점유/제동/조향 변화·STOP, 기본 local_path .28 m와 정상 추종 기준은 유지한다. 반대벽을 새로 선택하는 v1 fallback은 제거했다. root의 revision2 회귀42개와 최종 helper15개 통과 JUnit SHA를 선택 영수증에 기록했고 같은 검사를 반복하지 않는다.

명령·월드·차량 YAML·expected header·7개 runtime 조향 감사·80 sim초 상한·8 sim초 무진행 STOP·원 판정은 r544 attempt2와 같다. 빌드는 `arena_autonomy` 한 패키지만 수행하고 입력 SHA를 전후 비교한다. 그 뒤 실제 import/경쟁0/동결 ZIP 불변을 확인하고 단1회 실행한다. 빌드·하네스 오류 또는 주행 실패를 자동 반복하지 않는다. 이전 원시/ZIP/판정과 미사용 도구 관측 목록은 별도로 보존하며 Jetson/실물 장비에 접속하지 않는다.
