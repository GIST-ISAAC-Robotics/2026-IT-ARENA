# 호스트 요청 상관 후보 첫 검사 — 실패 보존

- [실행](run_checks.sh), [로그](tests.log): **176 passed·1 failed·exit 1**, 소스 해시 차이 0 B. 실패 당시 소스 스냅샷을 보존했다.
- 실패는 `test_bad_ack_never_establishes_permission[bad_ack2]`의 시험 보조 함수 `ack(request, **kwargs)`에 `request` 필드 덮어쓰기를 넘길 때 Python 인수가 중복된 `TypeError`다. 후보가 잘못된 요청 ID를 허용했다는 실패가 아니라 **해당 반례가 실행되지 못한 시험 오류**다.
- 후속은 보조 인수 이름을 구별하고 로컬 STOP 의도 유지/명시 CLEAR 확인 등의 반례를 추가해 새 경로에서 실행한다. 이 v1을 통과 결과로 바꾸지 않는다. 실물 송신/보드/ROS 실행 없음.
