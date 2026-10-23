# writer 정리 오류 수정

관련 **262회귀·1.38 s**, 실제 ROS 3조건 통과·소스 불변·runner exit 0. Python 기록 경로의 종료를 finally 한 곳으로 옮기고, 종료 오류를 기존 오류 뒤에 추가했다. 종료 재시도나 기존 실패 덮어쓰기는 없다.

| 조건 | close 호출 | manifest 오류 | 자식 종료 |
|---|---:|---|---:|
| 정상 160표본 | 1 | 없음, complete=true·CDR 감사 통과 | 0 |
| close 후 예외 | 1 | 종료 오류, complete=false·감사 거절 | 1 |
| 3표본 뒤 write 오류 + close 후 예외 | 1 | 쓰기와 종료 오류 모두, complete=false·감사 거절 | 1 |

세 경우 처리되지 않은 writer 스레드 예외는 없었다. 이전 실패는 `writer_cleanup_baseline_v1`에 보존했다. 예외 주입 전에 실제 MCAP close를 완료하므로 실제 저장장치 고장·멎음·파일 손상 복구·강제 전원 차단의 검증이 아니다. 동작/시간/물성/보호 설정은 변경하지 않았다.
