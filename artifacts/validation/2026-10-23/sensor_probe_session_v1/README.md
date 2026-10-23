# 선택형 센서 기록 세션 감독 — 실제 ROS 제한 시험

관련 **134회귀·3.59 s**, 실제 ROS 정상 기록/주입된 close 멎음 2조건 통과·소스 불변·runner exit 0. 이후 최종 검토의 명시적 무결성 bool 판정/CLI 추가 회귀는 이 실행 이후 변경이므로 별도 전체 회귀로 확인한다.

- 정상: ready 이후 2초 계획 기록, SIGINT 요청, 자식 exit 0·CDR 감사 통과·complete=true. 제어 명령/장치 없이 합성 IMU/scan만 기록했다.
- 멎음: 실제 writer의 close 대신 종료되지 않는 Event 대기를 주입하고 진입 사건을 확인했다. 시험 전용 0.5 s→0.3 s 종료 대기 후 자기 자식 그룹에 SIGKILL, exit -9·complete 실패·완료 manifest 없음. publisher/fixture 스레드도 정상 종료했다.
- 정상 CLI는 30초 준비 대기, 75초 정상 종료 대기, 5초 TERM/5초 KILL 후 확인이 기본이다. 실제 75초 대기·물리 디스크 고장·OS의 uninterruptible sleep은 검증하지 않았다. 단위 시험은 SIGKILL에도 살아 있는 경우를 종료로 거짓 표시하지 않는지 확인한다.

강제 종료된 자료는 로컬에 보존하고 정상 데이터로 승인하지 않았다. root/user 전체 ROS 프로세스를 검색해 죽이지 않으며 새로 시작한 recorder PID/그룹만 대상으로 했다. 실물/Jetson/주행 성능 검증은 아니다.
