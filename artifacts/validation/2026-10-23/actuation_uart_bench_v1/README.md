# 출력 비활성 UART 주기 어댑터 — 전체 PC 회귀와 변이 검사

**4,128통과·실패/건너뜀 0**, 172.90 s, 검사 exit 0, 실행 전후 `src/scripts/tests/config` 해시 차이 0 B다. 직전 [제어 주기 최종본](../motor_cycle_final_v1/README.md)의 4,122개에 새 [`test_actuation_uart_bench.py`](../../../../tests/test_actuation_uart_bench.py) 6개가 더해졌다. 기존 파일은 문서(README)만 바뀌었고 코드는 새 파일 추가뿐이다.

- 공유 가짜 I/O 사례 **23개**를 ASan/UBSan과 `-Os`에서 각각 실행했다. 계획 거절·외부 I/O 비활성·tick 독립/따라잡기 없음·ARM/DRIVE 거절·STOP 뒤 CLEAR 거절·수신 바이트/시간 상한(첫 읽기 보장 포함)·사건 범람/큐 포화·넘침/프레임 오류/break 세션 경계·송신 0/부분/오류/과대 보고·수신 오류·시각 역행·무수신 비단절·입력 폐기 실패·조립 기한·수신 시각 위치다.
- 정적 검사: 이식 헤더의 SDK/출력 API 부재, 제어 tick 호출 위치 2곳, 대상 앱의 출력/GPIO API 부재·TX ring 0·비차단 읽기·기본 계획 비활성, 빌드 스크립트의 `build/size` 한정, 콘솔 설정 불변.
- 변이 15종(통신 주기마다 tick·지난 기한 따라잡기·비활성에서 I/O·바이트 상한 4배·시간 상한 제거·첫 읽기 보장 제거·사건 상한 4배·큐 포화 무시·오류 사건 무시·읽기 뒤 사건 확인 제거·주기 시작 시각으로 수신 표시·부분 쓰기를 완료로 보고·쓰기 오류 무시·시각 역행 검사 제거·입력 폐기 생략): 정상 대조 통과, **15/15**가 컴파일 성공 뒤 사례 판단 차이로 검출됐다. 시간 초과·충돌은 검출로 세지 않는다. [요약](mutants/summary.json)

가짜 I/O·합성 시각이며 실제 UART/ISR 타이밍·드라이버 동시성·전기 신호·대상 실행이 아니다. 운영 ROS/Gazebo·센서/물성/보호 한도는 바꾸지 않았다. `firmware_verified=false`, `ready_to_drive=false`.

이 실행 뒤 `scripts/build_actuation_bench.sh`의 기호 기대값 한 줄만 바뀌었다([대상 v1 실패](../actuation_uart_bench_idf_v1/README.md)). 그 파일을 읽는 관련 검사 6개는 [v2](../actuation_uart_bench_idf_v2/README.md)에서 다시 통과했다. 실행은 Claude Haiku 5.5 별칭(자기 보고, 하네스 확인 아님) 보조가 맡았고 원시 결과는 Claude Opus 5.5가 대조했다.

[실행 스크립트](run_checks.sh), [전체 로그](full_tests.log), [변이 로그](mutants.log), [종료값](checks_exit.txt), [해시 차이](source_hash_diff.txt), 소스/바이너리 해시와 원시 스냅샷은 같은 폴더에 보관한다.

**후속 검토(같은 날):** GPT 최종 검토가 이 소스의 계약 불일치 3건(사건 큐 포화 관측 순서·호출별 사건 예산·주기 종료 시각 누락)을 찾았다. 위 결과는 당시 소스에 대한 것이며 수정 검증으로 소급하지 않는다. 이 폴더의 원시 스냅샷으로 결함을 재현한 결과와 수정본은 [검토 수정 v2](../actuation_uart_bench_review_v2/README.md)를 따른다.
