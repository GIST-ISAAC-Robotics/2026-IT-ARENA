# C++ 수신 상태기계 후보 1차 대조(인계 시점 기록)

- [명령](run_checks.sh): WSL Ubuntu, g++ 13.3.0, C++17 `-Werror -pedantic`, 수신기 driver ASan/UBSan과 `-O2`. [컴파일 로그](compiler.log) 비어 있음. [실제 종료](runner_status.txt) 0, [소스 해시 차이](source_hash_diff.txt) 없음. 실행 파일만 Git 제외.
- [관련 회귀](tests.log) 215 통과, [전체 회귀](full_tests.log) 3,610 통과, skip 0.
- [대조](export/report.json): 기존 포팅 벡터 19·전송 12·링크 정책 10·새 수동 반례 22·legacy 3 사례와 명령 receive 사례 3,667건, **이벤트 8,680개의 result·전체 status를 Python과 비교해 차이 0**. stderr 0 B. [`-O2`](export_o2/report.json)도 같음.
- Python과 일부러 다른 C++ 경계 7건(시계 상한 `INT64_MAX-2×10^7`, 음수 시계, 세대·임대 순번 소진 → `counter_exhausted`)은 손으로 적은 C++ 기대값과 일치했고, 같은 입력의 Python 동작을 함께 기록했다.
- [변이체](mutants/mutants.json) 13/13 검출(판정 순서, invalid_drive 무시, receive의 watchdog 갱신, 조향 리셋, 임대 퇴출 방향, 고정 중 물리 STOP, estop_held 무시, 무진행 감시 제거, 만료 임대 미정리, int64 차분 wrap(UBSan 보고 동반), 중복 피드백, 원천 나이 무시, 신선도 경계).

## 증거 실행 전 경위

예비 실행에서 (1) 링크 정책의 257 B 입력 예산 사례를 driver가 256 B로 나눠 위반이 사라진 driver 오류, (2) 검증기가 비JSON 출력에서 예외로 멈춘 문제, (3) 변이체 3개 미검출(고정 중 물리 STOP·만료 임대·극값 이전 표본 사례 부재)과 제 fixture 오류(ARMED 상태에서 임대 만료 전 고장)를 고친 뒤 위 실행을 했다.

## 인계 시점 한계

사용량 경고로 10/22 08:26 UTC 무렵 중단했다. 기술 보고서 §15는 중간 기록만 있다. ROS/MCU 연결·전기 출력·ESP32 툴체인·최악 실행시간은 미검증이며 `firmware_verified=false`, `ready_to_drive=false`.
