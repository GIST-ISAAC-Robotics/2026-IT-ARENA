# 단일 모터 PI C++ 대조

**148통과·실패/건너뜀 0, exit 0**, 7.74 s. 실행 전후 `src/scripts/tests/config` 해시 차이 0 B. 실제 장치·ROS 노드·Gazebo는 실행하지 않았다.

- Python 참조의 비유한 중간값을 출력 0·적분 초기화·제동 요청으로 처리했다. 수정 전 3실패는 [numeric v1](../speed_pi_numeric_v1/README.md)에 보존한다. 추가 정수 중간값의 float 환산 overflow도 검사했다.
- `speed_pi.hpp`는 단일 모터축에서 환산한 평균 바퀴 속도 하나를 받는다. 정상 수식/적분 과누적 방지·0목표/비허가 초기화를 유지한다. ARM/STOP 상태기계·watchdog·PWM/DIR를 대신하지 않는다.
- ASan/UBSan과 크기 최적화 빌드 각각 8×500개 합성 순서, 포화·반대 부호 오차·초기화·NaN/Inf·dt 경계·잘못된 설정을 Python과 대조했다. 정상 범위는 수정 전 수식을 보관한 4,000표본과도 일치했다.
- 비유한 검사를 없애는 fast-math/finite-math-only 빌드는 명시 거절한다. 정상 대조는 `-ffp-contract=off`를 사용한다.
- MCU 검사 앱과 공유하는 PI 12사례는 **호스트에서만 실행**했다. 기존 코어/연결부/TX/출력 비활성 사례도 유지한다.

게인 0.6/1.2·최대 dt 50 ms는 참조값이며 실물 튜닝값이 아니다. 음수 effort는 실제 역회전·전기 제동 명령이 아니다. 운영 ROS/Gazebo 모터 루프에는 C++ 후보를 연결하지 않았다.

[실행 스크립트](run_checks.sh), [검사 로그](tests.log), [종료값](checks_exit.txt). 원시 소스 스냅샷·해시 목록은 같은 폴더에 보관한다.
