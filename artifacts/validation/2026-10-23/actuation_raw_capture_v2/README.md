# PC 원시 바이트 기록·오프라인 감사 v2

**82통과·실패/건너뜀 0**, 7.29초, exit 0·실행 전후 소스 해시 차이 0 B다. [v1](../actuation_raw_capture_v1/README.md)의 import 경로 설정 실패를 보존하고 `PYTHONPATH`를 명시했다. 구현 코드/시험의 변경 없이 본문 검증을 수행했다.

- 파일/pipe 입력, 분할/합친 실제 C++ 송신 후보 프레임, CRC 손상/콘솔 잡음/형식 거절, 중복/boot 변화/부분 꼬리를 검사했다.
- 크기·chunk·시간 상한, 조용한 pipe의 50 ms 종료, EOF·Ctrl-C·읽기 오류·시계 역행·동일 시각을 구별했다.
- raw/metadata/footer/완료 표식의 손상·누락·중복·offset·파일 혼합·추가 꼬리, 각 저장 파일의 부분 쓰기/디스크 실패는 무결성 성공으로 처리하지 않았다. 기존 출력 폴더 덮어쓰기 거절도 확인했다.
- 실제 tty/serial·MCU·ROS·Gazebo·Jetson은 사용하지 않았다. C++ sender는 PC host 프로세스이며 물리 통신이 아니다. 검사는 새 수집기와 기존 보관기 관련 회귀를 함께 실행한 결과다.

`record_integrity_valid`·EOF 기록 완료와 패킷/센서/실차 검증을 구별한다. 완료 표식의 해시는 인증 기능이 아니며 driver 이전의 손실·정확한 frame byte 위치/취득 시각·전송 지연은 판별하지 않는다. [계약·사용법](../../../../docs/hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#7-1023--명령-없는-pc-원시-바이트-기록).

[실행 명령](run_checks.sh), [관련 검사 로그](related_tests.log), [종료값](checks_exit.txt), [소스 해시 차이](source_hash_diff.txt), 실행 당시 소스 스냅샷과 해시는 같은 폴더에 보존했다. GPT가 구현·검토, GPT-6 Luna가 실행·회수했다.
