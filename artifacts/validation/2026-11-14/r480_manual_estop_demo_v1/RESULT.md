# 수동 jog STOP 연결 간단 확인

**이미 구현된 연결을 확인했고 기존7개 검사가 통과했다. 누락을 발견하지 않아 코드·새 테스트·보호 기능을 추가하지 않았다.** Windows 순수 Python/모의 확인이며 실물 비상정지 시연 성공은 아니다.

- 패널 전체 STOP: `race_ops_node.py:306`에서 수동 모드에도 허용하고 `manual.stop`, `manual_prepare=None`, race STOP 발행 및 MCU STOP 요청을 한다. 수동 전용 STOP도 같은 파일344행에서 자식 중지/준비 해제/MCU STOP을 요청한다.
- 수동 자식: `race_ops_manual.py:166`의 `stop()`이 대기 요청을 취소하고 prepared identity를 지운 뒤 실행 중인 자식 stdin에 `STOP`을 보내고 닫는다. 한 번 끝난 jog는 별도 prepare 없이 두 번째 pulse를 시작할 수 없다. `development_jog.py:107`도 held 상태를 거부한다.
- 물리 스위치: runtime firmware `main.cpp:76`이 GPIO8을 읽는다. `firmware_runtime_preview/main/stop_switch.hpp:8`은 HIGH/개방을 held로 해석하며 endpoint→`ActuationReceiver.set_estop`으로 전달한다. `actuation_receiver.hpp:126`에서 pressed는 ESTOP, 해제는 상태 전환 없이 latch를 유지한다. CLEAR/ARM은 held 중 거절되고 별도 단계다. transition은 목표속도를0으로 만든다.
- 실제 출력 연결: runtime firmware77행은 held일 때 `motor_zero()`,117행은 준비되지 않음/미ARM/서보 비활성/목표0일 때 다시 `motor_zero()`를 호출한다. 여기까지는 소스 연결 확인이며 GPIO 전압·접점·MD20A 출력이나 실제 바퀴 정지를 측정한 것은 아니다.

실행한 기존 검사는 `receipt.json`의 정확한7개다: 패널 대기 run 취소, 자식 STOP/EOF 전달, 재준비 없는 추가 pulse 거절, physical held 해제와 CLEAR/ARM 분리, held jog 거절, stale 패널 상태에서도 STOP 허용, host STOP의 대기 ARM/DRIVE 제거. **7 passed /0 skipped /exit0 (0.48s)**. 실제 ROS 노드/HTTP 종단 시연은 실행하지 않고 node의 두 STOP 분기를 읽었다.

기존 r449의 격리 pytest9.0.2를 C:/Python314/python.exe에서 재사용했다. 패키지 설치0, WSL/Jetson/ROS/펌웨어 빌드/센서/GPIO/모터 접근0이다. 관련 원본 hash 변경0, 실제 실행 자식/원격 PID 생성0이며 전경 Python이 정상 종료했다. 상세 명령과 원본 SHA는 `run_selected.py`, `receipt.json`, 원 결과는 `pytest.log`/`pytest.xml`에 있다.

조립 후에는 짧은 실제 jog 중 **물리 비상정지**, **패널 전체 STOP/수동 STOP**, **물리 스위치 해제만으로 자동 재시작하지 않음**을 실제 바퀴/출력으로 확인하면 된다. 이 실물 확인을 이번7개 코드/모의 검사로 대신하지 않는다.
