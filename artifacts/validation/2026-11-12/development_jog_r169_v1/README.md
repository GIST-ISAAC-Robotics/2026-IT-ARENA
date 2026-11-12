# 개발 전용 짧은 조작 도구

2026-11-12. **Jetson 접근 없이 노트북에서 구현·검증했다.** 차량을 실제로 움직이지 않았다.

`python3 scripts/development_jog.py forward --speed .15 --seconds .5 --speed-limit .2`는 계획만 출력한다. `--send`를 명시해야 ROS 명령을 보낸다. 방향은 `forward/left/right`, 전진 속도는 명시 상한 이하(상한0.05–0.3 m/s), 길이는 벽시계0.05–2초, 좌/우 조향은±0.2rad다. 후진/WASD/브라우저 버튼은 아직 없다.

실제 발행에는 자율 제어기 OFF, `/drive`의 단독 발행자, `/drive/safe`의 `lidar_safety`와 구독 연결, STOP 서비스, 신선한 ARMED/0목표 MCU 상태가 필요하다. 도구가 ARM/CLEAR/RESET을 하지 않는다. 기존 `/drive`→독립 보호→`/drive/safe` 경계를 사용한다. 끝나거나 MCU STOP/세대/소유자 변경·상태 만료·다른 /drive 발행자가 생기면0명령과 명시 STOP을 보낸다. 처음부터 자율 발행자가 있어 거절한 경우에는 그 제어에0명령/STOP을 끼워 넣지 않는다. STOP 성공 응답과 새 ESTOP/0목표 상태를 기록하며, 실제 바퀴 정지 확인으로 확대하지 않는다.

자율 launch/제어 루프에는 import·타이머·키 입력·네트워크 대기를 추가하지 않았다. 개발 상한은 이 도구의 명시 `--speed-limit`이고, r162 자율 시험 상한과는 각각의 독립 실행 모드에서 설정한다. 두 값을 공유하는 콘솔 모드 전환은 아직 구현하지 않았다. 경기 중 원격 수동운전 허용으로 해석하지 않는다.

## 검증

- 순수/CLI13검사: 방향·상한·시간·입력 오류·초기 ARM·MCU STOP/epoch/만료·기본 미리보기 무발행.
- 실제 ROS와 기존 `LidarSafety`, **가짜 MCU 상태/STOP endpoint**의6사례 통과: 전진/좌/우 출력이 보호층을 통과, 가까운 앞 장애물에서 보호 출력0, 외부STOP에서 자동 재ARM 없음, 다른 발행자가 있으면 명령/STOP0건.
- `ros_result.json`과 각 사례 result/log가 기준이다. 실제 가상MCU 전송 전체·Gazebo 차량 움직임·실제 MCU/모터 방향·제동 검증은 아니다.
- 실행하면 `build/development_jog/<시각>/request.json,result.json`에 명령과 종료 상태를 남긴다. 수동 도구가 기본 자율 실행에 비용을 더하지 않도록 독립 스크립트로 유지한다.

다음은 실제 MCU 공통 상태/STOP 인터페이스가 연결된 후 낮은 속도로 방향/중립/종료를 확인하고 필요하면 기존 콘솔에 버튼을 붙이는 것이다. 현재 사용자 모터 시험 중인 Jetson에는 도구를 전송하거나 실행하지 않았다.
