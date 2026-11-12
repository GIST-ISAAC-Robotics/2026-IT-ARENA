# r234 — 연결된 MCU의 수동 식별 및 Jetson 상태

11/12 약13시의 **3초 수동 수신**이다. UART 명령 송신0바이트, DTR/RTS false, 배타 포트, 종료 후 닫힘을 확인했다. 포트 재개방 자체가 ESP32 리셋을 유발할 수 있으며 실제 부팅 메시지가 있었다. 모터·서보 동작, 펌웨어 교체, 권한/전원/클럭 변경은 하지 않았다.

현재 수신 앱은 **`motor_probe_v1`**이다. CRC를 통과한 info1·status3·motor3 프레임 중 motor는 모두 active=false·armed=false였다. IMU 스트림은 관측되지 않았다. 이는 IMU의 물리 연결 유무를 확인한 결과도, 통합 운영 펌웨어가 준비됐다는 결과도 아니다. GPIO4/5·quadrature_x4·필터1000ns·PWM/DIR0은 앱의 보고값이다. 현재 모터를 구동하지 않았으므로 엔코더0은 기능 시험 판정에 쓰지 않는다.

[첫 요약](summary.json)은 부팅 접두부가 붙은 info를 제외해6프레임으로 집계했다. 원본을 수정하지 않고 기존 모터 벤치 해석기로 [보존 바이트 감사](audit_saved.py)를 추가하여 [7프레임·앱 식별](independent_audit.json)을 확인했다. 원시 바이트는 Git 제외 `build/r234_uart_private.bin`과 Jetson의 이 시험 폴더에 보존한다. SHA는 두 결과에 기록했다.

읽기 전용 [장치 상태](device_state.log)는 MAXN_SUPER(mode2), CPU6개·schedutil, idle 약45–46°C·GPU0%·RAM 약2/7.6GB였다. sysfs와 idle 표본의 CPU 주파수가 달라 동적 주파수 조절 상태임을 확인했다. 부하 시 열/전력 제한의 부재를 증명하지 않으며 설정을 변경하지 않았다.

다음 실물 통합은 현재 앱 소유권과 엔코더 신호 미검출을 먼저 해소해야 한다. 이 식별 결과를 근거로 임의 펌웨어 교체나 0속도 피드백을 만들어 주행을 허용하지 않는다.
