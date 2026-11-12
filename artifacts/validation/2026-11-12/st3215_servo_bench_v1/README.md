# ST3215-HS 무부하 실물 시험 v1

2026-11-12, GPT 직접 실행. 단일 미장착·서보혼/조향 링크 없는 서보, GPIO17/18 UART→Adapter A, 사용자 보고 모터 전원 분리 조건이다.

- **통과 범위:** 점퍼 두 개 A 확인 뒤 ID1/1 Mbps·위치 모드,10초 읽기·작은 각도1회 및50초5회 왕복·복귀, 명령 공백1.5초 뒤 토크 OFF. 성공4실행 피드백1,592개/요청·응답1,645개, 체크섬·순번 공백·시간 초과·서보 상태 오류0. 사용자가5회 움직임/복귀 육안 확인.
- **초기 실패 보존:** 점퍼 미확인 당시 ID1×8속도 및 전체254 ID×8속도에서 응답0. A 위치로 맞춘 뒤 성공한 결과와 분리한다.
- **한계:** 조향 장착·부하/구속 전류·정밀 각도·최대율/순수 응답 지연·실제 버스 단선/MCU 리셋 중 토크 해제는 미검증이다. 전압/온도/위치는 서보 내부 보고값이다. 진단 수신 창30 ms를 서보 자체 지연으로 해석하지 않는다.
- **종료:** torque0·포트 소유자0·ACL 유지, 기존 IMU 앱의1 MB partition 전체 바이트 복구. IMU WHO0은 사용자 확인된 센서 분리 상태이며 IMU 재시험 통과가 아니다. 엔코더/모터 시험은 수행하지 않았다.

[기술 보고서](../../../../docs/sensors/ST3215_HS_HARDWARE_2026_11_12.md), [summary.json](summary.json), [audit.json](audit.json)에 형식·상태·원 실패·소스/바이너리/증거 해시를 남긴다. 원시 UART/전체 feedback/Flash는 Git 제외 `build/servo_hardware_20261112/`에 보존한다.

수신기와 독립적인 재감사:

```powershell
python scripts/audit_bus_servo_probe.py build/servo_hardware_20261112/results --output build/servo_hardware_20261112/re_audit.json
python -m unittest discover -s tests -p test_bus_servo_probe.py -v
```

실물 재시험은 [`firmware_servo_probe`](../../../../src/arena_vehicle_interface/firmware_servo_probe/README.md)의 격리 절차를 따른다. 기본 host mode는 read이며 motion과 watchdog은 각각 명시 선택한다. `--cycles 5`가50초 왕복 재현값이다. 운영 제어기에 이 앱을 연결하거나 승인 없이 모터 전원을 투입하는 재현 명령이 아니다.
