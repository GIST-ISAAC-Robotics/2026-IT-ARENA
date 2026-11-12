# MD20A · 모터 단독 저출력 시험

2026-11-12, GPT가 직접 수행했다. **약5%·500 ms 반복5회의 실제 회전과 PWM0 구간 정지를 사용자가 확인했다. 실제 구동 중 엔코더 입력 변화는0으로 실패가 남는다.**

- 첫250 ms 구동은 사용자가 보지 못했으므로 육안 확인에 포함하지 않는다. 후속500 ms×5회는 약4초 시작 간격으로 반복했다.
- 첫 사전 검사는 모터 전원을 분리하고 수행했다. 허가/범위/만료/CRC/재전송 거절·타이머 종료·명시 STOP을 확인했으며 실제 회전으로 세지 않는다.
- 전압·전류는 미측정이다. 사용자 요청으로 전압 측정을 생략했고, 회전수/감속 시간/제동거리/유지 토크를 정량 검증하지 않았다.
- 종료: 사용자 CH1 분리, PWM0·허가 해제, 포트 close·장치 소유자0, 기존 장치 권한 유지. 보드에는 motor_probe_v1 대기 상태가 남는다.

[상세 보고서](../../../../docs/sensors/MD20A_MOTOR_HARDWARE_2026_11_12.md) · [범위/사용자 관찰 집계](summary.json) · [독립 원문 감사](audit.json).

raw UART·전체 JSONL·Flash·이전 호스트와 첫 실패는 Git 제외 `build/motor_hardware_20261112/`에 보존한다. 원시 묶음 SHA-256은 `0363a50ff11c5a4d08726f95682a10fae7e5ecbd6a54d7dda129551f2b8a8a70`다. 첫 호스트의 부팅 접두부 수신 실패는 후속 성공으로 덮어쓰지 않았다.

독립 감사는48,816프레임 원문/JSONL 일치·CRC/순번 오류0·boot1을 확인했다. 구동 전 대기 중 A에7 µs 간격의 상승/하강 한 쌍·순계수0이 있었고 원인은 미확인이다. 실제 구동6회 구간의 에지0과 구분한다.

```powershell
python -m unittest discover -s tests -p test_motor_probe.py -v
python scripts/audit_motor_probe.py build/motor_hardware_20261112/captured/session_v2 --output build/motor_hardware_20261112/audit.json
```
