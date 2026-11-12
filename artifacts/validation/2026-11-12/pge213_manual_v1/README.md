# PGE-213 첫 수동 시험 — 입력 미검출

2026-11-12, GPT가 직접 실행했다. 모터 전원은 분리했고 사용자가 모터 축을 약1회전 수동 회전했다. 정확한 회전수/각도 표준으로 보지 않는다.

**결과는 엔코더 검증 미통과다.** 상태9,965개를 받았으며 CRC/JSON 오류·순번 공백·MCU 재부팅·ring drop은 없지만 PCNT/ISR 검출 에지와 count 변화가 모두0이었다. 최초 상태00은 읽었으나 v1의 후속 `ab`는 ISR 저장값이며 별도 GPIO 폴링이 아니다. 실제 신호 전압이 항상0 V였다는 주장이나0 PPR 환산은 하지 않는다.

- [기술 보고서·현재 재개점](../../../../docs/sensors/PGE213_ENCODER_HARDWARE_2026_11_12.md)
- [첫 수신 집계·사용자 측정](first_attempt_summary.json)
- [원문 CRC/JSONL 독립 감사](first_attempt_audit.json)

원문 UART·전체 JSONL·Flash·앱/SDK/이전 소스는 Git 제외 `build/encoder_hardware_20261112/`에 보존한다. 원시 묶음 해시를 집계와 보고서에 남긴다. 첫 앱v1 실패는 유지하며 매 상태마다 GPIO를 직접 읽는 v2는 빌드만 완료했고 아직 업로드하지 않았다. 실제 보드 상태/후속 측정은 기술 보고서의 현재 재개점을 따른다.

```powershell
python scripts/audit_encoder_probe.py build/encoder_hardware_20261112/first_attempt/manual_capture --output build/encoder_hardware_20261112/re_audit.json
python -m unittest discover -s tests -p test_encoder_probe.py -v
```

전원/채널 대응·엔코더 자석/축 결합과 실제 펄스를 확인한 뒤 방향·분해능·최대율을 검증한다. 첫 UART 기록이 정상이라는 이유로 센서 입력 자체를 통과로 바꾸지 않는다.
