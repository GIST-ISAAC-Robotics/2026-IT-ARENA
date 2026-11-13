# ESP32-S3 · 엔코더 수동 진단 앱

모터 전원을 분리한 상태에서 PGE-213→CF-0108→GPIO4/5 입력을 관측한다. GPIO4=A, GPIO5=B는 입력/내부 pull 없음이다. GPIO15 PWM·GPIO6 DIR을 앱 시작 시 LOW로 초기화하지만 ROM/리셋 구간 파형을 보장하지 않으므로 모터 전원은 분리한다. UART1/서보 명령은 구현하지 않는다.

PCNT 두 채널을 x4 quadrature로 구성하고1,000 ns glitch filter·±30,000 watch point/누산을 사용한다. 독립 GPIO 양쪽 에지 ISR은 A/B 상태, 각 채널 상승/하강 횟수와 software quadrature count를 기록한다. 상태 부호는 A가 상위 비트, B가 하위 비트다. `00→10→11→01→00`을+4로 정의하며 차량 전진/CW 부호는 아직 채택하지 않는다. 두 비트가 동시에 달라진 관측은 invalid로 기록하고 두 카운트를 임의 복원하지 않는다. ISR은255개 유효 용량의 유한 ring에 기록하고 포화 시 drop을 센다.

통신은 검증된 UART0 GPIO43/44·921600 8N1이다. 프레임은 `8자리 hex CRC32 + 공백 + JSON + LF`이며 CRC는 JSON 원문 바이트에 적용한다. `info/status/edge`는 같은 boot/sequence를 공유한다. MCU timestamp는 monotonic us다. v1의 `ab`는 ISR이 유지한 상태이고 v2에는 매 상태 프레임에서 GPIO를 별도 폴링한 `raw_ab`를 추가했다. PCNT와 ISR 상태는 서로 다른 순간에 읽으므로 움직이는 도중 두 count를 무조건 같은 값으로 요구하지 않고 정지 후 대조한다.

- `?`: 구성·상태 조회.
- `S`: 약20 ms 간격 상태 송신 시작. 누적 카운터는 지우지 않음.
- `X`:1 Hz 상태 송신으로 복귀. 에지 관측은 유지되며 모터/서보 동작은 없음.
- 활성 송신은600초 뒤 자동 대기한다. 재부팅 후 자동 활성화하지 않는다.

[`probe_esp32_encoder.py`](../../../scripts/probe_esp32_encoder.py)는 원문/전체 프레임을 비공개 폴더에 보존한다. 실행 중 해당 폴더의 `phase.json`으로 사람이 지시한 수동 회전 구간을 표시하고 `STOP` 파일로 종료한다. phase는 호스트 수신 시각 기준 라벨이며 물리 에지 취득 시각과 구분한다. 기록 시범:

```text
python probe_esp32_encoder.py --port /dev/ttyACM0 --output <새 비공개 시험 폴더> --seconds 300
```

7 PPR/28 counts per motor revolution은 제조사 표기에 따른 비교 후보다. 손 회전수와 실제 count를 검증하기 전 자동 환산값으로 채택하지 않는다. GPIO digital 상태·PCNT 계수만으로 HIGH/LOW 전압·상승/하강 품질·고속 펄스 보존·실제 취득시각 계약을 검증했다고 하지 않는다. 최종 앱 복구/종료도 별도로 기록한다.

[PCNT 공식 문서](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html), [현행 배선](../../../docs/hardware/ESP32_GPIO_WIRING_2026_11_11.md). 빌드는 기존 ESP-IDF v6.1에서 `idf.py build`를 사용한다.
