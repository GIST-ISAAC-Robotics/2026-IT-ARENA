# ESP32-S3 peripheral 결합 preview

**구동 가능한 운용 펌웨어가 아닙니다.** r305 실제 비구동 결합과 r315 GPIO8 STOP 준비의 소스를 재현 가능한 위치로 옮겼습니다. 원 시험 소스/결과는 그대로 보존하며, 이 디렉터리로 옮긴 뒤의 빌드와 실제 보드 실행을 구분합니다.

## 포함하는 것

- UART0 단일 소유자로 IMU·계약 offer/ACK·원시 상태 송신. 서보 UART1은 mode/position 읽기만 수행합니다.
- GPIO4/5 PCNT 원시 count와 고정 위상5ms 제어 호출. PCNT 원시0을 유효한 속도 피드백으로 꾸미지 않습니다.
- PWM15/DIR6 초기0, `output_ready=false` 고정. `ARM`/`DRIVE`로 모터나 서보를 구동할 수 없습니다. 출력용 토크 ON/goal 송신은 연결하지 않았습니다.
- GPIO8 내부 풀업·O/개방 STOP 입력, I/도통 복귀 뒤 래치 유지. GPIO를 읽는 소프트웨어이며 독립 전력 차단 회로가 아닙니다.
- 모터축28카운트·감속비 `2.5×81/17`·65mm 구성. 실제 방향/count per rev·장착 조향은 아직 교정 전입니다.

## 빌드

WSL Ubuntu에서 설치된 ESP-IDF6.1/ESP32-S3 도구체인을 사용하는 명시 빌드입니다. ST 드라이버는 스크립트에 기록한 commit과 깨끗한 작업 디렉터리를 확인합니다. 새 빌드 디렉터리는 공백 없는 절대경로를 사용합니다.

```bash
bash scripts/build_runtime_preview.sh \
  /home/jinhyeong/.cache/it-arena/runtime-preview-새이름 \
  /home/jinhyeong/.cache/it-arena/lsm6dsox-pid_20261111 \
  /절대경로/새결과폴더
```

소스 snapshot·전후 hash·binary hash·크기·종료 코드를 남기며 **flash는 하지 않습니다**. 소스가 빌드 중 달라지면 실패로 종료합니다. 보드 설치 시 현재 앱과 장치 소유 상태 확인/백업/원본 복원을 별도로 수행해야 합니다.

## 운용 연결 전에 남은 일

1. 실장 엔코더 부호·실제 count per rev와 배선/관측을 확인하고 PCNT를 유효 피드백으로 연결합니다.
2. 차량 조향각↔서보 위치 실측표를 받습니다. [`steering_mapping.hpp`](../portable/steering_mapping.hpp)는 비대칭 보간 준비물일 뿐이며, 실제 측정표/자동 중립/출력 연결은 없습니다.
3. 모터 PI의 음수 effort는 역회전 명령이 아닙니다. MD20A 전기 제동 매핑·실측 게인·속도 상한을 확인한 별도 구현이 필요합니다.
4. GPIO8/명시 STOP·통신 상실·출력0·조향 응답을 실제 하중에서 확인하고 운용 준비 상태를 연결합니다. 이 preview의 정상 수신만으로 구동 준비를 허가하지 않습니다.

상세 이력은 [실행·센서 통합 보고서](../../../docs/reports/RUNTIME_SENSOR_INTEGRATION_2026_11_12.md), 확정 회로는 [GPIO 배선 기준](../../../docs/hardware/ESP32_GPIO_WIRING_2026_11_11.md)을 따릅니다.
