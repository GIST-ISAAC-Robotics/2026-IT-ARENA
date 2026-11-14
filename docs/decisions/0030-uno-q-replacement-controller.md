# ADR 0030: 현장 UNO Q로 ESP32 제어기 대체

> 경기 종료 후 분류: **현장 미사용 대체안/준비 이력**. 대회용 최종 기준은 [ESP32 소스 보존본](../releases/FINAL_ESP32_2026_11_14.md)이다. 아래 내용은 당시 오프라인 준비 기록이며 실차 적용 결과가 아니다.

- 날짜: 2026-11-14
- 상태: 사용자 요청에 따른 선정·오프라인 구현. 실물 설치/통전 검증은 별도다.
- 선행: [ADR 0002](0002-jetson-esp32-actuation-boundary.md), [ADR 0023](0023-mcu-actuation-contract-reference.md), [ADR 0026](0026-motor-shaft-encoder-baseline.md).

## 결정과 이유

ESP32의 LED 소등·전원부 발열 이후 사용자에게 예비 ESP32가 없고 UNO Q는 현장에 있음을 확인했다. 기존 보드를 재사용하지 않고 **Jetson + UNO Q의 STM32U585**를 선택한다. 사용자 지시대로 물리 STOP 스위치는 포함하지 않으며, 나머지 구동·센서 기능과 소프트웨어 정지 계약을 유지한다.

Jetson은 기존 인지·판단과 USB 서보 통신을 맡는다. MCU는 TIM3 엔코더, TIM4 모터 PWM/DIR, SPI2 IMU, 명령/제어주기 만료를 맡는다. Jetson J12 UART와 UNO Q D0/D1 USART1을 1 Mbps로 직접 연결한다. 내부 MPU↔MCU 115200 baud Router로 IMU 전체 스트림을 전달하지 않는다. UNO Q 전원은 M01 CH6의 12 V를 VIN으로 공급하며, 운영 중 USB를 연결하지 않는다. 핀·전원 설계는 [UNO Q 회로](../hardware/schematics/uno_q_wiring_v1.html)에 둔다.

선정 배선은 엔코더 신호를 Jetson의 TXB0108 경로로 넣을 때 남는 TXS/TXB 구동 정합 문제를 피한다. USB 서보 경로는 기존 Adapter(A)를 B 점퍼 두 개로 설정한다. 서보의 출력 허가는 MCU가 직접 관측하지 못하므로 **boot/generation/MCU lease에 묶인 짧은 외부 서보 상태(ESV1)**를 추가한다. 오래된 USB 관측이나 호스트 수신 시각으로 유효기간을 갱신하지 않는다. 수락된 DRIVE와 서보 목표 ACK 순번이 맞아야 모터를 허용한다.

기존 portable IA 명령·ACK·lease와 `/drive`→독립 보호층→`/drive/safe` 경계는 유지한다. 실제 모터 방향·엔코더 부호·PI 게인은 새 보드에서 확인하기 전 임의 확정하지 않는다. 명시적인 벤치 단발과 설치 프로필 기반 주행을 구별하며 기본 상태는 미무장/PWM0이다.

## 구현 선택과 한계

직접 HAL 오류 처리와 비동기 UART를 위해 native Zephyr 앱을 사용한다. Arduino의 기존 MCU loader 위에 native Zephyr ELF를 그대로 올리는 방식은 호환되지 않으므로 MCU flash 백업·검증·복원을 갖춘 standalone 업로드를 준비한다. 이는 Linux eMMC OS 교체가 아니며, 현재 보드에 실행하지 않는다. stock Arduino 앱 업로드로 돌아갈 때는 MCU loader 복원이 필요하다.

물리 STOP 제외는 독립적인 전력 차단을 소프트웨어로 대체했다고 주장하는 결정이 아니다. 배선 오류·전원 강하·MCU reset 중 출력·동시 부하·실제 명령 만료는 실물 시험이 남는다. M01 입력 24–36 V 정격과 6S의 24 V 미만 구간 문제도 남는다. 최신 코드·빌드·설치 절차와 검증 결과는 [이관 문서](../hardware/UNO_Q_DEPLOYMENT_2026_11_14.md)에 기록한다.
