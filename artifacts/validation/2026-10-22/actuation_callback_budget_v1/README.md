# 실제 콜백의 offer 발생량 감사

실제 `VirtualMCU.control_tick/on_tx/send_offer`, `ActuationHost.on_drive/on_rx`를 **ROS 노드 없이** 합성 시간·즉시 배송 fixture에서 호출했다. 코드를 읽어 50 Hz로 추정한 이전 예산에 명령 수신 후 offer가 빠져 있었음을 확인했다. 이전 [예산 v1](../../2026-10-21/actuation_link_budget_v1/README.md)은 당시의 단순 50/50 Hz 산술 결과로 보존한다.

1초 관측 구간, tick 200 Hz·모터축 피드백 100 Hz·명령 50 Hz, 식별자 32 ASCII자, UART 8N1 가정이다. 출력 준비는 참으로 대체했으며 `/actuation/status`는 wire로 옮기지 않았다. 장치·DDS·USB·직렬 큐의 측정이 아니다.

| 입력 | host→MCU | MCU→host offer / ACK | MCU→host bytes/s | 460800 부하 | 921600 부하 |
|---|---:|---:|---:|---:|---:|
| 무명령 대기 | 0 | 50 / 0 | 30,445 | 0.661 | 0.330 |
| DRIVE 50 Hz | 50 | 100 / 0 | 65,818 | **1.428** | 0.714 |
| 이전 wire 반복 배송 | 50 | 100 / 0 | 64,442 | **1.398** | 0.699 |
| 매 DRIVE에 request 부여 | 50 | 100 / 50 | 70,818 | **1.537** | 0.768 |

반복 배송은 정상 host의 정책이 아니라 명시한 고장 입력이다. 49개를 거절하고 `command_expired/FAULT_LATCHED`를 유지했으나 거절 콜백도 offer를 생성했다. 정상 DRIVE는 ACTIVE를 유지했다. 명령마다 생성하는 `request`는 추가 ACK 부하를 확인한 사례이며 현행 DRIVE의 빈 request와 구별한다.

`checks_exit.txt=0`, 관련 **20 pytest 통과**, 소스 해시 불변. [보고서·원시 합성 콜백 표](audit/report.json), `checks.log`, 소스 스냅샷/해시에 근거를 남겼다. 부하 1 이상은 이 표본의 순수 직렬화조차 지속 처리할 수 없다는 뜻이다. 1 미만도 CPU/USB/큐/실제 baud 지원·마감 보장이 아니다. runtime/baud/timeout/보호층은 바꾸지 않았다.

후속은 주기 offer와 사건 알림/ACK·상세 진단의 역할을 분리하고 고정 크기 송신/병합 정책 후보를 설계하는 것이다. 늦은 명령을 새것으로 재포장하거나 보호 시간 한도를 늘려 통과시키지 않는다. 사용자의 마무리 요청으로 추가 구현은 하지 않았다.
