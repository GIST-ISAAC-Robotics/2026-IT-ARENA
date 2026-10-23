# PC SHM 크기의 제한 대조 — 기록기 없이 큰 Image 전달

같은 PC·소스·Image 크기·BEST_EFFORT depth 30으로 두 조건을 1회씩 비교했다. 각 조건은 1280×720 RGB/BGR(교대 행 padding 16 B), 10회·회차 wall 대기 0.2 s·종료 drain 1 s다. 실제 카메라 FPS를 재현한 시험은 아니다. 명시 XML + SYSTEM_DEFAULT discovery를 사용하되 두 프로세스의 **실제 builtin false·SHM 단독**과 크기를 읽기 전용 C++ inspector로 확인한 뒤 발행했다.

| 조건 | 수신 | 수신된 payload 해시 | 종료 |
|---|---:|---|---|
| SHM 기본 크기 선택(descriptor 0) | 4/10 | 모두 송신과 같음 | 자식 0 |
| SHM 16 MiB(descriptor 16777216) | 10/10 | 모두 송신과 같음 | 자식 0 |

검사 실행 자체는 exit 0이며 `diagnostic_completed=true`다. 기본 조건의 `all_sent_received=false`를 성공으로 바꾸지 않는다. 기록기가 없어도 누락이 재현됐으므로 기록기의 파일 쓰기만을 원인으로 볼 수 없고, 메모리 크기 대조가 추가 근거가 됐다. SHM 실제 점유/패킷 경로·누락 내부 원인을 완전히 계측한 결과는 아니다. 과거 system(UDP+SHM) 실패를 이번 SHM 단독과 동일 조건으로 합치지 않는다.

공통 원문/의존 소스는 각 조건의 `sources/`, 실제 설정은 `image_transport_*_active.json`, 송신/수신 목록은 별도 JSON, 고정 inspector 라이브러리/버전은 `../sensor_transport_pc_v1/`에 있다. 전역/운영 기본·QoS·Jetson/실물은 변경하지 않았다. 다음 5입력 기록 대조는 한 번의 명시 시험이며 자동 기본 채택/고속 처리량 인증이 아니다.
