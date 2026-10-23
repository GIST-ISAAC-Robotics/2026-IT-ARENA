# ESP32-S3 구동 계약의 검사 전용 빌드

`portable/` 코어를 실제 Xtensa/ESP-IDF 링크 과정까지 확인하는 **검사 앱**이다. 구동 펌웨어가 아니며 UART 명령 수신·모터 PWM/DIR·서보·엔코더·IMU 드라이버는 없다. 자동 업로드 스크립트를 제공하지 않는다. 코드가 빌드된다는 사실과 보드 실행·실물 정지 검증은 별개다.

## 구성

- `main/smoke_cases.hpp`: 호스트와 대상 빌드가 공유하는 합성 검사. 단일 모터축 **28카운트·15:1·65 mm**, 부팅 구동 금지·ARM/DRIVE·분할 STOP·조향 유지·CLEAR→별도 ARM·제어 tick 만료와 JSON/숫자 표현을 검사한다.
- `main/ingress_cases.hpp`: 연결부를 통과하는 ARM/DRIVE·단절 STOP·옛 세션 거절·명시 재허가·링크 고장과 준비값 생략/재허가 관문 12개. 기존 코어 20개와 구별해 출력한다. 큰 연결부 객체도 정적 저장 공간을 쓴다. 준비값 true는 합성 검사 인수이며 실물 출력 감시가 아니다.
- `main/tx_cases.hpp`: 송신 정책 후보 9개. 최악 payload 상한, 부팅 알림/offer 임대 발행 시각, 요청 ID escape·astral 바이트, 부분 쓰기 중 STOP에서 시작한 프레임 유지, ack 큐 한도/STOP 축출, 세션 경계 꼬리·ack 폐기, 막힌 sink에서 임대 추가 발행 없음, 시계 역행·sink 계약 위반·escape가 필요한 boot의 영구 침묵. 실제 UART가 아니라 고정 배열 sink에만 쓴다.
- `main/bench_cases.hpp`: 출력 비활성 `ActuationBenchEndpoint` 15사례. ARM/DRIVE 거절·STOP·피드백 없는 CLEAR 거절·링크/시계/sink 고장을 검사한다. 세 개의 endpoint와 배열 sink는 검사 앱의 정적 저장 공간이며 운영 펌웨어 메모리 사용량이 아니다.
- `main/pi_cases.hpp`: 단일 모터 속도 PI 12사례. 정상 계산·포화·비허가/0목표 초기화·잘못된 dt/설정·비유한 입력/중간값을 확인한다. 정규화 계산만 하며 PWM/DIR로 연결하지 않는다.
- `main/motor_cycle_cases.hpp`: 수신기와 PI의 출력 없는 결합 25사례. 정지·만료·준비 상실·허가 세대·피드백 환산·정상 감속을 합성 시각으로 대조한다. 준비값 true는 실제 HAL의 정상 신호가 아니며 이 후보를 구동 출력에 연결하지 않는다.
- 출력 비활성 UART 주기 어댑터의 공유 사례·SDK 연결은 별도 [`firmware_bench`](../firmware_bench/README.md)에 둔다. 이 앱의 `Report`·모터축 구성·송신 시험값 헤더를 재사용하며 이 앱의 사례 수는 바꾸지 않는다.
- `main/main.cpp`: 합성 검사를 한 번 호출하고 검사 수/실패 마스크·소요 시간·남은 스택의 진단을 출력한 뒤 반환한다. 성공해도 항상 `firmware_verified=false`, `ready_to_drive=false`를 표시한다. 시간·스택 값은 실제 실행 전에는 없다.
- `sdkconfig.defaults`: ESP32-S3 검사 대상, 8192 B main stack, 예외/RTTI/PSRAM 비사용. 실제 제어 task 크기·실물 flash/PSRAM 설정을 확정한 값이 아니다.
- 큰 parser/명령 scratch는 전역 저장 공간을 사용한다. 이것만으로 전체 라이브러리 무할당·마감 준수를 보장하지 않는다.
- 고정 boot ID `smoke-only`와 합성 엔코더/시각을 실제 차량 세션에 복사하지 않는다.

## 빌드·실행 범위

ESP-IDF **v6.1**의 고정 소스와 권장 `esp-15.2.0_20251204` 컴파일러를 이번 호환성 확인에 사용한다. 최종 MCU 개발 프레임워크를 팀 합의 없이 확정하는 뜻이 아니다. SDK/툴체인은 저장소와 별도 캐시에 두며 시스템 Python·ROS를 바꾸지 않는다.

이 저장소의 Windows 경로에는 공백이 있다. ESP-IDF component 경로 제한 때문에 빌드는 공백 없는 새 작업 폴더에 `portable/`와 이 폴더를 형제 디렉터리로 복사한 뒤 수행하고, 복사 전후 해시·실제 명령·실패 로그를 남긴다. `idf.py build`만 허용하며 `flash`, `monitor`, 장치 조회/연결은 이번 준비 범위에 없다.

호스트에서는 [`actuation_smoke_host.cpp`](../../../scripts/actuation_smoke_host.cpp)를 두 include 경로(`portable`, `firmware_smoke/main`)와 C++17로 빌드한다. ASan/UBSan의 stderr도 확인한다. 호스트 통과는 Xtensa에서 같은 수치/동작이 나온다는 증명이 아니다.

실제 v6.1/Xtensa ELF·BIN 빌드는 [v3](../../../artifacts/validation/2026-10-22/actuation_idf_smoke_v3/README.md), 연결부를 포함한 [v4](../../../artifacts/validation/2026-10-22/actuation_idf_smoke_v4/README.md), 송신 정책 사례를 더한 [v5](../../../artifacts/validation/2026-10-22/actuation_idf_smoke_v5/README.md), ARM 준비 관문을 더한 [v6 증거](../../../artifacts/validation/2026-10-22/actuation_idf_smoke_v6/README.md)에 보존했다. 기본 2 MB flash 구성은 검사값으로, 납품 N16R8 전체 메모리 구성의 확정값이 아니다. 상세 결과와 다음 이관 조건은 [구동 계약 §16](../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#16-1022-후속--esp32-s3-목표-컴파일과-검사-앱-링크)에 기록한다. 앱 결과가 좋아도 실차 전기 STOP·물리 정지 검증 전에는 차량 구동을 허가하지 않는다.
