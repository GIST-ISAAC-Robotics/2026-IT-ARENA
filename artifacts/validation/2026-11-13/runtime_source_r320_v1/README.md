# 출력 비활성 MCU 결합 소스의 공통 진입점 r320

r315 firmware8파일, r283/r287 portable2파일, r319 조향표1파일을 공통 src에 바이트 동일하게 복사했다. 원 시험 소스/실패/증거는 보존했다. 기존 공통 구현의 바이트는 바꾸지 않았으며 새 기본 실행/flash·모터/서보 출력 허가가 아니다.

- 공통 진입점: `src/arena_vehicle_interface/firmware_runtime_preview`.
- 새 빌드 명령: `scripts/build_runtime_preview.sh`. ST commit 고정, 소스 전후 hash/snapshot, 이미지/크기/종료 기록이며 flash는 하지 않는다. 전후 소스 변경이 있으면 실패 종료한다.
- GPIO8 STOP은 r315 코드 그대로다. PWM15/DIR6은0, output_ready=false, PCNT는 원시 진단뿐이며 유효 정지 속도로 합성하지 않는다.
- 서보는 mode/position 읽기만 연결한다. 조향 보간표는 값이 없고 펌웨어 출력 경로에 연결하지 않았다.

ESP-IDF6.1/ESP32-S3 전체 빌드 exit0, 소스 변경0. 이는 공통 경로의 빌드 검증이며 새 보드 실행/실제 STOP/기구 조향/제동 성능 증명이 아니다. 이전 r305 비구동 실제 대표와 r315 합성 STOP 검사를 소급 변경하지 않는다.

다음은 조립 시 실제 count/rev·부호·중립/조향·출력/STOP 확인이다. 그 대기 동안 현재 실행 지연을 같은 저장 입력으로 개선한다. 명목10Hz와 실제 처리율·명령 나이/만료는 별도로 확인한다.
