# ESP32-S3 검사 앱 v8 — PI 포함, 빌드만

ESP-IDF v6.1·Xtensa 15.2.0으로 기존 코어/수신 연결부/TX/출력 비활성 사례와 PI 12사례를 포함한 앱을 링크했다. **exit 0·빌드 전후 소스 해시 차이 0 B**다. PI 12사례는 별도 호스트 검사에서 실행했으며 대상 MCU에서는 실행하지 않았다.

- ELF 3,871,544 B, BIN 195,712 B.
- 정적 DIRAM 131,039 / 341,760 B. 여러 시험 객체/배열을 담은 검사 앱의 값이며 운영 메모리 요구량이 아니다.
- `ESP_ROM_ELF_DIR` 미정의 진단 두 건과 `nm -u`의 `__cxx_fatal_exception`, `start_app`, `start_app_other_cores`는 로그에 보존한다. 링크·이미지 생성 성공을 이 진단의 해결이나 보드 부팅 검증으로 쓰지 않는다.
- flash/monitor·장치 조회/업로드·PWM/DIR·센서·모터·Jetson 작업은 하지 않았다. `firmware_verified=false`, `ready_to_drive=false`.

명령: `bash scripts/build_actuation_smoke.sh artifacts/validation/2026-10-23/actuation_idf_smoke_v8 /home/jinhyeong/.cache/it-arena/idf-6.1-smoke/source-v8`.

[빌드 로그](build.log), [크기](idf_size.txt), [미해결 심볼](undefined_symbols.txt), [종료값](build_exit.txt), 소스/바이너리 해시와 스냅샷을 같은 폴더에 보관한다.
