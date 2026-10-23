# ESP32-S3 검사 앱 v9 — 제어 주기 결합 최종본 포함, 빌드만

ESP-IDF v6.1·Xtensa 15.2.0으로 기존 코어/수신 연결부/TX/출력 비활성/PI 사례와 제어 주기 25사례를 포함한 앱을 링크했다. **exit 0·빌드 전후 소스 해시 차이 0 B**다. 빌드 입력 19개 파일의 해시는 [전체 PC 회귀](../motor_cycle_final_v1/README.md)의 실행 전 해시와 같다. 사례들은 호스트에서만 실행했으며 대상 MCU에서는 실행하지 않았다.

- ELF 3,912,800 B, BIN 202,064 B.
- 정적 DIRAM 131,039 / 341,760 B(v8과 같음). 여러 시험 객체/배열을 담은 검사 앱의 값이며 운영 메모리 요구량이 아니다.
- `-fstack-usage` 함수별 정적 프레임: [빌드 캐시에서 복사한 `.su`](main_cpp_stack_usage.su) 기준 최대 **3,952 B**(`arena_smoke::run`), 다음 2,528 B(`motor_cycle_case`)다. `app_main`·`run_bench`는 각각 304 B다. 앞선 인계의 “최대 304 B”는 이 파일과 맞지 않아 정정한다. 함수 프레임의 합·SDK 내부·인터럽트 사용량을 포함하지 않으므로 어느 값도 실제 task stack 사용량이 아니다.
- `ESP_ROM_ELF_DIR` 미정의 진단 두 건과 `nm -u`의 `__cxx_fatal_exception`, `start_app`, `start_app_other_cores`는 로그에 보존한다. 링크·이미지 생성 성공을 이 진단의 해결이나 보드 부팅 검증으로 쓰지 않는다.
- flash/monitor·장치 조회/업로드·PWM/DIR·센서·모터·Jetson 작업은 하지 않았다. `firmware_verified=false`, `ready_to_drive=false`.

명령: `bash scripts/build_actuation_smoke.sh artifacts/validation/2026-10-23/actuation_idf_smoke_v9 /home/jinhyeong/.cache/it-arena/idf-6.1-smoke/source-v9`.

[빌드 로그](build.log), [크기](idf_size.txt), [미해결 심볼](undefined_symbols.txt), [종료값](build_exit.txt), 소스/바이너리 해시와 스냅샷을 같은 폴더에 보관한다. `.su` 사본은 README 작성 때 같은 빌드 캐시에서 복사했으며 두 파일의 SHA-256 `4304f6a1…40d7dc`이 같다.
