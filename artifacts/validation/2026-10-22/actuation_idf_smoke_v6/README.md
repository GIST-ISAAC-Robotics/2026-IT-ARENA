# ARM 관문 포함 ESP32-S3 검사 앱 링크

- 기존 격리 ESP-IDF v6.1 / Xtensa `esp-15.2.0_20251204`와 새 `source-v6` 작업 폴더를 사용했다. 실행 절차는 [빌드 스크립트](../../../../scripts/build_actuation_smoke.sh)와 [빌드 로그](build.log)에 남았다.
- [exit 0](build_exit.txt), [소스 해시 차이](build_source_hash_diff.txt) 0 B. 코어 20·연결부 12·송신 9 합성 사례를 담은 앱의 ELF/BIN **링크까지만** 수행했다.
- BIN **188,256 B**, ELF **3,789,816 B**, IDF image **188,135 B**. [메모리 보고](idf_size.txt)의 정적 DIRAM은 **83,807 / 341,760 B**다. 동적 stack/heap·실제 실행시간/WCET는 이 숫자로 확인되지 않는다.
- `ESP_ROM_ELF_DIR` 미설정 경고 2회와 [미해결 심볼 목록](undefined_symbols.txt)의 SDK `-u` 항목 3개를 보존했다. 링크 실패는 아니며 보드 실행으로 확인한 것도 아니다.
- 빌드 도구가 출력한 `flash` 안내는 실행하지 않았다. 장치 연결·업로드·실제 UART·모터 출력 없음. `hardware_executed=false`, `firmware_verified=false`, `ready_to_drive=false`.
- [바이너리 해시](binaries.sha256), `build_source_snapshot.tar.gz`, [ELF 헤더](elf_header.txt)를 보존했다. BIN/ELF/map 등 로컬 대형 빌드 산출물은 이 폴더의 Git 제외 규칙을 유지한다.
- [ARM 관문 및 한계](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#20-1022-후속--arm-출력-준비-관문).
