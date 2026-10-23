# 출력 비활성 연결부 포함 ESP32-S3 검사 앱 링크

- [빌드 스크립트](../../../../scripts/build_actuation_smoke.sh), [로그](build.log), [exit 0](build_exit.txt). 기존 격리 ESP-IDF v6.1/Xtensa 15.2.0과 새 `source-v7` 사본을 사용했다. [소스 해시 차이](build_source_hash_diff.txt) 0 B.
- 코어 20·수신 연결부 12·송신 9·출력 비활성 벤치 15개의 공유 사례를 담은 앱이 **링크**됐다. 보드 실행은 하지 않았다.
- BIN **193,264 B**, ELF **3,854,788 B**, [정적 DIRAM](idf_size.txt) **131,039 / 341,760 B**. 벤치 검사용 endpoint 세 개와 배열 sink 등 정적 시험 저장 공간을 포함한다. 실제 단일 endpoint 펌웨어의 사용량이나 stack/heap·WCET 측정값이 아니다.
- `ESP_ROM_ELF_DIR` 미설정 경고 2회, [SDK `-u` 심볼 3개](undefined_symbols.txt)를 보존했다. 빌드 실패는 아니며 실물 실행으로 검증한 것도 아니다.
- `flash/monitor`·포트 조회·UART/모터 구동 없음. `hardware_executed=false`, `firmware_verified=false`, `ready_to_drive=false`.
- [바이너리 해시](binaries.sha256), `build_source_snapshot.tar.gz` 보존. 바이너리/맵·원시 대형 파일의 Git 제외 정책은 유지한다.
