# ESP32-S3 검사 앱 v5 — 송신 정책 사례 포함

v3/v4와 같은 격리 SDK(ESP-IDF v6.1 `fff9895c…`, Xtensa esp-15.2.0_20251204)로 `scripts/build_actuation_smoke.sh`를 새 공백 없는 작업 폴더 `build-v5`에서 실행했다. `build_exit.txt=0`, `build_wrapper_exit=0`, 소스 해시 차이 0 B. v4의 원시 증거/바이너리를 덮어쓰지 않았다.

- 앱 공유 사례: 코어 20 + 연결부 10 + **송신 정책 9**(`main/tx_cases.hpp`). 송신은 고정 배열 sink에만 쓰며 UART에 쓰지 않는다.
- BIN 187,984 B(IDF 보고 image 187,875 B), DIRAM 정적 배치 83,807/341,760 B. 정적 `tx_scratch` 27,888 B(수신기 3개·송신 객체 4개·sink 4 KiB, 시험 구성). 실행 stack/heap·마감은 미측정이다.
- ELF header EXEC/Xtensa, `nm -u` 3줄(이전 판과 같은 SDK `-u` 이름), 심볼에 `ActuationTxScheduler` 함수 존재. `build.log`, `idf_size.txt`, `binaries.sha256`, `build_source_snapshot.tar.gz`, before/after hash를 보존했다. 큰 ELF/BIN/map/compile DB·심볼은 로컬 제외(`.gitignore`).
- 호스트 공유 사례 9개는 [송신 정책 v2](../actuation_tx_policy_v2/README.md)의 `test_actuation_smoke.py`에서 sanitizer/`-Os`로 통과했다. **목표 MCU에서 실행하지 않았다.**

flash/monitor·장치 접근·시리얼 포트·모터·Jetson·전원 변경 없음. 빌드 성공은 보드 실행·전기 STOP 검증이 아니며 `firmware_verified=false`, `ready_to_drive=false`다.
