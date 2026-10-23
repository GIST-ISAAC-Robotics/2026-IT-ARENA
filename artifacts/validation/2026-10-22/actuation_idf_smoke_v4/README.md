# ESP32-S3 검사 앱 v4 — 수신 연결부 포함

[v3](../actuation_idf_smoke_v3/README.md) SDK/컴파일러를 재사용해 수신 연결부와 10개 합성 검사를 추가한 앱을 새 폴더 `build-v4`에서 빌드했다. `build_exit.txt=0`, 소스 해시 차이 없음. v3의 원시 증거/바이너리를 덮어쓰지 않았다.

- BIN 177,184 B, IDF reported image 177,067 B, ELF 3,673,856 B(디버그 정보 포함).
- IDF DIRAM 정적 배치 55,919/341,760 B. IRAM 분류 16,384 B 사용은 해당 링크 구간의 보고값이며 전체 RAM 소진이나 task stack 사용량이 아니다. 실행 스택/힙·마감은 미측정이다.
- `build.log`, `idf_size.txt`, ELF header·심볼·`binaries.sha256`, `build_source_snapshot.tar.gz`, before/after hash에 근거를 보존한다. 큰 ELF/BIN/map/compile DB·심볼은 로컬 제외 정책을 적용했다.
- [호스트 v2 검사](../actuation_ingress_v2/README.md) 52개가 통과했다. 앱 공유 사례는 코어 20개 + 연결부 10개이며 **목표 MCU에서는 실행하지 않았다**.

실제 GPIO/PWM/DIR/엔코더/센서/서보·UART 명령 수신은 없다. flash/monitor·장치 접근을 하지 않았다. SDK 검사 빌드는 납품 보드 메모리/핀 설정 확정이 아니며, `firmware_verified=false`, `ready_to_drive=false`다.
