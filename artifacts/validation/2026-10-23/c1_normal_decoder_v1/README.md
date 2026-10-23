# normal 응답 원문 handler 제한 재현

실행: `python3 scripts/audit_c1_normal_decoder.py --output artifacts/validation/2026-10-23/c1_normal_decoder_v1` (WSL Ubuntu 24.04, C++17, UBSan). 기존 output은 덮어쓰지 않는다.

원문: [고정 SDK 자료](../c1_normal_decoder_sources_v1/README.md). `native/`는 빌드 당시 원문·시험 shim·probe·컴파일 명령/출력, `team_source/`는 당시 실행기와 probe 보존본이다. `*.input.json`은 **합성 응답 payload/가짜 host 시계**이고 `*.output.json`은 해당 원문 callback 출력이다. 실제 센서 수신 기록이 아니다. `completion_chunk_offset`은 점이 완성된 입력 청크의 시작 위치이지 해당 점의 첫 바이트 위치가 아니다. split된 점은 이전 청크까지 함께 읽어야 한다.

13조건·12판정 통과: [report.json](report.json). 이것은 결함/한계가 의도한 대로 재현됐다는 의미도 포함하며 운영 적합성 통과가 아니다. 같은 50개 점은 순서/각도/거리/quality가 같지만, 200 μs 간격 해석 시 timestamp span 9,800 μs, 일괄 같은 μs 해석 시 0 μs(49개 인접 중복), 일괄 1 μs씩 해석 시 49 μs다. 즉 이 경로의 점별 timestamp는 host decode 시각을 따라간다.

잘린 마지막 점은 buffer에 남고 reset은 이를 버린다. 0거리/quality는 그대로 반환한다. normal 본문에는 CRC 검증이 없어 유효 헤더 뒤 거리 bit flip도 값으로 나온다. 낮은 합성 host 시각에서 unsigned 지연 차감 underflow와 360도 초과 wire 각도의 축약도 원문 그대로 관찰했다. 실제 C1이 이런 값을 생성한다는 뜻이 아니다.

`timing_verified=false`, `hardware_verified=false`, `ready_to_drive=false`. 실제 response/firmware 식별과 원본 수신/clock 근거 없이 점별 보정 경로로 승격하지 않는다. 기존 G1/구동/물성/보호 설정은 변경하지 않았다.
