# ESP32 통신 단절 사건 증거 묶음

2026-11-15 로컬 보존 파일을 수집했다. [분석 문서](../../../../docs/hardware/ESP32_FAILURE_REVIEW_2026_11_15.md)가 기준이다.

- [manifest.json](manifest.json): 원본19개 경로·크기·SHA256. 원본 수정0, 장치 접근0.
- `evidence/`: 작은 결과 JSON9개 byte 동일 사본. 엔코더 방향 원 보고와 정정은 별도 보존.
- [observations.json](observations.json): 출력 시간·종료 상태·손 회전 계산 결과.
- [timeline.json](timeline.json): 당시 보고서의 시각표와 고정 commit/blob. 원시 전체를 새로 읽어 검증한 시각표가 아님.
- [reconnect_command_excerpt.json](reconnect_command_excerpt.json): 회수된 명령2건의 공개 발췌. 원 JSONL은 로컬 묶음에 보존.
- [rom_query_excerpt.txt](rom_query_excerpt.txt): USB 장치 식별 경로를 가린 오류 발췌.
- [local_bundle.json](local_bundle.json): 원시 파일·펌웨어를 포함한 로컬 ZIP 경로와 해시. Git 공개 파일이 아님.
- [validation.json](validation.json): 원본·공개 사본·로컬 ZIP의 SHA256, 고정 보고서 Git blob과 문서 링크 검사.

수집기는 [collect_esp32_incident.py](../../../../scripts/collect_esp32_incident.py)이며 새 폴더만 생성한다. 전원을 켜거나 UART/SSH/부트로더를 다시 조회하지 않는다. 전체 사건 UART와 kernel journal, 전기 파형이 없어 원인·고장 순간을 확정할 수 없다.
