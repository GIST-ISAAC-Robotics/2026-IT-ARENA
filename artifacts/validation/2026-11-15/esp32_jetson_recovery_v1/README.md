# Jetson에서 회수한 ESP32 통신 단절 증거

2026-11-15, 기존 호스트 키를 확인한 SSH 접속으로 Jetson의 `mounted-dc-test.5qyIBL` 시험 폴더를 복사했다. [회수 목록](recovery_manifest.json)의 원격 SHA-256과 전송 묶음의 파일 **83개**가 모두 일치한다. 전원·포트·구동 상태 변경이나 재시험 없이 기존 파일만 읽었다.

| 공개 자료 | 확인할 내용 |
|---|---|
| [사건 단일 세션](incident_evidence.json) | UART CRC·프레임 14,145개 대조, 마지막 PWM 0·미무장, 27.314초 뒤 조회 및 새 응답 0 |
| [명령 이력](command_history.json) · [수신 집계](uart_timeline_summary.json) | 9개 명령 스트림의 61개 레코드, 최초 수신 기준 1초 구간 134개 집계. 명령 ID·CRC·세션 토큰을 생략 |
| [USB 시간표](kernel_usb_events.json) | `kern.log`에서 1-1 분리·재등록·ACM 12개 발췌. 벽시계는 초 단위이며 괄호 안 커널 시각은 단조 시계. 시스템 journal에는 해당 시간대 항목이 없었음 |
| [시험 결과 원본 JSON 15개](test_results/) | 5%·7.5%·무전원 15%, 손 회전 전후와 방향 정정, 재접속·종료 결과 |
| [빌드 출처](build_provenance.json) · [플래시 기록 발췌](flash_verify_excerpt.json) | 노트북 `stage-path.txt`와 IDF 로그에서 동일 빌드 임시 경로 확인. Jetson 시험 경로와 별개이며 공개 경로에서 계정명만 가림 |

[사건 분석](../../../../docs/hardware/ESP32_FAILURE_REVIEW_2026_11_15.md)은 당시 밀리초 시간표와 이번에 재확인한 시각의 정밀도를 구분한다. 마지막 응답 이후 전기적 상태를 측정한 자료는 없다. 통신 증거만으로 발열 원인이나 부품이 고장난 시각을 단정하지 않는다.

원시 UART·전체 커널 시간창·장치 식별자가 있는 로그·펌웨어 백업·`stage-path.txt` 원본은 노트북의 Git 제외 경로와 **저장소 밖** `IT ARENA evidence backup 2026-11-15/incident-originals.zip`에 보존한다. [외부 묶음 해시](../local_preservation_v1/summary.json). 별도 묶음은 같은 노트북에 있으므로 저장소 삭제에 대비한 사본이다.
