# Jetson에서 회수한 ESP32 통신 단절 증거

**현재 원본 위치:** [비공개 GitHub 증거 릴리스](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA-evidence/releases/tag/archive-2026-11-15). 원격 해시 확인 후 로컬 중복본은 정리했으며 아래 경로는 회수 당시 위치다.

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

후속 확인에서 고장 폴더 바깥의 직전 서보 캡처 두 건, 서보 플래시 준비, 15% 시험 당시 호스트 코드 및 신선도 수정본을 더 회수했다. 일반 파일 **57개**의 원격 SHA-256을 확인했고 원시 기록은 같은 백업 폴더의 `jetson-incident-supplement.tar.gz`에 둔다. [추가 회수 목록](supplement_manifest.json)에는 [시험 당시 호스트 코드](supplement/start15_candidate__probe_esp32_motor.py), [실행기](supplement/start15_candidate__run_guarded_motor_bench.py), [후속 수정본](supplement/start15_host_fresh_v2__probe_esp32_motor.py)의 공개 사본과 해시가 있다.

별개로 [Jetson에 설치했던 r500의 입력 소스](supplement/r500-installed-input-sources.zip)와 [설치 확인 자료](supplement/r500-install-receipts.zip)를 당시 해시와 일치하는 노트북 사본에서 공개했다. 이는 경기 전 Jetson 준비 이력이며 [대회용 최종 ESP32 소스](../../../../docs/releases/FINAL_ESP32_2026_11_14.md)나 고장 당시 모터 진단 앱과 구분한다.
