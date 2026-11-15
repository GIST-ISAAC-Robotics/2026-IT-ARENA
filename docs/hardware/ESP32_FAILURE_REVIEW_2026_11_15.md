# ESP32 통신 단절·발열 사건 증거 정리

대상은 2026-11-14 조립 후 DC 시험 중 MCU 응답이 끊긴 사건이다. 2026-11-15 첫 정리에서는 노트북 파일을 모았고, 후속 회수에서 **Jetson의 시험 원본 83개와 사건 시간대 커널 로그**를 읽기 전용으로 복사했다. ESP32 조회·리셋·출력·재현 시험은 하지 않았다. [경기 무주행 결과](../reports/COMPETITION_RESULT_2026_11_14.md)와 고장 원인 분석을 구분한다.

## 현재 확인할 수 있는 결론

마지막 정상 MCU 수신 뒤 **27.314343111초**에 15% 시험을 위한 상태 조회가 기록됐다. 원 명령에서 이 단계는 `?`와 약 2.008초 뒤 `X`뿐이며 **재연결 뒤 구동용 ARM/PULSE가 없다.** UART 원시 바이트를 CRC 검사해 해석 프레임 14,145개와 모두 대조했다. 조회 이후 새 프레임은 0개, 이후 재접속 UART도 0바이트였다. [회수 증거](../../artifacts/validation/2026-11-15/esp32_jetson_recovery_v1/README.md)

앞서 **모터 전원을 분리한 15% 논리 출력 검사**는 1회 존재한다. 따라서 ‘15% 명령이 한 번도 없었다’고 전체 기록을 요약하면 틀린다. 전원 없는 보호 검사와 전원을 다시 연결한 뒤 시작하지 못한 구동 시험을 구분해야 한다.

LED 소등·발열·보드 단독 단락 의심은 사용자의 현장 전달이다. 전압·전류·온도 파형과 안정된 저항 실측값이 없으므로 **전기적 고장 순간과 원인은 확정할 수 없다.** 역기전력, 전원·배선 문제 또는 MCU 보드 손상 중 하나로 단정하지 않는다.

## 사건 시각

아래 밀리초 단위 절대 시각은 [당시 DC 보고서](MOUNTED_DC_TEST_2026_11_14.md#통신-중단-시각-대조--한국시간)의 monotonic↔realtime 대응 기록을 보존한 값이다. [기존 시각표](../../artifacts/validation/2026-11-15/esp32_incident_v1/timeline.json)의 원문은 커밋 `0af38406664de39ba7152bb5922397b2f48f6bbe`에 고정했다. 후속 회수는 UART·명령의 단조 시각 차이와 커널 로그의 초 단위 벽시계를 직접 확인했다. 남은 journal에 해당 시간대 기록은 없었다.

| 한국시간 11/14 | 기록된 사건 | 이번 로컬 증거 |
|---|---|---|
| 13:35:48.387 | 엔코더 손 회전 결과 표본 | 결과 JSON과 방향 정정 있음 |
| 13:36:26.551 | 마지막 유효 MCU 프레임. PWM 0·미무장·비활성 | seq 14144를 UART 원본과 대조. 파일 최종 수정 13:36:26.549953205 |
| 13:36:53.865 | 15% 구동 전 상태 조회, 새 응답 없음 | 명령 로그에 `?`·`X`와 오류. 새 프레임 0 |
| 13:40:18 | USB 1-1 첫 분리, 이후 재등록·재분리 | `kern.log` 벽시계는 초 단위, 대괄호 단조 시각은 3381.416278초 |

USB 분리는 마지막 수신보다 약 3분 52초 뒤였다. 회수된 `kern.log`에서 포트 1-1의 분리 4회, 재등록 4회, ACM 등록 4회를 확인했다. 포트 이벤트만으로 변환칩·본체·전원 중 어느 부분에서 이상이 시작됐는지는 알 수 없다.

## 출력·엔코더 증거

아래 시간은 MCU의 `ended_us - started_us`다. PWM 파형이나 모터 단자 전압을 외부 계측한 값이 아니다. 전원 연결 상태는 당시 사용자 확인/시험 단계의 기록이다. [원본 JSON 사본·계산값](../../artifacts/validation/2026-11-15/esp32_incident_v1/observations.json).

| 시험 | 모터 전원 | 요청 | 논리 출력 지속 | 종료 사유 | 종료 상태 |
|---|---|---|---|---|---|
| 5% | 연결 보고 | 5초 | 360.041ms | `no_encoder_start` | duty0·PWM pin0·미무장 |
| 7.5% | 연결 보고 | 0.5초 | 500.052ms | `deadline` | duty0·PWM pin0·미무장 |
| 15% 보호 검사 | 분리 확인 | 5초 | 360.029ms | `no_encoder_start` | duty0·PWM pin0·미무장 |
| 15% 실제 구동 준비 | 재연결 보고 | 구동 전 조회 | 시작하지 못함 | response timeout | 새 상태 확인 불가 |

세 출력 검사 모두 엔코더 증가가 0이다. 5% 시험 뒤 사용자는 무회전을 보고했고 7.5% 육안 회전은 확인되지 않았다. 그 후 손 회전에서는 PCNT/software 모두 **+690**, invalid/drop 증가 0을 확인했다. 원 JSON의 ‘전진’ 보고는 후속 사용자 정정으로 **후진**이다. 원본을 덮어쓰지 않고 [원 결과](../../artifacts/validation/2026-11-15/esp32_incident_v1/evidence/manual-encoder-result.json)와 [방향 정정](../../artifacts/validation/2026-11-15/esp32_incident_v1/evidence/operator-direction-correction.json)을 함께 보존했다.

## 재접속과 상태 판정

- [구동 전 오류](../../artifacts/validation/2026-11-15/esp32_incident_v1/evidence/start15-precommand-error.json): `waiting_for_ready`, `response timeout; no automatic retry`.
- [재접속 오류](../../artifacts/validation/2026-11-15/esp32_incident_v1/evidence/start15-reconnect-error.json): `startup`에서 같은 timeout.
- [회수된 명령2건](../../artifacts/validation/2026-11-15/esp32_incident_v1/reconnect_command_excerpt.json): `?`→5.010867012초 뒤 `X`. 이 파일 범위에 ARM/PULSE 없음. 원 JSONL은 로컬 보존한다.
- [종료 기록](../../artifacts/validation/2026-11-15/esp32_incident_v1/evidence/start15-reconnect-closed.json): `port_closed=true`, **`stop_confirmed=false`**, `last_motor=null`. 포트 닫힘은 MCU의 새 정지 응답이 아니다.
- [부트로더 조회 발췌](../../artifacts/validation/2026-11-15/esp32_incident_v1/rom_query_excerpt.txt): `No serial data received`. 장치 식별 경로만 가렸고, 원본 SHA256은 manifest에 남겼다. 당시 reset을 사용했으나 flash 쓰기는 없었다는 범위는 원 보고서를 따른다.

당시 호스트의 `state.json`은 새 MCU 수신 없이도 갱신될 수 있었으므로 파일 수정 시각만으로 통신이 살아 있다고 판단할 수 없었다. 구동기 내부의 새 info 응답 요구가 실제 출력을 막았으며, 후속에 MCU 수신 시각 기반 신선도 검사와 포트 종료/정지 확인 구분을 보완했다. 이 수정은 고장 원인 규명이 아니다.

## 자료 보존과 미확보 항목

[증거 manifest](../../artifacts/validation/2026-11-15/esp32_incident_v1/manifest.json)에 19개 로컬 파일의 원래 경로·크기·SHA256을 기록했다. 9개 작은 결과 JSON은 byte 그대로 공개 사본을 만들었다. 원시 명령·빌드 로그·기록 당시 앱 및 직전 앱 백업까지 포함한 별도 로컬 묶음은 `build/esp32_incident_20261115/esp32_incident_local_evidence.zip`이다. [묶음 해시](../../artifacts/validation/2026-11-15/esp32_incident_v1/local_bundle.json). 원본 파일은 수정하지 않았다.

후속 회수 원본은 [회수 목록](../../artifacts/validation/2026-11-15/esp32_jetson_recovery_v1/recovery_manifest.json)에 기록했다. Jetson의 시험 폴더와 노트북의 `build/mounted_motor_start15_5s_20261114/stage-path.txt`는 각각 **시험 수신 자료 위치**와 **ESP-IDF 빌드 임시 경로**로 구분한다. 원시 UART·전체 명령·커널 시간창·빌드 로그 및 `stage-path.txt` 원본은 저장소 밖 `IT ARENA evidence backup 2026-11-15/incident-originals.zip`에도 보관했다.

고장 세션과 함께 해석할 직전 서보 캡처 및 실제 호스트 실행 코드는 [추가 회수 목록](../../artifacts/validation/2026-11-15/esp32_jetson_recovery_v1/supplement_manifest.json)에 정리했다. 사건 폴더의 83개와 별개로 일반 파일 57개를 기존 SHA-256과 대조했고, 원본은 같은 백업 폴더의 `jetson-incident-supplement.tar.gz`에 보존했다.

분석에 아직 없는 자료는 다음과 같다.

- 당시 journal 원본 엔트리. 회수한 `kern.log`의 초 단위 USB 기록으로 벽시계를 재대조했다.
- 당시5V/3V3·모터 전원 전압/전류 파형, 발열 위치·온도
- USB·외부선을 모두 분리한 보드의 단위가 있는 안정된 저항값
- 실제 전원/접지/커넥터 연결 상태 사진과 장치별 전원 투입 순서

사용자가 전달한 저항 표시 0.1↔100은 단위·안정값이 확인되지 않아 확정 단락값으로 표기하지 않는다. 이 문서는 증거를 모으는 단계이며 고장 책임이나 단일 원인을 결론 내리지 않는다.
