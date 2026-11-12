# r239 — 실제 IMU와 MCU 계약 동시 수신, 첫 실행 실패

사용자가 모터 미연결·배터리 없음과 ESP32 변경 허용을 확인한 상태에서 시험했다. 출력 코드가 없는 `firmware_sensor_link`의 app만 설치했고 readback 해시를 대조했다. 시험 후 원 IMU probe 앱 `8c9fb788…ff27`로 다시 복구하고 readback 일치·포트 소유자 없음을 확인했다. bootloader·partition·NVS·전력 설정은 바꾸지 않았다.

## 확인한 것

- 실제 Jetson ROS host의 `offer_time_policy=receiver_owned_lease`로, Jetson 시각1794459986763813 μs와 MCU 부팅시각2881630 μs를 구별해 요청/응답했다.
- ARM은 `output_not_ready`, CLEAR는 `not_ready`로 거부, 명시 STOP 두 번과 정리 STOP은 `software_stop` 응답. 네 서비스 왕복은 약11.3–12.9 ms였다.
- IMU→공통 `MotionInput` 3,855개 발행/수락 일치·거부0, 실제/계약 boot 일치, 자식 세 프로세스 정상 종료. 엔코더 샘플0이며 가짜0피드백을 넣지 않았다. DRIVE 송신0.
- 빌드 성공, 원문/명령 프레임 교대의 분할·0바이트·부분 송신·유한 큐 등 PC 검증16개 통과. 같은 시간의 r238 기존 계약/출처 회귀394개 통과.

## 실패와 원인 범위

**무손실 시험은 실패했다.** 11,830개 IMU 원문 중 인접 순번1713→1715,1717→1857에서 총140개가 빠졌고 CRC 오류2건이 있었다. SPI/IMU FIFO/MCU 송신 큐 오류 계수와 계약 제어 마감 누락은 모두0이었다. 두 경계는 아직 모터 명령을 보내기 전이었다.

원문 수신 기록에서 순번1650→1651 사이 호스트 읽기 간격 **331.076 ms**가 있다. 뒤에서 한 번에 회수된 데이터 안에 두 순번 공백과 약0.226초 gyro 간격이 나타난다. MCU는 그 사이 순번을 계속 발행했고 이후 전송도 지속했다. 호스트가 오래 읽지 못한 구간과 손실이 대응하지만, 당시 단계별 계측/USB 드라이버 손실 계수는 없으므로 정확한 지연 원인이나 USB 계층을 단정하지 않는다. 공통 입력 callback 나이 최대340.47 ms도 실패 한계로 보존한다.

후속은 동일 펌웨어에서 UART 입출력을 ROS 처리와 분리해 제한 큐로 받고, 원 수신 시각과 큐 지연을 유지하는 비교다. 지연된 값을 새 시각으로 바꾸지 않는다. 기존 보드 송신 속도/IMU 범위·FIFO·제어/임대 기한은 바꾸지 않는다.

## 근거

`bridge_summary.json`, `receiver_summary.json`, `supervisor.json`, `flash_summary.json`, `restore_summary.json`과 `source_manifest.json`을 보존했다. 원문은 Git 제외 `build/r239_recovered/`, 압축 회수본 `build/r239_results_private.zip` SHA256 `15449a5673f6c2022dd6cd0ab0afdf8c69b2ee12d85c6aeac20ce27946f3d2d6`다. 업로드 묶음은 `upload_receipt.json`, ESP app 해시는 `2c27283343d8019670d270b1fd3a39b905c787ec3bfa5f4104ee4d5ae1a66915`다.

실물 시각/장착 교정·엔코더/모터 운용·주행·전체 제어 마감 통과는 아니다. 제한 수신 성공과 무손실 실패를 합치지 않는다.
