# r237 — IMU 앱 복구와 현재 공통 ROS 입력 재연결

사용자가 **모터 미연결·배터리 없음**을 확인하고 ESP32 펌웨어 교체/설정 변경을 허용했다. r234에서 남아 있던 `motor_probe_v1` 대신 기존 검증 IMU 앱을 복구했다. 모터/서보 명령은 보내지 않았다.

## 실제 복구

실제 partition table의 factory app `0x10000 / 0x100000`을 확인하고 현재 app1MiB와 table을 먼저 보존했다. 11/11에 검증한 `arena_imu_probe.bin` 214,016바이트(SHA `8c9fb78850dcdb56d6c9e14e11e91e8e852dcdf78718038cc85dd6690f81ff27`)만 app offset에 썼다. 같은 길이의 실제 Flash readback이 완전히 일치했다. bootloader/partition/NVS를 쓰지 않았다. [복구 코드](restore.py)·[결과](restore_summary.json).

## 실제 입력 결과

기존 r161 무구동 ROS 수신기를 현재 공통 소스와 격리 domain190에서 실행했다. 첫 셸은 Windows CRLF 때문에 `pipefail`에서 **장치/수신기 시작 전에 실패**했다. 내용은 그대로 LF로 바꾸어 새 실행 경로에서 수행했고 첫 로그·원 셸·[수정 해시](capture_eol_fix.json)를 보존했다.

- 센서20.001초: gyro **3,982개·199.093Hz**.
- 준비 포함 공통 `MotionInput` gyro **4,032개 수락·거부0**.
- 원 UART **12,396프레임**을 독립 재해석하여 ROS 값·header 시각 재구성과 일치.
- CRC/sequence 공백/SPI/송신/FIFO overrun **0**. 부팅 접두부684바이트는 별도로 집계.
- 명시 `X` 후 비활성 응답, 수신기/브리지 exit0, 포트 소유자 없음. 앱은 다음 `S` 전까지 ODR OFF/대기다.

[원 브리지 요약](bridge_summary.json)·[공통 입력 요약](receiver_summary.json)·[독립 감사](audit.json)·[감사 코드](audit.py).

**센서가 지금 응답한다는 사실을 확인했다.** 읽기시각 추정과 미장착 축을 사용했으므로 실제 취득시각·장착 교정·자율 주행 승인은 아니다. 엔코더 입력0개이며 0속도를 만들어 공급하지 않았다. 운영 MCU/구동 계약과 IMU 앱은 아직 별개다.

원시 UART·전체 표본·Flash 백업은 Git 제외 `build/r237_recovered/` 및 Jetson의 `/home/q/it_arena_imu_restore_r237_20261112/`에 보존한다. 회수 ZIP SHA `a71682000a8fc4c0b5ab5f78963d780b673300d40cfa0b8650eaa51d78683dbd`를 대조했다. [실행 소스](capture_source_manifest.json)와 [묶음](capture_bundle_receipt.json)은 최초 CRLF 셸 기준이며 후속 LF 변경은 별도 해시로 연결한다.
