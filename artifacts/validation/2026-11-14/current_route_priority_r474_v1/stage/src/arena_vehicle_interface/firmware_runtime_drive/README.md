# ESP32-S3 구동 연결 후보

**빌드·가짜 입력 검증용이며 현재 보드에 올리지 않았다.** 기본 `installed_board_profile()`은 미교정이라 유효한 속도 피드백·구동 허가·서보 목표 송신을 만들지 않는다. 실제 장착값을 연결하면 명시 ARM/DRIVE가 물리 출력을 만들 수 있는 앱이다. 미교정 상태에서는 진단용 wire 설정으로 `profile_valid=false`를 송신하며 운영 한계나 구동 준비가 확인됐다는 뜻이 아니다. 기존 [비구동 preview](../firmware_runtime_preview/README.md)와 혼동하지 않는다.

## 연결

- 한 소유 작업에서 UART0 호스트 계약, GPIO8 STOP, GPIO4/5 PCNT, UART1 서보, PWM15/DIR6, 기존 IMU 취득/송신을 처리한다. 송신은 즉시 FIFO 수락량만 소비하고 UART 응답 대기는 하지 않는다.
- 모터축28카운트·`2.5×81/17`·바퀴65mm는 공통 환산을 따른다. 프로필의 장착 부호로 PCNT 누적값을 정규화한다. 차분 간격20ms는 준비값이며 실제 응답 최적값이라는 뜻은 아니다. 제어 호출은 고정 위상5ms, 기존 명령/피드백 만료는 유지한다.
- `ready`는 이번 호출의 유효 프로필·서보 관측 증거다. ARM 후 현재 위치 읽기→그 위치 유지 목표 ACK→torque1 ACK를 거쳐 `engaged`가 된다. 이 준비 중 모터 출력과 PI 적분은0이다. ACK는 바퀴 이동 완료 확인이 아니다.
- 이번 ARM 세대에서 실제 DRIVE를 받았을 때만 CAD 조향 목표를 적용한다. 속도0의 DRIVE도 조향할 수 있다. ARM만으로 중앙 복귀하지 않는다.
- PI는 전진 duty 범위를 사용하고 DIR은 확인된 전진 극성으로 고정한다. 역회전 측정값에서도 전진 복원 출력을 만들 수 있으며, 정상 과속의 duty0은 STOP 상태 전이가 아니다. MD20A PWM0은 단락 제동 요청이며 실물 감속 크기는 미측정이다.
- STOP/만료/서보 준비 소실 때 PWM0을 요청한다. GPIO8 복귀만으로 재출발하지 않는다. 서보의 미송신 목표는 폐기하고 시작한 거래의 늦은 ACK로 이전 ARM이 복원되지 않게 한다. 자동 중앙 복귀/torque0는 하지 않는다.
- 서보 거래 오류는 출력 준비 상실로 처리한다. 허가 해제 상태에서 TX 완료와 RX 무응답30ms를 확인하고 RX를 비운 뒤 읽기 전용 Probe로 돌아간다. 이 시간만으로 실제 늦은 ACK가 없음을 보증하지 않는다. 원 명령을 재전송하거나 CLEAR/ARM 없이 다시 움직이지 않는다.

## 조립 후 연결할 값

`main/board_profile.hpp`에 실제 장착 엔코더 부호, 모터 방향, 시작 duty 상한, 속도 상한, PI 게인과 서보 설정을 넣는다. 서보 `zero`는 [영점 도구](../../../scripts/steering_zero_profile.py)의 `record` → `confirm-mount` → `export-cpp`로 기록한 값을 사용한다. CAD 중앙 명령 한계·wheelbase128mm/track123.9mm·내륜30°를 같은 프로필에 연결한다. 장착 안전 범위 밖을 자동 래핑하거나 영점을 EEPROM에 쓰지 않는다.

raw servo speed/acceleration은 무부하 시험값을 운영 기본으로 채택하지 않았다. [단위와 기존 Phase12 확인](../../../artifacts/validation/2026-11-13/mcu_engagement_advice_r383_v1/README.md)을 참고하되 실제 장착 응답은 별도다. 유효 프로필 작성과 장치 업로드/실물 출력 시험을 같은 승인으로 취급하지 않는다.

## 빌드와 남은 한계

```bash
bash scripts/build_runtime_drive.sh \
  /home/jinhyeong/.cache/it-arena/runtime-drive-새이름 \
  /home/jinhyeong/.cache/it-arena/lsm6dsox-pid_20261111 \
  /절대경로/새결과폴더
```

preview의 peripheral 초기화 보조/IMU/송신기를 공유한다. 빌드 도구는 원문 snapshot·전후 해시·binary 해시를 남기며 flash하지 않는다. 기본 프로필에는 장착값이 없어 실제 운용을 허가할 수 없다. [r387 결과](../../../artifacts/validation/2026-11-13/runtime_drive_r387_v1/README.md)를 따른다.

MCU 정지/전원 손실에서의 독립 PWM 차단, 실제 LEDC 파형과 조향 부하, UART 지연, 동시 IMU/PCNT 주기, 전체 장착 STOP은 미검증이다. 치명 UART/HAL 오류는 PWM0 요청 뒤 앱을 정지하지만 그 요청의 물리 성공을 보증하는 별도 회로가 아니다. 이 후보를 실차 검증 완료로 표시하지 않는다.
