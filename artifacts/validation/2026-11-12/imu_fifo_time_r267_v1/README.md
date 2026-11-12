# r267 — 저장 IMU FIFO 시각과 MCU 읽기 구간 분리

기존 r254 원문만 재해석했다. 새 장치 취득·펌웨어/ROS/운영 설정 변경은 없다. 결과는 [result.json](result.json), 재현은 [audit.py](audit.py), 관련14검사는 [tests.log](tests.log)다. Windows Python의 pytest 미설치 오류는 별도 로그로 보존하고 WSL에서 검사했다.

[ST AN5272 Rev5](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)의 §9.2.9·§9.11을 현재 비압축·동일 BDR·timestamp DEC_1 설정과 대조했다. 전체 tag의 parity/counter와 timestamp→gyro/accel 슬롯을 확인한다. 누락/overrun 뒤에는 새 timestamp를 기다린다. 다른 압축/FIFO 형식으로 일반화하지 않는다.

- 원12,396 프레임의 다른 취득과 혼동하지 않는다. **이번 입력은 r254의11,973 IMU 프레임**이며 gyro/accel 각각3,951개를 빠짐없이 대응했다. 값·순서·원문 해시는 불변이다.
- 센서 tick 간격은3,950구간 모두192였다. MCU 읽기 간격은3.986~6.151ms로 달랐다.
- 별도 timestamp 레지스터 읽기20회의 MCU begin/end 구간을 사용했다. 센서 tick의 nominal25µs는 계산 좌표이며 실제 tick 길이로 고정하지 않았다.
- 명시한 affine 시계/양자화/읽기 오차 가정 안에서 tick은26.16366~26.16429µs, 각 MCU 취득시각 구간 폭은0.259~0.299ms였다. 해당 구간은 **Jetson/ROS 절대 오차 범위가 아니다**.

모든20개 읽기를 한꺼번에 사용한 사후 분석이다. 이 결과만으로 온라인 사용이나 실제 취득 교정을 완료했다고 판단하지 않는다. 과거 정보만 사용하는 선택형 연결·실제 ROS 검사는 [r268](../imu_fifo_runtime_r268_v1/README.md)에서 따로 수행했다.
