# 실제 IMU에서 공통 ROS 입력부까지의 격리 연결

2026-11-12. 실제 ESP32-S3 진단 v2의 FIFO 원문을 보존하면서 자이로를 `sensor_msgs/Imu`로 전송하는 진단 연결이다. 모터/서보·운영 제어·펌웨어 변경은 없다. `/hardware_preview/imu`, `lsm6dsox_sensor`, localhost 전용 DDS domain191을 사용한다.

MCU 읽기 시각과 호스트 수신 시각의 최소 차이로 전송 지연 변동을 줄이고 USB 묶음 내 표본 간격을 보존한다. 이 값은 **센서 취득시각이 아닌 읽기시각 추정**이다. FIFO 지연·시계 배율·차체 장착축/영점은 교정하지 않았으며 `time_contract_verified=false`를 유지한다. 가속도/자세는 메시지의 unavailable 표시로 구분한다. 원 가속도/FIFO 시각은 원시 기록에 보존한다.

새 boot·호스트 시계 변경에서 이력을 분리하고, 누락 표본을 0이나 보간으로 채우지 않는다. 관련32검사가 통과했다. 원 `MotionInput` 구독을 사용하되 encoder를 만들어 넣거나 LiDAR 보정을 실행하지 않는다. 물리 차량의 yaw축 검증이 아니다.

## 실행과 증거

- `source_manifest.json`:159개 배포 파일의 SHA-256. 동결 소스 묶음은 제외 경로 `build/imu_preview_r161_20261112.tar.gz`에 보존.
- `receiver.py`, `run.py`, `run.sh`: 구독 준비 후20초 실제 입력. 종료 시 센서 `X`/inactive와 자기 프로세스 종료, 한시적 장치 ACL 원복 확인.
- [실행](bridge_summary.json)·[공통 수신](receiver_summary.json)·[원문 대조](audit.json)·[종료](supervisor.json)·[권한 복원](acl_result.json). 원시 UART/표본 JSONL은 기존 Git 제외 정책을 유지한다.

## 제한 실물 결과

20.001초에 gyro3,981개·199.038 Hz를 발행했다. 준비 구간 포함4,031개를 공통 `MotionInput`이 전부 수신·수락했다. 원 UART12,396프레임을 다시 독립 디코딩하고 읽기시각 매핑을 재구성하여 모든 자이로 값과 ROS header를 대조했다. 값/시각 차이0, CRC·sequence공백·SPI·TX·FIFO overrun0이다. 시작 텍스트 등 비프레임684바이트는 지우지 않고 원문에 남았다.

ROS header 간격 중앙5.000 ms·최대6.005 ms, 추정 읽기시각 대비 콜백 나이 중앙1.508 ms·최대5.273 ms였다. **센서 실제 취득에서 제어까지의 지연이 아니다.** encoder0개·구동 발행 없음·장착 미교정이다. 상태25개 모두 시각 미승인 표시를 유지했다. 프로세스2개 exit0, `X` 뒤 비활성 응답, 원 ACL 복원과 포트 점유 없음, 동결159파일 불변을 확인했다.

단일 IMU·짧은 정지 수신 대표다. 전체 센서/제어 동시 부하·실제 운동/영점·C1과 시계 관계·기동/장기 운용은 미완료다. r160 CPU 개선과 지연 교통 실패를 이 결과로 완료 처리하지 않는다. 다음은 기존 운영 도구의 시험 속도 상한을 연결하여 실차 최초 저속 시험 준비를 진행한다.
