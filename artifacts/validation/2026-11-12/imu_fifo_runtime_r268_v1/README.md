# r268 — IMU FIFO 시각의 선택형 온라인 연결

**기본 MCU 읽기 시각 경로를 유지**하고 `--imu-clock fifo_tick`에서만 센서 FIFO tick을 사용한다. 두 preview 브리지에 연결했으며 실제 ROS 시험은 원 IMU 앱을 사용하는 `bridge_esp32_imu_preview.py`에서 수행했다. 통합 sensor_link 브리지의 새 선택 옵션은 정적/관련 회귀만 확인했다. 운영 구동 허가·장착 교정·전체 시간 계약 완료가 아니다.

## 처리 방식

1. timestamp·gyro·accel의 원 전체 프레임 순서를 대조한다. 부팅/재시작·누락/overrun 뒤 이전 FIFO 슬롯을 빌리지 않는다.
2. **이미 도착한** timestamp 레지스터 읽기 구간만 최대8개 사용해 센서 tick→MCU 시계를 추정한다. 초기2개 report 전에는 추정 표본을 발행하지 않는다. nominal25µs를 실제 tick 길이로 고정하지 않는다.
3. MCU→호스트 offset은 기존 실제 `read_end`와 실제 I/O 수신시각으로 추정한다. 이를 취득시각 추정 중점으로 덮어쓰지 않는다. FIFO 추정은 별도 중점/구간/anchor 순번으로 기록한다.
4. 센서 값·원 read/receipt·원 부팅 정보는 보존한다. ROS 절대 취득시각의 정확도는 여전히 미검증이며 `time_contract_verified=false`다. MCU 구간 폭을 호스트 오차 보증으로 사용하지 않는다.

## 저장 입력

[r254 원문 재생](result.json)은 gyro3,951개 중 초기200개 이후3,751개를 발행했다. 기존 읽기 추정의3,901개와의 차이는 초기 준비 구간이다. 뒤쪽 입력 없이도 앞1,776출력이 동일하며 모든 추정은 표본보다 앞선 register anchor만 사용했다.

| 항목 | 기존 읽기 기준 | FIFO 온라인 추정 |
|---|---:|---:|
| 같은 표본의 간격 최소~최대 | 3.986~6.151ms | 4.986~5.055ms |
| 간격 중앙 | 5.000ms | 5.023ms |
| 원 읽기 시각 대비 이동 중앙/최대 | — | 0.707/2.188ms 이전 |

수신 callback 처리 비용은 PC p95 약0.063ms, 1Hz fit 갱신 p95 약0.922ms였다. 첫 계산 최대10.664ms를 포함해 보존하며, 이 비용은 Jetson 측정값이 아니다.

## 실제 Jetson/ROS 20초

[실제 브리지](bridge_summary.json)·[공통 MotionInput](receiver_summary.json)·[독립 원문 감사](live_audit.json):

- gyro3,982개/20.000초, **199.099Hz**. 준비 완료 이후3,794개 발행/공통 수락, 거부0.
- 표본 간격 약4.961~5.058ms. 콜백에서 본 추정 표본 나이는 중앙2.899ms·p95 3.449ms·최대27.666ms. 이는 실제 취득 지연의 절대 측정값이 아니다.
- UART12,396프레임을 독립 해석해 값·순서·재구성한 ROS 시각·clock sidecar가 모두 일치했다. CRC/순번/SPI/송신/FIFO overrun 오류0.
- STOP 문자 `X` 뒤 비활성 응답, 두 프로세스 exit0, 포트 소유자 없음. 펌웨어/센서 BDR/모터 출력/전력 설정은 바꾸지 않았다.
- 첫 시도는 포트의 q 사용자 ACL이 없어 열기 전에 실패했다. 해당 포트만 q 읽기/쓰기 ACL을 복구하고 별도 실행했다. 원 실패·빈 수신 기록을 보존했으며 성공으로 덮어쓰지 않았다.

관련 [81검사](tests.log)를 통과했다. 첫 추가 검사에서는 `arena_autonomy` Python 경로가 빠져 수집에 실패했으며 수정 후 결과와 분리했다. 원문/전체 ROS 표본은 Git 제외 `build/r268_recovered/`와 Jetson `/home/q/it_arena_imu_fifo_r268_20261112/`에 보존한다. [회수 해시](result_bundle_receipt.json)·[실행 소스](capture_source_manifest.json)·[변경 전 소스](before_sources.zip)를 남겼다.

## 남은 경계

현재 값은 호스트까지의 **추정 시간**이다. 실물 C1과의 상대 위상/장착/움직임 대조와 엔코더가 없으므로 deskew나 주행 결과로 확대하지 않는다. 현재 제어기에서 이 값을 운영에 사용할 명시 계약은 후속이다. 초기 시계 준비는 센서 활성화 후 약1~2초이며 센서를 상시 수신해 출발 전에 준비할 수 있다. START 때 센서를 재시작해야 한다는 요구가 아니다.
