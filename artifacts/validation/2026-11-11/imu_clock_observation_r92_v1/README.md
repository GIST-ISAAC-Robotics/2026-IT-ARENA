# 저장 IMU 기록의 시계 대응

2026-11-11. 새 장치 조작 없이 기존 수신 성공5분 기록을 분석했다. 목적은 ROS/운동 보정 이관 전 시계 환산의 실제 차이를 좁히는 것이다. 운영 시계·영점·장착 축은 변경하지 않았다.

`analyze.py`는 원시 timestamp59,721개와 MCU 읽기 시작·호스트 수신 시각을 대조한다. 같은 boot·UART sequence 연속성을 확인하고, **처음60초로 얻은 직선 관계를 이후 약240초에 그대로 적용**한다. 원문은 Git 제외 `build/esp32_imu_hardware_2026_11_11/remote/continuous_no_gc/frames.jsonl`이며 전후 SHA-256을 확인했다. 집계는 [report_with_slots.json](report_with_slots.json), 첫 시계 집계는 [report.json](report.json)이다.

| 항목 | 관측 |
|---|---:|
| 센서 timestamp → MCU 읽기의60초 추정 비율 | 26.1635678 μs/tick |
| 이후240초 잔차 절댓값 p95 / 최대 | 0.531 / 1.640 ms |
| MCU 읽기 → 호스트 수신의60초 추정 비율 | 0.999997209 |
| 이후240초 잔차 절댓값 p95 / 최대 | 0.317 / 4.167 ms |
| 명목25μs 환산의5분 경과시간 차이 | −13.342초 |
| 공장 판독 `25/0.955` 후보의5분 경과시간 차이 | +0.165초 |

단순25μs 고정 환산은 이 장치의 기록에 맞지 않는다. 공장 후보도 시계 동기 없이 장시간 고정해 쓸 만큼 정확하다고 결론 내리지 않는다. **현재 시뮬이13초 틀어져 있거나 실차 제어가 이미 그 값으로 구동된다는 의미가 아니다.** 운영 연결 전의 차이를 확인한 것이다.

FIFO 완결59,720묶음은 timestamp→gyro→accel 순서였다. 세 항목의2bit `TAG_CNT`는 같은 슬롯을 가리켰고 다음 timestamp마다 modulo4로1씩 증가했다. ST AN5272 §9.4의 FIFO 슬롯 정의와 부합한다. MCU가 gyro/accel을 실제로 읽은 시각은 timestamp 읽기보다 중앙183/361μs 뒤였으므로, 각각의 읽기 시각을 취득 시각으로 간주하면 읽기 순서의 지연까지 섞인다. [ST AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf).

## 한계와 다음 적용

- 잔차는 **읽기/수신 시계 사이의 변동**이며 물리 취득 오차나 센서 필터 지연의 상한이 아니다. 절대 지연·시작 오프셋은 이 기록만으로 분리하지 못한다. 자이로/가속도 내부 필터의 위상까지 같다고 주장하지 않는다.
- 기록된 한 boot·정지 환경의 관계다. wrap는 실물에서 발생하지 않았고 재부팅·다른 온도/부하에 그대로 적용하지 않는다. 저장60초 fit의 계수를 제품 설정에 쓰지 않는다.
- 다음 실물 어댑터는 원본 FIFO 슬롯/내부 timestamp를 유지하고, boot별로 센서→MCU→호스트 시계 대응과 지연 불확실성을 분리한다. 운영 연결 전에는 진단 토픽/기록으로 대조한다. 축/영점은 최종 장착 뒤 확인한다.
- [기존 원문 감사](../esp32_imu_input_v1/audit_no_gc.json)의 끝점 비율26.1635898μs와 독립적으로 대조했고 알려진 직선/유보구간 계산·영시간축 거절을 검사했다. [검증](verification.json). 새 실물5분 시험이나 시뮬 주행은 하지 않았다.

제조사 문서§9.11은 매FIFO 슬롯의timestamp를 사용하는 기본 상관 방식을 설명하고, overrun 뒤 새timestamp 전 데이터는 제외하도록 한다. 이는 **센서 내부 상대시간 재구성**의 근거이며 Jetson과의 절대 시계 동기 보장은 아니다. 문서§6.4의 공장주파수 식과 이번 장치판독−30도 대조했다.

## 미연결 FIFO 연관 후보

[`fifo_candidate.py`](fifo_candidate.py)는 위 명시 설정(압축OFF·DEC_1)의timestamp/gyro/accel을 같은 슬롯으로 연관한다. gyro/accel을 강제로 한 표본에 합치지 않고 원래 값·읽기 시각·호스트 수신 시각·내부tick을 각각 유지한다. 시계 초 환산·ROS 발행·보정·자세 생성은 하지 않는다. 잘못된tag parity, 슬롯 불일치, wire누락/overrun, boot/내부시계 되감김 뒤에는 새timestamp 전 표본을 연관하지 않는다. 이 후보의누적SPI/TX오류 처리는 진단용 보수 정책이며 운영 복구 정책으로 채택하지 않았다.

원문 전체에 적용한 [저장 비교](fifo_candidate_replay.json)는 측정창gyro/accel 각각59,721개·원값 보존·스트림 시간역행0을 확인했다. 경계7검사도 통과했다. 후보는 이 산출물 폴더에만 있고 운영 `src/`·펌웨어에는 아직 연결하지 않았다. `.time_basis=sensor_ticks_unmapped`이므로 이를 검증된ROS 취득시각으로 사용할 수 없다.

후속 검토에서 같은 boot 안에서도 진단 수집을 중단·재시작하면 내부 시계가 다시 시작할 수 있다는 경계를 추가했다. 반복된 중단 상태는 한 번의 새 epoch로 묶고, 다음 timestamp 전 표본은 연관하지 않는다. 이 경계를 포함한8검사를 WSL Python에서 통과했다(Windows 기본 Python에는 pytest가 없었다). 재현 가능한 [`replay_fifo.py`](replay_fifo.py)로 같은 원문을 다시 처리한 [v2 비교](fifo_candidate_replay_v2.json)에서도 측정창 gyro/accel 각각59,721개·원값 보존·시간역행0을 유지했다. 초기 수집 시작1회를 별도 epoch로 집계한 진단만 추가됐으며 기존 결과·원문은 보존했다. 실물 재시작 시험이나 운영 복구 정책 검증은 아니다.
