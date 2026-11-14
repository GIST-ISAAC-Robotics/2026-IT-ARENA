# r523 — controller 입력 소비 예산 보완과 잔여 명령 만료

**controller drain만 2→6 ms로 보완한 1회에서 active `motion_stale`는 2→0, 양수 `/drive` 최장 공백은 604→162 ms, `/drive/safe`는 501→103 ms로 줄었다.** 다만 짧은 명령 만료가 증가하여 관찰된 0 목표 시간은 1.92→3.48초로 늘었다. 입력 적체에 의한 긴 재개 정지를 줄인 결과이며, 전체 주행 손실이 개선됐다는 증명은 아니다. 추가 ROS 시험 없이 종료했고 Jetson 소유권을 반환했다.

## 원인과 최소 변경

[r522 trace OFF](../jetson_traceoff_continuity_r522_v1/RESULT.md)의 직접 control 호출 85.22 ms 뒤, 기존 2 ms drain은 첫 호출에서 wheel 5·gyro 4개만 소비하고 시간 상한에 걸렸다. 평가시각 22.148690초에 gyro는 22.085초, 나이는 63.69 ms였다. 다음 호출도 gyro 나이 42.51 ms로 기존 30 ms 수명을 넘었다. 원 입력 gyro 5 ms·wheel 10 ms 간격에는 이 공백이 없었다.

첫 drain 전까지 발행된 미소비 후보는 wheel 9·gyro 18개였다. 이는 발행/소비 stamp를 비교한 수이며 **실제 DDS unread queue 깊이를 직접 측정한 값은 아니다.** 현재 wheel 우선 교대 소비에서 gyro를 30 ms 이내로 따라잡으려면 약 20개 소비가 필요하다. r522 적체 호출 28개의 표본당 소비 비용은 중앙 0.2160 ms, p95 0.2334 ms, 최대 0.2377 ms였다. 20개 비용은 p95 4.667 ms·최대 4.753 ms로 추산되어, 약 1.25 ms 여유를 둔 **controller 6 ms**를 선택했다. 일반적 최악시간 보장은 아니다. [계산 및 원 stamp](budget_evidence.json), [산출 코드](budget_analysis.py).

- guard는 **2 ms**, 표본 수 상한은 **64개**, 원 stamp·FIFO 순서·callback 검증·남은 표본 보존을 유지한다. 빈 큐에서 기다리지 않고 즉시 끝낸다.
- drain 뒤 제어 평가용 `now`를 읽는 순서를 유지한다. callback의 기존 시각 검증도 유지하고 미래 표본을 현재값으로 위조하지 않는다. 시간 한도는 표본 사이에 검사하므로 한 callback만큼 초과할 수 있다.
- 기존 `SingleThreadedExecutor` 및 motion subscription과 control의 직렬화 전제를 유지한다. 병렬 callback 허용 구조에 그대로 확장하는 변경은 아니다.
- 고정 센서 30 ms·명령 200 ms 수명, 보호 조건, `/drive`→guard→`/drive/safe` 경계는 바꾸지 않았다.

[계약 회귀](test_budget_candidate.py) 3개는 원 stamp와 실측 표본 비용을 사용한 적체 모델의 2 ms 실패/6 ms 회복, 원 순서·잔여 표본 보존, 64개 상한, 정상 짧은 입력의 동일 결과·조기 종료를 확인한다. 로컬 및 ARM에서 통과했다. 합성 큐 회귀를 DDS 내부 큐의 실측으로 취급하지 않는다. [ARM 실행 로그](remote_run.log).

## 동결 범위와 1회 결과

r522의 동결 파일 13개를 대조해 **12개 동일**, `node_runner.py`의 role별 budget 인자 한 줄만 바꾸고 회귀 파일을 추가했다. [복사 감사](source_copy_audit.json), [manifest](manifest.json). 상세 trace 경로 등록·생성자 검증·시작 header/footer 4,252 bytes·경량 사건 집계는 유지했고 상세 입력 직렬화만 OFF인 r522 조건을 그대로 사용했다.

원 설치 `/home/q/it-arena-field-r500-20261114-a`의 소스 ZIP SHA는 `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`이다. 같은 원 입력·원 epoch/frame/`simulation_acquisition` 시각, domain 158·`/replay_r508`, native simulation runtime/guard/RaceOps, 합성 RGB 848×480·20 Hz와 실제 detector를 사용했다. SHM 단독 16 MiB는 실행별 선택이며 global 환경을 바꾸지 않았다. **r518 fit·r521 벽 재획득·최신 공통 전체 설치본은 포함하지 않는다.** 실물 센서/host/MCU/출력, 실제 B0390 영상, B3 wait/start, 실제 주행 또는 전체 10 Hz 완료 증거가 아니다.

명목 선택은 20초 `[7.62,27.62)`지만 마지막 원 gyro/wheel은 27.515/27.514초이다. 아래 active 비교는 같은 원 입력 유효 구간 **`[7.62,27.52)` 19.9초**이며 시작/EOF는 분리했다. 각 조건 1회로 실행 순서·완료 계산 수는 달라졌다. [비교 집계](comparison.json), [집계 코드](compare.py).

| active 지표 | r522 controller 2 ms | r523 controller 6 ms |
|---|---:|---:|
| `/drive` 0 / 양수 | 7 / 163 | **4 / 176** |
| `/drive/safe` 0 / 양수 | 118 / 833 | **195 / 766** |
| 모든 `/drive` 최대 공백 | 185.11 ms | **182.95 ms** |
| 양수 `/drive` 간격 중앙 / p95 / 최대 | 114.94 / 147.38 / 603.54 ms | **106.42 / 125.29 / 161.80 ms** |
| 양수 `/drive/safe` 최대 공백 | 501.21 ms | **103.26 ms** |
| 양수 `/drive` 원 stamp 나이 중앙 / p95 / 최대 | 62.86 / 114.80 / 121.28 ms | **98.61 / 118.54 / 140.64 ms** |
| 양수 명령 도착 나이 200 ms 초과 | 0 | **0** |
| controller / guard motion_stale | 2 / 0 | **0 / 0** |
| guard command_stale = stale_input | 81 | **173** |
| 모든 control 호출 비용 p95 / 최대 | 20.72 / 85.22 ms | **18.95 / 69.30 ms** |

completion-notice에서 직접 호출되는 control도 마지막 행에 포함한다. timer callback만 계측한 수치로 바꾸지 않는다.

r523 drain은 중앙 0.481 ms·p95 1.320 ms·최대 **3.709 ms**, 최대 **21개**를 소비했다. 기존 2 ms보다 오래 소비한 호출은 5회이며 6 ms/64개 상한 종료는 0회다. drain 전 최신 stamp가 평가시각보다 30 ms 이상 오래된 호출 8회도 이후 MOTION_STOP 없이 처리됐다. 이를 모두 2 ms에서 반드시 실패했을 것이라는 반사실적 증명으로 확대하지 않는다. [실제 drain 집계](drain_outcome.json), [산출 코드](drain_outcome.py).

| 회수한 계산 완료 비용 | r522 | r523 |
|---|---:|---:|
| plan 건수 / 중앙 / p95 / 최대 | 166 / 82.04 / 114.14 / 129.88 ms | **158 / 69.90 / 99.02 / 131.72 ms** |
| current 건수 / 중앙 / p95 / 최대 | 259 / 16.75 / 21.05 / 42.80 ms | **182 / 16.78 / 21.81 / 39.05 ms** |
| plan / current 제출→회수 p95 | 116.13 / 30.59 ms | **103.81 / 32.55 ms** |

회수 전에 폐기된 모든 작업까지 포함한 전수 계산 비용은 아니다. 두 노드 각각 gyro 4,381·drive motor 2,191개 accepted, rejected 0, timing drop 0이다. active controller는 SENSOR_STOP 1·DECISION_PENDING 26·HANDOFF_FOLLOW 3·RUNNING 1,240이고 guard는 missing_input 1·clear 788·stale_input 173이다. 첫 scan 전에 accepted gyro/drive 및 frame/topic/time 계약 관문을 통과했다.

시작 전 controller SENSOR_STOP/guard missing_input 각 101회는 별도이다. 원 입력 소진 뒤 controller MOTION_STOP 16·SENSOR_STOP 38·DISABLED 16·DECISION_PENDING 2, guard motion_stale 9·stale_input 61·clear 1(command_stale 표시 2)은 위 active 성능에 합치지 않았다.

## 남은 짧은 0 명령: 저장 로그만 분류

active guard 만료 173회의 **원 `/drive` 나이**는 최소 200.045 ms·중앙 211.979 ms·p95 234.091 ms·최대 247.407 ms였다. 이 값은 guard control receipt의 `command_source_s`로 계산했다. `/drive/safe`의 새 header는 guard 발행 시각이므로 원 명령 나이 대신 사용하지 않았다. [원시 stamp 근거와 구간별 집계](guard_stale_classification.json), [분류 코드](classify_guard_stale.py).

| 첫 양수 `/drive/safe` 뒤 관찰값 | r522 | r523 |
|---|---:|---:|
| 종료까지 관찰된 0 명령 구간 | 46회 | **115회** |
| 구간 길이 중앙 / p95 / 최대(관찰자 수신 기준) | 34.84 / 60.64 / 482.93 ms | **27.43 / 46.58 / 85.47 ms** |
| guard 발행 header 기준 구간 최대 | 470.15 ms | **54.39 ms** |
| 완료된 구간의 총 0 목표 시간 | 1.922초 | **3.406초** |
| 마지막 구간의 active 부분만 추가 | 0 | **0.076초** |
| 출발 뒤 active 관찰시간 대비 0 목표 비율 | 9.90% | **17.95%** |

r523 완료 구간은 0 표본 1개인 경우 65회, 2개 48회, 3개 2회다. 마지막 27.443679초부터의 구간은 다음 양수 명령이 원 입력 유효 범위 안에 없으므로 **우측절단**했으며 27.52초까지의 76.32 ms만 포함했다. 이 마지막 구간의 4개 0 표본은 stale_input 3·clear 1로, 모두 같은 사유로 합치지 않는다. EOF 이후 길이는 계산하지 않았다.

원 명령은 도착 시 중앙 약 98.61 ms를 이미 사용했고, 다음 양수 명령까지 중앙 약 106.42 ms가 걸렸다. 남은 유효시간과 갱신 간격이 맞지 않아 기존 200 ms 경계에서 짧은 만료가 반복되는 양상이다. 실제 관찰자 수신 기반 추정에서도 이전 양수 명령 만료 뒤 다음 `/drive`가 도착한 구간은 127회였다. 이 추정은 guard 내부 command callback 도착 시각 그 자체가 아니며, 중앙값의 합만으로 개별 인과를 증명하지 않는다.

**6 ms 보완은 적체 후 긴 재개 정지에 대한 최소 변경으로 이관을 제안한다.** guard 173회는 입력 gyro stale와 다른 잔여 문제다. 후속은 원 명령을 더 신선하게 계산·전달하고 유효기간 안에 갱신하는 경로를 다뤄야 한다. TTL 연장·원 stamp 갱신으로 통과시키거나 한 숫자씩 반복 시험하지 않았다. 실제 차량 속도·관성·모터 제어를 연결하지 않았으므로 위 0 목표 시간을 물리 정지시간·정지거리·완주 손실로 환산하지 않는다.

## 공통 이관과 회수

[최소 2파일 패치](common_transfer/controller_budget.patch), [기준/후보 SHA](common_transfer/manifest.json)를 준비했다. 패치 SHA-256은 `1b01e42cb2e903111ea9d7044a3e7ea5beb3f8c90377e6f63ffd2f8a58cb2089`이다.

- `lidar_motion_ros.py`: `drain_pending(*, budget_s=.002)`로 기본값을 유지하면서 기존 소비 함수에 전달한다.
- `g1_g2_node.py`: control 직전 `drain_pending(budget_s=.006)` 한 호출을 지정한다. guard 호출은 기존 기본 2 ms이다.

생성 당시 기준 공통 소스에 patch 적용 가능 여부를 확인했다. **공통 소스에는 적용하지 않았다.** r520 동결 실행 종료 뒤 root가 r524의 같은 `g1_g2_node.py` 변경과 함께 통합해야 한다. 기준 SHA가 바뀌면 전체 후보 파일로 덮어쓰지 않고 최소 변경을 이관한다.

실제 시작 확인은 한 번 수행했다. main/controller/guard/ops 및 계산 자식 등 소유 PID는 11758/11774/11775/11776/11823/11824/11831/11837이었다. [시작 영수증](start_receipt.json). STOP service 성공·자식 정상 exit 0·강제 종료 0, **8 PID 모두 없음**, 작업 폴더 잔존 process 0, 원 소스 721파일·설치 234파일 불변을 확인했다. [회수 영수증](resource_return.json).

회수 ZIP 1,206,989 bytes의 SHA-256은 `ffe641cb2060f84920ad0f9f101179cb245cd2614f940f0f32061fdd707598d0`이며 원격/로컬 일치했다. 원시 수신/계측 기록·ZIP은 Git 제외다. Jetson 반환 뒤 차량 이동을 위한 root의 종료 요청을 받았으며 **새 SSH·원격 실행 없이 로컬 로그 분석만 마쳤다.** 본 보고서의 PID0은 반환 당시 확인이고 전원 종료 확인을 대신하지 않는다.
