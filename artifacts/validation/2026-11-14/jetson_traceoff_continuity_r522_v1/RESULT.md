# r522 — 상세 trace OFF 단1회 대조

**상세 trace를 분리하자 active 양수 명령의 최장 공백은 1.326→0.604초로 줄었지만, 연속 실행 문제를 모두 해결하지는 못했다.** 기록 OFF에서도 completion-notice를 통해 직접 호출된 control이 85.22 ms 걸렸고, 바로 뒤 gyro backlog로 controller `motion_stale` 2회가 남았다. 상세 trace가 부하 요인이지만 유일한 원인이라고 판정하지 않는다. 추가 시험은 하지 않았고 Jetson 독점권을 반환했다.

## 변경·고정 범위

[r508 최종 SHM16+drain+proof stage](../jetson_input_continuity_r508_v1/RESULT.md)의 동결 파일 12개를 대조했다. **11개는 바이트 동일**하고 `node_runner.py`의 trace 생성 wrapper만 바꿨으며, `trace_off_sink.py`를 추가했다. [복사 감사](source_copy_audit.json), [전체 manifest](manifest.json).

- 원 설치 `/home/q/it-arena-field-r500-20261114-a`의 소스 ZIP SHA `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`. 원 소스 721파일·설치 Python/공유라이브러리 234파일 불변이다.
- 같은 원 입력·원 stamp/epoch/frame, native simulation backend, 동일 runtime/guard/RaceOps, 합성 RGB 848×480/20 Hz와 실제 detector, buffered timing·제어 receipt를 유지했다. ROS domain 158와 `/replay_r508` 이름도 그대로 사용하여 불필요한 계약 치환을 피했다. SHM 단독 16 MiB는 4개 실제 participant에서 실행별로 확인했고 global 설정은 바꾸지 않았다.
- **r518 fit·r521 벽 재획득·최신 공통 설치본은 포함하지 않는다.** 물리 센서/포트/host/MCU/출력, 실제 영상, B3 wait/start, 폐루프 실차 검증이 아니다. 이 결과로 최신 전체 실차 또는 10 Hz 완성을 주장하지 않는다.

초기 계획은 `g1_trace_path=''`만 주는 것이었으나 코드 확인에서 현재 선택 구성의 short-prefix/parallel-hold/G1G2 생성자가 절대 trace 경로를 필수로 요구함을 발견했다. **빈 경로의 실패 실행은 하지 않았다.** 실제 stage는 원 생성자의 경로·모드·구성 검증을 그대로 수행한 뒤 원 writer를 닫고, 상세 payload를 순회·freeze·직렬화·저장하지 않는 시험용 sink로 교체했다. 경로 등록과 시작 header/footer **4,252 bytes**는 남는다. 실행 종료 후에도 파일 크기가 그대로임을 확인했다.

제어용 입력 snapshot/IPC freeze, 센서 시각 검증, 현재 경로 검사, 상태·명령·STOP 기록은 유지했다. `status_detail` 등 제어/상태 경로의 freeze까지 제거한 시험이 아니다. sink는 사건/폐기 사유의 작은 카운터만 기록한다. source·TTL·보호 조건·네트워크 기본값·전력·패키지를 수정하지 않았다.

## 결과

선택 구간은 20초 `[7.62,27.62)`이고, 마지막 원 gyro/wheel은 27.515/27.514초다. 이전 시험과 같은 **원 입력이 계속 존재하는 `[7.62,27.52)`의 19.9초**를 active 비교로 사용한다. EOF 입력 소진은 별도 집계한다. 각 조건은 1회이며, 실행 경로·계산 건수가 달라 통계적 성능 보장으로 확대하지 않는다. [비교 집계](comparison.json), [집계 코드](compare.py).

| active 지표 | r508 trace ON | r522 상세 trace OFF |
|---|---:|---:|
| `/drive` 0 / 양수 발행 | 17 / 161 | **7 / 163** |
| `/drive/safe` 0 / 양수 발행 | 213 / 765 | **118 / 833** |
| 모든 `/drive` 사이 최대 공백 | 242.91 ms | **185.11 ms** |
| 양수 `/drive` 사이 p95 / 최대 공백 | 131.90 / 1326.28 ms | **147.38 / 603.54 ms** |
| 양수 `/drive/safe` 사이 최대 공백 | 1388.15 ms | **501.21 ms** |
| 양수 `/drive` 원 명령 나이 p95 / 최대 | 114.23 / 161.77 ms | **114.80 / 121.28 ms** |
| 양수 원 명령 나이 200 ms 초과 | 0 | **0** |
| controller motion_stale 호출 | 3 | **2** |
| guard motion_stale 호출 | 0 | **0** |
| guard command stale / stale_input 호출 | 128 / 128 | **81 / 81** |
| 모든 control 호출 시간 p95 / 최대 | 28.83 / 104.07 ms | **20.72 / 85.22 ms** |

명령 수는 실제 차량 이동량·정지 사건 수가 아니다. `/drive/safe` header는 guard의 새 발행 시각이므로 그 나이를 원 명령의 종단 지연으로 사용하지 않는다. timing timer만 보면 OFF 최대가 **28.08 ms**지만, completion-notice의 직접 control 호출이 빠진 값이다. 위 표는 그 호출도 포함하는 별도 receipt를 사용했다.

수신한 계산 완료 항목의 비용은 다음과 같다. 이전 세대에서 회수 전에 폐기된 모든 작업을 포함한 전수 비용은 아니다.

| 완료 비용 | trace ON | 상세 trace OFF |
|---|---:|---:|
| plan 건수 / 중앙 / p95 / 최대 | 158 / 68.95 / 92.52 / 103.59 ms | **166 / 82.04 / 114.14 / 129.88 ms** |
| current 건수 / 중앙 / p95 / 최대 | 169 / 16.51 / 21.51 / 34.27 ms | **259 / 16.75 / 21.05 / 42.80 ms** |
| plan 제출→회수 p95 | 104.05 ms | **116.13 ms** |
| current 제출→회수 p95 | 34.21 ms | **30.59 ms** |

OFF에서 current 완료가 169→259로 늘어 부하와 실행 순서가 같지 않다. plan p95는 오히려 100 ms를 넘으며, 상세 기록을 끄면 모든 단계가 빨라진다는 결론은 성립하지 않는다.

## 잔여 정지와 입력 무결성

최장 양수 공백은 **22.004→22.608초**로 active 구간이다. 직전 ROS reference 22.058초의 직접 control 호출이 **85.22 ms**였고, 22.149/22.163초의 2 ms drain 이후에도 gyro 원 stamp가 각각 22.085/22.120초로 남았다. 실제 나이 약 63.69/42.51 ms가 기존 30 ms 한도를 넘어 정지했다. 시각을 갱신하거나 TTL을 늘리지 않았다.

이 구간의 발행 지연 최대는 약 4.44 ms이며 인근 HTTP 요청은 3.72 ms라, 원 입력 공백이나 동기 HTTP 대기로 85 ms 정체를 설명할 근거는 없다. 전체 발행 지연 최대는 scan 5.03 ms, gyro 16.85 ms, wheel 17.39 ms이다. 무엇이 직접 control의 잔여 85 ms를 차지했는지는 이 시험에서 세분하지 않았다.

- 두 노드 모두 gyro **4,381/4,381**, wheel **2,191/2,191**, rejected **0**. timing drop **0**.
- 시작 구간: controller SENSOR_STOP 101회, guard missing_input 101회. 첫 scan 전 accepted gyro/drive 관문과 전체 frame/topic/time 계약 검사는 통과했다.
- active 구간: controller SENSOR_STOP 1, DECISION_PENDING 44, HANDOFF_FOLLOW 4, RUNNING 1,211, MOTION_STOP 2. guard missing_input 1, clear 870, stale_input 81(command stale 81), motion_stale 0.
- 입력 소진/EOF 구간: controller MOTION_STOP 16, SENSOR_STOP 38, DISABLED 16, RUNNING 2. guard motion_stale 9, stale_input 61, clear 1이다. 이 구간의 command_stale 표시는 0이며 scan 수명 검사에서 먼저 거절된다. 이를 active 연속성 실패와 합치지 않는다.
- lightweight sink의 discarded-result reason 카운터는 0이다. 실제 worker 내부 parallel discard 47회와는 별개다.

## 회수

실제 시작 PID 확인은 한 번만 수행했다. main 11369, controller 11396, guard 11397, ops 11398, resource tracker/vision/계산 자식 11445/11446/11454/11460이다. [시작 영수증](start_receipt.json).

STOP service 성공, 자식 exit 0, 강제 종료 0, 위 **8 PID 모두 없음**, 작업 폴더 잔존 process 0, 고정 소스/설치 SHA 불변, 물리 포트·출력 0을 확인했다. [자원 반환](resource_return.json). Jetson 독점권을 반환했고 추가 실행·새 태스크·공통 수정은 하지 않았다.

회수 ZIP은 **1,198,455 bytes**, SHA-256 `fd5cfef6f050ce77373e98a4e82ae2df734a97a70afa43f012eb0701fc78a264`로 원격/로컬 일치한다. 원시 수신/계측 기록과 ZIP은 Git 제외다. 상세 로그를 가볍게 분리하는 운영 모드는 후속 설계 대상이며, 현재 운영 기본이 이미 trace OFF라고 가정하지 않는다.
