# r543 guard 상태 payload와 선택형 축약 검토

**비용을 줄일 근거가 있습니다. 공통 미반영 stage입니다.** r530 상태 1,015행 중 640행에서 최초 래치의 같은 61,410 B 좌표 자료를 반복했습니다. 이 자료만 별도 상세 파일에 한 번 보존하고 참조하는 후보에서, 상태와 상세 합계는 45.139 MB에서 6.341 MB로 **85.95%** 줄었습니다. 저장된 모든 상태를 원래 값으로 복원했습니다. 현재 제어 판단·기억·원시 기록·r539 동결 소스는 변경하지 않았습니다.

## 원본 크기와 생성 원인

원본 `field_wall_resource_drive_r530_v1/run1/safety_status.jsonl`은 45,190,271 B입니다. 이는 평가 주행12 sim초만 잘라낸 파일이 아니라 시작·STOP을 포함한 reference 0.02–20.32 sim초 전체 기록입니다. 행당 수집기 수신 시각/JSON 감싸기를 제외한 실제 `data` payload 합계는45,138,617 B입니다.

| 항목 | 저장 자료에서 확인한 값 |
|---|---:|
| 행 수 | 1,015 |
| 상태 payload 중앙값 / 최대 | 69,094 B / 82,620 B |
| 동일 correspondence_context가 들어 있는 행 | 640 |
| 서로 다른 correspondence_context | 1개, 61,410 B |
| 반복 context 전체 바이트 | 39,302,400 B, payload의87.07% |
| stage 상태 payload 합계 | 6,278,907 B |
| stage 상세 파일 | 61,618 B |
| 원 상태로 완전 복원 | 1,015/1,015행 |

`lidar_safety.py:125`의 제어 timer는20 ms이며 `control()`은 `/drive/safe`를 먼저 발행한 뒤 `lidar_safety.py:286`에서 상태 전체를 `json.dumps`하여 `/safety/status`에 발행합니다. 줄어드는 것은 상태 인코딩·전송·소비자 JSON parsing 비용입니다. 현재 콜백의 구동 발행은 인코딩보다 앞에 있으므로 감소분을 곧바로 같은 명령의 발행 지연 개선이라고 주장하지 않습니다. 다음 입력/콜백과 CPU 여유에는 영향을 줄 수 있으나 실측이 필요합니다.

`partial_observation.py:509`는 최초 래치에서 `.tolist()`로 독립적인 `latch_evidence.correspondence_context`를 만들고, `:173`은 같은 객체를 이후 report에 넣습니다. 현재 검색 범위의 쓰기는 새 캡처 또는 `None`으로 교체이며 내부 좌표를 갱신하지 않습니다. 대표 전체 latch 증거65,057 B 중61,410 B가 이 정적 좌표입니다. 동적 `retained_memory_command`, `known_structure`, anchor/clearance 진단도 존재하지만 이번 후보는 제거하거나 새로 요약하지 않았습니다.

## Windows 직렬화 비용

Python/OS 정보·원본 SHA·바이트 분포·상세 시간은 `measurement.json`에 있습니다. 각 대표에서10회 준비 후150회 측정했습니다. 아래는 인코딩/파싱 함수의 중앙값이며, ROS·DDS·네트워크·Jetson·제어 기하 비용을 포함하지 않습니다.

| 대표 | full → compact B | full dumps → compact encode ms | full loads → compact loads ms |
|---|---:|---:|---:|
| 작은 상태 | 222 → 556 | 0.0020 → 0.0045 | 0.0017 → 0.0030 |
| 중앙 크기 상태 | 69,094 → 8,180 | 1.2072 → 0.0673 | 0.7628 → 0.0456 |
| 최대 상태 | 82,620 → 21,706 | 1.3129 → 0.2029 | 0.8246 → 0.1508 |

중앙 크기 인코딩 p95는1.7941→0.0982 ms입니다. 작은 상태에는 메타데이터 비용이 추가됩니다. 최초 상세 캡처의 단1회 측정은 **5.8631 ms**였으며, 최초 JSON/SHA/queue 준비와 해당 상태 인코딩이 들어갑니다. 반복 비용 감소와 최초 비용 증가를 구분해야 합니다. 저장 JSON을 읽으면 매 행 새 객체가 되므로, 계측 전에 내용이 완전히 같은 context를 하나의 객체로 연결해 현행 최초-capture/불변 객체 계약을 재현했습니다. 이 준비 비용을 guard의 매 콜백 비용에 포함하지 않았습니다.

## 소비자와 호환 경계

| 소비자 | 실제 사용하는 범위와 판단 |
|---|---|
| 패널 `race_ops_node.py:257`, `race_ops_core.py:150/352` | JSON을 파싱하고 이유·swept clearance를 표시합니다. 상위 필드 그대로 유지하므로 UI 변경이 필요 없습니다. |
| 평가 actuator `scheduled_actuator.py:21–48` | `reason`과 수신 신선도를 사용하며100 ms timeout이 있습니다. **상태 발행 빈도를 줄이지 않습니다.** |
| planner | 조사한 `src`의 `/safety/status` 구독자는 패널과 평가 actuator뿐입니다. planner는 이 상세 상태를 제어 입력으로 받지 않습니다. |
| 교통/기본 평가 `validate_g1_traffic.py`, `validate_basic_autonomy.py`, `g1_partial_trial.py:56` | 상태/partial 진단을 복사해 기록하고 이유·unknown·관측 필드로 판정합니다. 그 필드는 유지하지만 원 좌표를 사용하는 추가 분석은 상세 복원이 필요합니다. |
| 기록·재생 `bag_contract.py`, `replay_observer.py`, sensor/fault 검증 도구 및 r530 수집기 | 전체 JSON을 저장하거나 재현합니다. 기본 full을 유지하거나 **compact 상태+상세 sidecar를 함께 회수하고 먼저 복원**해야 합니다. compact 파일만으로 기존 full 재현을 주장할 수 없습니다. |

## 최소 opt-in stage

`stage.py`의 `StatusEncoder`/`DetailArchive`와 `integration.patch`는 검토용입니다. 채택한다면 모듈을 `arena_autonomy/guard_status_payload.py`로 이관하고 `lidar_safety.py`에 초기화·인코더 호출·종료 회수만 연결하는 범위입니다. 기본 `full`은 기존 `json.dumps(status)`와 문자열까지 같으며 파일/worker를 만들지 않습니다.

명시 `compact_v1`만 다음처럼 동작합니다.

1. 정적 correspondence context를 최초 한 번 JSON/SHA로 고정해 기존 `QueuedLog`에 전달합니다. 새 절대 경로를 `x` 모드로 열어 원 기록을 덮어쓰지 않습니다.
2. 상세 writer의 write+flush가 확인된 뒤에만 좌표 필드를 `correspondence_context_ref`로 바꿉니다. **pending/failed/dropped이면 전체 좌표를 그대로 발행**합니다. 파일 열기 자체가 실패하면 명시 compact 구성을 시작하지 못합니다.
3. 원 내부 report/context는 변경하지 않고 새 얕은 dict만 만듭니다. latch 이유·시각·witnesses와 현재 속도·점유·제동·조향 진단은 보존합니다. STOP/명령 만료/epoch/UNKNOWN/기억 해제 조건에는 관여하지 않습니다.
4. 상태마다 별도 schema·archive ID·현재 저장 상태·기록 누락/실패/대기 수를 표시합니다. 상세 record ID/SHA로 회수하며 `load_details`/`restore_status`가 누락·변조를 거절합니다. 종료 시 writer를 회수합니다.

현재 소유자가 정적 context를 내부에서 변경하지 않는다는 코드 계약에 따라 객체 동일성으로 반복 인코딩을 피합니다. 향후 해당 객체를 제자리에서 변경하도록 소유 코드가 바뀌면 이 계약도 함께 바꿔야 합니다. file flush는 OS에 전달됐다는 뜻이며 전원 상실에 대한 fsync 보증은 아닙니다. 주기적으로 전체를 다시 보내는 keepalive나 별도 상세 ROS 토픽/서비스는 추가하지 않았습니다.

r524와의 연결은 같은 설치 선택 YAML에서 **각 노드에 별도로 명시**하면 됩니다. planner의 compact 선택만 보고 guard를 자동 변경하지 않습니다.

```yaml
local_pursuit:
  ros__parameters:
    g1_trace_detail: compact_v1
    # 기존 g1_trace_path 및 나머지 선택값 유지
lidar_safety:
  ros__parameters:
    safety_status_detail: compact_v1
    safety_status_evidence_path: /absolute/new-run/guard_status_contexts.jsonl
```

이 YAML은 제안이며 공통 설치 helper나 선택본에 반영하지 않았습니다. full 진단이 필요한 r539/기존 시험은 그대로 유지합니다. 채택 후 최초 ROS 연결에서는 신규 파라미터·상세 회수·기존 full 소비 계약만 확인하고, 실제 비용 개선은 Jetson 재허용 뒤 같은 입력/부하에서 판단해야 합니다.

## 검증과 산출물

- `test_stage.py`: 기본 full 문자열 동일성, 저장 대기/실패/누락의 전체 증거 유지, 동적0/만료 상태 보존, 객체 교체 시 새 캡처, 상세 write 확인/무결성·복원 등 **6개 통과,0.18 s** (`pytest.xml`).
- `analyze.py`: 저장1,015행 모든 JSON 값 복원 일치, 원 파일 SHA 불변, 대표3종 직렬화 비용. 추가 주행은 없습니다.
- `lidar_safety.before.py` / `lidar_safety.stage.py` / `integration.patch`: 현재 노드에 대한 작은 이관 후보, 구문 컴파일 통과. 실제 ROS 노드 실행 검증은 아닙니다.
- `source_manifest.json`: 읽은 소스와 stage SHA. `compact_statuses.jsonl`·`measured_contexts.jsonl`은 원본을 지우지 않은 새로운 파생 산출물입니다.

공통 파일·문서·r539 실행 입력 변경0, Jetson/WSL/서버/물리 명령0입니다. 파일 소유권은 이 r543 폴더에만 한정했습니다.

후속 검토: 상세 파일은 기존 실행별 고유 폴더의 새 절대 경로를 사용해야 합니다. 같은 경로로 노드를 재시작하면 x 모드가 기존 파일을 보존하며 거절합니다. 별도 `stage_integration/`에서 이 경우 경고와 full fallback을 연결하는 후보를 검토하며, 기존 공통 미반영 상태와 최초 계측/복원 근거는 유지합니다.
