# r543 선택형 상태 축약의 노드 연결 stage

**공통 미반영입니다.** 실제 `lidar_safety.control()`의 stage 사본과 새 `guard_status_payload.py`를 연결했으며 Windows 계약 검사16개가0.78초에 통과했습니다. 원 r543의 바이트 감소·최초 캡처 비용·1,015행 복원 근거는 그대로 유지하고 다시 계측하지 않았습니다. 채택/새 상세 옵션의 활성화는 root의 후속 판단입니다.

## 파일과 연결

- `guard_status_payload.py`: 기존 `QueuedLog`를 쓰는 상세 보존/복원과, 인코더 오류를 기존 full JSON으로 되돌리는 작은 wrapper입니다.
- `lidar_safety.pystage`: 현재 공통 노드를 복사한 뒤 초기화·상태 직렬화·종료 회수만 연결했습니다. 상태 발행 직전까지 실제 `control()` AST가 baseline과 완전히 같습니다.
- `lidar_safety.before.py`, `integration.patch`, `manifest.json`: baseline·최소 패치·SHA 근거입니다. 실제 이관한다면 helper 목적지는 `src/arena_autonomy/arena_autonomy/guard_status_payload.py`입니다.
- `test_integration.py`, `pytest.xml`: 실제 before/stage control 메서드를 실행한 비ROS 계약 검사입니다.

기본 `safety_status_detail=full`은 factory에서 `None`을 반환하고 기존 `json.dumps(status)`를 그대로 호출합니다. 상세 경로를 검사하거나 파일·writer thread를 만들지 않으며 상태 필드/문자열·발행주기·구동 출력도 바꾸지 않습니다.

`compact_v1`만 `safety_status_evidence_path`를 사용합니다. 기존 실행별 고유 폴더의 **새 절대 경로**를 전달해야 합니다. 기존 파일을 x 모드로 열 수 없거나 경로/옵션이 유효하지 않으면 한 번 경고하고 full로 시작합니다. 같은 path 재시작으로 노드를 죽이거나 원파일을 덮어쓰지 않습니다. 새 경로를 임의로 추정해서 만들지도 않습니다.

런타임 인코더 예외·상세 저장 failed/dropped/missing은 한 번 경고하고 해당 인스턴스를 full 경로로 유지합니다. 상세 write+flush가 아직 확인되지 않은 pending 상태도 전체 좌표를 발행합니다. 기존 구동 발행은 상태 인코딩보다 앞에 있고, 이번 wrapper는 `/drive/safe`·STOP·현재 점유·시각·기억 판단을 변경하지 않습니다. 경고 출력 자체의 예외도 구동 콜백으로 전달하지 않습니다.

## 집중 검사

| 대표 | 확인 결과 |
|---|---|
| 기본 full | archive 생성 함수를 호출하지 않음, 새 파일0. 실제 control 양수/0/만료/입력누락/장애물 대표에서 원 상태 JSON 문자열과 완전히 동일 |
| 명시 compact | 같은5대표에서 `/drive/safe` 속도·조향·원 stamp·frame 동일. 상세 복원 뒤 모든 동적 상태 키/값 동일 |
| 같은 파일 재시작 | 기존 상세 파일 바이트 보존, 경고1회, factory가 full 경로를 반환 |
| 빈/상대 경로 | 파일 생성 없이 경고와 full fallback |
| 인코더 실행 예외 | 실제 control의 양수와 다음0명령 유지, full JSON 동일, 반복 경고 없음 |
| 비동기 디스크 쓰기 실패 | 기존 queue worker에서 실패를 기록한 뒤 실제 control 양수 유지·full JSON 동일·다음0명령 유지·경고1회 |

기존 `test_independent_guard_route.node`의 ROS 대역을 재사용했습니다. projection·관측 감사·swept limit·제동·명령 신선도와 출력은 실제 control 본문을 실행합니다. 압축할 latch 상세의 수명만 재현하도록 partial policy의 반환값을 작은 fixture로 고정했으므로, 새 기억/경로 알고리즘의 수학적 검증이나 ROS 노드 기동 검증은 아닙니다. 전역 NumPy 대체나 공통 monkeypatch는 하지 않았습니다.

```powershell
$env:PYTHONPATH='src/arena_autonomy;src/arena_vehicle_interface;tests;scripts;artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'
$env:PYTHONIOENCODING='utf-8'
python -m pytest artifacts/validation/2026-11-14/guard_status_payload_r543_v1/stage_integration/test_integration.py -q --tb=short -p no:cacheprovider --junitxml=artifacts/validation/2026-11-14/guard_status_payload_r543_v1/stage_integration/pytest.xml
```

## 유지하는 한계

원 r543의 반복 인코딩 감소는 이전 Windows 표본의 측정이며, 이번 예외 wrapper를 포함한 실제 Jetson/ROS 지연을 새로 측정한 결과가 아닙니다. 최초 캡처5.8631 ms 단일 측정과 파일 flush≠전원 상실 fsync 보증이라는 한계도 유지합니다. compact 기록은 상세 sidecar와 함께 회수하고 복원해야 합니다. 실패 후에는 자동으로 compact를 재시도하지 않으며, 다음 실행에서 새 run 경로를 명시합니다.

공통 코드·설정·r544 실행 입력·장치 변경0, WSL/Jetson/네트워크 서버 실행0입니다. 원 r543 전체 로그 복원·성능 시험을 반복하지 않았고 새로운 주행 필수 조건도 추가하지 않았습니다.
