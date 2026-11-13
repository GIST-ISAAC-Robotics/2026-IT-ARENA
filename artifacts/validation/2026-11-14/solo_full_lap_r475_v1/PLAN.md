# r475 단독 전체랩1회 실행 준비

**현재는 Windows의 runner/계획 준비만 완료했다.** r474 최종 집중 회귀와 저장 입력 확인이 진행 중이므로 공통 source scope·ZIP·`prepared.json`을 만들지 않았다. WSL/Jetson·빌드·ROS·Gazebo 실행도 없다. 최종 r474 결과와 WSL 소유권을 주 담당에게 인계받은 뒤에만 아래 준비/동결/단독1회를 진행한다.

## 목표와 동일 조건

목표는 r469가9.90 m에서 멈춘 추가 불확실성 정지를 r474의 현재 경로 우선 판단으로 해소하고 **원 전체랩 평가까지 마치는지** 확인하는 것이다. 9.90 m 통과만으로 완주 성공을 기록하지 않는다. r466 마커v2와 r470 native는 최신 공통 코드에 포함하되 **`vehicle_marker_observer_enabled=False`**를 유지한다. 실제 차량 후방 마커의 기존 모델 기하를 제거한다는 뜻이 아니다.

r469의 runner·평가기·finalize·시설/기하·기준 header·launch recipe13개는 byte exact로 복사했다. config는 scope 설명과 이 artifact의 launch 경로만 변경했다. 아래 조건과 원 판정은 그대로다.

- actor0, old610 시설3개, grid slot3, 기존 단독 속도/경로 옵션.
- 차량/기하·C1/B0390·물성·독립 보호·`/drive`→보호층→`/drive/safe` 경계.
- RTF .25, 최대80 sim초, **8 sim초 무진행 또는 원 접촉 기준에서 소유 STOP1회**, 기존 STOP 확인·정상 종료.
- 원 plan/current/부모 발행 추적·header 대조·원 실패 판정·입력 무결성·소유 PID 회수.
- **전체랩1회만 실행**, 같은 조건으로 성공할 때까지 자동 반복하지 않는다. r469 원 실패/로그/PID/실행 영수증은 복사하지 않았다.

`config.json`에 있는 예전 speed_profile 설명문도 내용 수정 없이 보존했다. 실제 설정은 r469의 고정 argv/launch 옵션과 대조하며 이 준비에서 속도를 변경하지 않는다.

## 준비한 변경

경로·실행 ID·baseline을 r469→r475로 바꿨고, 원 수집/동결/실행 구조를 재사용했다. source delta의 과거 고정5파일 조건은 제거하고 **최종 r469 대비 실제 delta 전체를 `root_source_review.json.approved_delta`와 exact로 대조**하게 했다. 미검토 변경이 있으면 동결 전에 중단한다. 최종 검토에는 `r474_final_checks_passed=true`, `vehicle_marker_observer_enabled=false`, 관련 검증/통합 근거 SHA가 필요하다.

native 확인은 과거 r469 binary와의 동일성을 요구하지 않는다. 공통에 이미 반영된 r470의 **검토된 현재 native SHA와 새 `current_mask_occupied` entrypoint**를 실제 import로 확인한다. marker observer는 r469 argv에서 선택하지 않으며 현재 선언 기본값이False임을 Windows AST로 확인했다. 현재 source를 동결했다는 의미는 아니다.

[원 recipe 대조](recipe_manifest.json), [경로/검토 변경 diff](recipe_review.patch), [복사 출처](baseline_copy.json)를 남겼다. Windows 정적 확인 결과는13개 byte exact, Python AST20개, recipe28파일이며 빌드/정책/주행 성공을 뜻하지 않는다.

## r474 최종 확인·WSL 반환 뒤의 순서

1. 주 담당에게 r474 최종 회귀/저장 입력 결과와 WSL 자원을 인계받는다. 그 뒤 `prepare.py --scope`로 **그 시점의** 공통 delta를 수집하고, 전부 검토한 `root_source_review.json`에 실제 근거 hash를 기록한다. 현재는 두 파일 모두 없다.
2. 기존 준비 wrapper로 `arena_autonomy`1패키지 빌드, 실제 import/native 검사, 고정 CLI의 ROS init 직전 차단, 소스/실행 입력 동결을 수행한다. r474 관련 회귀를 이유 없이 다시 전수 실행하지 않는다. 빌드가 검토된 공통 입력을 바꾸면 동결을 강행하지 않는다.
3. 동결 hash와 단독1회 범위를 확인한 `root_execution_review.json`을 기록하고 고정 실행 wrapper를 한 번 호출한다. 원 `registered.json`/`run1`/`run.pid` 존재 검사는 중복 실행을 막는다.
4. 기존 `observe.py`로 실제 진행과 원 STOP 조건만 관측한다. 필요하면 기존 `stop_once.sh`를 한 번 호출하고 원 evaluator의 STOP/종료를 기다린다. 실패를 성공으로 바꾸거나 강제 종료를 정상 종료로 표기하지 않는다.
5. 원 report/trace, source hash, 소유 process group/PID, WSL 반환을 회수한다. r474로9.90 m를 통과했는지와 전체랩 결과·명령 만료·접촉·STOP을 분리해 기록한다.

준비·실행 명령은 아래와 같으며 **이번 준비 작업에서 호출하지 않았다.**

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_full_lap_r475_v1/build_v1/prepare_once.sh'
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_full_lap_r475_v1/execution_v1/execute_once.sh'
```

마커 보조 기능의 성능, 다른 출발/벽 조건, 교통 대표, Jetson 시간, 실차 주행은 이번1회에 합산하지 않는다. 원 r469의 실패와 r474 저장 입력 결과도 새 폐루프의 성공으로 소급하지 않는다.
