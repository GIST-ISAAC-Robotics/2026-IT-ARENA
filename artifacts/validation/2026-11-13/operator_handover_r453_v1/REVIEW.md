# r453 조립 후 운영 인계의 필수 연결 점검

2026-11-13, GPT. 공통 소스가 r452 실행 준비로 동결된 동안 문서·CLI·관련 결과만 읽었다. 새 기능, 공통 수정, 시험, CLI 실행, WSL, Jetson, 장치/출력 접근은 하지 않았다. 아래 판단은 코드·문서 대조이며 실물 실행 성공의 주장이 아니다.

## 결론

기존 기능을 다시 구현할 필요는 없다. **필수 인계 연결 2건을 문서에서 보완**하면 된다. 최소 명령 조합은 [MINIMAL_HANDOVER.md](MINIMAL_HANDOVER.md)에 제안했다. 생성한 설치 파일이나 장착 값을 이번 작업에서 선택·작성하지 않았다.

### 1. 설치 상한과 estimated scope를 함께 선택하는 명령이 분산돼 있다

- `docs/autonomy/RACE_OPERATOR_RUNBOOK.md:13,63`과 `docs/hardware/PHYSICAL_HANDOVER_APPROVED_LAYOUT.md:150,168`은 r415/r439의 운영 명령과 r444 결과를 각각 연결한다.
- `installed_limit_chain_r439_v1/RESULT.md:57–83`의 실행 예제는 생성한 params-file/host 상한을 사용하지만 planner의 `g1_current_corner_budget`는 아직 `off`다. r444 결과7–9행은 새 scope의 선택 계약을 설명하지만 이를 설치 상한 명령과 합친 예제는 아니다.
- 현재 `crop_edge_selection.py:14–18,62–71`은 `hardware_runtime_candidate_v1`, `use_sim_time=false`, `native_estimated_v1`, `both`, `host_estimated_v1`, `motion_scan_timing_verified=false`를 요구한다. `runtime_node_backend.py:14–25`는 estimated scope와 corner=off 조합을 거절한다. r396만 단독 추가하거나 timing을 true로 바꾸는 것도 유효한 선택이 아니다.
- `local_pursuit.py:97–103`의 r446 입구 검사 때문에 일반 `local_pursuit` 실행 파일은 이 선택을 거절한다. 기존 `g1_g2_handoff_trial` 진입을 유지해야 한다.

**최소 보완:** r439의 같은 생성 YAML/host 배열을 유지하고, 명시 estimated 후보를 선택했을 때만 양쪽 노드에 scope/crop를, G1 노드에 r396을 함께 전달하는 명령을 한 곳에 적는다. 기본 비활성 선택도 그대로 남긴다. 생성 YAML을 사후 편집하면 r439 무결성 검사가 거절하므로, 문서 예제의 ROS override를 명시하거나 별도 기본 YAML을 먼저 선택하고 새 산출물을 생성한다. 이미 생성한 manifest를 고쳐 맞추는 방식은 제안하지 않는다.

### 2. 수동 단발의 로그 경로와 사건 내보내기 방법이 연결되지 않았다

- runbook51–53행의 수동 패널 명령에는 `log_path`, `manual_output_root`가 없다. `race_ops_node.py:159–178,228–232`는 빈 log_path에서 디스크 로그를 쓰지 않는다. 단발의 request/result/stdout은 별도로 `build/panel_manual` 아래에 남는다(`race_ops_manual.py:23,141–155,210–219`, `development_jog.py:210,277`). **단발 증거가 전혀 없다는 뜻은 아니다.** 기존 수집 경로가 한 묶음으로 안내되지 않은 문제다.
- 수동 구성에는 자율 planner가 없으므로 정상적으로 `/autonomy/status`도 없다. `race_ops_core.py:107–109`의 사건 `sim_s`는 이때 null이다. runbook34행에 안내한 `race_event_export.py --request ...`는 null이 아닌 sim_s를 요구하므로 수동 사건에 그대로 사용할 수 없다(`scripts/race_event_export.py:13–17`). 요청을 생략한 전체 export는 이 조건을 요구하지 않고 wall_s를 보존한다.
- 자율 구성은 이미 상태/이유/거동/기준 측방/큰 목표속도 감소, 보호층 이유를 기록한다(`race_ops_core.py:118–156`). 기존 `--decisions`는 판단 trace를 함께 읽는다(`scripts/race_event_export.py:36,50–51`). 이는 사고 자동 판정 또는 실제 충돌/정지 증명이 아니며, 새 로깅체계를 만들 이유는 없다.

**최소 보완:** 수동 명령에 `log_path`와 `manual_output_root`를 명시해 host·패널·jog를 같은 RUN_OUTPUT 아래 두고, 수동은 `--request` 없는 전체 export와 jog 원 request/result/stdout을 함께 회수하도록 적는다. 자율은 기존 요청 중심 export에 `--decisions`를 연결한다. host `events.jsonl`, `summary.json`, `registered.json`, 원 UART 자료는 같은 host 하위 폴더에서 보존한다. 패널 표만으로 MCU STOP ACK·차체 실제 정지·명령값/실측값을 합치지 않는다.

## 유지할 완료 항목과 사람에게 남는 입력

- r429 패널 속도 상한/양방향 단위/별도 수동 준비→pulse→STOP, r434 기본 OFF 미리보기, r439 설치 상한 생성, r444 estimated 입력 계약, r446 일반 planner의 잘못된 입구 거절은 이미 구현·이관 근거가 있다. r429/r434 결과의 당시 stage 표현을 공통 미구현으로 오독하지 않았다. 각 integration.json과 현재 파일을 대조했다. 기존 검증을 반복하지 않았다.
- 자율의 일반 STOP→START와 정지를 유지하는 RESET, 최초 WAIT_GREEN의 수동 출발 계약에는 이번 범위에서 새 필수 누락을 찾지 못했다. 수동 구성은 START가 아니라 매 단발 전 명시 준비가 필요하고, 자율 publisher를 켠 채 수동을 사용하는 구성은 기존 코드가 거절한다.
- `PHYSICAL_HANDOVER_APPROVED_LAYOUT.md:87–91`은 설치 앱/포트와 측정 영점·좌우 한계/방향·모터/엔코더·출력 설정을 조립 후 입력으로 남긴다. `runtime_installed_profile.py:63–74`는 명시 board/servo 필드와 assembly ID를 요구하며 누락값을 채우지 않는다. build/verify가 물리 교정을 확인하는 것은 아니다.
- 현재 r401의 기하, 센서 위치/축, gyro bias=0, wheel scale=1, 부호·기어비/반경은 선택 설계/가정과 실제 확인을 구분해야 한다. 이번에는 이 값을 교정값으로 바꾸거나 합성 영점/profile을 만들지 않았다. clock policy JSON과 motor feedback JSON도 사람이 확인한 실제 파일을 지정하는 입력으로 남긴다.
- r439는 **설치 조향 상한**을 연결한다. 속도 상한은 기존 운영/host/firmware 선택과 별개이며 r401의1.5m/s를 실차 확인 속도로 취급하지 않는다. 실장 profile의 속도·출력 값이 아직 정해지지 않았다면 값 생성이나 ARM 준비를 완료했다고 표시하지 않는다.

## 검토 경계

Windows 파일 읽기와 이 폴더의 문서/소스 hash 기록만 수행했다. 실행 가능한 파서/ROS 인자와 기존 분기를 읽어 조합했으나 Bash/ROS/HTTP/센서/구동을 실행하지 않았다. 본 문서가 r429의 가짜 MCU 대표나 r444의 무입력 ROS 생성자 결과를 새 통합 실행으로 승격하지 않는다. 새 주행·장치 재검증은 제안하지 않으며 주 담당의 문서 통합만 남긴다.
