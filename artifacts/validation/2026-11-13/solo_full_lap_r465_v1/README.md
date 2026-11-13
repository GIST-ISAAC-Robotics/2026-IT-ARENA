# r465 최신 정책 통합 뒤 actor0 단독1회 준비

**고정1회 실행은9.54 m 정체로 미완주했고 소유 STOP·자식14개 종료·잔여PID0·종료 시점1,348개 입력 불변을 회수했습니다.** [결과](RESULT.md)의 원 report false/run exit1을 보존하며 재실행하지 않습니다. r457의23.35m 미완주와 원 source/report/실패·재시도 기록도 원 폴더에 그대로 보존합니다.

## 같은 조건과 재사용

- actor0, old610 신호·ID20·ID30 시설3개, Contact plugin.
- 최종 CAD vehicle207×150mm 보수 외곽/물리199.7mm, wheelbase128mm·track123.9mm, C1 전방배선6mm·native10Hz/sequential/both, 동일 자기 마스크·RGB/IMU/엔코더.
- `g1_open_1p5`, control50Hz, RTF .25, physics step1ms, road_level, marker blank20/30, slot3(x9.89/y3.52/yaw1.5708), red1초.
- 전체 lap44.64m·최대80sim초, 최초8sim초 무진행 소유 STOP, 기존 접촉/노면 중단, 실제 STOP/MCU/기한진단·정상 종료·입력 무결성 기준. 이동 후 .05m/s 미만 연속2.5sim초의 기존 통과 기준도 그대로입니다. 랩 진행과 전체 판정은 분리합니다.
- 기존 명시 controller/runtime 선택과 complete plan trace ON, `/drive`→독립 보호→`/drive/safe` 경계를 유지합니다. 방향/교통/속도증가/추가 물성 변경은 없습니다.

`solo_runner.py`, `solo_evaluation.py`, `solo_finalize.py`, 시설·기하·header·launch helper 등13개는 r457 byte exact 사본입니다. `run.py`는 자기 경로·표시·baseline 기록만 변경했고 `config.json`도 scope와 artifact 진입 경로만 바꿨습니다. STOP/모니터/회수 스크립트는 소유 경로·태그만 바꿨습니다. r457의 원 run1, 등록·PID·receipt, 실패로그와 재시도 wrapper는 복사하지 않았습니다. 새 `build_v1`/`execution_v1`에는 코드만 있습니다. [최초 복제 SHA](baseline_copy.json)의 draft SHA는 최초 복제 시점 기록이며, 최종 recipe SHA는 [recipe manifest](recipe_manifest.json)를 따릅니다.

## 후속 통합과 exact source 검토

통합 대상은 r463의4모듈/신규 시험, r464 `local_follow.py`, r461 `race_ops_page.py` 표시 변경의7파일입니다. [실제 source scope](source_scope.json)와 [주 담당 검토](root_source_review.json)가 전체 before/after SHA를 기록합니다.

통합 완료 뒤 주 담당이 아래 명령으로 **r457 동결 source 대비 실제 모든 변경/신규/삭제 SHA**를 기록했습니다. 동결된 현재 자료에 다시 실행하지 않습니다.

```powershell
& 'C:/Python314/python.exe' -B artifacts/validation/2026-11-13/solo_full_lap_r465_v1/prepare.py --scope
```

주 담당이 `root_source_review.json`을 작성합니다. 필수 스키마:

```json
{
  "root_reviewed": true,
  "source_scope_sha256": "source_scope.json의 실제 SHA256",
  "baseline_archive_sha256": "source_scope.json의 baseline_archive_sha256",
  "approved_delta": [
    {"path": "실제 상대 경로", "before": "원 SHA 또는 null", "after": "현재 SHA 또는 null"}
  ],
  "geometry_sensor_speed_physics_unchanged": true,
  "integration_evidence": {
    "artifacts/validation/2026-11-13/관련 작업/integration.json": "해당 파일 실제 SHA256"
  }
}
```

`approved_delta`는 source_scope의 delta 배열과 경로·순서·before/after까지 exact여야 합니다. 검토 근거로 r463/r464/r461의 최종 통합 영수증을 실제 경로로 넣습니다. 임의 기본값/작은 숫자 목록으로 다른 delta를 숨기지 않습니다. common 수집 범위는 r457과 같고 기존 frozen config/시설을 유지합니다. `--verify-source`가 이 목록과 현재 source를 재대조합니다.

## 빌드·준비·동결 — 완료

WSL 소유권과 준비 실행 지시를 받은 담당이 다음 고정 wrapper를1회 수행해 exit0으로 회수했습니다. 아래는 실행 기록이며 반복 명령이 아닙니다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/solo_full_lap_r465_v1/build_v1/prepare_once.sh'
```

`arena_autonomy`1회 colcon build → 실제 설치/import 경로와 변경 모듈 SHA/native ABI → 같은 고정 CLI를 `rclpy.init` 직전에 차단 → source archive/manifest 동결 → source 재검사·자기 프로세스 회수입니다. 기존 byte exact evaluator/helper 회귀를 새로 전수 반복하지 않습니다. 새 정책 자체 회귀는 r463/r464가 담당합니다. native 원소스 변경이 없는 범위라 기존 native SHA도 r457과 대조하며, 예상치 않은 변화는 통합/빌드 원인을 먼저 확인합니다.

`prepared.json`, `sources.zip`, `build_v1/receipt.json`을 생성했고 동결 후1,348개 입력 재대조를 통과했습니다. ROS init/Gazebo 없음, 소유 PID0을 회수했습니다. 실행 전 source가 달라지면 같은 목록으로 계속 진행하지 않습니다. 실패는 원 로그를 보존하고 원인 변화 없는 재실행을 하지 않습니다. WSL E_UNEXPECTED는 실제 시작 여부/등록/PID를 먼저 확인하며 이전 r457 실패를 이번 실행 결과로 복사하지 않습니다.

## 고정1회 실행·회수 — 완료, 반복 금지

주 담당 최종 검토의 `root_execution_review.json`은 `root_reviewed=true`, `execution_limit=1`, `repeat_on_failure=false`, `prepared_sha256`, `source_archive_sha256`를 포함합니다. 해당 검토와 실행 지시/WSL 단독 배정 뒤 아래 명령을1회 수행했습니다. 아래는 실행 기록이며 다시 실행하지 않습니다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/solo_full_lap_r465_v1/execution_v1/execute_once.sh'
```

관측은 같은 폴더 `execution_v1/observe.py`, 필요 시 STOP1회는 `execution_v1/stop_once.sh`, 종료 회수는 `execution_v1/check_owned_processes.py`입니다. `run.py`는 STOP 소유 argv와 맞춰 `python3 <상대 run.py>`로 실행하고 bytecode는 환경변수로 차단합니다. source/import 검사에는 `-B`를 사용합니다.

원 report false/선행 STOP latch/시작 실패를 보존합니다. 진행거리·정체 구간·정상 가속 복귀·부모 양수 발행·guard/MCU·완전 제출/완료/footer 오류·STOP·자식exit/PID/입력 불변을 분리해 회수합니다. r460 변형 주행은 nominal 완주 전 계속 보류합니다.

## 현재 확인

[recipe 체크](recipe_check.log)는 Windows Python exit0입니다.13개 exact 사본, config/CLI의 자기 경로 외 값 불변, 시설3개, 소유 argv, Python19파일 AST, bash LF를 확인했습니다. 실시간/native/ROS/주행 검증으로 합산하지 않습니다. [준비 결과](RESULT.md)를 참조합니다.
