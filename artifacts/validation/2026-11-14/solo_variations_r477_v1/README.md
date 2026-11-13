# r477 최소 환경 차이 두 조건 준비

**현재: 사용자 요청으로 첫 조건을 STOP·정상 회수하고 중지했다.** 최대41.96m·미완주, 접촉 표본0·입력 불변·소유 PID/partition0이며 WSL을 반환했다. 벽 변형은 미실행이다. 종료 원인과 재개점은 [PAUSED.md](PAUSED.md)를 따른다. 아래 준비 기록은 보존한다.

**후속 준비 완료:** r475 성공·회수 gate 확인 뒤 기존 Linux preflight와 source 동결을 exit0으로 마쳤다. 첫 `start_left20_yaw_plus3`의 root 검토 대기이며 주행은 시작하지 않았다. 최신 SHA·검토 경로·회수 범위는 [PREPARED.md](PREPARED.md)를 따른다. 아래 Windows 준비/미완료 문구는 최초 준비 당시 기록이다.

**Windows 준비만 완료했다. 실행 승인·최종 source 동결·WSL/ROS/Gazebo 실행은 하지 않았다.** r460의 기존 변형/평가/STOP 구현을 재사용하고 nominal gate를 최신 r475에 연결했다. r475가 미완료/실패이거나 회수가 끝나지 않으면 gate가 닫히며, 완료 후에도 root가 실제 시작 여부를 판단한다.

| 조건 | 변경 | 유지 |
|---|---|---|
| `start_left20_yaw_plus3` | slot3 기준 차량 왼쪽20 mm·yaw+3° | 벽·시설·기하·센서·속도·제어 |
| `wall_surface_inward10` | `walls_main_left_1` 두께50→70 mm, 주행면 쪽10 mm·외측10 mm 확장 | 벽 중심/길이/높이·노면·마찰·차량·정책 |

단 한 번의 nominal 결과에만 맞춘 배치 의존성을 최소 두 조건으로 살펴보는 준비다. 수치는 실측 오차 분포가 아니다. 오른쪽 시작 변형이나 새 플랫폼·새 판정은 추가하지 않았다. 차량/맵 정답은 평가와 Gazebo 배치에만 쓰며 제어 입력으로 보내지 않는다.

## 재사용과 확인 범위

- r475의 runner/evaluator/finalize·시설·header·기본 launch 등11파일을 byte-exact로 복사했다. config는 scope와 artifact launch 경로만 바꿨다. 명령은 runner/config/output 세 경로만 r477로 교체한다.
- r460의 원 XML 변형→기존 runtime override→old610 beam 처리/RTF .25 및 slot 선택 직후 spawn 지역변수 수정 구조를 그대로 사용한다. 새 r477 환경변수는 artifact launch만 읽는다.
- Windows 원 XML/AST·평가용 polygon 꼭짓점 검사12항목을 두 조건으로 실행했다. 시작 변형은 벽 polygon0개 변화, 벽 변형은1개 변화이며 runtime/evaluator 벽129개와 시설3개 연결이 같다. 원 r460의 Shapely/ROS 미검증 한계를 유지한다.
- r475의1,365개 동결 입력 전체와 source archive/prepared/선택 scene을 대조해1,368개 기준 입력을 보존한다. 공통·원 r460·r475에는 쓰지 않았다. source archive 재동결 전 추가 빌드는 예정하지 않으며, 현재 설치/import 일치 확인은 Linux preflight 후 root가 판단한다.
- 1랩 또는 최대80 sim초, 8 sim초 무진행 소유 STOP, 원 접촉/노면/STOP·정상 종료 기준과 complete plan trace를 유지한다. 조건별1회이며 실패 시 자동 재실행하지 않는다. 두 조건 자동 연속 실행 루프는 없다.

근거는 [정적 결과](static_v1/result.json), [연결 manifest](connection_v1/manifest.json), [기준 파일 계보](recipe_copy.json), [변경 diff](recipe_review.patch), [준비 결과](preparation_result.json)다. Windows가 생성한 `static_v1` 월드는 비교 증거이며 Linux 실제 실행 입력으로 사용하지 않는다.

## 닫힌 nominal gate와 필요한 root 필드

`nominal_success()`는 **r475**의 다음 원 파일/조건을 모두 요구한다. 없는 결과를 성공으로 채우거나 실패 판정을 수정하지 않는다.

1. `run1/report.json`: `passed=true`, `shutdown_clean=true`, `evaluation.gates.full_lap=true`, `evaluation.no_actor.facility_count=3`.
2. `integrity.json`: `changed_inputs=[]`, `archive_unchanged=true`.
3. `execution_v1/owned_processes_after.json`: `remaining_owned_work=[]`.

성공·회수 뒤 root가 WSL 단독권을 배정하면 기존 import/launch description/시설 연결 preflight와 최종 freeze를 수행한다. 그때 생성되는 `execution_prepared.json`/`execution_sources.zip`의 SHA를 조건별 `cases/<case>/root_execution_review.json`에 넣는다. 필요한 필드는 [템플릿](root_execution_review.template.json)의 `root_reviewed`, `case`, `execution_limit=1`, `repeat_on_failure=false`, `wsl_exclusive_owner_confirmed`, `prepared_sha256`, `source_archive_sha256`다. 템플릿의 false/null은 승인/실제 SHA가 아니며 지금 실제 review 파일은 만들지 않았다.

## 남은 단계와 고정 명령

Windows 연결 확인은 다음과 같다. 아래 명령은 **후속 사용용이며 본 준비에서 Linux 명령을 실행하지 않았다.**

```powershell
py -B artifacts/validation/2026-11-14/solo_variations_r477_v1/prepare_execution.py --check
```

root의 r475 성공·자원 회수 판단 후 배정된 담당이 Ubuntu-24.04에서 다음을 수행한다. preflight는 ROS import/launch description 생성만 하며 init/spin/발행/Gazebo 시작을 하지 않는다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
a='artifacts/validation/2026-11-14/solo_variations_r477_v1'
python3 -B "$a/runtime_preflight.py"
python3 -B "$a/prepare_execution.py" --freeze
```

실제 시작은 root가 결과와 freeze SHA를 검토한 뒤 조건별1회다. 아래 두 실행 명령은 선택 가능한 개별 명령이며 자동 연속 실행 지시가 아니다.

```bash
bash "$a/execution/execute_once.sh" start_left20_yaw_plus3
# 첫 조건 회수/결과 판단 뒤 별도 선택:
bash "$a/execution/execute_once.sh" wall_surface_inward10
```

소유 모니터와 STOP/회수 명령은 같은 조건 이름을 사용한다.

```bash
case_name='start_left20_yaw_plus3'
python3 -B "$a/execution/observe.py" "$case_name"
bash "$a/execution/stop_once.sh" "$case_name"
python3 -B "$a/execution/check_owned_processes.py" "$case_name"
```

관측/판정/STOP/정상 종료·소유 PID·입력 불변을 각각 회수한다. 원 r475 결과가 실패이면 실행을 시작하지 않고, 이 준비만 보존한다. 본 문서가 추가 주행 승인을 대신하지 않는다.
