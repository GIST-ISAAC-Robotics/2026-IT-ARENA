# r460 단독 완주 이후 배치·벽 형상 대표

**Windows 실행 연결·정적 기하 검사를 완료했습니다. 실제 ROS launch·빌드·주행은 하지 않았습니다.** r457 최신 기하·정책의 nominal actor0 전체 랩 성공과 자원 반환 뒤 주 담당 검토로 사용할 3개 대표입니다. nominal 미실행/실패이면 r460 실행도 보류합니다. 수치는 실측 오차 분포가 아닌 작은 환경 차이에 대한 시험 가정입니다.

| 조건 | 변경 | 유지 |
|---|---|---|
| `start_left20_yaw_plus3` | 기준 slot3에서 차량 기준 왼쪽20mm, yaw+3° | 코스·벽·시설·센서·제어·속도 |
| `start_right20_yaw_minus3` | 오른쪽20mm, yaw−3° | 동일 |
| `wall_surface_inward10` | `walls_main_left_1` 두께50→70mm: 주행면 쪽10mm 돌출과 외측10mm 확장 | 벽 중심/길이/높이·노면·시설·마찰·차량 |

원 scene/world와 초기 [정적 manifest](prepared/manifest.json), 이미 병합된 visual에 적용하려다 실패한 [원 기록](prepared/runtime_probe_initial_failure.json)은 보존했습니다. `prepared/wall_surface_inward10.sdf`는 초기 차이 확인용이며 최종 old610 실행 월드가 아닙니다.

## 연결과 유지한 기준

`run.py <case>` → r460 `course.launch.py`/`launch/*` → 메모리에 복제 로드한 공통 `simulation.launch.py` 순서입니다. 공통 파일과 r457 recipe는 수정하지 않습니다. `IT_ARENA_R460_VARIATION`은 artifact launch만 읽는 평가 조건이며 노드 파라미터로 전달하지 않습니다.

`variation_launch.runtime_wrapper`는 원 XML의 collision/visual 두 size를 먼저 바꾸고, 원 source.parent 기준으로 상대 URI를 절대화한 임시 사본을 기존 `_runtime_world_overrides`에 넘깁니다. 원 함수도 같은 URI 절대화를 수행하므로 임시 디렉터리로 기준 경로가 바뀌지 않습니다. 그 뒤 r457과 같은 중복 `tl_beam` 제거와 RTF .25를 적용합니다. 일반 `world_override`는 사용하지 않으며 기존 시설/road_level/마커 조합 검사를 해제하지 않습니다. 임시 raw 파일은 사용 후 정리되고 최종 runtime XML/OBJ는 기존 증거 경로에 보존됩니다.

spawn은 공통 모듈 사본의 `_launch_setup`에서 선택한 slot을 대입한 바로 다음 지역 변수 세 개만 바꿉니다. 임의 slot 재배치나 제어기에 위치/지도 정답 전달은 하지 않습니다. 평가기 STRtree는 launch 전에 만들어지므로 `connection_v1/<case>.sdf`의 동일 collision을 읽습니다. 종료 평가에서 실제 runtime 월드의 129개 정적 벽 polygon과 다시 대조하고 불일치는 harness 오류로 남깁니다. old610 시설3개 STL slab은 r457 그대로 추가합니다.

r457 runner/evaluator/finalize·시설·expected header는 사본을 재사용했습니다. 전체1랩/80sim초, 8sim초 무진행 시 소유 STOP, 접촉·노면·STOP·정상 종료 기준을 유지합니다. 원 false 판정이나 선행 STOP latch를 성공으로 바꾸지 않습니다. 조건별1회이며 실패를 원인 변화 없이 자동 반복하지 않습니다. 원 r457과 같은 C1/차량/물성/속도/정책을 사용하며 complete plan trace도 명시 ON을 유지합니다.

## 현재 검사와 한계

[정적 결과](static_v1/result.json), [결과 설명](RESULT.md), [연결 manifest](connection_v1/manifest.json), [복제 원 SHA](baseline_copy.json)를 참조합니다. Windows 표준 라이브러리로 실제 공통 XML 함수 및 r457 wrapper를 실행해 세 변형을 대조했고, evaluator의 원 rectangle/obstacle 함수를 Polygon 꼭짓점 반환 방식으로 검사했습니다. 이는 Shapely 공간 연산·실제 ROS 생성/실행·Gazebo 물리·센서 주행 검증이 아닙니다. Windows 생성 `static_v1/*.sdf` 및 OBJ는 비교 증거이며 Linux 실행 입력으로 사용하지 않습니다.

`connection_v1`은 원 상대 URI를 유지한 평가용 사본과 코드/입력 SHA입니다. 최종 실행 source ZIP과 `execution_prepared.json`은 아직 생성하지 않았습니다. 후속 nominal 성공·자원 반환 후 아래 import 확인과 동결이 필요합니다. 새 빌드는 이 변형의 필수 변경이 아니며 common/install은 r457 빌드와 일치해야 합니다.

## 후속 명령 — 현재 실행 보류

Windows 연결 무결성 확인은 저장소 루트에서 다음과 같습니다.

```powershell
& 'C:/Python314/python.exe' -B artifacts/validation/2026-11-13/solo_variations_r460_v1/prepare_execution.py --check
```

주 담당이 r457의 `passed/full_lap/shutdown_clean/입력불변/PID0`와 WSL 반환을 확인한 뒤, 배정된 담당이 bash에서 수행합니다. 이 문서는 새 사용자 재승인 절차를 요구하지 않습니다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
a='artifacts/validation/2026-11-13/solo_variations_r460_v1'
python3 -B "$a/runtime_preflight.py"
python3 -B "$a/prepare_execution.py" --freeze
```

preflight는 import/launch description/시설 polygon 연결만 확인하며 ROS init/spin/발행/Gazebo를 시작하지 않습니다. 주 담당은 조건별 `cases/<case>/root_execution_review.json`에 `root_reviewed=true`, `case`, `execution_limit=1`, `repeat_on_failure=false`, `wsl_exclusive_owner_confirmed=true`, `prepared_sha256`, `source_archive_sha256`를 기록합니다. 이 SHA는 `--freeze` 결과의 정확한 값을 씁니다.

```bash
bash "$a/execution/execute_once.sh" start_left20_yaw_plus3
# 별도 소유 모니터에서 같은 조건을 관측하고 원 기준으로 필요할 때 STOP1회:
python3 -B "$a/execution/observe.py" start_left20_yaw_plus3
bash "$a/execution/stop_once.sh" start_left20_yaw_plus3
# 정상 회수 후:
python3 -B "$a/execution/check_owned_processes.py" start_left20_yaw_plus3
```

두 다른 이름도 같은 명령에 조건명만 바꿉니다. 세 조건을 자동 연속/동시에 시작하는 루프는 없습니다. `run.py`는 STOP argv 일치를 위해 `-B` 없이 실행하며 bytecode는 환경변수로 차단합니다. 실행 로그/PID/exit·등록·정지·무결성·형상 대조는 각 `cases/<case>`에 분리합니다. 전체랩, 최장 정체/저속, 급감속, 벽·시설 겹침/노면, STOP·자식 종료를 각각 회수합니다. 대표3조건의 통과도 실차 오차 전체나 모든 배치의 안전 보장은 아닙니다.
