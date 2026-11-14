# r517 업데이트 레시피 READY — 최종 소스 고정·설치 지시 대기

2026-11-14. **준비만 완료했다. 새 ZIP0·공통 수정0·Jetson/WSL/장치 접근0.** r514→r515 및 후속 제어 변경의 최종 선택을 root가 전달하기 전에는 아래 생성/전송/설치 명령을 실행하지 않는다. 별도 설치 체계를 만들지 않고 r491/r507 수집과 r500의 새 독립 루트·ROS overlay·기존 ARM 의존성 재사용을 따른다.

## 1. 현재 확인한 연결 상태

기준 Jetson 루트는 `/home/q/it-arena-field-r500-20261114-a`다. **r500의 당시721소스/231설치 Python/17진입점 확인은 이전 버전의 설치 증거**다. 이후 r504 CLI, r505 패널, r498 계산 최적화3파일, r512/r514 `moving_pass.py`는 r500 설치에 포함되지 않았다. 현재 Jetson을 다시 읽거나 최신 설치라고 확인하지 않았다.

[관측 목록/SHA](observed_delta_not_frozen.json)는 현재 공통722파일 중 r500 대비 추가1·변경6개이며 **최종 동결본이 아니다**. r507은 r502 때의 `moving_pass.py`이므로 최종 업데이트에 그대로 사용하지 않는다.

| 연결 대상 | 업데이트 경로/조건 |
|---|---|
| r498 `local_follow.py`, `pass_release_trial.py`, `traffic_solo_gap.py`; 최종 `moving_pass.py`; r505 `race_ops_page.py` | `arena_autonomy` 새 overlay의 Python 파일 및 import 경로가 새 root 아래여야 함. source ZIP 복사만으로 설치가 갱신되지 않음 |
| r504 `scripts/steering_zero_profile.py` | 새 root의 scripts CLI를 명시 사용. `goal --help`에 `--command-limit-rad` 확인 |
| `scripts/mounted_servo_setup.py` | 현재 별도 세션 원문 그대로 보존. `probe_bus_servo.py`·`steering_calibration.py`는 수집에 포함, pyserial은 r500 `deps` 재사용. 기존 별도 세션의 원격 디렉터리·설치 앱을 덮어쓰지 않음 |
| ARM native | r500의 검증 `.so` 재사용 전 setup.py＋C++4개 SHA가 새 manifest와 같아야 함. x86 바이너리는 source ZIP에 넣지 않음 |
| C1/pyserial | r500 `bin/stream_c1_native_preview`, `deps`를 새 root로 복사하고 SHA 대조. 시스템 pip/apt/전력/네트워크 설정 변경 없음 |
| firmware 경계 | 기존 수집기는 `.c`를 제외하여 `firmware_servo_probe/main/main.c`가 빠진다. 호스트 실행 의존성은 완비지만 **servo-probe 펌웨어 재빌드 완전본은 아님**. runtime_drive의 main.cpp/빌드 도구/실장 생성기는 포함. 기존 side-session probe 앱과 PC firmware 빌드 경로를 보존하며 이번 호스트 업데이트에서 flash하지 않음 |

## 2. 최종 고정 뒤 한 번만 새 소스 묶음 생성

root의 소스 선택/고정 전달 후에만 실행한다. 원 r491/r507 ZIP과 SHA를 보존하고 출력은 새 `final_v1`을 사용한다. 생성 전후 수집 목록/각 파일 SHA, ZIP CRC·전체파일 SHA, embedded manifest, 필수 리소스6개·누락0, private/실행 바이너리 제외를 r507과 같은 방법으로 대조한다. `installed_values`는 모두 null이다.

```powershell
$recipeRoot = 'artifacts/validation/2026-11-14/runtime_update_recipe_r517_v1'
$toolDir = Join-Path $recipeRoot 'stage/scripts'
New-Item -ItemType Directory -Path $toolDir -ErrorAction Stop
Copy-Item -LiteralPath 'artifacts/validation/2026-11-14/runtime_handover_r507_v1/stage/scripts/prepare_runtime_handover.py' -Destination $toolDir -ErrorAction Stop
python -B "$toolDir/prepare_runtime_handover.py" --repo . --output "$recipeRoot/final_v1" --archive
```

사본 도구의 원 SHA는 `1d44f8f1cffc218538347406805451599731ec83514ca039abbb1ce5b45fa226`이다. 복사 후 이 값과 일치해야 한다. r517에서 사본을 아직 생성하지 않았다. 최종 delta는 `observed_delta_not_frozen.json`을 덮어쓰지 말고 별도 파일로 남긴다. 새 ZIP/manifest SHA는 생성 후 실제 값으로 기록한다.

## 3. 승인된 업데이트의 새 설치 루트

아래 `UPDATE_ROOT=/home/q/it-arena-field-r517-20261114-a`는 **미생성 제안 경로**다. 설치 담당이 실제로 없는 새 경로인지 확인한다. 기존 r500을 고치거나 symlink/current 기본값을 변경하지 않는다. 접속/전송은 기존 `scripts/jetson_access.ps1 -Action run|put|get`와 저장된 기존 호스트 키를 사용한다. 인증정보를 새 파일/로그에 복사하지 않는다.

전송 대상은 최종 ZIP, 해당 실제 SHA, 이 폴더의 `verify_update.py`뿐이다. 기존 방식의 명령 형태:

```powershell
scripts/jetson_access.ps1 -Action put -Source "$recipeRoot/final_v1/runtime_handover_sources.zip" -Destination '/home/q/runtime_handover_r517.zip'
scripts/jetson_access.ps1 -Action put -Source "$recipeRoot/verify_update.py" -Destination '/home/q/verify_update_r517.py'
```

Jetson 단일 담당이 기존 실행·소유권 회수를 확인한 뒤 깨끗한 Bash에서 진행한다. **아래는 미실행 레시피**다.

```bash
set -eo pipefail
BASE=/home/q/it-arena-field-r500-20261114-a
UPDATE_ROOT=/home/q/it-arena-field-r517-20261114-a
ARCHIVE=/home/q/runtime_handover_r517.zip
: "${ARCHIVE_SHA256:?최종 로컬 ZIP에서 확인한 실제 SHA}"
export BASE UPDATE_ROOT ARCHIVE ARCHIVE_SHA256
python3 - <<'PY'
import hashlib,json,os,zipfile
from pathlib import Path,PurePosixPath
root=Path(os.environ['UPDATE_ROOT']);archive=Path(os.environ['ARCHIVE'])
assert not root.exists()
assert hashlib.sha256(archive.read_bytes()).hexdigest()==os.environ['ARCHIVE_SHA256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    assert all(not PurePosixPath(n).is_absolute() and '..' not in PurePosixPath(n).parts for n in z.namelist())
    manifest=json.loads(z.read('source_manifest.json'))
    assert not manifest['required_missing'] and all(v is None for v in manifest['installed_values'].values())
    assert set(z.namelist())==set(manifest['files'])|{'source_manifest.json'}
    assert all(hashlib.sha256(z.read(n)).hexdigest()==h for n,h in manifest['files'].items())
    root.mkdir()
    z.extractall(root)
assert all(hashlib.sha256((root/n).read_bytes()).hexdigest()==h for n,h in manifest['files'].items())
PY
cp -a "$BASE/deps" "$UPDATE_ROOT/deps"
cp -a "$BASE/bin" "$UPDATE_ROOT/bin"
```

native는 r500 설치본에서 재사용한다. 기존 cache 경로를 추측하지 않고 실제 동일 binary 파일을 열거하여 상대 경로를 보존한다. 이때 새 root의 파일만 만든다.

```bash
python3 - <<'PY'
import hashlib,json,os,shutil
from pathlib import Path
base=Path(os.environ['BASE']);root=Path(os.environ['UPDATE_ROOT'])
old=json.loads((base/'source_manifest.json').read_text())['files']
new=json.loads((root/'source_manifest.json').read_text())['files']
native_sources=['src/arena_autonomy/setup.py']+[
 'src/arena_autonomy/arena_autonomy/_native/'+n for n in
 ('python_module.cpp','depth_kernel.cpp','polygon_kernel.cpp','current_mask_kernel.cpp')]
assert all(old[n]==new[n] for n in native_sources)
name='_numeric_kernels.cpython-310-aarch64-linux-gnu.so'
binary=base/'install/arena_autonomy/lib/python3.10/site-packages/arena_autonomy'/name
expected='5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(binary)==expected
cache=list((base/'build/arena_autonomy').rglob(name))
assert cache, 'r500 native build cache path must be inspected; do not guess a cache path'
assert all(sha(p)==expected for p in cache)
targets=[root/'src/arena_autonomy/arena_autonomy'/name]+[root/p.relative_to(base) for p in cache]
for target in targets:
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(binary,target)
    assert sha(target)==expected
PY
cd "$UPDATE_ROOT"
source /opt/ros/humble/setup.bash
colcon build --base-paths src/arena_autonomy src/arena_vehicle_interface \
  --packages-select arena_autonomy arena_vehicle_interface --executor sequential \
  --event-handlers console_direct+ > overlay_build.log 2>&1
```

실행 담당은 원 r500처럼 build PID·종료코드와 로그를 보존한다. cache가 없거나 native 원문이 바뀌면 기존 ARM 빌드 판단으로 돌아가며, 다른 바이너리를 임의 대입하지 않는다. 이 레시피는 r500 재사용 방식을 정리한 것이며 실제 cache 경로나 colcon 명령을 이번에 원격 실행해 검증한 것은 아니다.

## 4. 영향 범위만 smoke, 기존본으로 복귀

새 root만 활성화한 깨끗한 Bash에서 실제 ROS domain을 명시한다. 원 r500 `activate_r500_v2.bash`와 같은 환경 연결이며 노드/장치/펌웨어/운용 YAML을 시작하지 않는다.

```bash
: "${ROS_DOMAIN_ID:?시뮬과 분리한 실제 선택 domain}"
source /opt/ros/humble/setup.bash
source "$UPDATE_ROOT/install/local_setup.bash"
export PYTHONPATH="$UPDATE_ROOT/deps:$UPDATE_ROOT/scripts:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
python3 -B /home/q/verify_update_r517.py --root "$UPDATE_ROOT" --baseline-root "$BASE" \
  --output "$UPDATE_ROOT/update_verification.json"
```

검증은 r500과 같은 모든 source/설치 Python SHA 대조, **변경된5개 모듈 import가 새 overlay 아래인지**, r504 `goal --help`, mounted `--help`, 재사용 native ABI/SHA·pyserial·C1 SHA만 확인한다. 새 ROS 생성자/spin·계획 재생·벤치마크·장치 열기·구동은 하지 않는다. 이미 통과한 전체 회귀/전체랩을 반복하지 않는다. [검증기](verify_update.py)는 이번에 AST 문법만 확인했으며 **ARM 실행 완료 도구로 표기하지 않는다**.

실패하면 새 root와 로그를 보존하고 그 셸/프로세스를 정상 종료한다. **기존 r500을 덮어쓰지 않았으므로 파일 삭제나 되돌림 복사는 불필요**하다. 새 깨끗한 셸에서 기존 `activate_r500_v2.bash`를 source하여 r500 환경으로 복귀한다. 현재 ROS node가 이미 실행됐다면 먼저 담당자가 기존 STOP·정상 종료 절차로 회수한다. `~/.bashrc`, 서비스, current symlink를 바꾸지 않으므로 자동 활성화 변경도 없다. r500 역시 실장값 미채택 상태라는 한계는 유지한다.

## 5. 실물 current-route 선택은 ZIP 교체와 별개

현재 `current_route_priority.enabled()`는 `traffic_recovery_trial=true`와 `guard_observation_policy=selected_r181`로 선택된다. r474/r481 현재 경로, r489 독립 guard, r494/r514 MovingPass 분기에 simulation 전용 환경 제한은 없지만 **실물 입력의 기존 scope 계약을 먼저 만족해야 한다**.

실제 선택 YAML의 planner·guard에 동일하게 필요한 기존 조합:

```yaml
execution_environment: hardware_runtime_candidate_v1
use_sim_time: false
lidar_compensation: both
lidar_input_representation: native_estimated_v1
motion_scan_timing_verified: false
motion_scan_time_policy: host_estimated_v1
g1_traffic_recovery_trial: true
g1_guard_observation_policy: selected_r181
partial_observation_policy: bounded_local_v4
g1_crop_edge_component_trial: true
g1_observation_input_scope: host_estimated_crop_v1
```

planner는 기존 두 계산 runtime(`g1_runtime_policy=cad_runtime_r377_v1`, `g1_decision_execution=process_candidate`, `g1_submission_policy=fresh_native_v1`, `g1_current_path_recheck=selected_v1`, `g1_current_recheck_execution=worker_event_v1`, `g1_numeric_execution=native_v1`, `g1_expiry_route_reference_trial=true`)와 `g1_current_corner_budget=r396_v1`도 필요하다. 해당 키는 최종 선택 YAML과 두 노드의 실제 읽은 값으로 확인한다. 실제 시각을 verified/시뮬 basis로 바꾸지 않는다. 최종 실장 frame·기하·속도 상한·B3·포트·clock-policy·motor-feedback JSON은 별도 실제 입력이다.

기존 r444 생성자 검증과 [r487 계약8호출](../runtime_estimated_handover_r487_v1/RESULT.md)을 재사용한다. r487은 저장 시뮬 수치를 명시 estimated 계약으로 옮긴 입력이며 **r481 수락·r482 재사용 분기는 미도달**이다. r489/r494/r514 최신 분기 전체의 실제 센서/연속 주행 검증으로 확대하지 않는다. r500은 운용 YAML을 미선택했으므로 이 조합이 자동 채택된 상태가 아니다. r401 참고 템플릿만으로 실행하지 않는다. 운영 명령은 r491의 설치 limits 검증·같은 선택 YAML·host `--clock-policy`/`--motor-feedback-config` 명시 경로를 따르며, 이번 업데이트에는 ARM/drive-wire 허가를 넣지 않는다.

**transport 상태:** r500 활성화 스크립트는 ROS domain/localhost와 Python 경로만 명시하고 SHM16MiB transport XML이나 관련 환경변수를 적용하지 않는다. 실물 launcher에서 해당 설정이 실제 적용됐다는 이번 증거도 없다. r508 비교는 진행 중·미채택이므로 원인을 DDS로 확정하거나 네트워크/환경 기본값을 바꾸지 않는다.
