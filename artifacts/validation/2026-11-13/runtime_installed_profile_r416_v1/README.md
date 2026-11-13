# r416 실장 프로필 → 운영 firmware 빌드 연결 후보

2026-11-13. **staged 준비이며 공통 적용·WSL 검사·IDF 빌드·실물 실행은 아직 하지 않았다.** r415에서 확인한 빈 `installed_board_profile()`에 외부 실장 입력을 명시 전달하는 연결만 추가했다. 영점·방향·PI/duty 등 실제 입력은 없으며 생성하지 않았다. 기본 빈 구성, protocol/main/STOP/boot와 preview의 비활성 의미는 유지한다.

## 변경 범위

| staged 파일 | 변경 |
|---|---|
| `scripts/runtime_installed_profile.py` | 기존 mounted JSON 검사/`cpp_zero_profile()`을 그대로 사용한다. 별도 board JSON의 명시 필드를 C++ 값으로 옮기고 원 JSON·header·해시를 새 폴더에 보존한다. 장치/ROS/serial/flash 코드는 없다. |
| `scripts/build_runtime_drive.sh` | 기존3인자에 선택형 `--installed-profile DIR`만 추가한다. 미지정은 기존 미교정 빌드이다. |
| `scripts/build_runtime_preview.sh` | 위 옵션을 **명시 drive 앱일 때만** 받는다. 프로필 원본/사본 검사와 기존 C++ `valid(map)` native 검사 후 IDF에 선택값을 전달한다. 미선택 때 추가 검사/컴파일을 실행하지 않고 기존 preview IDF 인자를 유지한다. |
| `firmware_runtime_drive/main/board_profile.hpp` | 기존 구조체/`valid`/진단 fallback 본문을 유지한다. 명시 compile definition에서만 생성 header의 구조체를 반환한다. header 파일 존재만으로 활성화하지 않는다. |
| `firmware_runtime_drive/main/CMakeLists.txt` | 기본 OFF인 compile option과 빌드 사본의 profile include 경로를 추가한다. |
| `tests/test_runtime_installed_profile.py` | 명시 필드/실장 assembly·기존 영점 export·변조·기본 비활성·합성 C++ 전달을 검사한다. 테스트 수치는 실물 설정 예제가 아니다. |

실제 수정된 common 파일은 없다. 부모 검토는 `staged.patch`, 기준 원문 해시는 `source_manifest.json.original_common_sha256`를 사용한다. 원4파일도 bundle의 `_r416/original/`에 보존한다. 새 helper/test는 artifact 경로를 참조하지 않아 공통 이관 후에도 사용할 수 있다.

## 입력 계약과 사용법

`runtime_installed_board_v1` JSON은 다음 필드가 모두 필요하다. 기본값 채우기나 생성용 실물 예제 파일은 제공하지 않는다.

- 최상위: `schema`, `assembly_id`, `encoder_direction`, `motor_direction_confirmed`, `forward_dir_high`, `maximum_duty`, `kp`, `ki`, `maximum_speed_mps`, `feedback_period_us`, `servo`.
- `servo`: `id`, `goal_speed`, `goal_acceleration`, `position_period_us`, `observation_max_age_us`, `command_limit`, `wheelbase`, `track`, `wheel_limit`.
- 단위는 기존 C++ 구조체 그대로다. 시간은 μs, 속도는 m/s, 길이는 m, 각도는 rad이며 `command_limit`는 차량 중앙등가 조향 한계다. PI와 ST3215 속도/가속 값도 기존 필드의 단위를 그대로 사용한다. `servo.zero`는 별도 기존 mounted steering JSON에서만 온다.

생성기는 exact 키·타입·finite/C++ 표현 범위와 양쪽 `assembly_id`를 검사하고 기존 mounted 영점 검사/전체 CAD 범위 검사를 호출한다. **새 PI/동역학/주기 기준은 만들지 않았다.** 생성 성공은 `RuntimeBoardProfile.valid()` 통과를 뜻하지 않는다. native C++ 판정은 선택형 빌드가 기존 함수를 그대로 호출하며, 실패하면 IDF 빌드를 진행하지 않는다. 미교정 default의 진단 동작은 계속 별개다.

```bash
python3 scripts/runtime_installed_profile.py prepare \
  --board-profile "$MEASURED_BOARD_JSON" --steering-profile "$MOUNTED_ZERO_JSON" \
  --assembly-id "$ASSEMBLY_ID" --output "$NEW_PROFILE_DIR"
bash scripts/build_runtime_drive.sh "$NEW_BUILD_DIR" "$PINNED_ST_SOURCE" "$NEW_EVIDENCE_DIR" \
  --installed-profile "$NEW_PROFILE_DIR"
```

위 변수는 사람이 확인한 실제 파일/장착 ID로 지정하는 운영 인계 형식이다. 영점 JSON은 기존 `steering_zero_profile.py record`/`confirm-mount` 결과를 사용한다. 별도 `export-cpp`와 byte가 같은 `mounted_steering_zero.hpp`, 나머지 구조체를 채우는 `runtime_installed_profile.hpp`, 원 JSON2개, `manifest.json`이 생성된다. 임의 사용자 메모/장착명을 C++ 코드에 삽입하지 않는다. 기존 폴더를 덮지 않는다. 빌더는 선택된5파일을 fresh build 사본과 evidence에 복사하고 해시/재생성 일치를 확인한다. 외부 입력을 수정하거나 header만 바꾸면 거절한다.

유효 프로필이 들어간 바이너리는 장치에 설치되면 기존 ARM/DRIVE 조건 아래 실제 출력을 할 수 있는 **운영 바이너리**다. 빌드가 출력 허가나 flash는 아니며 자동 ARM을 추가하지 않았다. 합성 검사 바이너리는 설치하면 안 된다. 부호·PI·상한·장착 영점이 확인되지 않으면 기존 빈 프로필 상태로 남긴다.

## 실행 담당 인계 — 아직 미실행

`sources.zip`은 staged를 덮어씌운 필요한 소스와 검사만 포함한다. 저장소 전체/ROS/센서 입력은 사용하지 않는다. `preparation.json`에 archive와 manifest 해시를 기록한다. 아래는 WSL 담당자 하나가 새 root에서 수행할 고정1회이다. 기존 SDK/도구체인 설치나 장치 접근은 하지 않는다.

```bash
set -euo pipefail
R416_ARCHIVE='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/runtime_installed_profile_r416_v1/sources.zip'
R416_ROOT='/home/jinhyeong/it-arena-r416-20261113-a'
R416_BUILD='/home/jinhyeong/.cache/it-arena/runtime-installed-r416-20261113-a'
R416_ST='/home/jinhyeong/.cache/it-arena/lsm6dsox-pid_20261111'
test ! -e "$R416_ROOT"
test ! -e "$R416_BUILD"
python3 - "$R416_ARCHIVE" "$R416_ROOT" <<'PY'
import hashlib,json,sys,zipfile
from pathlib import Path
archive=Path(sys.argv[1]); root=Path(sys.argv[2])
receipt=json.loads((archive.parent/'preparation.json').read_text())
assert hashlib.sha256(archive.read_bytes()).hexdigest()==receipt['archive_sha256']
root.mkdir()
with zipfile.ZipFile(archive) as z: z.extractall(root)
assert hashlib.sha256((root/'source_manifest.json').read_bytes()).hexdigest()==receipt['manifest_sha256']
PY
cd "$R416_ROOT"
python3 _r416/verify.py --root "$R416_ROOT" --output "$R416_ROOT/result" \
  --build-dir "$R416_BUILD" --st-source "$R416_ST"
```

`verify.py`는 새 검사와 기존 영점 export의 관련2검사만 실행한다(240초 상한). 이어 **합성 입력이라고 표시한 tests fixture만** 새 results 하위에 생성하여 명시 주입 IDF 빌드를1회 수행한다(1,200초 상한). 기본 profile은 native에서 비활성/비ARM을 확인하며, 같은 default IDF 빌드를 다시 반복하지 않는다. 공유 preview는 옵션 거절/셸 문법과 분기 diff로 확인하고 preview 앱을 실행하지 않는다. native 검사→IDF 순서이며 한 단계 실패하면 다음 단계를 진행하지 않는다. timeout 때 자신이 시작한 process group만 TERM 후 최대10초, 필요 시 KILL 후 최대10초 회수한다. 상한은 약25분이며 실패 root/로그를 유지한다.

회수 대상은 `result/report.json`, tests/build 로그, `source_before/after.json`, `result/build`의 profile/소스/바이너리 해시와 `result.zip`이다. `result`에는 `synthetic_profile_only=true`, `hardware_executed=false`, `firmware_flashed=false`를 남긴다. 코드 입력 전후 불변과 native/IDF 결과를 구분한다. 검사 성공도 영점 실측·실장 출력·물리 STOP의 확인으로 확대하지 않는다. 원 실패 위에 재실행하지 않으며 새 revision은 root 검토 후 별도 경로로 만든다.

준비 단계에서 수행한 검사는 Python AST와 셸 파일 LF 형식·staged diff/자료 범위의 정적 확인뿐이다. 실제 pytest/native/IDF 결과는 담당자가 회수한 뒤 별도 결과 문서에 기록한다. 인계 후 이 묶음은 root 명시 반환 전까지 문서 포함 동결한다.
