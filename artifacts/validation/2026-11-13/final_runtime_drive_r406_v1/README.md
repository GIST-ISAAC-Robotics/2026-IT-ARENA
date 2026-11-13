# r406 최종 기하·공통 runtime 제한 주행 준비

**준비만 완료. 빌드·pytest·ROS·WSL·Gazebo를 실행하지 않았다.** 작성 범위는 이 폴더뿐이며 r401 동결 파일과 공통 소스는 수정하지 않았다. r405 저장 입력 대조·root의 공통 이관/변경 검토를 끝내기 전에는 주행하지 않는다. r377 원 실패·판정·자료는 보존한다.

## 실행 경로와 변경

`run.py` → 기존 `scripts/validate_g1_traffic.py` → 이 폴더의 `course.launch.py`/하위 launch → **공통 `g1_g2_handoff_trial`** → 공통 두 계산 evaluator를 사용한다. 실제 노드에 `--launch-prefix`, `node_entry.py`, 과거 artifact candidate import chain을 넣지 않는다. launch 경로 선택과 시설 평가용 wrapper는 평가 프로세스에만 있다. 공통 정책 내부의 기존 plan 자식 설치/복구는 r398/r400/r405 계약을 따른다.

[proposed.patch](proposed.patch)는 공통 `simulation.launch.py`에 두 launch argument와 controller parameter를 추가하는 8줄뿐이다. `g1_runtime_policy`와 `g1_current_corner_budget` 기본값은 모두 `off`이며 이번 명령에서만 각각 `cad_runtime_r377_v1`, `r396_v1`을 선택한다. process/fresh/worker/native·retained reference 설정은 r377대로 명시한다. 실제 제출 receipt는 `runtime_dual_latest_v1`이다.

| 구분 | 원 r377 | r406 |
|---|---|---|
| 마커/가림 | `--pilot-marker-z 0.140`, 5구간 | override 쌍 제거, 명시 선택 final vehicle YAML의 Z `0.09 m`, 4구간 |
| 실행 연결 | `node_entry.py`와 artifact 정책 chain | common G1G2 runtime factory, `g1_current_corner_budget=r396_v1` |
| 최종 기하 전달 | 당시 Follow/Lateral 기본값이 남은 경로 | r388의 `.128/.1239/.0261 m` 축거/윤거/타이어폭 및 현재 공통 기하 전달 |
| 원 시각/기어/속도 | verified simulation, native C1 10 Hz, 11.9117647059:1, g1_open_1p5 | 동일. 원 frame `laser_frame`, `verified_only` 유지 |

기존 marker override의 최소 `.115 m` 검사를 완화하지 않는다. 기본 YAML을 명시 선택하고 그 값 `.09 m`를 검사한다. marker4 변경과 후보/공통 변경이 함께 있으므로 개선을 r405 단독 효과로 환산하지 않는다. r396의 기존 5가림 저장 비교는 별도 분리 근거로 유지한다. r401은 실물 추정시각 프로필이므로 이 시뮬레이션에서 그대로 사용하지 않으며, 동일 최종 Follow/Execution 값만 기대 header의 근거로 사용했다.

원 `config.json`과의 차이는 설명 `scope`/`launch.entry_g1`뿐이고 `actors.json`은 바이트 동일하다. 두220mm 상대, `official_v2026_11_06`, old610 시설, ID20/30 누락, `road_level`, 80sim초, RTF0.25, 가상 MCU, 같은 물성/속도/가속/STOP·노면·접촉·전체거리 기준을 유지한다. config에 남은 옛 `.135 m` 평가 fallback은 수정하지 않는다. 기존 평가기는 명시 vehicle YAML의 `ego_geometry`를 먼저 넣어 실제 `.128/.1239 m`를 평가에 사용한다.

시설 STL/위치·광원 beam 제거·RTF wrapper는 r377과 같다. 평가 slab의 4개 함수는 원 함수 본문을 복사하고 의존 이름만 로컬 연결했다. AST 동등성 검사로 이를 확인하며 제어기에 지도·평가 자세·actor 각본을 보내지 않는다. 기본 sensor/frame/속도/미관측/TTL 정책을 새로 정하지 않는다.

## 공통 변경 검토와 소스 보존

[SOURCE_DELTA.md](SOURCE_DELTA.md)의 범위를 읽고 `source_review_draft_v1.json`의 **각 경로와 이전/예정 SHA256**을 검토한다. 이 초안은 준비 시점 r405 staged를 포함한 투영본으로 변경32개·snapshot 추가54개이며 동결 승인본이 아니다. 기존 공통 text의 전체 diff는 `common_delta_projected_v1.patch`에 별도로 보존했다. `added_to_snapshot`은 원 archive에 없었다는 뜻이며 모두 이번에 새로 작성됐다는 뜻은 아니다. 경로 접두어를 허용 목록으로 취급해 다른 변경을 무시하지 않는다.

r405 검증/공통 이관과 이 폴더의 launch patch 이관·필요 빌드가 끝나면 root가 다음 순서로 고정한다. 소스 inventory는 stdlib/해시/AST만 쓰며 실행하지 않는다. 기존 JSON을 덮어쓰지 않는다.

```bash
python3 -B artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/prepare.py \
  --inventory artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/source_review_actual_v1.json
```

root가 실제 목록을 확인하고 같은 내용을 새 `source_review_approved_v1.json`에 기록한다. 최상위 `status`는 `reviewed`, 각 `changes` 항목에는 구체적인 `reason`을 추가한다. 경로·해시·kind·순서는 수정하지 않는다. `--freeze`는 실제 전체 delta와 exact 비교하므로 새 변경·누락·해시 변동이 있으면 중단한다. r405 `integration.json`의 모든 common 파일과 staged launch의 바이트 일치도 요구한다.

```bash
python3 -B artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/prepare.py \
  --freeze artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/source_review_approved_v1.json
```

`prepared.json`과 `sources.zip`이 새로 생성된다. 공통 src/scripts/tests, 이 하네스·설정·patch, 직접 참조한 원 시설 함수/mesh/설정, r405 이관 receipt와 검토 목록을 보존한다. 과거 candidate 전체 계보를 실제 runtime dependency로 다시 끌어오지 않는다. 기존 r377 `sources.zip`은 덮어쓰거나 다시 생성하지 않고 baseline hash만 기록한다. 이 뒤 source 파일은 실행/회수까지 동결한다.

## 실행 담당 인계 명령

한 WSL 담당자에게만 빌드/ROS/Gazebo 자원을 배정한다. 아래 cwd는 repository root이며 기존 ROS overlay와 native 확장이 먼저 준비되어 있어야 한다. ROS setup의 PYTHONPATH를 지우지 않는다. 테스트에는 센서/주행 프로세스가 없고 실제 launch 치환만 수행한다.

```bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
# root가 지정한 기존 install/setup.bash도 이 지점에서 source한다.
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/prepare.py --check
python3 -m pytest -q -p no:cacheprovider \
  artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/test_preflight.py \
  artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/test_launch_selection.py
```

준비한 검사는7개 순수 검사와2개 실제 launch 치환 검사다. r405의 계산 동일성/노드 spawn 검사를 반복하는 묶음은 아니다. 외부 상한180초, skip·실패·source mismatch가 있으면 주행으로 넘어가지 않는다. 이후 root의 1회 자원 배정에 따라 다음 상대 경로 명령 그대로 실행한다.

```bash
python3 artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/run.py
```

원 startup wall150초·전체 wall1500초·STOP wall120초·green 초과20sim초를 유지한다. `registered.json` 또는 `run1/`이 있으면 재실행을 거절한다. 같은 장치에 병렬 빌드/성능/시뮬레이션 부하를 추가하지 않는다. 평가 기록에서 8sim초 무진행 또는 접촉·노면 이탈을 관측하면 원 r377과 같이 소유 시험에 STOP을 요청한다.

```bash
python3 artifacts/validation/2026-11-13/final_runtime_drive_r406_v1/stop_owned.py
```

이 helper는 같은 cwd/고정 argv의 소유 PID만 찾는다. STOP은 조기 종료 이유로 기록하고 원 full-run 실패를 지우지 않는다. 새 자동 완주/정지 판정이나 timeout 완화는 없다. 평가기의 정상 STOP·자식 회수·잔여 PID를 확인한 뒤 자원을 반환한다.

## 회수와 판단

- `run1/report.json` 등 기존 평가 결과: 전체 기간/거리/랩·접촉·바퀴 노면·STOP·실제 정지·상대 추적·하네스 유효성을 원 판정대로 보존한다.
- `launch_command.json`, `beam_replacement.json`, `rtf_audit.json`: 실제 명령·시설·RTF 확인.
- `runtime_audit.json`: 기대 전체 header exact, common factory/`r396_v1`/`runtime_dual_latest_v1` 기록, 후보 진단·계산 허가·부모 checked-route 발행을 구분한다. 후보 계산 성공만으로 실제 guard/MCU 적용을 주장하지 않는다.
- `registered.json`, `sources.zip`, `integrity.json`: 실행 입력/소스/원 평가 returncode와 전후 불변.

configuration 증거 불일치·footer 누락·capture error·source 변경은 유효한 후보 주행 증거로 쓰지 않는다. 후보 양수 적용이 없으면 적용 효과 미검증으로 남긴다. 원 약10m 정체를 넘어도 전체거리/STOP/접촉/노면/하네스 실패를 성공으로 바꾸지 않는다. 한 대표 결과를 회수한 뒤 같은 조건을 반복하지 않으며 실시간성·실차·전체 코스 안전으로 확대하지 않는다. 실제 모터/서보/serial/ARM/flash 작업은 포함하지 않는다.
