# r533 최종 업데이트 준비 — source-only 묶음 생성 / 설치 없음

**후속 root 선택:** r532/r529/r526/r524/r523/r535 공통과457회귀를 고정하여 `selected_v1` source-only 묶음을 생성했다.729파일, ZIP SHA256 `1358c38798a8cb286f93e2ba5f3aa28fc79ab24571d8dc0864d5b36401878e71`, CRC/멤버SHA/현재소스대조 통과. Jetson 전송·설치0이며 실측 프로파일 미제공으로 `hardware_selection_prepared=false`다. 아래 초기 stage 설명의 미생성 상태는 이 후속 결과로 갱신한다. 실제 주행 통과를 뜻하지 않으며 r530 실행 결과는 별도로 확인한다.

2026-11-14, GPT. [r517 레시피](../runtime_update_recipe_r517_v1/READY.md)와 [r519 정책 생성](../hardware_policy_handover_r519_v1/RESULT.md)을 현재 공통의 명시 `compact_v1` 선택에 연결했다. **집중 검사9개 통과 후 root가 source-only ZIP을 생성했다. Jetson 종료 상태에서 접속·설치·실물 출력은 하지 않았다.** r517/r519 원본과 공통 scripts는 변경하지 않았다.

## 준비된 연결

- [정책 서식](draft_v1/policy_overlay.yaml)은 planner `g1_trace_detail=compact_v1`, 기존 native 두 계산/r396/estimated crop를 명시한다. planner/독립 guard는 같은 `selected_r181`, traffic recovery, B3 및 `false / host_estimated_v1` 계약을 사용한다. compact는 재생 입력이 아니며 기존 절대 `g1_trace_path`가 필요하다.
- [실측 입력 서식](draft_v1/required_inputs.json)의 assembly·frame/장착 변환/IMU bias·port·clock/motor-feedback 경로는 null이다. servo zero·board/MCU 상한도 [상태](draft_v1/status.json)에 null로 남는다. 추월 profile·속도·차량 기하는 root가 선택한 base에서 보존하며 임의 채택하지 않는다.
- [prepare_update.py](stage/prepare_update.py)는 보존한 r491 collector의 SHA를 검사한 뒤 원 `collect/inventory/write_bundle` 함수를 재사용한다. 현재 패키지의 새 하위 모듈도 수집하고 공개 helper·compact overlay·null 서식3개만 `handover/`에 추가한다. r401은 계속 참고 템플릿이며 운용 YAML로 선택하지 않는다.
- 실제 입력을 함께 주면 기존 `runtime_installed_profile.prepare` → [정책 helper](stage/prepare_hardware_policy.py) → `runtime_installed_limits.prepare/verify`를 연결한다. 결과는 `private/`에 분리해 ZIP에 넣지 않는다. helper는 저장소 stage와 이관된 `handover/` 양쪽에서 scripts root를 찾는다.

## 실제 영점·장착 한계 입력

현행 `mounted_servo_setup.py measure/zero`의 `straight.json`은 **중립만** 기록한다. 수동 세 점의 `summary.json`도 바로 설치 프로파일이 되지 않는다. 실제 운영 여유·방향을 확인한 뒤 기존 명령으로 새 프로파일을 만든다. `mounted_limits.json`은 아래에서 정한 출력 파일명이며 새로운 별도 스키마가 아니다.

```powershell
python -B scripts/steering_zero_profile.py confirm-mount $measuredStraight `
  --direction $confirmedDirection --safe-min-count $confirmedMinimum --safe-max-count $confirmedMaximum `
  --confirm-position-mode-and-mounted-limits --output $mountedLimits
```

`$mountedLimits`가 아래 `--steering-profile` 입력이다. 방향/count/기계 여유를 r401·CAD·합성 fixture에서 채우지 않는다. board JSON의 assembly·servo command 범위·MCU 상한 역시 별도 실제 선택 입력이다. 기존 generator가 neutral-only, assembly 불일치, 장착 범위 밖 명령을 거절하고 같은 조향 상한을 MCU 헤더/두 ROS 노드/host 인자로 연결한다. MCU 빌드·flash·실제 속도 상한·측정 정확도 확인은 별도이며 파일 생성으로 완료되지 않는다.

## root 최종 소스 선택 뒤 한 명령

아래 source-only 명령은 root가 최종 소스를 선택한 뒤 `selected_v1`에 한 번 실행했다. 후속 소스가 바뀌면 다른 새 디렉터리를 사용한다. `$finalSelection`은 그 선택을 식별하는 label, `$freshOutput`은 새 디렉터리다. 기존 r491/r507 ZIP과 r500을 덮어쓰지 않는다.

```powershell
$env:PYTHONPATH = (Resolve-Path 'build/jetson_tools/invoke/vendor').Path
$env:PYTHONDONTWRITEBYTECODE = '1'
python -B artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/prepare_update.py prepare `
  --final-source-selection $finalSelection --output $freshOutput
```

이 기본 명령은 **source-only**다. 실제 실장 입력도 준비됐다면 같은 명령에 다음 네 인자를 모두 추가한다. 없으면 null 서식만 남기며 생성기를 가짜 값으로 통과시키지 않는다.

```powershell
  --base-params $reviewedRuntimeYaml --hardware-inputs $measuredHardwareJson `
  --board-profile $actualBoardJson --steering-profile $mountedLimits
```

출력은 source ZIP/manifest, r500 대비 경로별 before/after SHA delta, 설치 미실행 영수증, policy draft다. 네 입력을 제공한 경우에만 `private/installed_profile` 및 `private/runtime_selection/selected/runtime_profile.yaml`·host limits가 추가된다. 공개 ZIP에는 실측 프로파일/port/clock 입력을 포함하지 않는다. 실제 실행은 기존 `g1_g2_handoff_trial`과 `lidar_safety`에 같은 selected YAML을 전달하고 host도 생성된 steering 인자를 쓰는 r519/r517 경로다.

```powershell
python -B artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/stage/prepare_update.py validate `
  --output $freshOutput --against-current
```

prepare는 생성 전후 source SHA, ZIP CRC/중복/경로/멤버 SHA/embedded manifest, 필수 리소스, 실행 바이너리·private 경로·기존 key signature 제외를 확인한다. 생성 중 소스가 바뀌면 검증 완료 영수증을 만들지 않는다. `validate`는 저장된 묶음 무결성을 확인하고 `--against-current`일 때 현재 소스와도 대조한다. key 검사는 알려진 패턴 확인이며 모든 비밀 탐지 보장은 아니다.

## 설치·복귀 경계와 검사

r500 `/home/q/it-arena-field-r500-20261114-a`와 그 원 ZIP/manifest SHA를 그대로 보존한다. 후속 설치가 허용되면 r517의 **새 독립 root·새 overlay 검증**을 사용한다. 복귀는 새 셸에서 기존 r500 활성화 파일을 선택하는 방식이며 현재 install/symlink/기존 루트를 수정하지 않는다. 이번에는 Jetson 접속 명령을 실행하거나 예약하지 않았다. r517의 native source SHA 대조·새 import 경로 확인을 생략하지 않는다.

기존 collector는 servo-probe `main.c`를 제외하므로 이번 묶음도 해당 펌웨어 재빌드 완전본은 아니다. side-session `mounted_servo_setup.py`와 별도 probe 앱/백업을 보존한다. 이 host 업데이트에는 flash를 포함하지 않는다.

[검사 로그](tests.log): 현재 공통 compact 생성자·양쪽 정책 계약, 원 r491 도구/r500 SHA 보존, **임시 합성 source**의 ZIP 생성·검증/변경·훼손 거절, 실제 입력 인터페이스의 합성 설치 연결·neutral-only 거절·원본 불변·private ZIP 제외 등9개 통과. 합성 ZIP/측정값은 테스트 종료 시 임시 디렉터리와 함께 제거했다. 생성된 ZIP 무결성은 확인했으며 실측 교정·ROS 실행·Jetson 성능은 검증하지 않았다.

**현재 준비 가능: 최종 선택 후 source-only 업데이트 묶음 생성. 아직 불가: 미제공 실측값을 포함한 운영 YAML 확정, 종료된 Jetson 설치·실물 활성화.**
