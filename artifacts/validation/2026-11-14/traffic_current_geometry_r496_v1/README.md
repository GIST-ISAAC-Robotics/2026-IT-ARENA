# r496 — r490 동일 교통 대표 단1회 실행 준비

**현재는 Windows recipe 준비만 완료했습니다. 공통 소스 동결·native 회귀·WSL 접근·실주행은 0회입니다.** r492 종료/WSL 반환과 root의 공통 통합 뒤에 진행합니다. `source_selection.json`은 아직 freeze/drive 허가 false이며 `prepared.json`, `sources.zip`, `registered.json`, `run1`은 없습니다. 새 판정선이나 성공할 때까지 반복은 추가하지 않습니다.

## 고정 조건과 보존 자료

r490에서 실제 사용한 runner/launch/STOP/관측/회수 도구17개를 복사하고 r496 경로만 바꿨습니다. 시설 SDF·시설 기하 함수·expected header는 byte 동일입니다. `recipe_provenance.json`은 원 파일과 복사 파일 SHA를 보존합니다. 원 r490 실패/run1은 수정하지 않습니다.

- 대표: `straight_moving_lead`, seed0, GREEN 기준 **10 sim초**, RTF .25. 전체랩이 아닙니다.
- 앞차: 최초4초 .30m/s, 4.0~4.2초 .40, 4.2~4.4초 .50, 이후 .58m/s. 오른쪽 .12m, 높이220mm, 초기 중심 간격1.6m. 실제 차량 동역학 대신 기존 평가 actor 이동 모형입니다.
- 기존 차량/센서/CAD 기하·자차 `g1_open_1p5`·`medium_v1`·시설3개·마커 옵션OFF·기존 간격/노면/접촉 판정 유지.
- 기존 최소 진행2.5m, 앞차 이동 중 최장정지2.5s, 최종간격1.2m 및 원 STOP 판정 유지. 실접촉/지속진행/실제간격/명령만료/STOP·종료를 구분하며 특정 최종mode를 성공 목표로 만들지 않습니다.
- 기존 `observe.py`의 벽접촉/8sim초 무진행 감시와 `stop_once.sh`의 owned STOP, `finish_collection.bash`의 원평가/동결/소유PID 회수를 재사용합니다.

## 예정 소스 변경과 동결 시점

r489 독립 guard, r494 현재 경로 점유 비교, float 직렬화 수정이 포함될 예정입니다. root가 r491 송신2파일과 r495 실장 max_wheel parameter2파일도 함께 통합할 예정이라고 전달했습니다. **포함 완료로 미리 판정하지 않습니다.** 통합 후 root가 r490 frozen manifest 대비 전체 delta를 `source_selection.json`의 control/noncontrol/additional sources/tests와 `included_integrations`에 기록합니다. 관련 없는 실물 도구 변경은 실제 시뮬 실행 경로 여부를 구분합니다.

`expected_r494_hashes.json`은 r494 원 beforehash/최종 stagehash를 그대로 복사한 검토용이며 현재 common을 동결한 파일이 아닙니다. 최종 이관본과 normalized stage 동일을 native runner에서 확인합니다. 준비 중 source_selection의 빈 change 목록이나 승인 false를 임의로 채우지 않았습니다.

## 필요한 제한 native 회귀

r486의 기존5파일 + 작은 current_return 파일을 그대로 재사용합니다.

1. `tests/test_moving_pass.py`
2. `tests/test_moving_pass_medium.py`
3. `tests/test_moving_pass_faster.py`
4. `tests/test_bypass_entry_trial.py`
5. `tests/test_bypass_entry_guide.py`
6. `tests/test_moving_pass_current_return.py`

`run_native_regression.py`가 실제 **common import 경로/SHA, 기존 native binary SHA, native_v1 활성 유지**를 기록합니다. r494 `common_probe`는 stage path를 삽입하므로 common native 검증에 사용하지 않습니다. r494 신규4개 저장/점유/인계 근거는 stage/common 내용 동일을 확인하여 보존하고 반복하지 않습니다. 새 C++/숫자 커널 변경이 없는 예정 범위에서는 재빌드/성능 행렬을 추가하지 않습니다. float·송신·실장 parameter의 자체 관련 검사는 root의 별도 이관 근거를 사용합니다.

## 통합·WSL 반환 후의 고정 명령

아래 명령은 아직 실행하지 않았습니다. 먼저 root가 실제 source delta와 WSL 단독 소유권을 기록해야 합니다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1/native_once.sh'
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1/prepare_once.sh'
```

기존 절차대로 최종 `prepared.json`/`sources.zip` SHA를 root가 검토하고 `root_execution_review.json`을 쓴 뒤 아래 단1회 명령을 실행합니다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1/execution_v1/execute_once.sh'
```

필요 시 기존 owned STOP 및 종료 회수:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1/execution_v1/stop_once.sh'
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1/execution_v1/finish_collection.bash'
```

등록/run1 gate와 소스 불변 확인은 r490 도구 그대로입니다. 다른 담당의 r492 종료 전에 동결하거나 실행하지 않습니다. Jetson/센서/자문 사용은 이 준비에 필요하지 않습니다.
