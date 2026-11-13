# r417 경계 성분 후보의 명시 공통 연결 준비

상태: **staged 준비·실행 전**. 공통소스·실물·WSL·ROS·Gazebo를 변경하거나 실행하지 않았다. 기본은 OFF이며 실물 운영 프로필도 OFF다. 기존 r414의 24검사, OFF 고유254 판단/기록 상태 exact, prefix37/첫 직접 대상 전 불일치0, 후반88입력 양수2→88은 [원 결과](../crop_edge_component_r414_v1/verify_native_v2/report.json)에 보존한다. 이는 누락 이력이 있는 저장 순서의 제한 대조이며 완주·실차 안전 증명이 아니다. 원 r414 v1 실패와 v2 fixture도 그대로 보존한다.

## 변경 범위

`staged/src/`의 8파일만 공통 이관 후보다. [proposed.patch](proposed.patch)와 [static_preparation.json](static_preparation.json)에 base/staged SHA를 기록한다.

- `partial_observation.py`: r414 검증 후보 바이트 그대로, SHA-256 `576966260591adf169c426f33c921df0586ca75f31e9e973c69be94b2c268425`. crop 밖 성분은 소실/위협 검사에 포함하고 free로 바꾸지 않는다. 근거리·6°/5%·TTL·속도·metric/central 기준은 r414 그대로다.
- `local_follow.py`: `FollowConfig.crop_edge_component_trial=False`를 마지막 필드에 추가한다. 옛 header/위치 인자·기존 기본값을 유지하며 bool/Recovery+v4 호환 검사, 기존 수치 검사 제외, 정책 생성에 전달한다.
- `crop_edge_selection.py`: 설정 검사 한 곳. ON은 v4+Recovery 및 `use_sim_time=true`, 비실물 환경, motion `both/verified/verified_only`, `native_simulation_v1`을 요구한다. OFF는 기존 환경/정책을 건드리지 않는다.
- `local_pursuit.py`, `lidar_safety.py`: 같은 `g1_crop_edge_component_trial=false`를 각자 선언/검사하고 독립 정책 객체에 전달한다. planner의 명시 G1 활성도 확인한다. 객체·이력·래치를 공유하지 않는다.
- `native_lidar_input.py`: 기존 표현을 get-or-declare로 읽는 작은 helper만 추가한다. guard 생성 전 선택 검사와 뒤의 구독 선언이 충돌하지 않게 한다. 이미 선언된 값/기존 `laser_scan` 기본값/`str` 표현을 유지한다. decoder·source frame·원시각·epoch·mask·구독 콜백은 무변경이다.
- `simulation.launch.py`, `g1_follow.launch.py`: 동일 이름의 false 기본 launch argument를 명시한다. simulation은 같은 bool을 planner/guard에 전달하고 native local_pursuit 선택을 확인한다. g1_follow는 기존 scoped=False 문맥의 bool SetParameter를 사용한다. 중간 local/demo 연결은 기존 문맥을 그대로 전달한다.

`FollowTrace`의 기존 `asdict`, `decision_process.make_controller`, 두 runtime child의 기존 factory/reset 경로가 새 bool을 보존하므로 해당 파일들은 수정하지 않는다. 현재 재검사에는 별도 허가나 알고리즘 변경을 추가하지 않는다. 노드 검사는 선택 환경의 관문이며 순수 FollowConfig 자체가 시간·물리 허가를 증명하는 것은 아니다. 정책 내부도 native `MeasurementScan`/`simulation_acquisition`/`verified_only`가 아닌 입력에는 성분 예외를 적용하지 않는다.

## 검사와 고정 실행

Windows에서는 준비 코드와 AST/해시만 검사한다. 실행은 지정 담당자가 반환받은 WSL 자원에서 **한 번** 수행한다. 기존 ROS 환경의 PYTHONPATH는 보존한다. 새 빌드·센서·serial·모터·Gazebo·주행은 없다.

```bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B artifacts/validation/2026-11-13/crop_edge_runtime_r417_v1/run_verify.py --output artifacts/validation/2026-11-13/crop_edge_runtime_r417_v1/verify_native_v1
```

runner는 기존 8개 관련 회귀 + 새 설정/원시입력/독립상태/native fresh plan·current factory + launch AST를 먼저 검사한 뒤, r401 YAML에서 명시적으로 바꾼 **시험용 native 시뮬 시간** fixture로 실제 G1G2Node/guard를 한 번 생성한다. 원 r401 실물 설정 파일은 변경하지 않는다. 실제 두 계산 자식/vision 자식 생성·정상 회수, 양 노드 ON, trace bool, 원 frame/취득 시각/source epoch/snapshot epoch, `/drive`→`/drive/safe` 소유를 확인한다. spin/control/입력 offer/ARM/양수 명령은 호출하지 않는다. 여기서 만든 ROS 프로필은 실행 대상 장치의 교정값으로 승격하지 않는다.

순수 정책 검사와 기존 회귀는 `test_support/arena_autonomy/__init__.py`의 **시험 한정 package 검색 overlay**로 staged 6모듈과 기존 나머지 모듈/빌드된 native `.so`를 함께 읽는다. 이 init/시험/runner는 공통 이관 대상이 아니다. 제품 코드에 artifact import나 global monkeypatch는 없다. `.so`는 manifest의 현 바이트로 구분하며 이번 시험은 새 C++ 빌드나 새로운 native 성능 검사가 아니다.

모든 선택 검사 통과·skip0·동결 입력 불변·자식 정상 회수가 수용 조건이다. 실패/timeout/불변성 위반 시 다음 단계 없이 중단하고 원 로그·report를 보존한다. 재시험은 새 output/새 revision으로만 한다. `verify_native_v1/{related,actual_ros}.{log,xml}` 및 `report.json`이 결과이며 기존 r414 254 replay를 다시 실행하지 않는다. ROS domain223은 graph 충돌 완화일 뿐 CPU 자원 격리나 장치 허가가 아니다.

## 다음 주행과 진단

후속 r418은 최종4mask·r406 조건에서 `g1_crop_edge_component_trial:=true`를 선택하고 실제 launch 치환/양 노드 전달을 확인한 뒤 별도 한 번 주행한다. 이 준비물이 그 주행 결과를 대신하지 않는다. 정책 진단은 `detail.partial_observation.crop_edge_component`이며 실제 runtime wrapper에 따라 `normal` 등 상위 경로가 붙는다. `source_indices`, `inside_source_indices`, `flank_source_indices`, `full_width_deg`, `combined_unknown_source_indices`, `combined_width_deg`, `combined_fraction`, `flank_ranges_m`, `scope`를 남긴다. 진단 존재는 성분 선택 근거이지 최종 허가나 진행 성공이 아니다.

속도/물성/평가 판정·현재 재검사·실물 프로필·source frame/timing을 바꾸지 않았다. 실제 guard의 새 주행 결과와 지속 진행·접촉·정지/종료·시간 비용은 후속 주행에서 판단한다. 양수 판단 수만으로 채택하거나 완주라고 쓰지 않는다.
