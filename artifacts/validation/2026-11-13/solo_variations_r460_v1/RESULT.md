# r460 3개 배치·벽 대표 실행 연결 정적 결과

**독립 artifact 연결 구현과 Windows 정적 검사를 완료했습니다. 공통/r457 입력 변경은0이며 WSL·Jetson·ROS·빌드·Gazebo·주행은 실행하지 않았습니다.** r457 nominal이 아직 실행 전인 상태에서 준비한 결과이므로, r460의 실제 실행 동결과 세 조건 주행은 보류합니다. 이 결과를 전체랩 또는 실차 검증으로 사용하지 않습니다.

## 구현 범위

r457 recipe의11개 파일은 byte exact 사본입니다. config의 scope/launch 진입 경로와 launch adapter의 조건 전달만 바꾸고, artifact 전용 raw XML/spawn 연결 및 조건별 실행·STOP·회수·동결 준비를 추가했습니다. [명세](changes.json)와 [검토 diff](review.patch)는 공통 적용용이 아니라 r460 artifact 검토용입니다. 원 `prepare.py`와 `prepared/*`, runtime visual 이미 병합 실패 기록은 보존했습니다. 일시 중지 당시 상태는 `PAUSED.md`/`paused_files.json`에 남아 있고 이 문서가 후속 완료 상태입니다.

- `variation_launch.py`, `launch/simulation.launch.py`: 원 XML의 벽 collision/visual을 common runtime override 전 변경합니다. 임시 source의 기준이 틀어지지 않게 원 source.parent로 상대 URI를 먼저 절대화합니다. common original과 같은 규칙이며, 이후 원 배치 처리·road_level·마커·시설 적용과 r457의 beam 제거/RTF .25를 유지합니다. 원 `world_override` 검사나 기본값은 수정하지 않습니다.
- `spawn_clone`: 선택된 slot 지역 대입 뒤 1개의 호출만 추가합니다. 시작점 변형은 sim 평가 사본에만 들어가고 제어기 파라미터·입력에는 들어가지 않습니다.
- `run.configure`: 사전 STRtree와 최종 평가가 같은 raw 변형의 정적 벽을 읽습니다. 종료 시 실제 runtime 정적 wall polygon을 대조하고 불일치는 harness invalid로 기록합니다. 시설3개 STL slab은 baseline과 같습니다.
- `prepare_execution.py`, `runtime_preflight.py`, `execution/*`: 조건별1회 recipe입니다. nominal 성공/회수 전 실행 동결을 막고, 후속 import 확인과 주 담당 검토 SHA를 요구합니다. 기존 STOP·실패 판정과 기준을 바꾸지 않습니다.

## 실행한 검사

명령은 저장소 루트의 Windows `C:/Python314/python.exe -B .../static_check.py`이며 환경은 `PYTHONDONTWRITEBYTECODE=1`, `PYTHONUTF8=1`입니다. [stdout](static_check_v1.log), [exit](static_check_v1.exit), [상세 JSON](static_v1/result.json)을 보존했습니다. **3조건/12검사 통과, exit0**입니다. Python20파일 AST도 확인했습니다.

실제 common XML 함수와 r457 `isolated` 함수 본문을 ROS import 없이 실행했습니다. 원 evaluator의 `rectangle/obstacle_polygons`도 원 코드 그대로 실행하되 Polygon 생성만 꼭짓점 tuple 반환으로 바꾸어 좌표를 비교했습니다. Shapely의 교차/면적 연산이나 ROS `_launch_setup` 전체 실행은 수행하지 않았습니다.

| 조건 | spawn (x, y, yaw rad) | baseline 대비 runtime 차이 | 평가 정적 벽 |
|---|---|---|---|
| left20/+3° | (9.870000000000136, 3.519999926535898, 1.62315987755983) | XML/OBJ 형상 변경0 | 129개 exact |
| right20/−3° | (9.909999999999865, 3.520000073464102, 1.51844012244017) | XML/OBJ 형상 변경0 | 129개 exact |
| wall50→70mm | (9.89, 3.52, 1.5708) | collision size1개, `batched_walls_0.obj`의 vertex8개만 변경 | 129개 중1개 변경, 변형 runtime과 exact |

각 runtime XML의4개 모델은 static track+시설3개입니다. 원 배치 처리 뒤 별도 wall visual이 제거되는 것이 정상이며 해당 모양은 OBJ에 반영됩니다. 다른 mesh/면/법선·노면·물성 차이는 없습니다. 메모리 복제된 spawn 함수는 기존 AST에 지역 호출1개를 넣는 차이만 확인했고 기준 함수 원문은 변하지 않았습니다. 기존 ROS 선택 목록도 exact이며 artifact 조건 환경변수가 기존 환경을 보존하는 것을 fake Popen으로 확인했습니다.

기준 source·필수 fixture·scene/world·recipe·archive 등 **1327개 입력 SHA가 검사 전후 일치**했습니다. 초기 정적 산출물과 원 probe 실패도 불변입니다. 실제 core 계산/판정/제어 코드를 수정하거나 재검증한 작업이 아닙니다.

## 동결 준비와 잔여

`prepare_execution.py`의 connection 생성도 exit0입니다. [connection manifest](connection_v1/manifest.json)에 baseline1327개, artifact32개, raw 변형3개와 조건별 고정 CLI를 기록했습니다. manifest SHA256은 `72c8406d85da2948ef06738f872223871f4b303da44e7b6075a4c7e80e7e4106`입니다. 실행용 `execution_prepared.json`/`execution_sources.zip`과 조건별 등록·PID·run1은 생성하지 않았습니다.

다음 필요한 순서는 r457 nominal 성공/정상 회수 → 배정된 WSL에서 비구동 import/launch description/시설 연결 확인 → 최종 동결/주 담당 검토 → 조건별1회 주행입니다. 자세한 명령과 receipt 필드는 [README](README.md)에 있습니다. nominal이 실패하면 이 단계는 진행하지 않습니다. WSL이나 장치 소유권을 취득하지 않았고 모든 Windows 명령은 전경 exit0으로 종료했으므로 이 작업의 잔여 소유 실행 PID는0입니다.

현재 수치는 작은 배치 차이의 시험 가정입니다. 실제 센서 오차/장착 오차 분포, 물리 접촉/노면 통과 또는 실차 안전을 입증하지 않습니다. 원8sim초 무진행 STOP·전체80sim초·완주 기준은 후속 주행에서도 유지합니다.
