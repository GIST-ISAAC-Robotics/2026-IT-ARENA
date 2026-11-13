# r441 동일 조건 crop 단일 저속 후속 제한 주행 준비

**주행 미실행·root 최종 검토 대기.** r437의 원 정체/종료 자료를 보존하고 같은 조건에서 r440 단일0.1m/s current 후속이 실제 부모 양수 발행과 진행으로 이어지는지 새 경로에서 한 번 확인한다. 성공 여부와 관계없이 재실행하거나 판정선을 바꾸지 않는다.

## 고정 조건과 변경

r437과 동일한 config(설명/새 artifact 경로만 변경), actors, expected header, launch 선택, 시설 함수, 원 평가 기준을 유지한다. 128mm 축거·123.9mm 윤거·65mm 바퀴·native C1 10Hz·4mask·old610·두220mm 상대·blank20/30·road_level·`g1_open_1p5`·80sim초·RTF0.25·최초8sim초 무진행 owned STOP은 그대로다. 속도·기하·물성·센서/시각·보호 한계를 조정하지 않는다.

공통 변화는 r437 동결본 대비 **r434 4파일+r440 crop1파일**, 기존3개 변경/신규2개, 합계5개뿐이다. `source_scope.json`은 각 before/after SHA와 이유, `source_delta.patch`는 실제 diff를 보존한다. r439 설치 상한 후보는 아직 stage이며 이번 입력에 없다.

- r434의 node/page/helper/test는 기본OFF 선택형 ROS 미리보기다. 이번 주행에서 패널·preview를 활성화하지 않으며 raw 센서 구독이나 JPEG worker를 추가하지 않는다. 모듈 import에서 기본OFF/worker 없음도 확인한다. 이4파일이 포함되므로 byte 단위로 crop1파일만 다른 시험이라고 부르지 않는다.
- r440은 기존 .2 crop proof 성공·자체 후측방 mask만 거절·front mask0·실측0..0.15에서 .1을 한 번 재검사한다. 기존35검사/409 저장 비교 결과는 r440에 보존했으며 반복하지 않는다. WSL 저장 계산의105.40ms 한 사례를 숨기지 않고 실제 만료/처리 지연 관측과 구분한다.

## 준비·실행 계약

WSL 배포판은 `Ubuntu-24.04`다. 단독 담당이 r437과 동일한 `colcon build --symlink-install --packages-select arena_autonomy arena_vehicle_interface arena_bringup` 및 실제 import 경로를 확인한다. 빌드 뒤5파일 exact delta를 다시 대조하고 동결한다. 기존 `test_preflight.py`, `test_launch_selection.py`의16개 source/config 사전검사를 한 번 실행한다. 기존 crop 부모발행 검사에 crawl 분리 집계도 함께 확인하며 Node/시뮬레이터는 실행하지 않는다.

`prepared.json`/`sources.zip`/빌드·사전검사 영수증과 SHA를 root에 보고한다. root의 최종 실행 확인을 받은 뒤에만 다음 고정 명령을 한 번 실행한다.

```bash
python3 artifacts/validation/2026-11-13/crop_crawl_drive_r441_v1/run.py
```

`run.py` 앞에 `-B`를 넣지 않는다. 원 STOP helper는 `/proc`의 첫 두 argv가 `python3 path/run.py`인지 확인한다. 공통 입력은 root를 포함한 모든 담당자가 실행 회수까지 변경하지 않는다. 원 startup150wall초/전체1500wall초/STOP120wall초·green 이후20sim초 margin을 유지한다. 최초8sim초 무진행·접촉/이탈 또는 원 종료 조건이면 소유 `stop_owned.py`를 한 번 호출하고 evaluator STOP·정착·자식 회수를 기다린다. helper의 고정 reason과 실제 중단 이유는 구분한다.

수락은 단계별로 나눈다. crop/crawl 계산 수락, 인접한 같은 ref/조향의 실제 부모 양수 route 발행, guard/MCU 적용, 실제 진행, 명령 만료/계산 지연, STOP/자식exit/잔여PID/입력 불변을 따로 회수한다. `audit_runtime.py`는 기존 crop 집계와 `current_crop_crawl` 집계를 분리하며 계산 수락만으로 발행/실제 진행을 주장하지 않는다. 원 평가의 성공/실패 판정은 바꾸지 않는다. 이번 준비·후속 대표를 Jetson/실차/전체 조건 검증으로 확장하지 않는다.
