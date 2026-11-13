# r452 GUARD 단일 저속 후속 제한 주행 준비

**준비/검증만 수행하며 Gazebo 주행은 아직 시작하지 않습니다.** r441 실패와 원 archive를 보존하고, 같은 조건의 한 번의 후속 대표를 준비합니다. root 최종 실행 확인 뒤에만 고정 명령을 한 번 실행하며 같은 실패를 자동 반복하지 않습니다.

## 고정 조건과 실제 변경 범위

r441과 같은 config(설명/새 artifact 진입 경로만 변경), actors, expected header, 시설 함수, STOP 본문과 원 평가 기준을 유지합니다. 128mm 축거·123.9mm 윤거·65mm 바퀴·native C1 10Hz·4mask·old610·두220mm 상대·blank20/30·road_level·`g1_open_1p5`·80sim초·RTF0.25·최초8sim초 무진행 owned STOP은 그대로입니다. 기하·물성·센서 시각·속도 profile·보호 한계를 변경하지 않습니다.

r441 동결본 대비 현재 공통 변화는 **26파일**입니다. r439 설치 상한 생성기/시험, r444 명시 estimated 계약, r446 일반 입구 제한, r449 완전 plan 기록, r451 GUARD .1 후보와 실제 launch trace bool 전달을 모두 포함합니다. 이관군이 한 파일에서 겹치는 경우도 그대로 기록합니다. 단일 파일 변경 또는 이전 빌드와 완전히 같은 source라고 주장하지 않습니다. `source_scope.json`과 `source_delta_postbuild_v1.patch`가 before/after SHA와 실제 diff를 보존합니다.

이번 simulation은 원 verified 계약을 그대로 사용합니다. 실물 estimated 선택·설치 상한 export·패널 수동/미리보기는 실행하지 않습니다. 새 r451 후보는 원 GUARD 상태/점유/1cm 한계를 유지한 단일 `.1 m/s` 재검사이며 정상 양수 주행 전체의 상한을 낮추지 않습니다.

기존 launch 선택에 **`g1_complete_plan_trace:=true`만** 추가합니다. 공통 launch와 노드 기본은 false로 유지합니다. 이 진단 옵션은 실제 plan 제출 원문·식별자·reset 이유와 별도 완료 출력, 실제 evaluator header를 보존합니다. 제출/완료는 발행/실제 진행 증명이 아니며, 추가 기록 비용도 실제 처리 지연과 구분합니다.

## 준비와 실행 계약

WSL `Ubuntu-24.04`에서 담당 한 명이 `colcon build --symlink-install --packages-select arena_autonomy arena_vehicle_interface arena_bringup`를 실행하고 실제 Python/native import 경로 및 ABI를 확인합니다. 기존 self-underlay override 경고는 원 로그에 보존합니다. 빌드 뒤 전체 actual delta를 다시 대조하고 root의 `root_source_review.json`을 확인한 뒤 동결합니다. 기존 사전검사 16개에 trace 충돌/선택/default/controller bool 연결 3개를 추가한 **19개**를 실행합니다. 이 검사에서는 simulation OpaqueFunction·ROS Node·프로세스를 실행하지 않습니다.

준비 산출물은 `prepared.json`, `sources.zip`, build/import/preflight receipt 및 source SHA입니다. root 최종 실행 확인 뒤에만 다음 명령을 실행합니다.

```bash
python3 artifacts/validation/2026-11-13/guard_crawl_drive_r452_v1/run.py
```

`run.py` 앞에 `-B`를 붙이지 않고 `PYTHONDONTWRITEBYTECODE=1`만 설정합니다. 원 STOP helper는 `/proc`의 첫 두 argv `python3 path/run.py`를 검사합니다. source는 실행 회수까지 동결합니다. 원 startup150wall초/전체1500wall초/STOP120wall초·green 이후20sim초 margin, 최초8sim초 무진행·접촉/이탈·원 종료 기준을 유지합니다. 조건에 닿으면 소유 `stop_owned.py` 한 번 후 evaluator STOP·정착·자식 종료를 회수합니다.

## 후속 분석 항목

원 등록 성공 기준을 바꾸지 않고 아래 단계를 별도로 집계합니다. 저속 목표는 일시 탈출이며, 계속 `.1`로 남는 것을 성공으로 간주하지 않습니다.

1. `selector_guard_crawl_retry` 계산 시도/수락, plan 제출/완료 식별자, 실제 current 재검사와 부모 발행 연결.
2. 후보 시작 뒤 실제 진행거리, 명령 목표와 측정 속도를 구분한 연속 `.1` 구간의 시간.
3. 이후 `.15 m/s` 초과와 `.35 m/s` 초과로 정상 가속에 복귀했는지 및 그 시각/진행거리. 복귀 관측이 없으면 없음 그대로 기록.
4. guard/MCU 적용, 접촉·이탈, command expiry/계산 지연, STOP·자식 exit·잔여 PID·입력 불변.

진행거리/trajectory 정답은 평가용 집계에만 사용하며 제어 입력으로 전달하지 않습니다. r441 원 실패·미완주와 r451 국소104행/함수 비교의 한계를 보존합니다. 준비나 단일 제한 대표를 Jetson/실차/전체 코스 검증으로 확대하지 않습니다.
