# r437 차량 조향 상한 연결 후 제한 주행 1회

r431의 원 `command_expired`와 모든 산출물을 보존하고, [r435 근거](../command_expiry_review_r435_v1/REVIEW.md)의 시뮬 출력기 조향 파라미터 누락을 수정한 대표를 새 경로에서 한 번 확인한다. 성공할 때까지 재실행하지 않는다. 새 빌드·동결·16개 사전 검사 뒤 주 담당의 실행 확인을 기다린다.

## 조건과 검토 범위

- r431과 같은 config·actors·expected header·launch selection·시설 함수·원 평가 기준을 사용한다. 설명/새 artifact 경로만 다르다. 128 mm 축거·123.9 mm 윤거·65 mm 바퀴·native C1 10 Hz·4mask·old610·두220 mm 상대·blank20/30·road_level·80 sim초·RTF0.25·`g1_open_1p5`를 유지한다.
- 공통 변경은 r431 동결본 대비 **r430 네 파일 + r429 여덟 파일 + r436 두 파일, 총14개(기존11개 변경·신규3개)**다. 원/새 SHA와 각 이유는 `source_scope.json`에 있으며 r429/r430 integration의 stage SHA와 정확히 일치해야 한다. 이전 r426 다섯 파일만 허용한다는 조건이나 공통 소스가 동일하다는 빌드 재사용 주장은 적용하지 않는다.
- r436은 기존 launch가 전달한 `max_steering_angle_rad`를 `GuardedSimActuator`에서 읽어 `AppliedGuard.max_angle`에 연결한다. 새 물리 한계나 deadline 완화가 아니다. 사전의 실제 ROS 생성자 비교는 r436에 보존돼 있다.
- r429 패널 수동 모드는 이번 자율주행에서 활성화하지 않는다. 추가 speed cap 기본값은 기존 maximum이다. r430 설치 profile export는 실행하지 않는다. 이14파일이 함께 포함되어 있으므로 조향 연결만 바뀐 byte-identical 단일 변수 시험으로 설명하지 않는다.
- 공통 src/scripts/tests는 주 담당이 실행 회수까지 고정한다. 이번 담당은 이 artifact와 지정 build/install/log의 빌드·실행 출력만 작성한다. Jetson·물리 장치·전원·flash는 사용하지 않는다.

## 준비와 실행 순서

1. r431 대비 전체 common 목록의14파일 exact delta 및 r429/r430 stage 해시를 대조한다.
2. WSL Ubuntu-24.04 단독 담당으로 `colcon build --symlink-install --packages-select arena_autonomy arena_vehicle_interface arena_bringup`을 한 번 새로 실행하고 build/import exit0과 실제 모듈 경로를 회수한다. `build_v1/`에 명령·로그·영수증을 남긴다.
3. 빌드 후 delta를 다시 확인하고 위 사전 승인된 범위의 해시/이유를 `source_review_approved_v1.json`에 기록한다. `prepare.py --freeze ... --integration .../source_scope.json`으로 동결한다. 검토 목록의 정확성 대조와 최종 주행 실행 승인은 구분한다.
4. ROS Jazzy 및 install을 source한 LF wrapper에서 명시 `PYTHONPATH`와 `PYTHONDONTWRITEBYTECODE=1`을 설정한다. `prepare.py --check` 후 기존 `test_preflight.py`, `test_launch_selection.py`의16개를 한 번 수행하고 skip0/실패0 및 동결 불변을 확인한다.
5. manifest·빌드·사전 검사 결과를 주 담당에게 보고한다. **실행 확인을 받은 뒤에만** `python3 artifacts/validation/2026-11-13/applied_limit_drive_r437_v1/run.py`를 한 번 실행한다. run.py에는 `-B`를 앞에 넣지 않는다. 원 STOP helper가 `/proc`의 `python3 path/run.py` 첫 두 argv로 소유권을 확인하기 때문이다.

원 startup150 wall초/전체1500 wall초/STOP120 wall초와 green 이후20 sim초 margin을 유지한다. 첫 **8 sim초 무진행**, 접촉·이탈 또는 원 종료 조건이면 소유 `stop_owned.py`를 한 번 호출하고 evaluator의 STOP·정착·자식 회수를 기다린다. 원 helper의 고정 reason과 실제 중단 이유는 분리 기록한다. 원 평가 판정을 성공으로 바꾸지 않는다.

현재 crop 계산 수락·부모 발행·guard/MCU 수용·실제 진행, 최초 과거0.45초과 조향의 출력기 수용, STOP/자식 exit·잔여PID·입력 불변을 각각 회수한다. 같은 실패 뒤 재시작하지 않는다. 한 대표는 실차나 전체 조건 검증이 아니다.
