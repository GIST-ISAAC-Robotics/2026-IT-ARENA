# r560 희소 상부 상대 OFF 대표

r555의 출발 전 launch 실패를 보존하고 별도 경로에서 준비한 실제 교통 대표다. root GO에 따라 native OFF/ON 파라미터 전달만 검사한 뒤, 실제 주행은 **OFF 1회**만 수행한다. ON 주행·다음 단독 시험은 승인 범위가 아니다.

`right_side_start`, seed0, actor1/0.5m/s/시작 전방0.5m·우측0.23m,12sim초,시설3,grid3,RTF.25,g1_open_1p5/medium_v1,8sim초 무진행 STOP과 원 판정선을 유지한다. 실측 count 파생 YAML·중앙 조향 상한 .28475436568260193rad, 기본 full 상태 기록도 같다. actor/field world/goal_eval/원 acceptance는 r555와 byte 또는 구조 비교로 동일 확인한다. 상대 형상과 굴곡은 가정 민감도이며 실측·실차 검증이 아니다.

제어 선택은 r553 동결본 + r554 bypass_entry_trial + r557 helper/current/retained 3파일, 총4개 변경이다. r557 stage8/native215 통과 근거와 최종 integrated manifest를 사용한다. r559 wrapper는 빈 OFF profile 파라미터만 생략하며 ON typed string, observer false, vision process와 local course 연결은 유지한다.

`assess.py`의 옛 고정 vehicle.final_cad_2026_11_13.yaml 대신 recipe/arguments에서 실제 선택한 vehicle config를 읽고 원 report의 경로·SHA를 교차 대조한다. 평가 수치/임계값은 바꾸지 않는다. `collect_result.py`와 `assess.py`는 case_window가 생성되지 않은 시작 실패를 unavailable로 기록하며 성공으로 바꾸지 않는다. r555 저장 report 사본으로 이 분기만 확인한다.

실행 순서는 `prepare_once.sh`의 r559 native smoke(40초 상한, 별도 ROS domain224, no Gazebo/control), 저장 미출발 수집 분기 확인, 실제12module import/native 불변/경쟁0 확인, source/ZIP 동결이다. root의 위임에 따라 실제 prepared/ZIP 대조 영수증을 쓴 뒤 `execution_v1/execute_once.sh`를 한 번만 실행한다. 실패 자동 반복은 없다. 기존7조향 launch/SDF 감사를 그대로 사용한다.

`execution_v1/finish_collection.bash`로 원 결과·STOP·소유PID0·입력 무결성을 회수한다. 사후 부가 평가와 원 판정을 구분하고, ROS 초기화 실패·주행 실패·실제 STOP·정상 종료·수집 오류도 각각 보존한다. WSL 소유자는 실행 담당1명이며 Jetson/ESP32/실물 접근은0이다.
