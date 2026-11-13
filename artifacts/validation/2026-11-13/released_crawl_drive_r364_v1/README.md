# r364 확정 기하·RELEASED 저속 후보 주행: 미완료

2026-11-13. r353와 같은 최종 기하·native10Hz·두220mm 상대·RTF0.25 조건에서 r361의 계획 단계 저속 후보만 추가했다. 실행은 한 번이다. **약10.12m에서8초 이상 정체하여 지정 STOP을 요청했다. 완주 실패이며 성공으로 재분류하지 않는다.**

23–31초의 상태는 `ROUTE_CURRENT_LARGE_MOTION_REGION_UNOBSERVED`80개와 `DECISION_PENDING`80개였다. 마지막 계획은 SOLO·0.50m/s로 전진을 제안했으나 현재 입력 재검사가0으로 거절했다. 현재 검사 안의 낮은 속도 후보도 `outside_small_body_side`였다. 이는 원 r353의 마지막 RELEASED 계획 정체와 다른 위치/단계다. r361 설치·종료 영수증은 확인했지만 이번 실패를 r361의 목표 구간 개선 증명으로 쓰지 않는다. [저장 분석](result_audit.json).

STOP 요청/확인은 성공했고 원 평가의 `passed=false`, `harness_valid=false`, `mcu_latched_before_stop: software_stop`, 전체 랩/영구 정체/전체 시간 coverage 실패를 보존했다. 조기 종료가 요청된 STOP과 원 평가의 자동 STOP 순서를 바꾸므로 두 결과를 구분한다. 수집 구간의 벽 충돌 표본은 없지만 전체 코스 무접촉을 증명하지 않는다. [원 결과](run1/report.json)·[조기 종료](early_stop.json).

노드 선행 검사는 최초 ROS PYTHONPATH 손실로 skip/실패했고, bash로 ROS 경로를 보존한 수정 명령에서1개 통과했다. 실제 실행 때1326입력 변경0·소스 보관본 불변, 후보 설치/정상 복구, 지정 ROS/Gazebo 잔여 없음과 실행 소유권 반환을 확인했다. 이후 공통 코드 변경은 이 원 실행에 소급하지 않는다.

다음 주행 전에 정체 시점의 현재 검사와 전체 운동 영역을 저장 입력에서 분리한다. 이유 변화 없이 동일 시험을 반복하지 않는다. 서보 CAD와 수동 신호 출발 작업은 별도 결과로 관리한다.
