# 일반 STOP 뒤 RESET 없는 START

제자리에서 잠깐 멈추는 조작은 **STOP→START**, 정지를 유지한 채 상태만 먼저 초기화하는 조작은 **STOP→복구 RESET**으로 구분했다. RESET은 프로세스 재부팅이나 새 경기가 아니다. START는 두 경우 모두 기존처럼 새 입력 경계·현재 경로·MCU 재ARM을 확인한다. 일반 명시 STOP 뒤에는 재배치했더라도 START가 새 입력으로 준비하므로 별도 RESET 버튼이 필수는 아니다. 경기 출발 사실을 유지하며 최초 신호 없는 START, 고장/물리 정지 해제, 새 STOP·연결 상실 취소를 우회하지 않는다. [계획](PLAN.md), [변경 전](before.zip), [소스 계약](../../../../config/tests/ordinary_resume_r1_sha256.json).

화면에는 제자리 재개와 RESET 용도를 설명했다. 일반 자동 시험에 화면을 띄우는 요구는 없으며 이번 실행도 같은 HTTP 명령 경로를 쓰는 자동 시험이다. 실제 브라우저 버튼 연결은 [별도 선두 주행](../post_free_facility_v1/README.md)에 있다.

## 검사·실행

- 관련 단위/노드/구동계약/소스 검사 **206통과**, 빌드 exit0. [검사](tests.log)·[종료 코드](tests.exit), [빌드](colcon.log)·[종료 코드](colcon.exit).
- run1은 RGB 출발·0.969m/s 이동 중 STOP 전달까지 진행했으나, 추가 정지 평가기가 `(t,x,y,yaw,z)` 5열을 4열용 함수에 넘겨 `ValueError: invalid ego pose shape/values`로 중단됐다. **재개 START는 미실행**이다. 제어기 실패로 부르거나 완료로 바꾸지 않는다. source 불변·정상 정리 확인. [실패 분석](run1_analysis.json)·[당시 평가기](run1_evaluator_before_fix.zip).
- 평가기 입력만 `(t,x,y,yaw)`로 고친 run2에서는 RESET 없이 재개 허가·같은 경기 진행·최종 정지를 확인했다. 다만 평가기가 일시적인 `policy_accepted`만 기다리는 동안 결과가 `permitted`로 넘어가 timeout된 **원본 exit1/분석 실패**를 유지한다([당시 평가기](run2_evaluator_before_fix.zip), [원본 분석](run2_analysis.json)). 후속 함수의 기존 준비/완료 대기로 충분하므로 중복 대기 한 줄을 제거했다. 기록에 이미 행동 증거가 있어 세 번째 주행은 하지 않는다.
- [같은 기록 대조](run2_reconciled.json)는 중간 단계 기록 누락 대신 실제 명령 수락·재개 결과·경기 ID·정답 자세로 행동을 확인했다. 새 입력 경계14.377초부터 첫 주행 허가 상태14.760초까지 **0.383초**, 최종 STOP 요청 전 본선 진행 **7.939m**다. 두 정지의 연속 기준·노면 외곽·소스 불변·정상 정리를 확인했다. 정지 속도1cm/s·연속0.2초, 제어기·주행 프로필·물성을 바꾸지 않았다.

실물/공용망 검증이 아니며 시험의 정답 자세는 평가 전용이다. 재개 요청 수락과 실제 주행 허가, 센서/경로 준비와 물리 정지는 구분한다.
