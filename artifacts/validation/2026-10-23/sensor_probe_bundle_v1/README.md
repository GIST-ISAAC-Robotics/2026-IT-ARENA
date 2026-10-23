# 센서 기록 도구 묶음 v1 — 독립 검사 실패 보존

관련 전체 회귀는 174개 통과했고 호환층을 적용한 실제 PC ROS 기록도 통과했다. 16파일·31,210 B archive의 파일/해시·Python 3.10 문법 검사는 통과했다. 그러나 `check_isolated.py`로 새 `build/` 폴더에 풀어 원래 repo를 Python 경로에서 제외한 검사는 **142통과·1실패**였다.

원인은 실수로 포함한 전체 A2 `test_bag_contract.py`가 의도적으로 제외한 `replay_sensor_bag.py`를 소스 점검 대상으로 읽기 때문이다. 런타임 기록기 의존 파일이 빠진 문제는 아니지만 **휴대 묶음의 시험 목록 오류**다. 재생기를 추가하지 않고 해당 전체 A2 시험 파일은 원래 저장소에서 계속 실행하며, 묶음에는 sensor_probe의 전용 계약·이전 재생기 거절 회귀만 남긴다. 실패를 skip/xfail로 바꾸지 않는다.

원래 archive·manifest·verification·isolated_report/log·추출 폴더는 보존했다. 다음 판본 v2에서 새 archive로 확인한다. 파일 검사 통과를 독립 실행/Jetson 성공으로 소급하지 않는다.
