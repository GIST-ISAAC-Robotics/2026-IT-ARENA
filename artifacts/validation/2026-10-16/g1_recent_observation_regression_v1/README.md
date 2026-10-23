# 최근 관측 후보 전체 회귀 v1 — 실패 보존

`run_pytest.sh`로 ROS 환경과 checkout 우선 Python 경로를 설정한 뒤 전체 tests를 실행했다. 2,672통과·7실패(85.54 s)다. `tests.log`의 7개 실패는 새 독립 보호층/평가기의 후속 hash 계약을 일부 옛 검사에서 따라가지 못한 연결 누락이다. 기능 합격이나 전체 회귀 성공으로 표시하지 않는다.

`pretest_manifest.json`, `pretest_source_snapshot.zip`에 실행 전 940파일을 보존했고 사후 대조 변경은 0이었다. 환경의 `ackermann_msgs` import는 정상이다. 실패 후 colcon·ROS 고장 시험·Gazebo를 시작하지 않았다.

후속 v2는 과거 계약을 덮어쓰지 않고 최신 소스·부모 hash를 검증하여 역사적 before와 연결한다. 소스/부모가 임의로 바뀌면 거절하는 반례를 추가한다. v2 결과는 별도 자료이며 이 실패를 수정하지 않는다.
