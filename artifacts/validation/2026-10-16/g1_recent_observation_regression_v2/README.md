# 최근 관측 후보 전체 회귀 v2 — 마감

`run_pytest.sh`의 ROS Jazzy/기존 overlay·checkout 우선 Python 경로로 전체 tests를 실행했다. **2,682개 통과**, 82.54 s, 종료 코드 0이다. `environment.log`는 ROS 메시지 import를, `tests.log`는 회귀 결과를 보존한다.

실행 전 `src/scripts/tests/config` 943파일을 `pretest_manifest.json`·`pretest_source_snapshot.zip`에 보존했다. 사후 해시 차이는 0이었다. v1의 7개 후속 hash 계약 연결 실패는 별도 묶음에 보존했으며, 이번에는 최신 소스/부모 hash 검사와 역사적 before 연결 및 변조 거절 반례를 적용했다. 이전 계약 JSON은 덮어쓰지 않았다.

사용자 마감 요청에 따라 colcon 빌드·ROS 고장 시험·Gazebo를 시작하지 않았다. 실제 연결과 20 ms 연산 마감·새 코스 주행은 미검증이다. 옵션은 기본 false이며, 세부 재개 절차는 [기술 보고서](../../../../docs/autonomy/G1_OBSERVATION_ENVELOPE_2026_10_16.md#9-마감과-재개-순서)를 따른다.
