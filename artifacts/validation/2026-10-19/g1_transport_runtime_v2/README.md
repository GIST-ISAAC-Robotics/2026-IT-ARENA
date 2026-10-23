# 최근 관측 장면 이송 최적화 검증

- 변경: `observation_support.transport_scene`의 광선별 `dataclasses.replace`를 같은 클래스의 생성자 호출로 바꿨다. 모든 필드·유효성 검사·좌표 계산 순서·취득 시각·상태를 유지한다.
- 함수 대조: [선행 AB/BA](../g1_transport_constructor_v1/run/report.json) 6개 실제 입력 쌍, 버전당 240표본에서 정확한 객체 값 차이 0. p50 2.281→1.860 ms, p95 2.354→1.922 ms. 제어 주기 전체 비용이 아니다.
- 최종 회귀: [pytest.log](pytest.log) **3,101개 통과**, 종료 코드 0. 원래 `replace` 구현을 별도로 고정해 자기 자신과의 비교를 피했다.
- [ON 재생](profile_on/report.json): 3,001/3,001 판단 일치. total p50/p95/최대 5.274/8.305/29.617 ms, 20 ms 초과 6회.
- [OFF 재생](profile_off/report.json): 2,837/2,837 판단 일치. total p50/p95/최대 5.033/7.928/15.665 ms, 20 ms 초과 0회.
- ON seq3151~3156 total: 29.617, 28.043, 27.152, 25.935, 26.602, 23.958 ms. 최근 관측 보완의 비용 문제는 남는다. OFF는 다른 저장 실행의 입력이므로 같은 번호끼리 속도 개선율을 산정하지 않는다.
- 재현: [run_checks.sh](run_checks.sh). ROS Jazzy와 로컬 설치 환경을 먼저 읽는다. [최초 실패](../g1_transport_runtime_v1/pytest.log)는 ROS 환경을 읽지 않아 collection에서 27오류·2skip으로 중단된 기록이며 삭제하지 않았다.
- [전후 소스 차이](source_hash_diff.txt)는 비어 있다. 각 재생 디렉터리에 입력/소스 hash와 소스 snapshot이 있다.

이 시험은 PC 저장 입력의 순차 재생이다. 독립 보호 노드·ROS 전달·Gazebo 폐루프·Jetson 마감을 포함하지 않는다. 이전 실패와 기본 false, 보호 경계, 센서/물성은 그대로 유지한다.
