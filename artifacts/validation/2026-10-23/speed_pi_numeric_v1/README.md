# 속도 PI 산술 반례 — 수정 전

유한한 입력이더라도 내부 차분·비례항·적분 증가량이 넘치는 세 반례에서 모두 `effort=1.0, brake_requested=false`를 반환했다. 기대한 출력 차단과 달라 **3실패·1통과, exit 1**이다. 정상 범위 계산 1건은 통과했다.

극단값은 실물 센서 측정이나 실제 주행 결함 발생을 뜻하지 않는 API 고장 반례다. 입력 유한성 검사 뒤 생기는 비유한 중간값을 포화 처리하는 문제를 분리했다. 소스 변경 전 스냅샷·원시 로그를 보존하며 실행 전후 해시 차이는 0 B이다. 실제 장치·ROS·Gazebo는 실행하지 않았다.

- [실행 명령](run_checks.sh), [실패 로그](tests.log), [종료값](checks_exit.txt)
- 소스: `source_snapshot.tar.gz`, `sources_before.sha256`, `sources_after.sha256`, `source_hash_diff.txt`
