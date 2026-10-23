# 센서 기록 휴대 묶음 v6 — 선택형 세션 감독 포함

- `sensor_probe_tools_version=4`는 목록 판본이며, 이 산출물의 v6와 구별한다. **23파일 / Python 22파일**의 고정 목록이다. 기존 15/19/21파일 묶음은 각 판본으로 검증하며 덮어쓰지 않았다.
- 관련 **350회귀 통과(3.78 s)**, 원래 저장소를 Python 경로에서 제외한 추출본의 **236회귀 통과(2.41 s)**. 추출 전 경로·파일 목록/해시를 검사했다.
- 설치 Jazzy와 추출본만 사용하는 사전점검·감사·간격/형식 진단·세션 도움말은 성공, 미완료 기록은 예상대로 거절했다. 파일/코드를 수정하지 않았다.
- 추가 실제 ROS 검사에서는 localhost 격리 domain 193의 합성 IMU/scan만 발행했다. **추출본 세션 도구의 ready 후 2초→SIGINT→정상 자식 종료→전체 CDR 감사**가 통과했다. fixture도 종료했다. 제어 명령/차량/실물 장치를 사용하지 않았다.
- `spawn_failed`는 실행 실패 보고서로 남기고, 시작 전 취소에는 자식을 만들지 않는다. 이후 KILL에도 살아 있는 가짜 자식은 계속 실패로 기록한다.
- 직전 [v5 실패](../sensor_probe_bundle_v5/README.md)는 시험 fixture가 실행 전 취소를 반환한 문제로 보존했다. 제품의 강제 종료 성공 조건을 완화하지 않았다.
- 실제 Jetson/Humble/Python 3.10 실행, 센서 드라이버·취득 시각·교정, 고속 영상 처리량, 실제 디스크 고장/복구는 미검증이다. Python 3.10은 문법 검사만 했으며 `ready_to_drive=false`다.

archive는 Git 제외 `build/sensor_probe_tools_20261023_v6.tar.gz`, 추출본/원시 ROS 기록도 `build/` 아래에 로컬 보존했다. 해시/목록은 `manifest.json`, 결과는 `verification.json`, `isolated_report.json`, `portable_runtime_report.json`, `portable_session_report.json`을 따른다. 전송·Jetson 접속·운영 기본 변경은 하지 않았다.
