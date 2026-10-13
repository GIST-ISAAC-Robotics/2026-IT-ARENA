# 공식 출발 배열·경기 허가 후보

2026-10-13. 기본 노드/launch에 연결하지 않은 순수 후보 시험. Gazebo·ROS 노드·Jetson 실행 없음.

- `audit.sh` → `layout_audit.json`: 기존 `StartSignal`에 공식 5열의 합성 RGB를 공급. 지름 5/8/12/20/28 px·단일/쌍 10조건에서 검출 0·오출발 0. 사진/실물 검증 아님.
- `test.sh` → `tests.log`: 새 `OfficialStartSignal`·`RacePermission`과 기존 비전 회귀 **105개 통과**. 8/12/20/28 px 단일/쌍 배열 출발 확인, 5 px 면적 부족 거절, 부분/무관 초록/소등/중복·되감기·공백/명시 STOP/세대 해제 검사. 실물/ROS 제어 연결 시험은 아니다.
- 최초 후보와 당시 기본 영상 처리 소스는 `source_snapshot.zip`에 보존했다. 이후 과정 전달 변경은 별도 v2 산출물로 구분한다.
- 현재 실행 신호등·제어기와 공식 원본은 변경하지 않는다. 임시 시간/기하 기준을 공식 사양이나 실물 측정으로 부르지 않는다.

설계·출처·남은 적용 조건: [B3 보고서](../../../../docs/autonomy/RACE_START_CONTRACT_2026_10_13.md).
