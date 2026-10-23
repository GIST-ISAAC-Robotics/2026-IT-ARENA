# 센서 점검 기록·감사 최종 관련 회귀

`run_checks.sh`: **138 passed in 1.00s**, 실패/skip 0, pytest/래퍼 exit 0. `tests.log`, `junit.xml`, `checks_exit.txt`, 전후 소스 해시 보존. `source_hash_diff.txt`는 비어 있다. 당시 전체 소스 snapshot은 기존 정책대로 로컬에 보존한다.

선택형 센서/시계·기존 기본 계약·별도 manifest·파일/메시지/receipt 대응·허용 목록·중복 JSON/비유한 값·변조·이전 재생기 거절을 확인했다. `test_sensor_probe_audit.py`의 바깥 ROS reader는 stub이며, 실제 ROS/MCAP는 [v3 저장 입력](../sensor_probe_ros_v3/README.md)을 바뀐 감사기로 다시 읽어 `saved_cdr_audit.json`에 남겼다. 실제 센서 시각/값·전체 제어/Jetson 성능 검사가 아니다.
