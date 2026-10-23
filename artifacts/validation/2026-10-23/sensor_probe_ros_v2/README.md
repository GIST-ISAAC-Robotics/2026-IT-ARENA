# 두 번째 ROS 검사: 시험기 graph 준비 경쟁

저장 bash 환경에서 관련 단위 회귀 112개는 통과했으나, ROS 시험기는 구독 endpoint matching 직후 노드별 구독 목록을 읽다가 `NodeNameNonExistentError`로 중단했다. 즉 endpoint 수신과 node-name graph의 전달 시점 차이를 시험기가 고려하지 않았다. 기존 12초 준비 마감 안에서 두 조건을 모두 기다리도록 시험기를 수정하며 timeout이나 기록/구동 판정선을 늘리지 않는다.

센서 payload를 발행하기 전 실패했고, `finally`에서 자기 기록기만 SIGINT 종료했다. `system/sensor_recording/manifest.json`의 `complete=false`, 원시 0 B·토픽별 0개는 그대로 보존한다. 첫 실패의 소스는 `sources/`, 상세 traceback은 [checks v1 ros.log](../sensor_probe_checks_v1/ros.log)에 있다. 기록기 결함이나 실제 센서 누락으로 판정하지 않는다. v3에서 새 기록으로 재개한다.
