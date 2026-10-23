# B0183 입력 검사 v2 — 수집 단계 실패

ROS Jazzy를 source하고 상속 `PYTHONPATH`를 유지했지만 `arena_vehicle_interface` 소스/기존 workspace overlay 경로가 빠져 pytest 수집 단계에서 exit 2로 중단했다. 실행기 exit 1·소스 불변이다. [stdout](stdout.txt)·[보고서](report.json)·[소스 해시](hashes_before.json)를 보존한다.

후속 [v3](../b0183_input_contract_v3/README.md)는 기존 `imu_frame_gate_v1/run_checks.sh`의 ROS/overlay/소스 우선순위 진입점을 사용했다. 패키지 설치나 실패를 숨기기 위한 검사 제외는 하지 않았다.
