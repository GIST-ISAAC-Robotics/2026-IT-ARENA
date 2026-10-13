# 모터축 관측 한계 감사 근거

2026-10-13. 새 Gazebo/ROS 노드 실행이 아니라 현재 소스의 오프라인 단위 회귀와 기존 G1/solver 원시 진단 분석이다. 운영 소스/모델/보호층은 수정하지 않았다.

- [기술 보고서](../../../../docs/autonomy/MOTOR_MOTION_OBSERVABILITY_2026_10_13.md)
- [분석 JSON](analysis.json): 입력·현재 소스 SHA, 원래 판정, 근접 시각 대응/누락, 0.1/0.3초 거리 차이 근사. `preserved_outcome`의 null은 원본에 그 이름의 필드가 없다는 뜻이며 실패/정지로 채우지 않았다. G1 실제 STOP 결과는 원본 `stop`/`stop_checks`에 있다.
- [실행 로그](analysis.log), [회귀 275개](tests.log), [실행 스크립트](run.sh)
- [최초 setup 실패](initial_setup_failure.json): `set -u`와 ROS setup 충돌로 시험 전에 중단. 해당 플래그만 제거하고 다음 실행 성공. 시험 실패를 숨기거나 판정선을 완화한 것이 아니다.

분석기: `scripts/analyze_motor_observability.py`, 새 회귀: `tests/test_motor_observability.py`.

입력은 10/7 `g1b_side_v1/side_left_s0`와 10/12 `solver_compare_v1/d1_dantzig`, `p1_pgs`의 보존 파일이다. 대용량 JSONL/수신 로그를 이 디렉터리로 복제하거나 Git에 추가하지 않는다. 입력 원문 해시는 분석 JSON, 당시 소스는 각 실행의 기존 snapshot에 있다. 이번 소스/시험기는 마감 시 `source_snapshot.zip`으로 보존하며 `seal.json`은 파일 보존·동일성 검사이지 주행 합격표가 아니다.

## 재현

WSL Ubuntu-24.04 / ROS Jazzy 환경에서 `run.sh`를 참고해 **새 출력 경로**를 사용한다. 현재 로그/JSON은 exclusive-create로 덮어쓰지 않는다. 모듈 검색 경로는 저장소의 `src/arena_autonomy`, `src/arena_vehicle_interface`, `scripts`다.

분석의 거리 차이는 진단 평균 속도와 평가용 차체 종방향 속도의 ZOH 적분이다. 실제 센서 보정점 오차·실물 안전·solver 단독 개선·한계 검출 완료로 해석하지 않는다. G1 실패와 PGS 미정지는 그대로 남는다.
