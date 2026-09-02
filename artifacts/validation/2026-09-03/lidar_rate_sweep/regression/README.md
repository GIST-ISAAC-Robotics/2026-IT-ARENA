# 2026-09-03 최종 회귀 검사

- ROS 패키지 5개 빌드 완료: `arena_description`, `arena_vehicle_interface`,
  `arena_gazebo`, `arena_autonomy`, `arena_bringup`.
  [빌드 기록](build_logs/build_2026-09-03_02-28-28/logger_all.log)
- 최초 Python/ROS **125개**, 종료 처리 보완 후 **129개 통과**, 실패 0개.
  [최초 JUnit](pytest.xml) · [최종 JUnit](pytest_shutdown_fix.xml)
- C++ `single_motor_equations` **1개 통과**. [JUnit 원본](ctest.xml)
- `build_official_track.py --check`: 공식 ZIP 24파일, 생성물 29개와 SHA-256 확인.
  ZIP 해시 `6f74322703554e2dbe87598ce00a85332e1c6227353e82b1255180d2b12e12cb`.
- `build_experimental_track.py --check`: 초기 전달본 23파일과 압축 파일 보존,
  파생물 해시·정적 경로 검사 통과.
- 공식·기존 실험·초기 원본 재현 월드의 `gz sdf -k` 세 번 모두 `Valid.`.
- 정식 LiDAR 20회 보고서의 입력 해시 **180개**는 현재 파일 160개와
  [v1 검사기 원문](../lab_matrix/test_driver_v1.py.txt) 20개로 추적합니다.
  차량 YAML·차량 모델·기존 벽 추종 제어기·트랙 파일에는 Git 변경이 없습니다.

## 종료 처리 보완

초기 검사기가 SIGINT를 그룹과 실행 부모 경유로 중복 전달했고, 부모 코드만
확인해 자식 종료 오류를 누락했습니다. 원시 v1 결과는 보존하고 종료 상태는
각 묶음의 `shutdown_audit.json`에서 정정했습니다.

[수정 후 20/30 Hz](../shutdown_fix_validation/README.md) 두 실행 모두
부모·자식 20개 정상 종료, 오류 0개를 확인했습니다. 30 Hz의 제동 이탈은 별도
주행 실패로 유지합니다. 조향 클래스·차량·평가 조건을 변경하지 않았습니다.

## 기본 주행 경로가 유지되는지 확인

[최종 짧은 데모 보고서](default_tof_demo/report.json)는 공식 트랙에서
`brisk`, LiDAR 10 Hz, ToF 안전층 켬, 기본 빨간 신호 8초를 사용했습니다.

- 진행 2.0398 m, 최고 지면 속도 0.723127 m/s.
- LiDAR 500점·10.0014 sim Hz, 올바른 범위·프레임.
- 조기 출발 없음, RGB·LiDAR 기반 자율주행 입력과 ToF·엔코더 기반 보호층 확인.
- 지정 거리의 정적 벽 겹침 표본 0개. 제동 중 충돌까지 평가하는 독립 고속
  시험기와는 평가 범위가 다릅니다.
- 정지 대기 약 0.893 sim s, 정지 중 변위 약 0.08761 m, 정상 종료.
- 원시 자료·영상·로그는 같은 폴더에 있습니다. 새 한 바퀴·장애물 회피 검증은 아닙니다.

고속 시험에서 사용한 ToF 해제·100 Hz 고정 속도 조향은 기본 데모에 적용하지
않았습니다. 컴퓨터 전원 종료는 수행하지 않습니다.
