# B3 정적 신호 광원 렌더링 v2 — bridge 정상 종료, Gazebo 실패 유지

v1 대비 `launch_ros.actions.Node`로 native bridge를 직접 소유하고 capture subscriber → bridge → Gazebo 순서로 종료했다. 남은 PID 정리는 실행 전 기록한 동일 시작 시각의 프로세스에만 허용하며 강제 정리를 정상 종료로 인정하지 않는다.

- `tests.log`: 관련 41개 통과.
- `run/report.json`: 6장면 × 3 distinct 프레임 확보. grid 2/3 모두 초록 검출·소등 후 유지·조기 출발 없음.
- bridge PID 794 정상 종료 0 확인. Gazebo PID 793은 서비스 `data: true` 뒤에도 20.088초 잔류, launch의 SIGINT/SIGTERM 후 SIGKILL·exit -9.
- 전체 `passed=false`, 시험기 exit 1. outer launch 0을 성공으로 쓰지 않았다. 최종 잔존 0, 추가 잔존 정리 없음, 소스 해시 일치.
- `run/source_snapshot.zip`/`fixture.sdf`/`launch.log`/PNG: 생성 조건과 실패 증거. 기본 배치·운영 모델 미변경.

픽셀 결과의 범위는 v1과 같은 진단 배열의 정적 영상 재생이다. bridge 고아 프로세스는 분리했으나 Gazebo 종료 원인은 미확정이다. 동일 시험을 더 반복하지 않는다. [기준 보고서](../../../../../docs/autonomy/RACE_START_CONTRACT_2026_10_13.md#51-정적-재질-영상은-검출-gazebo-종료는-실패).
