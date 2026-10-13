# B3 정적 신호 광원 렌더링 v1 — 종료 실패 보존

- `run.sh` / `tests.log`: 관련 회귀 38개 통과.
- `run/report.json`: 6장면 × 3프레임 확보, 두 그리드의 픽셀 재생 통과. 전체 `passed=false`.
- `run/launch.log`: Gazebo 서비스 ACK 뒤 미종료, PID 829는 최종 SIGKILL·exit -9. 중간 `ros2 run` 프로세스 종료 후 native bridge PID 840이 잔류했다.
- `run/cleanup_followup.json`, `.log`: 정확한 실행 인자와 PID/시작 시각/그룹을 확인하고 그 bridge에만 SIGTERM. 정리 후 잔존 0. 원래 실패 report는 변경하지 않았다.
- `run/source_snapshot.zip`, 보고서 해시: 당시 소스 보존. 공식 원본/운영 월드 변경 없음.

−90°·z=0.30 m는 진단 배치다. static off/red4/green 영상에 새 재생 시각을 부여한 것으로 동적 점등/공식 원위치 시야/실물 검증이 아니다. [기준 보고서](../../../../../docs/autonomy/RACE_START_CONTRACT_2026_10_13.md#51-정적-재질-영상은-검출-gazebo-종료는-실패).
