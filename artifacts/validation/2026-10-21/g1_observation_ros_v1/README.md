# 관측 최적화·진단 정정 뒤 ROS 연결 검사

- 실행: [run.sh](run.sh), `g1_recent_observation_candidate`, `full`, `evidence_expiry`, wall cap 600 s. 실패 시 자동 반복하지 않는 1회.
- [보고서](run/validator_output/report.json): **43/43 통과**, [실제 종료](run/runner_status.txt) 0, [소스 불변](run/source_hash_status.txt). 사전 전체 소스와 내부 검사기 소스를 각각 보존했다.
- 세 제어기 재시작의 추적 행은 모두 완전했다. 최종 명시 STOP·그래프 수렴 뒤 정지를 확인했고 생성한 ROS/영상 자식 잔존은 없었다.
- 합성 센서·실제 ROS 프로세스·간단한 종방향 plant 검사다. 새 Gazebo 주행, Jetson/펌웨어/실차, 모든 지연의 원인 해결, 50 Hz 마감 보장이 아니다.
- 전원 모드는 변경/재측정하지 않았다. 과거 실행의 Turbo 설정 확인을 이번 실행의 실측으로 복사하지 않는다.

이전 v5/v6/v8 실패·간헐 지연은 [분리 보고서](../../../../docs/autonomy/PIPELINE_FAULT_VALIDATION_2026_10_04.md#7-1019-제어기-재시작-중-mcu-고정--v5v6-원인-분리와-시험기-수정)에 보존한다.
