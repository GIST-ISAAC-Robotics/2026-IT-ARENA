# r436 출력 감시의 차량 조향 한계 연결

r431 command_expired의 선행 원인은 [r435](../command_expiry_review_r435_v1/REVIEW.md)에서 분리했다. MCU가 수용한 중앙 등가각0.4578436017rad를 가상 출력 감시만 고정0.45rad로 거절했다. 이후 output_ready=false가 호스트 갱신을 막았고 MCU 명령 만료로 이어졌다. 원 output_status 연속 기록은 없어 최초 ACK의 세부 수신시각까지 재현한 것은 아니다. 원 상태를 원 소스에 넣은 비교에서 이 각도 거절을 재현했다.

`GuardedSimActuator`가 기존 launch의 `max_steering_angle_rad`를 읽어 `AppliedGuard.max_angle`에 전달하도록 수정했다. launch·차량·MCU의 선택 각도는 바꾸지 않았다. 기본값0.45rad와100ms stream 감시도 유지했다. 보호각을 임의로 넓힌 수정이 아니다.

- 기존13+신규3 회귀16개 통과. 선택 상한±끝점 수용·그 바로 바깥 거절, 축소0.2rad와기존0.45rad, 잘못된 설정 거절을 포함한다.
- 실제 ROS 노드 생성에서 미지정0.45·축소0.2·현재 CAD0.4578436017036438을 각각 전달해3설정 확인. 구동 허가는 하지 않았다.
- 최초 shell wrapper의 `set -u`가 ROS setup의 미정의 선택 변수에서 중단해 시험 미실행이었다. wrapper를 `set -e`로 정정한 뒤 위3설정 시험을 수행했다.
- 새 주행은 후속 r437의 별도 결과로 판단한다. 이 결과만으로 최신 기하 전체 랩을 통과했다고 하지 않는다.

근거: `result.json`, `ros_parameter_result.json`, 보존된 r435 원 상태 비교. 실물 장치·flash·전원 설정 변경 없음.
