# 점별 각도/시각 투영 후보

- `run_checks.sh`: 기존 `test_lidar_motion`와 신규 `test_lidar_point_review` 합계 **55통과·exit 0·소스 해시 차이 0 B**.
- 비균일/각도 wrap·정속 직선/회전, 기존 균일 경로와 동일 출력, 인덱스/원시 관측 나이 보존, 거절/미관측 조건을 확인했다.
- 현행 65 mm 바퀴·임시 축거 135 mm·평균 바퀴 속도 한 채널로 신규 합성 이력을 만들었다. 실제 모터/IMU/타임스탬프·슬립을 측정한 것이 아니다. 옛 `test_lidar_motion`의 당시 모형은 재현용으로 그대로다.
- 원래 `lidar_motion.py`와 ROS 입력은 불변이고 새 모듈을 운영 노드가 import하지 않는다. Gazebo·장치·Jetson·드라이버 실행 없음.
- 실행 소스/로그/해시는 이 폴더에 보존한다. [API 계약과 한계](../../../../docs/sensors/C1_DRIVER_TIME_CONTRACT_2026_10_23.md#5-점별-입력-투영의-오프라인-후보).
