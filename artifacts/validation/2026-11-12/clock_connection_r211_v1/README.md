# 주행 기동 실패 뒤 시계 전송 진단

r210에서 물리 pose는 들어오지만 필수 ROS 센서/주행 상태가 없었다. 차량 없는 별도 빈 Gazebo world와 기존 ROS bridge, `use_sim_time=true` Python node를 사용했다. /clock2,001개·2초 진행·timer101개를 확인했고 두 프로세스 모두 강제 종료 없이 종료됐다. 직접 /drive·구동기·실물을 사용하지 않았다.

ROS 기본 시계 경로가 지금 작동한다는 제한 확인이며 r210의 간헐 원인을 특정하거나 실제 차량/센서 연결을 검증한 것은 아니다. 최초 probe의 설치 버전에 없는 qos_profile_clock import는 구동 전에 실패해 명시 BEST_EFFORT QoS로 바로 고쳤다. 결과는 `result.json`이며 r210 원 실패는 보존한다.
