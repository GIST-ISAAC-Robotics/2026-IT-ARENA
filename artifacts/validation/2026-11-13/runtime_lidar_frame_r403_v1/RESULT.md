# r403 결과

원 v1은 실행기가 source한 ROS Python 경로를 버려 rclpy import에서 실패했다. 소스148개는 불변이며 원 실패를 verify_v1에 보존했다. 실행기만 상속 PYTHONPATH를 보존한 v2에서 **93 passed, skip0**, 외부 arena import0, 원소스/준비파일 변경0을 확인했다. [실제 결과](verify_v2/extracted/results/report.json), [실행환경 수정](EXECUTION_ENVIRONMENT_FIX.md).

주 에이전트가 검토 후7모듈과2검사를 공통에 이관했다. 기본 frame은laser_frame이며 실제 C1은원 이름을 양노드에 명시한다. 다른frame의과거관측혼합은cache reset/동일frame필터로 차단한다. 입력 오차/시간/기하 한계는 그대로다. 실제ROS 프로필 및 센서 연결은 r401/r404 후속이며 물리 장착 교정·실차주행으로 확대하지 않는다.
