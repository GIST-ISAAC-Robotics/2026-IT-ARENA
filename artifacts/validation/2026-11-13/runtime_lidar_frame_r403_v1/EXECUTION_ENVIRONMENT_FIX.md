# r403 실행 환경 복구

최초 verify_v1은 rclpy import 단계에서 실패했다. 격리 소스의 PYTHONPATH를 구성하며 source한 ROS setup의 Python 경로를 버린 실행기 문제였다. 원 검사와 소스148개는 불변이다. execute_isolated_v2.py는 격리 소스 경로 뒤에 기존 PYTHONPATH만 보존하며 manifest_v2로 별도 동결한다. 새 root/receipt에서 같은 필요한 검사만 다시 수행한다. 최초 실패와 실행기를 덮어쓰지 않는다.
