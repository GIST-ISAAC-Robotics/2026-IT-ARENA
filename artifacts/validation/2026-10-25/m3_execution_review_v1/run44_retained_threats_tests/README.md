# 회귀 실행 환경 오류

운영 소스의 수용 검사에 들어가기 전, pytest 수집 단계에서 `rclpy`/`xacro` import 4오류로 exit 2가 됐다. 스크립트가 ROS 환경을 source한 후 PYTHONPATH를 교체해 ROS 패키지 경로를 지웠기 때문이다. 소스/시험 코드를 바꾸지 않고 기존 환경 경로를 뒤에 보존한 run45를 별도로 실행한다. 원본 XML·stdout·소스 snapshot을 보존한다. 이 결과를 코드 회귀 통과나 코드 결함 검출로 세지 않는다.
