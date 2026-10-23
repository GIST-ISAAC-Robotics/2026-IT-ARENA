# 첫 ROS 검사: 실행 환경 오류

2026-10-23 첫 inline PowerShell→WSL `bash -lc` 실행에서 `import rclpy`가 `ModuleNotFoundError: No module named 'rclpy'`로 실패했다. `sources/`는 당시 소스 보존본이다. ROS 초기화·가상 발행·기록기 자식 프로세스 시작 전 실패했으며 성공 자료가 아니다.

읽기 전용 확인에서 `/usr/bin/python3` 3.12.3과 `/opt/ros/jazzy/lib/python3.12/site-packages/rclpy` 자체는 존재했다. 긴 inline 환경 문자열을 피하고 기존 검증과 같은 저장 bash 스크립트에서 ROS/overlay/source PYTHONPATH를 적용해 새 v2에서 재개한다. 새 패키지 설치·장치 접근 없음. 첫 실패를 지우거나 v2 결과로 소급하지 않는다.
