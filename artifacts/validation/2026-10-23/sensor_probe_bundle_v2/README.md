# 센서 기록 도구 휴대 묶음 v2

`run_checks.sh` exit 0, 관련 회귀 **174통과·1.51 s**. 기존 전체 회귀와 합산하지 않는다. 15파일·28,286 B archive의 목록/해시·14 Python 파일의 Python 3.10 문법 검사가 통과했다. archive는 `build/sensor_probe_tools_20261023_v2.tar.gz`에 있으며 SHA-256/파일별 해시는 `manifest.json`에 있다.

`check_isolated.py`는 먼저 archive 검증을 통과한 뒤 새 `build/sensor_probe_isolated_kxhbrt9t`에 풀었다. 원래 저장소 경로를 제외한 `python3 -I -B`에서 포함된 핵심 모듈의 실제 경로가 묶음 내부임을 확인하고 **100회귀·0.48 s**를 통과했다. 추출본·결과·로그를 보존했다.

v1의 전체 A2 시험 의존 파일 누락 실패는 그대로 보존한다. v2에서는 센서 전용 범위를 벗어나는 그 시험 파일만 묶음에서 제외했으며 원래 저장소의 전체 A2 회귀는 위 174개에서 계속 검사했다. 재생기를 묶음에 추가하거나 실패를 skip으로 바꾸지 않았다.

PC Python 3.12의 독립 소스 검사다. Python 3.10 실제 실행·ARM/Humble/Jetson·장치 연결·전달·주행 검증이 아니며 자동 설치/압축 해제/구동 기능은 없다. 호환층의 실제 Jazzy ROS 기록 결과는 `../sensor_probe_ros_v6/report.json`, 공개 Humble API 원문은 `../rosbag_humble_contract_v1/`에 있다.
