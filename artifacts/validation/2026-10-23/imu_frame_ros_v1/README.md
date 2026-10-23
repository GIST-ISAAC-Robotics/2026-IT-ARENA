# IMU frame 입력 관문의 실제 ROS 전달 보충

- 기존 PC 메시지 객체 시험에 더해, 별도 복사형 설치본의 `MotionInput`을 실제 DDS 구독으로 검사했다. 격리 localhost domain 194이며 합성 `/clock`, `/imu/data`, `/drive_motor/encoder`만 사용했다.
- **25조건 통과·오류 0·정상 종료·설치 소스 해시 불변**. 두 어댑터(명시 `imu_link`, 기본 빈 문자열)를 같은 입력에 연결했다.
- 올바른 frame과 장착 roll 투영, 15:1·65 mm 단일 모터 환산, 다섯 잘못된/빈 frame 거절과 기본 호환, 거절 시 이력/시각 미갱신, 연속 거절 뒤 `motion_stale`, 거절했던 동일 시각의 정상 입력 수용, 신선한 구간 복귀를 확인했다.
- 이름이 맞아도 NaN/포화는 거절했고 좌우 바퀴 이름을 단일 모터로 받지 않았다. 가상 시계 되감김 뒤 이전 운동 이력을 버렸다.
- `/drive`, `/drive/safe`, `/actuation/*`를 만들지 않았다. 실제 제어기의 정지 출력·물리 차량 정지 경로를 시험한 것은 아니다.
- 원래 기본 frame 검사는 빈 문자열이며 자동 활성화하지 않았다. 이름 일치는 실제 IMU 설치 방향·단위·교정·취득 시각의 증명이 아니다. Gazebo/Jetson/실물 장치 실행 없음.

조건/입력 이벤트·설치 소스 해시는 `report.json`, 실행은 `run.py`, `run.sh`, `run.log`, `exit.txt`를 따른다. `ROS_LOCALHOST_ONLY`의 deprecated 안내는 보존했으며 현재 Jazzy가 해당 격리를 계속 적용한다고 출력했다.
