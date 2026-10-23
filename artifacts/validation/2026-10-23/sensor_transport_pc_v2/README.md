# 명시 SHM 프로필 + Jazzy LOCALHOST 병합 — 입력 전 차단

첫 sender의 실제 transport 목록이 **SHM 2개 + UDPv4 1개**여서 원래 SHM 단독 기대와 달랐다. 이미지나 receiver를 시작하지 않고 exit 1로 중단했다. 검사 기준은 바꾸지 않았고 16 MiB 조건은 미실행이다.

설치 버전과 대응하는 `rmw_fastrtps` 8.4.4 원문 participant.cpp의 LOCALHOST 분기는 기존 XML user transport 목록에 SHM과 UDP를 추가한다. [원문](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp#L185-L214). 로컬 복사본은 `../sensor_transport_pc_sources_v1/participant.cpp`, SHA-256 `b5ed71b30dc68aba1d415ce7d969d08283da784b5e61d48e6bc72338e955bf89`다.

후속 v3는 자식 환경에서만 `ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT`를 사용하여 RMW의 추가 transport 병합을 피하고 **builtin false·명시 SHM 단독**의 실제 적용을 각 프로세스에서 검사한다. SYSTEM_DEFAULT 자체를 로컬 격리라고 부르지 않으며 격리는 검사된 SHM 전용 transport가 담당한다. 기존 Humble 재생 설정/검사나 전역 설정은 수정하지 않는다.
