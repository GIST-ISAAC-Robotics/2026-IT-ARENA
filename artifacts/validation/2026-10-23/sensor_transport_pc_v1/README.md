# PC 전송 감사의 첫 실행 — 입력 발행 전 차단

Humble에서 썼던 읽기 전용 C++ inspector 원문을 설치된 PC Fast DDS 2.14.6에 다시 빌드했고 exit 0이었다. 하지만 Jazzy 기본 localhost participant는 SHM(기본 크기 선택 0)·UDPv4 두 개이고 UDP descriptor의 interface whitelist는 빈 목록이었다. 기존 Humble 관문의 `127.0.0.1` 제한 기대와 달라 **입력을 발행하기 전에 exit 1**로 차단했다. 자식 receiver/영상 전송은 시작하지 않았다.

`ROS_LOCALHOST_ONLY`의 실제 데이터 경로 제한을 이 descriptor 하나로 단정하지 않는다. 전역 설정을 고치거나 기존 Humble 검사를 느슨하게 하지 않는다. 후속 v2는 명시 SHM 단독의 기본 크기/16 MiB만 대조한다. 이는 기존 system(UDP+SHM)과 같은 기준선이 아니며 이번 큰 영상 누락 원인을 곧바로 확정하는 대조도 아니다. 라이브러리/패키지 버전/실패 관측을 보존했다.
