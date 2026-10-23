# CameraInfo 같은 프로세스 raw 전달

한 표본의 설치 rclpy `publish(bytes)`→동일 프로세스 raw callback에서 333 B가 그대로 전달됐다. 원본 hash/모든 메시지 필드 동일, 두 CDR 파일은 로컬 보존한다. 기록기·장치·제어기 없이 실행한 한 조건이며, 다른 프로세스/DDS 경로의 불변성을 보증하지 않는다. 후속은 [별도 프로세스 대조](../camera_info_wire_v2/README.md)다.
