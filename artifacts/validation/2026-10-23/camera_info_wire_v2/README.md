# CameraInfo 별도 프로세스 raw 전달

기록기 없는 두 프로세스에서 동일한 종류의 CameraInfo 한 표본을 전달했다. 송신 **333 B**, 수신 **336 B**이며 원래 333 B는 모두 같고 끝에 `00 00 00`이 붙었다. 역직렬화한 모든 필드는 같고 자식은 exit 0이다. raw CDR은 로컬 보존, 해시/차이 위치/환경 경계는 `report.json`에 있다.

이 관측은 sender CDR bytes와 ROS callback CDR bytes의 경계를 분리할 근거다. recorder가 받은 bytes와 MCAP의 동일성 검사를 느슨하게 하지 않는다. 이 한 표본만으로 모든 DDS/배포판의 padding 정책을 일반화하지 않는다. 영상 누락의 원인은 별도다.
