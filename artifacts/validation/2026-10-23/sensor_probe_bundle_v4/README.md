# 종료 오류 수정·입력 형식 관찰 포함 휴대 묶음 v4

관련 **313회귀·1.97 s**, 독립 복사본 **206회귀·0.63 s** 통과. 21파일·46,867 B, 목록 형식 `sensor_probe_tools_version=3`다. archive는 `build/sensor_probe_tools_20261023_v4.tar.gz`이며 파일별/전체 해시는 `manifest.json`에 있다. 기존 15/19파일 묶음도 변경 없이 새 검증기로 통과했다.

`runtime_check.sh`는 원래 저장소/overlay를 Python 경로에서 제외하고 묶음 소스와 설치 `/opt/ros/jazzy`만 사용했다. 사전점검·CDR 감사·간격 진단·입력 형식 관찰은 exit 0, 기존 미완료 기록은 exit 1과 진단 파일 미생성이다. 노드 생성 금지·소스/저장 원본 해시 불변을 확인했다.

새 기록기 close 경로는 `writer_cleanup_fix_v1`의 실제 ROS 3조건으로 별도 확인했다. archive에 원시 기록·드라이버·접속 정보·제어기·DDS 변경을 넣지 않았다. Jetson 전달/실행·Python 3.10 실제 실행·실물/고속 기록 성능·주행 허가는 아니다. 이전 v3와 실패 자료는 보존한다.
