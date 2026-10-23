# 5개 입력 형식 회귀와 첫 ROS 결과

120회귀 통과·1.83 s. ROS v7은 Image 17/20 누락으로 실패하여 runner exit 1이다. 소스 snapshot은 ROS v7의 `sources/`, 검사 직전 해시는 `sources_before.txt`, 원래 판정/로그는 `ros.log`와 ROS v7에 있다. 이후 성공 경로의 hash 대조/진단 명령은 실행하지 않았다. ROS v7 자체의 소스 불변 검사는 통과했다.

v1 CDR padding·v2 float32 기대값 오류는 각 실패 폴더에 보존했다. v3에서는 수신 bytes 비교를 느슨하게 하지 않았으며 내부 완료와 전송 손실을 구분한다. 후속 낮은 발행 빈도는 별도 조건이지 이번 누락 해결/고속화 승인으로 표시하지 않는다.
