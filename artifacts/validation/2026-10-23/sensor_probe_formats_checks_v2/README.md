# 센서 형식 검사 v2 — wire float32 비교 전제 실패

관련 회귀 **115통과·5실패**, exit 1. ROS 기록은 시작하지 않았다. v1의 CDR padding 전체 바이트 비교를 필드 비교로 바꾼 뒤, LaserScan scalar의 Python double 값과 wire float32 값 차이로 실패했다(예: 0.1 → 0.10000000149011612). 실패한 소스·hashes·로그·junit은 보존했다.

후속은 ROS 필드 선언이 `float`인 scalar만 IEEE 754 float32로 환산한 기대값과 **정확히** 비교한다. 임의 허용 오차를 늘리지 않고 Image 데이터·header·단일 모터축 등 다른 필드는 그대로 비교한다. 실제 기록 검사는 한 번 만든 송신 bytes와 수신 bytes의 동일성을 계속 요구한다. 원본 기록기와 단위/보호 기준을 바꾸지 않았다.
