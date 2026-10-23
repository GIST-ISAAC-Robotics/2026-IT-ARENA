# 고정 원문: normal 응답 decoder

출처: [Slamtec/sllidar_ros2 34300099fadfc772965962dec837bf436706188f](https://github.com/Slamtec/sllidar_ros2/tree/34300099fadfc772965962dec837bf436706188f).
2026-10-23 원문을 내려받았으며 소스/헤더의 저작권·BSD-2-Clause 조건은 각 파일과 [LICENSE](LICENSE)에 보존했다. 실행기에서 사용하는 모든 원문 파일은 [`audit_c1_normal_decoder.py`](../../../../scripts/audit_c1_normal_decoder.py)의 SHA-256 목록으로 검사한다.

`dataunnpacker_commondef.h`, `dataunpacker.cpp`는 바깥 의존성/시계 구현 확인용 원문이며 컴파일하지 않는다. 컴파일 트리에서는 commondef만 **시험용 의존성 shim**으로 대체하고, handler 본문과 wire 자료형·타이밍 구조체·가상 인터페이스는 원문 그대로 사용한다. 두 base 생성자/소멸자는 원문의 빈 구현과 동일한 stub이며 전체 unpacker registry는 실행하지 않는다.

실행 범위는 응답 본문 `0x81`의 5바이트 normal node다. 응답 descriptor 파싱·scan mode 질의/선택·USB·serial·ROS·스레드는 포함하지 않는다. 실제 C1이 이 응답 형식을 쓴다는 확정 자료가 아니다. 다른 응답 decoder에 이 결과를 일반화하지 않는다.
