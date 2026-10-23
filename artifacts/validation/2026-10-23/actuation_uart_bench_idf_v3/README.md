# UART bring-up 검사 앱 대상 빌드 v3 — 빌드 성공, 큰 정적 프레임 발견으로 미채택

`build_exit.txt`는 0이고 빌드 전후 소스 해시 차이 0 B다. 검토 수정 v1 소스이며 BIN 220,112 B, 정적 DIRAM 105,182 B, SDK UART 함수 6개가 링크됐다.

그러나 [`.su`](main_cpp_stack_usage.su)에서 `UartScratch::make`의 정적 프레임이 **24,832 B**였다(v2 빌드에서는 208 B). 가짜 I/O에 시계 포인터를 더한 뒤 `io = FakeIo{}` 임시 객체(약 24 KiB 배열 포함)를 stack에 만들어 대입하도록 컴파일된 것으로 해석한다. 이 앱은 공유 사례를 8,192 B main stack에서 실행하므로 대상에서 실행했다면 넘쳤을 가능성이 크다(추정이며 실행하지 않았다). 호스트 회귀는 큰 기본 stack 때문에 이를 드러내지 못했다.

수정: 제자리 소멸·재생성으로 바꾸고, 빌드 스크립트가 함수별 정적 프레임 2 KiB 초과를 거절하게 했다. [v4](../actuation_uart_bench_idf_v4/README.md)에서 확인한다. 이 폴더는 발견 근거로 보존한다. flash/monitor·장치 조회 없음.
