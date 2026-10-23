# C++ 프레임 코어 1차 대조

- [명령](run_checks.sh): g++ 13.3.0, C++17, `-Wall -Wextra -Werror -pedantic`, ASan/UBSan.
- 관련 **184회귀 통과**, [실제 종료](runner_status.txt) 0·소스 해시 불변. C++ 추가 직전 전체 회귀는 별도 [3,570개](../actuation_link_budget_v1/tests.log)이며 이번에 전체를 다시 실행한 것은 아니다.
- [대조 결과](export/report.json): 243사례·2,294연산, C++/Python 프레이밍 차이 0. 기존 Python JSON 경계 2,039회 차이 0. 실제 binary 종료 0, sanitizer 오류 보고 없음.
- 최대 크기·모든 STOP 분할 위치·단일 비트 손상·노이즈·부분 헤더·합법/불법 길이·초과 청크와 기존 상태 계약 12사례의 바이트 순서를 포함한다.
- CRC-valid 비JSON 원문도 C++는 callback에 전달한다. JSON 검증까지 이관됐다는 뜻이 아니며, Python에서 별도 필터링한 결과만 기존 decoder와 대조했다.
- [입력](export/stdin.txt)/[출력](export/stdout.txt)/[stderr](export/stderr.txt)·사전 전체 소스·선택 소스 ZIP·컴파일 버전·명령·binary SHA-256을 보존한다. 로컬 binary만 Git에서 제외한다.
- 실제 보드, 부분 조립 timeout, JSON/상태기계, UART 작업/시간 성능, 런타임/전기 출력은 미검증·미연결이다.

사용자 마감 요청 시 작성 중이던 묶음만 검사했다. 이후 새 시험은 시작하지 않았다.
