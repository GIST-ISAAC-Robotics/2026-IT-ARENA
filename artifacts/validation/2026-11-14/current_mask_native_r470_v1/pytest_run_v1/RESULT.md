# r470 재사용 pytest10개 실제 ARM 실행

기존 pytest6.2.5와 검증된 r470 native binary를 재사용해 **10 passed, 실패0/skip0**을 확인했다. pytest가 표시한 실행 시간은0.05초이며 제어기 처리시간 계측이 아니다. 성능 행렬과 전체 Evaluator는 다시 실행하지 않았다.

신규 `/home/q/it-arena-r470-tests-a`에 회귀 파일과 실행기만 배치했다. 실제 import는 기존 `/home/q/it-arena-r470-a/stage/src/arena_autonomy/arena_autonomy/`의 Python 연결부와 native 확장이다. test SHA는 `ecce49902cdd3dcad22e46578d79c0a6fcd0bde6898f3d872530cb805aa7e341`, binary SHA는 `5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff`로 전후 동일하다. 기존 manifest464파일도 변경0이다.

pytest cache와 외부 plugin 자동 로드를 끄고, JUnit의 실제 testcase10개·실패/skip0을 확인했다. 새 빌드·패키지 설치·공통 소스 변경·WSL·센서·ROS·구동·전력 변경은 없었다.

소유 PID62051은 exit0, 강제 종료0, 잔존 process group0이다. 관련 실행과 열린 센서 포트0을 확인하고 **Jetson 자원을 반환**했다. [원 receipt 요약](summary.json)에 test별 결과와 소스/프로세스 회수를 남겼다. 원 JUnit·로그·import 경로는 Git 제외 `remote_result/`에 보존했고 회수 ZIP SHA는 `2152d4c3252e713efb58196a8ae1b942c86e6460e31d66db66815707ca624fb6`이다.

공통 통합은 미실행이다. 사용자 일시 중단 지시에 따라 새 실행 없이 stage·패치·[준비된 통합 명령](../REGRESSION_HANDOFF.md)을 주 담당에게 인계하고 종료한다.
