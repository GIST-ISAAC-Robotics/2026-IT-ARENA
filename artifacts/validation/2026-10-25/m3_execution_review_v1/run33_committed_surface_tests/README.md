# run33 — 노면 확약 후보 첫 관련 회귀

GPT가 구현·시험을 작성하고 GPT-6 Luna가 고정 명령을 한 번 실행했다. 299통과/1실패, 소스 불변이다. 시뮬레이션은 하지 않았다.

실패는 새 끝 보간 시험에서 의도한 경계 자세를 찾지 못한 `assert found`다. 코드/기준을 완화하지 않고, 회전 사각형 횡반폭과 이동을 이용해 경계를 0.30625 m에 정확히 놓는 독립 기하 입력으로 바꿨다. 첫 시험/소스는 `source_tested.tar`·로그에 보존하고 수정 후 검사는 run34로 분리했다.

수정 전 전체 소스 사본은 `source_before/`·`source_before.json`, 검사 당시 소스는 `source_tested.tar`·`before.sha256`, 실제 명령·종료코드는 `command.txt`·`exit_code.txt`다. 실패한 첫 검사를 후속 통과로 덮어쓰지 않는다.
