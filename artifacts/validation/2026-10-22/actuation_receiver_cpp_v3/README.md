# 변이체 주입 위치 사전 검사 — 사례 오기 실패 보존

v2의 물리 STOP 변이 주입 위치를 현행 코드에 맞추고, 위치가 정확히 한 곳인지 확인하는 사전 회귀를 추가했다. 코어·driver·API 실행 파일은 v2와 동일하며 재사용 해시 검사는 통과했다.

[사전 회귀](mutation_anchors.log)는 **14 통과·1 실패**, [전체 실행 종료](runner_status.txt)는 exit 1이다. 빈 일치 사례로 쓴 문자열 `no anchor`에 실제로 `anchor`가 들어 있어 기대값이 틀렸다. 원래 코어 오류가 아니라 시험 데이터 작성 오류다.

변이체 본검사·O2 API·전체 회귀는 미실행이며 [전후 소스 차이](source_hash_diff.txt)는 없다. 후속에서는 입력 문자열만 `unrelated text`로 고치고 [v4](../actuation_receiver_cpp_v4/README.md)로 진행한다. 이 실패와 당시 소스/로그는 보존한다.
