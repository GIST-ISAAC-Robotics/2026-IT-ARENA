# 센서 기록·호환·진단 후 전체 PC 회귀

`run_checks.sh` exit 0, **4,538통과·실패/skip 0·265.94 s**. `sources_before.sha256`와 `sources_after.sha256`가 같고 `source_hash_diff.txt`는 비어 있다. A1 C++ 검사는 보존한 바이너리/원문 해시를 확인한 뒤 재사용했다. 소스 snapshot·전체 로그·JUnit을 보존했다.

후속으로 추가할 코드는 이 회귀에 포함되지 않는다. 원래 G1 공식 실패·실물 미검증·큰 영상 기본 전송의 누락을 통과로 바꾸는 결과가 아니다. 새 Gazebo/Jetson/실물·운영 기본 활성화 없이 PC 코드의 전체 회귀만 수행했다.
