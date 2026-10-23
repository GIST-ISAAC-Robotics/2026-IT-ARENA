# 점별 시각 후보 이관 묶음 v2 — 로컬 준비

기존 10/13 묶음은 덮지 않고 `build/jetson_g1_profile_20261015_v2.tar.gz`를 새로 만들었다. 명시한 왼쪽 uniform 후보/첫 U자 지연 후보 JSONL 두 개와 최소 의존성·점유 감사 도구만 포함한다. 원시 기록·archive는 Git 제외, manifest·요약·소스 증거는 보존한다.

- 관련 회귀 39개 통과, [25파일 내용/해시 검증](verification.json) 통과.
- [새 격리 추출본 smoke](smoke.json)에서 ROS 없이 해당 추출본 모듈 import, 374개 판단 재생과 점유 감사 통과. 원본 묶음의 모든 멤버 해시 불변이다.
- [격리 재생](isolated_profile.json)은 의존성 확인을 위한 반복 1회이며 비용 비교의 추가 표본으로 합산하지 않는다. [격리 점유 감사](isolated_occupancy.json)는 수정된 작업 전후 소스 해시 계약으로 실행했다.
- Jetson 전송/설치/실행·전원 변경 없음. ARM·Python 3.10 실환경 및 ROS 전체 부하 검증은 별도다.

[manifest](manifest.json), `run.sh`, `smoke.py`, 단계별 로그를 보존했다. 이후 코드 변경은 이 묶음에 소급 포함되지 않는다.
