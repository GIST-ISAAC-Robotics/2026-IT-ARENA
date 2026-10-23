# 10/23 센서 기록 도구 통합 회귀 v2

- `run_checks.sh`에서 현재 `src/ scripts/ tests/ config/` 해시와 소스를 보존한 뒤 전체 PC 회귀를 실행했다.
- **4,656개 통과, 실패/skip 0, 191.68 s**, `checks_exit.txt=0`, `source_hash_diff.txt`는 비어 있다.
- 기존 네이티브 시험 바이너리는 해당 C++ 소스/바이너리 해시가 같은 것을 확인한 뒤 재사용했다. 이 실행에서 새로 빌드했다고 해석하지 않는다.
- writer 단일 정리·추가 오류 보존, 입력 형식 관찰, 휴대 묶음 목록 호환, 선택형 세션 감독의 최신 CLI 검사까지 포함한다.
- Gazebo 주행, 설치본 모듈 일치, 실제 Jetson/Humble, 물리 디스크 고장, 실물 센서/차량 성능은 이 회귀로 검증하지 않았다.
- 상세 결과는 `full_tests.log`, `junit.xml`, 소스/바이너리 대조는 `*.sha256`와 `reused_*_check.log`를 따른다. 원시 소스 압축은 기존 Git 제외 정책으로 로컬에 보존한다.
