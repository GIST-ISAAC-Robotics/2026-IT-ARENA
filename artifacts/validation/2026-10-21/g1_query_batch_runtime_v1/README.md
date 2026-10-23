# 외곽 질의 묶음 계산의 운영 회귀

- 실행: [run_checks.sh](run_checks.sh), WSL Ubuntu 24.04 / ROS Jazzy 환경의 Python 검사·저장 재생. 새 Gazebo/실물 실행 아님.
- [pytest.log](pytest.log): 3,495 passed, 실제 종료 0.
- [ON](profile_on/report.json): 기존 3,001판단 일치. [OFF](profile_off/report.json): 기존 2,837판단 일치. 실제 종료 각각 0. 상세 시각은 각 디렉터리 산출물에 보존한다.
- [소스 비교](source_hash_diff.txt)는 비어 있으며 입력/소스 전후 해시도 일치한다.
- ON total p50/p95/최대 5.213054/8.313330/24.764310 ms, 20 ms 초과 4건. OFF 4.989355/7.678251/16.735601 ms, 초과 0건. 마감 보장/새 주행 성공 아님.
- 수정 전 소스는 [후보 증거](../g1_query_batch_candidate_v1/)에 보존했고 [후속 계약](../../../../config/tests/recent_observation_query_batch_r0_sha256.json)으로 연결한다.

해석·한계는 [기술 보고서](../../../../docs/autonomy/G1_OBSERVATION_CONTRACT_2026_10_21.md#5-외곽-조각-필요조건의-동일-결과-묶음-계산)를 따른다.
