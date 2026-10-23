# 깊이 각도 창 함수 대조

명령: `bash artifacts/validation/2026-10-21/g1_depth_window_v2/run.sh`.

[보고서](run/report.json)와 [당시 소스](run/source_snapshot.zip)에 12실제 입력, 8,987질의, 버전별 240표본을 보존했다. 저장 2,615판단과 모든 후보 불리언이 일치했고 입력·소스 불변, 종료 0. p50 3.786949→1.351864 ms. 가장 가까운 원점–질의는 0.048991 m.

운영 적용·원점 계약 정정은 각각 [후속 함수 적용](../g1_depth_runtime_v1/), [원점 계약](../g1_origin_contract_v1/)의 별도 소스/검증이다. 함수 비용을 ROS/Jetson/전체 마감 통과로 확대하지 않는다. 상세는 [기준 보고서](../../../../docs/autonomy/G1_OBSERVATION_CONTRACT_2026_10_21.md).
