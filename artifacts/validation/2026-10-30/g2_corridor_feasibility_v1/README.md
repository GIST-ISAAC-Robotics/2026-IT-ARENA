# G2 중앙 통로 성립 진단

2026-10-30. 운영 후보가 아닌 진단 묶음이다. 결론과 표는 [중앙 통로 후보 보고서](../g2_inner_boundary_v1/README.md)에 한 곳으로 모았다.

- [analytic.py](analytic.py), [analytic.json](analytic.json): 평행·요각 자세의 차체 중심 구간 해석식.
- [counterfactual.py](counterfactual.py), [closed1/report.json](closed1/report.json): 표적 여유만 바꾼 폐루프 14사례. 벽 55 mm·경로·추종·독립 보호·STOP은 그대로다.
- [rerun_early.py](rerun_early.py), [rerun2/](rerun2/): 오염 사례 재측정.

**무효 표시:** closed1의 `const_100_early`는 작업자 재사용으로 표적 여유 패치가 누적되어 v=0 동작을 했다. 재측정 값은 0.8443 m 정체다. 표적 여유가 0이 아닌 다른 사례는 바깥 패치가 값을 다시 덮어써 영향이 없다. 원본 파일은 수정하지 않았다.
