# Fusion 조향 운동 측정 산출물

- [상세 결과·부호·한계·시뮬 인계](../../../../docs/hardware/STEERING_FUSION_2026_11_13.md)
- [서보 입력별 좌우 바퀴 조향표](steering_table.csv): 96행, 각도와 rad. Fusion 입력 양수는 우회전, 바퀴 방향 양수는 좌회전.
- [Fusion 원 결과](fusion_sweep.json): 두 분기, 각 목표·조인트·바퀴 변환·복귀·오류 상태.
- [첫 시도 원 결과](fusion_sweep_first_attempt.json): 킹핀 360°/0° 등가를 처리하지 않아 양쪽 분기 사이에 중단한 실패를 보존.
- [조인트 정보](fusion_inventory.json): 원 조인트 설정, 회전축, 부품 연결. 조향 입력의 최소/최대 활성화.
- [Fusion 측정 코드](fusion_probe.py)와 [CSV 생성·감사](analyze.py). 측정 코드는 활성 Fusion 문서의 운동을 변경하고 중립으로 되돌린다. 원본을 저장하지 않으며 재실행 시 별도 출력 경로를 쓰고 열린 문서의 다른 작업 상태를 먼저 확인한다.
- [독립 평면 링크 계산](planar_check.py)과 [결과](planar_check.json): Fusion의95개 입력과 대조하고 혼 중립 오프셋 제거를 가상으로 비교한다. 실제 형상을 바꾸지 않는다.

검증: `python artifacts/validation/2026-11-13/steering_cad_fusion_v1/analyze.py` → 96행, 입력·단조성·킹핀과 바퀴축 방향 일치·회전상 중립 복귀 확인. CAD 간섭 검사·실물 측정·Gazebo 주행은 수행하지 않았다.
