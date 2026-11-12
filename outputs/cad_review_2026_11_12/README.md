# 11/12 패키징 STEP 검토 산출물

상세 결론은 [검토 보고서](../../docs/hardware/PACKAGING_STEP_REVIEW_2026_11_12.md)에 있다.

- `source.json`: 원본 경로·크기·SHA-256. 원본은 Downloads에 보존.
- `assembly.json`, `geometry.json`: 배치 트리와 B-rep 치수/체적. **체적을 실제 질량으로 취급하지 않음.**
- `sections.json`: 지상 높이별 단면과 광축 차폐. `scan_comparison.png`는 기존/현재의 다른 배선 조건을 명시한다.
- `wire_cases.json`: 기둥 외곽 추가2/4/6 mm 민감도. 실제 배선 형상이 아니다.
- `clearances.json`: 선별8쌍 거리/공통체 결과. 전체 간섭 검증 아님.
- `stability.json`: 부품별 질량/위치 가정, 조합 범위, 정적 하중 이동 근사.
- `cad_*.png`, `shelf_*.png`, `jetson_*.png`: 검토용 색상 렌더.
- `wiring_route_candidate.png`: 기둥 앞/안쪽→상판 아래→내부 틈 상승의 제안 경로.
- `extract_step.py`, `analyze_geometry.py`, `sections.py`, `clearance_checks.py`, `wire_envelopes.py`, `stability_estimate.py`, `render_review.py`, `route_figure.py`: 이번 분석 소스.

의존성은 저장소 밖 임시 venv의 Python3.14, cadquery-ocp8.0.1.1.0, numpy, matplotlib, vtk다. `extract_step.py`→`analyze_geometry.py`→`sections.py`→각 후속 계산/렌더 순으로 실행한다. `sections.py --reuse`는 기존 단면으로 그림만 다시 생성한다.

`part_*.brep`, `mesh_*.npz`, `*.log`는 재생성 가능한 로컬 캐시로 이 폴더의 `.gitignore`에서 제외한다. 제조사/커뮤니티 상세 CAD의 대량 복제본을 공개 저장소에 추가하지 않는다.

최초 추출에서 OCP8의 컬렉션/Face 캐스팅/경계 API 차이로 중단한 뒤 수정해 다시 추출했다. 초기 전체 거리 검사 중 상세 모터 surface 모델의 계산이 오래 걸려 중단했고, 최종 `clearances.json`은 모터/Jetson 전수 최소거리 검사가 아닌 선별8쌍이다. 실패/제외를 전체 통과로 해석하지 않는다.
