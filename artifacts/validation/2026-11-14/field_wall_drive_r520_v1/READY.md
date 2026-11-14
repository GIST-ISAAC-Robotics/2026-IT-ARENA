# r520 생성·정적 준비 완료 — 미동결·미실행

2026-11-14, GPT. [선택 조건](PLAN.md)에 따라 첫 직선 양쪽 두 벽의 국소 파생 SDF와 r515 기반 실행 레시피를 준비했다. **WSL·ROS import·Gazebo·제어 재생·동결·주행은0회**다. r521 결과 및 최종 공통 소스 선택은 root 결정 대기다.

## 형상과 정적 결과

- 원본 SHA `4c2136c7eec1615293db9f0cd8a5acb5133fc5f433c4a16f292b527bca851b3b` 불변. 현재 [파생 SDF](field_world.sdf) SHA `9bf5ba7491153b8ef045a96e2428f44bffc9d308f4d3431c25bee1f30ee166a2`.
- 원129개 벽 중127개 collision/visual은 동일하다. 선택2개를 국소 곡선·연결부·원두께 잔여구간348개로 나눠 총475개다. 벽 외 XML subtree는 동일하며 마찰·색·높이를 상속했다.
- 양벽의 도로측 간격은 표본상1.010 m로 유지됐다(최대 수치 차이1.1×10⁻¹² m). 곡선 내측면의 sine 대비 최대 표본 오차는0.3648 mm이며50 mm chord의 이론 선형보간 오차 한계는0.3701 mm다.
- 각 벽15연속 패널·14접합 틈. 의도한20 mm world-y 틈 안 점유는 수치 오차 수준2.7×10⁻²⁴ m², 실제 표면 사이 최소거리는20.1516 mm다. 패널 안 구멍0, 국소 양 끝도 buffer 없이 연결·구멍0이다.
- 선택 구간 밖 두 대상 벽의 원 형상과 차집합은 각 약1.9×10⁻⁹ m²다. 원본 yaw 소수점·XML 재표기와 경계 분할의 미세 수치 차이를 기록한 것이며, 나머지127개 벽은 XML identity 검사다.
- [601 actor·초기ego 검사](geometry_right_side.json) 통과. actor의 파생 벽 최소간격0.169962 m, 시설 최소간격0.047500 m, 벽/시설 겹침0·도로밖0. 초기ego는 벽간격0.316784 m·actor간격0.296500 m·벽/시설/actor겹침0·도로밖0이다.

최초 생성본 경계의 미세 삼각 공백은 [실패 원본](preparation_failures/boundary_v1/FAILURE.md)에 보존했다. 후속 형상은5 mm 두께를 유지하면서1 mm 평탄 연결부,0.5 mm 곡선 끝 inset, 기존 벽 안1 μm 겹침으로 연결했다. 의도한 접합 틈에는 연결부를 넣지 않았다. [정적 상세](field_static_geometry.json), [원본·형상 감사](field_geometry_audit.json).

## 연결과 남은 순서

[run.py](run.py)의 평가 polygon과 [simulation.launch.py](launch/simulation.launch.py)의 runtime source 인자가 같은 파생 SDF를 가리킨다. runtime 변환 전 source를 넣으므로 collision/visual이 함께 반영되고, 실행할 때 파생/실행 벽 collision digest를 확인한다. `world_override`는 사용하지 않는다. 현재 확인은 정적 소스 연결이며 실제 WSL import·runtime 생성 검증으로 확대하지 않는다.

[source_selection.json](source_selection.json)은 r518 `bypass_wall_fit.py`와 r5084파일, 관련 기존 시험2개·신규 시험2개를 기록한다. root의84/196검사 완료 전달을 근거로 하며 시험을 다시 실행하지 않았다. `freeze_authorized`, `drive_authorized`, `wsl_exclusive_owner`는 모두false이고 최종 소스 선택은pending이다. [준비 검사](preparation_receipt.json)는 이 상태와 입력 차이를 기록한다.

30 mm 진폭·1 m 주기·20 mm 접합 틈은 미실측 현장에 대한 선택 가정이다. 전체 현장 벽 재현, 주행 성공, 실시간 성능 또는 실차 보장을 뜻하지 않는다. 최종 후보 선택과 실행 지시는 root가 맡는다.
