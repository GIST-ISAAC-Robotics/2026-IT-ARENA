# r520 현장 굴곡 벽 가정의 오른쪽 옆차 대표 — 정적 준비

사용자 전달에 따르면 현장 벽은 대부분5T이고 큰 S자 굴곡·접합 틈이 있다. 굴곡 진폭과 틈 크기는 실측되지 않았다. 아래30 mm/20 mm는 root가 선택한 **가정 민감도 조건**이며 현장 실측값이 아니다.

- 대상은 `walls_main_left_5`와 `walls_main_left_25` 두 벽의 world-y3.0–11.5 m 구간이다. 나머지127개 벽과 바닥·잔디·시설·차량·물성은 유지한다.
- 기존50 mm 벽의 도로측 nominal면을 고정하고 두께5 mm로 만든다. 양벽 모두 같은 global-x 방향으로 `0.030 sin(2π(y−3.0)/1.0)`을 적용한다. 양 끝offset0이며 의도적인 반대 위상 협착을 더하지 않는다.
- 50 mm 표본 간격의 상자 chord, 600 mm마다 world-y폭20 mm의 틈을 둔다. 같은 패널 내부 상자는1 mm씩 겹쳐 연결하고, 국소 구간 양 끝은1 mm 평탄 연결부를 둔다. 접합 틈에는 겹침·연결부를 넣지 않는다.
- 원본은 덮어쓰지 않는다. [파생 SDF](field_world.sdf), [형상/127벽 감사](field_geometry_audit.json), [틈·경계·너비 검사](field_static_geometry.json)를 별도 보존한다.
- 같은 파생 SDF를 evaluator `obstacle_polygons`와 artifact-local launch의 `_runtime_world_overrides(source,...)`에 연결한다. source를 기본 runtime 변환 전에 주입하므로 collision/visual이 함께 반영된다. `world_override`는 사용하지 않는다. 실제 실행 시 runtime collision SHA를 대조한다.
- r515와 같은 actor+0.50 m/오른쪽−0.23 m·0.50 m/s·220 mm높이, 시설3, RTF목표0.25, `g1_open_1p5`/`medium_v1`, 12 sim초, STOP와 판정선을 유지한다. [601 actor+초기ego 정적 기하](geometry_right_side.json)는 새 벽으로 다시 계산했다.
- 공통 후보는 r518·r508 승인5파일과 기존 시험2파일/신규 시험2파일을 [source_selection.json](source_selection.json)에 기록했고 r521 통합 여부는 root 결정 대기다. 관련84/196검사는 root가 완료했으며 여기서 반복하지 않았다. 최종 SHA 확정 전 동결·WSL import·실행은 하지 않는다.

최초 생성본의 미세 경계 공백은 [실패 근거](preparation_failures/boundary_v1/FAILURE.md)에 보존했다. 현재 준비는 정적 기하와 연결 소스 검사이며 Gazebo 주행, 실시간 성능, 실물 결과가 아니다.
