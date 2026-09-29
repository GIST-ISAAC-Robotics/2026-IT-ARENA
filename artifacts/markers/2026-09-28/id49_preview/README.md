# DICT_4X4_50 ID 49 생성 및 등록

- 사용자 선택과 게시 승인에 따라 ID 49 마커 PNG와 등록 문구를 2026-09-28 공식 이슈 #1에 게시했다. 등록 단계에서는 시뮬레이션을 변경하지 않았고, 이후 사용자 요청으로 차량의 임시 ID 10도 ID 49로 교체했다.
- 공식 생성기: https://github.com/MOSW626/istech-it-arena/blob/main/tools/make_vehicle_marker.py (2026-09-28 조회, 파일 blob SHA `fc5598c1bb7d27d7ea3229fdde70e5841eddf32f`). 소스 사본은 `make_vehicle_marker.py`.
- 명령: `python make_vehicle_marker.py 49`.
- 생성기 기본 설정으로 ID 49 재검출·36칸 격자 정렬 검사 통과.
- PNG는 1000×1000 px. SVG는 판 전체 50×50 mm, 코드 34.8 mm, 셀 5.80 mm. 인쇄 시 100% 배율을 사용한다.
- 생성 이미지의 정적 검증이며 실물 검출 거리·주행 성능 검증은 아니다.

## 등록 결과

- [게시된 댓글](https://github.com/MOSW626/istech-it-arena/issues/1#issuecomment-5858748518), 작성자 `leejinh0225`, 팀 `ISAAC (GIST)`.
- 승인된 초안에 1000×1000 PNG 한 장을 첨부했다. `DICT_4X4_50`, ID 49, 흰 여백 포함 5×5 cm, 차량 뒷면 부착을 명시했다.
- GitHub 댓글 재조회로 본문과 첨부 URL을 확인했고, 브라우저에서도 이미지 로딩 완료 및 원본 크기 1000×1000을 확인했다.
- 게시 본문은 [posted_comment.md](posted_comment.md), 조회 근거는 [posted_comment.json](posted_comment.json), 화면은 [issue_comment_posted.png](issue_comment_posted.png)에 보존했다. SVG는 로컬 인쇄용으로 보존했다.

## 게시 전 준비 기록

- 게시 대상: https://github.com/MOSW626/istech-it-arena/issues/1
- 본문: `issue_comment_draft.md`. Chrome의 해당 이슈 댓글 입력란에도 같은 본문을 준비했다.
- 첨부 준비 파일: `marker_DICT_4X4_50_id49.png` 및 `marker_DICT_4X4_50_id49.svg`.
- 최초 준비 단계에서는 파일 선택기 첨부가 확장 프로그램의 로컬 파일 접근 설정 때문에 실패했다. 이미지 붙여넣기는 자동 승인 검토가 당시 사용자 요청의 ‘올리기 직전’ 범위를 초과하는 외부 업로드로 판단하여 실행 전에 거부했으므로 중단했다. 이후 사용자가 이미지 첨부와 게시를 명시적으로 승인하여 클립보드 이미지 첨부 및 댓글 게시를 완료했다.
- `issue_comment_draft.png`는 댓글 본문이 입력된 미게시 상태의 화면이다. 브라우저 초안의 영구 보존을 보장하지 않으므로 로컬 본문도 보존한다.

## 차량 모델 반영 — 2026-09-28

- `src/arena_description/config/vehicle.yaml`·차량 Xacro·런치 검증 조건을 ID 49로 맞췄다. 등록 PNG 원본은 수정하지 않았다. 기존 SDF 셀 렌더링을 유지하면서 6×6 셀 배치가 등록 PNG와 픽셀 단위로 일치하는지 검사했다.
- 판 전체 50×50 mm, 코드 34.8×34.8 mm, 셀 5.8 mm, 각 변 흰 여백 7.6 mm다. 후방 중앙 `xyz=[-0.1005, 0, 0.09] m`·후방향 자세는 기존 임시 배치를 유지했다. 질량·관성·충돌체·제어기·코스 마커는 변경하지 않았다.
- 마커 관련 정적 회귀 2개 통과: ID/코스 ID 비중복·크기·PNG 일치·셀 방향/후방 배치·추가 충돌체 없음 확인. 최초 치수 검사에서는 부동소수점 합을 완전 일치로 비교해 실패했으며 `1e-12 m` 허용 오차로 비교하도록 수정했다. 모델 치수를 검사에 맞춰 바꾼 것은 아니다.
- 최초 촬영 실행은 필수 `lidar_compensation:=both` 인자 누락으로 시뮬 시작 전에 거절됐다. [실패 기록](../../../screenshots/2026-09-28/rear_id49/report.json)을 보존하고 현재 센서 구성의 정상 실행 인자로 재시도했다. 촬영은 주행 제어를 켜지 않은 별도 실행이다.
- 재시도에서 1600×2000 px(4:5) Gazebo 원본 사진 두 장을 저장했다. [후방 정면](../../../screenshots/2026-09-28/rear_id49_v2/rear_portrait.png)·[후방 사선](../../../screenshots/2026-09-28/rear_id49_v2/rear_three_quarter.png) 모두 기본 OpenCV 검출기로 ID 49가 검출됐고 육안으로 좌우 반전/도안 일치와 구도를 확인했다. 사진 합성·픽셀 후처리는 하지 않았다. 전체 모델 질량은 2 kg이며 직전 폐루프 런타임과 관성/충돌체 XML 15개가 동일했다.
- **촬영 성공과 정상 종료 실패를 분리한다.** `capture_passed=true`지만 전체 `passed=false`다. 사진 브리지 2개는 SIGINT 종료 대기 후 SIGTERM으로 정리됐고, Gazebo는 서버 종료 요청 뒤 세그멘테이션 오류(`-11`)로 종료됐다. launch 자식 10개 중 9개 정상 종료·1개 실패이며 잔존 프로세스는 없다. 기존 GPU 종료 문제와 원인이 같은지는 이번에 분석하지 않았다. [실행 보고서](../../../screenshots/2026-09-28/rear_id49_v2/report.json)·[원문 로그](../../../screenshots/2026-09-28/rear_id49_v2/capture.log)를 보존한다.
- 추가 차량/센서 회귀 실행은 승인 서비스의 502 오류로 시작하지 못했다. 위 마커 전용 회귀 2개와 실제 렌더링 확인을 수행한 범위로 기록하며, 전체 회귀나 마커 교체 후 주행 재검증 완료로 확대하지 않는다. 오늘은 정리 요청에 따라 추가 개발을 종료한다.
