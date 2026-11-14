# r537 희소 상부 상대 한 종류의 실행 준비

**정적 준비 완료, 주행·Gazebo·Jetson 실행0.** 공통 생성기/제어기는 변경하지 않았다. 실제 상대 형상은 미상이며 아래 치수·마커 부착 높이는 민감도 시험 가정이다.

기존 `sparse_upper_posts`를 그대로 재사용했다. 마커 추가 전 SDF는 [r341 보존본](../../2026-11-13/sparse_upper_r341_v1/sparse_upper_posts.sdf)과 byte-identical이다. 하부는200×150×80mm, 하단20mm·전체 높이220mm, 상부는8×8mm 지주4개와4mm 덮개다. C1 z134.6mm에서는 지주4개만 교차한다. 감지 형상은6부품이지만 **collision은 기존200×150×220mm의 꽉 찬 보수 외접 상자**다. 정밀 부품 접촉 모델과 다르며 이를 일치한다고 설명하지 않는다. 실제 하부의 평면 외곽과 평가용200×150mm 외곽은 일치한다.

현재 `c1_standard10_native_angles_v1.json`의 첫512광선 배열 하나로만 비교했다. 같은 실제 각도·원천 index를 유지하며 원시 제어 입력으로 쓰지 않았다.

| 자차 기준 상대 중심 | 센서–상대 중심 거리 | 희소 상부 반환 | 꽉 찬 외곽 반환 |
|---|---:|---:|---:|
| 오른쪽 앞(0.50, −0.23)m | 0.483m | 9 | 38 |
| 정면(0.80, 0)m | 0.725m | 4 | 20 |
| 정면(1.20, 0)m | 1.125m | 2 | 12 |

현재 CAD 지주/전방6mm 배선의 최대X는44.612mm이며3mm 움직임·2mm 거리 여유를 더해도 C1 X75.4mm 뒤다. 이 세 대표의 반환 방향은 전방이므로 해당 명목 단면과 교차하지 않는다. 실제 배선·반사율·빔 폭·움직임·광선 사이 영역의 검증은 아니다. 기존 r341의 정지 Gazebo 2점 반환은 과거 근거이며 이번 새 렌더 검증으로 세지 않는다.

## 동일 actor의 후면 마커 OFF/ON

`actor.sdf`에는 현재 공통 profile의 **DICT_4X4_50 ID1** 한 개를 붙였다. 코스0/20/30/45 및 자차49와 다르다. r466이 보존한 KAIST 제출 PNG의1000×1000px/검정 bbox696×696px를 다시 확인했다. 전체 인쇄50mm, 검정 코드34.8mm, 흰 여백 한쪽7.6mm이며 원본 이미지는 `marker_source.png`로 바이트 그대로 보존했다. 기존 `experimental_facilities.marker_cells`와 `box`를 재사용했고 셀 비트가 원본 이미지와 같은지 검사했다. 외부 texture/mesh URI가 없어 미해결 렌더 자산 경로는 없다.

코드 바깥면은 차량 후단X=−100mm와 일치한다. 중심Z=200mm/흰 판175–225mm로 C1 단면을 가리지 않는다. 판/잉크는 모두 기존 collision 상자 안에 있다. 기존 `rectangle` 및 근사 PnP의 이상적인 픽셀 꼭짓점 감사에서 후단 기준점과200×150mm 영역이 맞았다(최대 수치 차이2.1e−12m). **이 값은 실물 거리 정확도나 Gazebo 검출 성능이 아니다.**

- OFF: 같은 마커가 붙은 `actor.sdf`를 쓰되 `r537_marker_profile:=`를 비운다. LiDAR 입력과 기존 제어만 사용한다.
- ON: 같은 SDF에서 `r537_marker_profile:=<저장소 절대경로>/config/perception/vehicle_marker_approx_b0390.json`을 지정한다. `marker_course.launch.py`가 실제 노드의 `vehicle_marker_profile_path`에 전달하며 영상 처리는 process이다. 기존 관측 전용 `vehicle_marker_observer_enabled`와 점유 보조 profile은 서로 다른 옵션이다.
- ON도 실제 이미지 검출/신선한 packet이 있어야 보조 영역이 생긴다. 미검출·가림·소실을 자유 공간이나 새 추월 허가로 쓰지 않는다. 실제 렌더 검출, 현재 실행 부담, OFF/ON 주행 비교는 미실행이다.

## 다음 실행 담당 인계

1. r530 종료/회수 후 그 실행 레시피를 **새 artifact 실행 사본**으로 준비한다. 원 r530 `run.py`나 등록/회수 파일을 덮어쓰지 않는다. `config.json`의 한 actor·`right_side_start`, seed0,12sim초,0.5m/s 일정을 사용한다. 기존 판정선은 바꾸지 않는다.
2. 새 사본의 평가용 actor 생성 연결에 `actor_fixture.wrap_actor_factory(existing_actor_sdf)`를 적용한다. 기존 시설 합성 wrapper와 함께 사용할 수 있으며 `eval_g1b_actor`만 변경한다. 실제 시작 자세는 기존 평가기가 공급하고, 정답/일정은 제어기에 전달하지 않는다. 원점의 `actor.sdf`는 정적 기준본이다.
3. 새 사본의 launch 진입점을 `marker_course.launch.py`로 연결한다. 기본 OFF 한 회를 우선한다. ON 비교 시 같은 SDF/기하/일정과 동일 소스 동결을 유지한다. 이 파일은 r530의 보존된 시설 launch를 읽기 전용으로 포함한다.
4. 실행 전 actor SDF·config·adapter·launch·등록 profile을 소스 동결에 포함한다. 실제 C1/영상 원문·마커 packet·`/drive/safe`·접촉/간격·STOP/종료를 회수한다. `audit_actor_outline_returns.py`의 기존 중앙±50mm 오목면 전용 판정을 이 지주형에 그대로 사용하지 않는다. 기존 원시 광선/평가 좌표 변환은 재사용할 수 있지만 판정 대상은 SDF의4개 지주다.

재현: 저장소 루트에서 `python artifacts/validation/2026-11-14/sparse_actor_recipe_r537_v1/prepare.py`. Windows 정적 확인만 수행한다. `audit.json`, `recipe.json`, `manifest.json`에 근거/입력 SHA를 남겼다. 새 제어기·전체 매트릭스·추가 장비 실행은 없다.
