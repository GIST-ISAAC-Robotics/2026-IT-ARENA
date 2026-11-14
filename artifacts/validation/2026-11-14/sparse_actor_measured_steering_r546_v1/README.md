# r546 희소 상부 상대·실측 count 조향 레시피

**Windows 준비만 완료했다. 동결/실행 승인 대기, Gazebo 동적 주행0·이번 actor 렌더 검출0·WSL/Jetson 접근0.** r544 단독 결과를 우선하며 이번 준비로 제어기를 변경하지 않는다.

## 단일 대표

- 기준: r539의 같은 굴곡벽, 오른쪽 앞차1대, `right_side_start`, seed0,12sim초. actor 시작 중심 전방0.50m/우측0.23m, 일정0.5m/s. 기존 판정·STOP·8sim초 무진행 종료·1회 제한을 유지한다.
- 차량: `src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml`, SHA256 `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021`. 중앙 명령 상한0.28475436568260193rad, 바퀴 각도는 CAD 환산이며 실측하지 않았다.
- 상대: r537 `sparse_upper_posts` + ID1 후면 마커를 그대로 복사했다. `actor.sdf` SHA256 `43a059f3a99e0cb0bdabcf359361e3f124c0fb1e4703444b68d5b08d4d920a2a`. 전체 인쇄50mm·검정 코드34.8mm·흰 여백 한쪽7.6mm, DICT_4X4_50. 자차49·코스 ID와 구분한다.
- **첫 실행은 마커 보조 OFF**: 같은 마커가 붙은 actor를 두고 `r537_marker_profile`을 빈 문자열로 유지한다. ON은 같은 actor에서 별도 root 승인 후 별도 실행 artifact로 준비한다. 이번1회 레시피에 ON 실행은 포함하지 않는다.

실제 상대 형상은 알려지지 않은 민감도 가정이다. 기존4개8mm 지주·얇은 상판과 C1 높이134.6mm의 관계는 r537 정적 결과를 재사용한다. collision은 기존200×150×220mm의 꽉 찬 **보수 외접 상자**이며 렌더 부품별 정밀 충돌형상이 아니다. actor 하단20mm, 상단240mm, 마커 중심200mm도 r537 가정 그대로다. 새 기하 생성이나 전수 광선 감사를 하지 않았다.

## 연결과 재사용

`arguments.json`의 `--vehicle-config`가 위 공통 YAML을 선택한다. `expected_header.json`도 선택 L로 한 항목을 갱신해 옛 상한으로 비교하는 오류를 피했다. 원 r539의 expected header와 결과는 보존했다.

실행 사본의 `run.py`는 기존 `actor_sdf`에 r537의 `wrap_actor_factory`를 적용한 뒤 기존 시설3개를 합성한다. actor의 실제 시작 자세·일정은 원 평가기만 사용한다. `traffic_runner.py`, `collect_evidence.py`, 굴곡벽 `field_world.sdf`, 시설 SDF는 r539와 byte-identical이다.

`launch_adapter.py`는 `marker_course.launch.py`를 포함한다. 이 파일은 r537 launcher를 재사용하면서 BASE만 이번 `course.launch.py`로 바꿨다. 기본 빈 profile·observer false·process vision 설정은 유지했다. 이후 같은 기존 g1_follow→simulation 경로로 진입하며 `/drive`→독립 보호층→`/drive/safe` 경계를 유지한다. actor 정답/outline/일정을 제어기에 입력하지 않는다.

`collect_evidence.py --help` 등 새 실행은 하지 않았다. 수집기는 원 도구 그대로이며 실제 실행 후 원 report·raw sensor·guard/명령·접촉/간격·STOP/소유 프로세스 종료를 회수한다. OFF에서는 마커 보조 검출 성공을 요구하거나 주장하지 않는다. ON의 실제 렌더 검출·신선도·추가 처리 부하는 별도 후속 대상이다.

## root 인계

`source_selection.json`의 freeze/drive/WSL 소유권은 모두 false이고 최종 소스 선택은 pending이다. `prepared.json`, `sources.zip`, `registered.json`, `run1`, `root_execution_review.json`은 없다. 무승인 `verify_recipe()`의 동결 진입 거절을 정적으로 확인했다.

root가 r544 검토 뒤 실행을 선택할 때 현재 공통 소스와 r539 frozen source의 차이를 검토해 `source_selection.json`을 완성하고 기존 준비/동결/단1회 실행 절차를 사용한다. 이번 레시피의 예상 제어기 변경은0이며 측정 조향 YAML은 추가 입력이다. 최종 설치/source freeze 검증은 이 Windows 준비 결과를 대신해 생략하지 않는다.

재확인 명령: `python artifacts/validation/2026-11-14/sparse_actor_measured_steering_r546_v1/check_recipe.py`. 이 검사는 JSON/해시/AST/연결과 미승인 상태만 검사한다. `preparation_receipt.json`, `recipe_provenance.json`, `recipe.json`이 준비 근거다.
