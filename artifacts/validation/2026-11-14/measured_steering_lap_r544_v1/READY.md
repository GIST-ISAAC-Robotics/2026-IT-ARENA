# r544 단독 전체랩 실행 준비

r534 미실행 레시피의 actor0·grid3·시설3개·80 sim초 상한·8 sim초 무진행 STOP·RTF .25·g1_open_1p5·medium_v1·원 판정을 유지한다. 월드 SHA `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`는 r530/r539와 같으며 첫 직선 두 벽만 미실측 가정으로 변형됐다. 기하를 새로 생성하지 않았다.

차량 설정은 공통 `src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml`을 명시 선택한다. SHA `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021`, 중앙 명령 상한 .28475436568260193 rad, YAML 내륜 관절 상한 .30666234131856146 rad다. count 범위에서 CAD로 파생했으며 실제 바퀴각 실측값이 아니다. 질량·속도·접지·센서와 월드 조건은 그대로다.

제어 소스는 r539 이후 r540의 정확히 같은 각도 계산 재사용 1파일을 포함한다. root의 portable C++ 401목표 및 관련 pytest110 통과 영수증을 재사용하며 같은 검사를 반복하지 않는다. r543 compact 계측은 채택하지 않고 기존 `/safety/status` 전체 원문을 기록한다. root가 최종 동결·WSL 독점·단독 1회를 승인했으며 실패 후 같은 조건 자동 반복은 하지 않는다.

[준비 조건](final_recipe_manifest.json), [소스 선택](source_selection.json), [헤더 변경 근거](header_baseline_change.json)를 보존한다. 헤더는 조향 상한 한 키만 바꿨다. [실행 조향 감사](steering_runtime_audit.py)는 launch가 실제 생성한 controller/guard/virtual MCU/actuator 파라미터와 runtime SDF 관절·plugin 값을 기록하며 값을 변경하지 않는다.

동결 ZIP에는 전체 관측 목록을 남기되 `prepared.json.frozen_inputs`에는 실행 필수 입력을, `observed_inputs`에는 범위 밖 참고 목록을 구분한다. 필수 입력과 ZIP 불변은 실행 조건이며 관측 목록 변경은 별도 기록한다. r539의 미사용 도구 변경에 의한 전체 목록 FAIL과 원 주행 PASS는 소급 수정하지 않는다.

원 보고서·STOP·정상 종료·소유 PID0·실제 입력 SHA·전체 status·runtime 조향 전달·완주/미완주를 각각 수집한다. 지금 이 문서는 준비 영수증이며 새 주행 성과가 아니다. Jetson/ESP32/물리 센서 접근은 하지 않는다.
