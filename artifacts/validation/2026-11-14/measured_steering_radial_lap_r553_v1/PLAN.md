# r553 실측 조향 단독 후보 준비

**Windows에서 명령과 PLAN만 준비한다. 현재 GO 전이며 WSL 진입·빌드·소스 동결·새 주행은 하지 않는다.** root의 r552 회귀 결과·최종 SHA·WSL 독점 소유권과 실행 GO를 받은 뒤 아래 순서를 수행한다. 사용자 STOP은 없으나 이 준비 문서를 GO로 취급하지 않는다.

기준은 r550 원 실행이다. actor0·grid3·시설3개, 같은 fieldworld와 measured YAML, `g1_open_1p5`·`medium_v1`, RTF 설정.25,80 sim초 상한,8 sim초 무진행 STOP, 기존 노면/독립 guard/STOP/원 판정을 유지한다. 중앙 controller/guard 상한은 .28475436568260193 rad, MCU/actuator 물리 중앙 상한은 .28475438456006436 rad, SDF 내륜 joint/plugin 상한은 .30666234131856146 rad다. 실제 바퀴각 실측값이 아니라 사용자 count 범위의 CAD 파생값이라는 한계도 유지한다.

이번 후보는 r549 revision3의 실제 ray 방향 정렬과 r552 저속 helper의 원판 운동 여유 및 current/retained에 대한 동일 재검사다. 기본 normal 고속 sweep와 독립 guard의 판단 변경은 없다는 인계를 받았으며, 실제 최종 diff/SHA는 GO 시 확인한다. 기존 revision2의 helper 최소 관측 .207 m를 유지한다. 다음 실행의 결과를 기존 r544/r550 실패에 소급하지 않는다.

준비 기준값:

- r550 원 보고서 SHA: `90bbf5f6b759226de9678403462222fabbcffb4a0d500a2d68546fd3337da397`
- r550 동결 ZIP SHA: `6dfc3632dfd84b24010dec8bf73eb41c92934137137cd6d56cae3ad63bc3474f`
- fieldworld SHA: `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`
- measured vehicle YAML의 r550 SHA: `dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021`
- native 바이너리 SHA: `1bbe89cdcff6662fda699847756ff9c776e1b70c0d406ec97df469666dc3b16c`
- expected header SHA: `258630d5a24657d86c7a4a06cc22cc3174cdd3ab0801df78beb5ab9f09b0928d`

`config/hardware/steering_zero.assembled_2026_11_14.json`의 `evidence_note` 정정으로 profile SHA가 바뀔 수 있다. r550 actual source map에는 이 hardware profile이 포함되지 않았고 실행은 위 vehicle YAML을 선택했다. GO 시 metadata-only diff인지와 실제 import/수집 포함 여부를 확인해 최신 SHA를 별도로 기록한다. 기능값이 같다는 확인 없이 오래된 profile SHA를 강제로 요구하거나 새 profile을 자동 채택하지 않는다. 이전 기록의 SHA는 수정하지 않는다.

GO 후 실행 순서:

1. 경쟁 Gazebo/ROS/pytest/build 프로세스0 및 WSL 소유권을 먼저 확인한다.
2. r550 레시피를 새 폴더에 복사하고 artifact 경로만 바꾼다. 새 최종 source manifest와 r552 회귀 영수증을 연결하고 기본 상태 기록 `full`,7개 실제 조향 감사와 기대 헤더를 유지한다. 기존 geometry/static/resource 영수증을 재사용한다.
3. 승인된 단일 패키지 빌드가 필요하면 `arena_autonomy`만 수행한다. native 바이너리 SHA를 전후 비교하고 변경되면 자동 채택하지 않는다. 실제 source import 경로/SHA를 확인한다.
4. 실제 사용 입력을 필수 목록으로 동결하고 전체 참고 inventory는 별도 관측 목록으로 남긴다. 다른 세션의 미사용 script/test 변경은 원 diff를 보존하되 주행 실패와 섞지 않는다. ZIP·필수 입력 SHA·명령·root 실행 인계를 기록한다.
5. [준비 명령 인수](arguments.json)의 단독1회를 실행하고 실제 PID/로그를 보고한다. 주요 진행·8 sim초 정체/기술오류·종료 시점에 보고하며 같은 실패를 자동 반복하지 않는다.
6. 원 STOP·실제 안정 정지·외부 조기 STOP의 원 부가 판정·정상 종료·PID0·입력 불변·actual RTF를 따로 회수한다. PID0 확인 후 WSL을 우선 반환하고 보고서와 대표 정체 입력 발췌는 Windows에서 마무리한다.

[명령 계획](commands.json)은 실행 순서를 위한 데이터다. 아직 레시피 파일·새 ZIP·실제 실행 입력 해시·run 폴더가 없으며 이 단계에서는 자동 실행하지 않는다. Jetson·ESP32·물리 센서에는 접속하지 않는다.
