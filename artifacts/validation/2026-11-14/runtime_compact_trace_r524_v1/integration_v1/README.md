# r523 입력 소비 예산·r524 명시 compact 기록 공통 이관

r523의 controller만6ms/guard기본2ms 제한 소비와 r524의 명시 `g1_trace_detail=compact_v1`을 공통에 연결했다. 원 stamp·64개상한·시각검사·제어 입력 snapshot/IPC는 유지한다. 기본 full 기록은 그대로이며 compact 선택은 대형 센서/경로 기록만 작은 사건 요약으로 대체한다. STOP/RESET·폐기·명령감소·기존 별도 guard/패널 로그는 보존한다.

[WSL 관련189검사](tests.log)가 통과했다. stage에서 발견한 기존 AST 테스트의 '인접2문장' 가정은 `complete_plan_trace`의 실제 조건문을 의미로 찾도록 테스트만 수정했다. 최초 셸 준비의 UTF-8 누락은 `setup_failure.log`에 보존했다. 소스 baseline 해시가 일치하지만 CRLF 때문에 git apply가 맞지 않아 검토된 stage 두파일을 해시확인 후 복사하고6ms 연결을 적용했다.

이후 r526이 같은 g1_g2_node의 current 전달 호출에 실제 dt만 추가했다. r526과 함께 실행한101회귀는 별도폴더 `fresh_current_delivery_r526_v1/common_transfer/wsl_tests.log`에 있다. 여기의 source.json은 r526 이전 이관 시점이다.

Jetson에는 아직 설치하지 않았다. r523은 긴 motion-stop을 줄였지만 짧은 command-stale0비율은 증가했으며, r526이 그 별도 전달대기를 다룬다. compact 자체의 실제Jetson 지연 개선량도 미측정이다. 다음 실물용 선택 YAML에서는 full 기본값을 바꾸지 않고 `g1_trace_detail: compact_v1`을 명시하는 것을 운영 후보로 선택한다.
