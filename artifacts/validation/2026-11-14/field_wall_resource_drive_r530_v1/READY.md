# r530 자원 경로 보정 준비

Windows 준비만 완료했습니다. **공통 제어 소스 선택·동결·WSL 진입·시뮬레이션은 하지 않았습니다.** r526 delivery/r528 guard/r529 warmup 최종 통합과 root의 실행 인계를 기다립니다.

새 [field_world.sdf](field_world.sdf)는 r520 원문에서 `albedo_map` 네 곳만 바꿨습니다. `aruco/aruco_id{0,20,45,30}.png`를 최초 원본 월드 디렉터리 기준으로 해석한 뒤 새 월드 위치 기준 상대 경로로 기록했습니다. Windows 절대 경로를 넣지 않아, 승인된 후속 WSL 실행에서 기존 common resolver가 같은 저장소 자산의 Linux 경로로 해석합니다. 20/30 마커는 기존 실행 설정에서 비워지지만 원본 네 자원 모두 확인했습니다.

- 새 월드 SHA256: `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`
- 보존 r520 월드 SHA256: `9bf5ba7491153b8ef045a96e2428f44bffc9d308f4d3431c25bee1f30ee166a2`
- [resource_manifest.json](resource_manifest.json): 네 경로 변경 및 원본 자산 SHA.
- [resource_check.json](resource_check.json): 월드 텍스처 4건과 시설 STL 참조 6건, 고유 파일 5개 모두 존재. XML 전체의 URI·texture·material 파일 항목과 파일형 값을 조사했습니다. Gazebo 플러그인 6개 참조는 패키지 심벌로 분리했으며 WSL 로더 검사는 아직 하지 않았습니다.
- [geometry_equivalence.json](geometry_equivalence.json): 네 텍스트를 복원하면 전체 월드가 동일합니다. 475개 벽 collision/visual, pose, surface, physics, 지주·시설·바닥·잔디 기하는 유지했습니다. 시설 SDF도 byte-identical입니다. 기존 601 actor 표본/initial ego 정적 검사 수치를 재사용하며 원 검사 월드 SHA를 그대로 남겼습니다.
- [r520_preservation.json](r520_preservation.json): 원 실패 보고서·동결 소스·원시 입력을 포함한 r520 111개 파일의 준비 전후 SHA가 같습니다. r520 실패 판정과 텍스처 오류를 수정하거나 소급하지 않았습니다.

레시피는 기존 right_side_start, actor 높이 220mm/속도 0.5m/s, 초기 +0.5m/−0.23m, 12sim초, RTF 0.25, `g1_open1p5`, `medium_v1`, 시설 3개와 STOP/원 판정선을 유지합니다. 변경된 설정 항목은 새 파일 경로·준비 시각 표기·scope뿐입니다. 벽 형상은 진폭 30mm·주기 1m·20mm 접합 틈이라는 기존 **미실측 가정 민감도** 그대로입니다.

[source_selection.json](source_selection.json)의 승인 세 항목은 모두 false이고 최종 선택은 pending입니다. [prepare.py](prepare.py)는 이 상태에서 동결을 거절합니다. 이후 root가 최종 소스 SHA/추가 파일/검사 근거와 WSL 소유권을 인계하면 [PLAN.md](PLAN.md)의 같은 1회 실행 절차를 사용할 수 있습니다. 런타임 월드 생성 직후 [launch/simulation.launch.py](launch/simulation.launch.py)가 새 텍스처와 생성 mesh를 포함한 선언 파일의 존재/SHA를 검사하고 `runtime_resource_receipt.json`을 남기도록 준비했습니다. 이 검사는 아직 실행되지 않았습니다.

최종 준비 검사: [preparation_receipt.json](preparation_receipt.json). GUI 구성·재질 수치·메시 형상·제어 공통 소스 변경은 없습니다.

후속 root 승인으로 [traffic_runner.py](traffic_runner.py)에 기존 `/safety/status` 전체 원문 기록을 추가했습니다. 공통 평가기는 덮어쓰지 않았고 상태 갱신·구독 depth10·판정·STOP은 같습니다. `current`와 `retained_memory_command`를 포함한 발행 JSON을 필터링하지 않으며 평가기 수신 `receipt_monotonic_s`와 원래 payload의 시각을 구분합니다. [safety_capture_preparation.json](safety_capture_preparation.json)은 다섯 개의 가역적인 logger 변경과 공통/로컬 SHA를 기록합니다. 실제 입력 스냅샷에도 이 로컬 runner·파생 월드·외부 자산이 포함됩니다. 아직 계측을 실행한 것은 아닙니다.
