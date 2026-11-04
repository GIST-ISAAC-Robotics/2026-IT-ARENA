# C 배치 적응 v2 — 인계 상태(2026-11-04, 5시간 사용량 경고로 중단)

작업 담당: Claude Opus 5.5 → GPT 주 에이전트(구현·검토)·GPT-6 Luna(실행)로 인계. **시뮬레이션·테스트·빌드는 한 번도 실행하지 않았다.** 이 워커가 띄운 프로세스·백그라운드 명령·하위 에이전트는 없다(시작 시 `pgrep`로 실행 중 시뮬 없음 확인).

## 고정 스냅샷

- 수정 전 HEAD: `before/HEAD.txt`(dd3c54fb…; v1 변경은 이미 이 커밋에 포함, 작업트리는 깨끗했음).
- 수정 전 바이트: [`runtime_before.zip`](runtime_before.zip) — partial_observation·local_follow·local_pursuit·lidar_safety·local_scene·local_path·simulation.launch.py·validate_basic_autonomy·validate_g1_traffic·test_source_provenance_r0_plus_b3. v1 계약 `lidar_mount_trial_r1`의 after 해시와 일치(local_pursuit 4569fb9f…, lidar_safety 6672bb70…, simulation 1da94496…).

## 변경 파일(미커밋, `git diff --stat` 4파일 +67/−3, 신규 1)

| 파일 | 상태 |
|---|---|
| `src/arena_autonomy/arena_autonomy/known_structure.py` (신규, LF) | 정책 본체 `structure_aware_v1`: `validate`, `parse_sectors`, `classify`, `structure_policy(base, sectors_json)` 클래스 팩토리, `StructureAwareMixin`(evaluate/_shadow/_inside/_structure) |
| `partial_observation.py` (CRLF 유지) | `structure_excused` 속성(기본 없음 → 기존 동작 동일)이 있으면 `_excused_audit`로 설명된 구조 광선만 결측일 때 ok, 결측 run 계산에서 제외 |
| `local_follow.py` | `FollowConfig.known_structure_policy='off'`·`known_structure_sectors_json=''` + 검증(v4 필수)·양수 검사 제외 목록, `LocalFollow`가 정책 클래스를 감쌈, `_partial_command`의 변환 공백 검사에서 설명된 구조 광선 제외, 구조 제한으로 0이 된 명령 사유 `follow_structure_shadow_stop` |
| `local_pursuit.py` | 파라미터 `known_structure_policy`(기본 off) → FollowConfig, G1 아닌 경우 거절 |
| `lidar_safety.py` | 같은 파라미터·검증·클래스 감싸기, 구조 제한이 0이면 사유 `structure_shadow_limited` |

`py_compile` 5파일 통과(Windows Python, 실행 검증 아님). 줄끝 CRLF 유지 확인.

## 정책 설계와 위험 경계(구현된 내용)

1. **구분:** 구조 각도(=v1 `lidar_blind_sectors_json`, mask_scan과 같은 센서축 규칙) 광선은 계속 NaN. 양옆 광선이 모두 유효 반환일 때만 일반 결측 예산에서 뺀다. 구조 밖 결측·옆이 끊긴 구조 각도는 기존 v4 그대로(1.2 m/s 초과 불허, 6°/3° 예산, 이전 점 래치). 거리 생성·자유 공간 채움 없음.
2. **그림자 기하:** 구조 광선의 운동 보정 취득 방향으로 그림자(평균 원점, 시작/끝 방향)를 만든다. 완전히 숨을 수 있는 폭 `2r·sin(각폭/2)`이 5 cm(v4 측방 선례 `maximum_chord_m`, 규정값 아님)를 넘는 거리부터 제동 지평까지의 먼 그림자를 3선 표본(3 cm)으로 만들고, 현재/직전 조향 `swept_limit`의 첫 간섭 거리로 `braking_speed` 상한만 둔다(점군·경로에 넣지 않음).
3. **숨은 기억:** 직전 스캔 반환(≤3 m)이 이번 스캔 그림자로 들어가고 현재 반환 2 cm 안에 재관측되지 않으면 정지 물체로 가정해 자차 운동으로만 옮겨 기억(최대 512점). 새 스캔에서 그림자를 벗어나면 해제. 기억 점의 `swept_limit`로 같은 제동 상한.
4. **남는 위험(측정 필요, 보증 아님):** 5 cm보다 좁고 이전에 보이지 않은 그림자 속 물체, 그림자 안에서 움직이는 물체(기억 정지 가정·정지 중 해제 지연 가능), 부분 노출 물체의 숨은 부분(보이는 옆면에서 ≤5 cm). 교통 우회 복귀 띠(`bypass_entry_trial._rejoin_band`의 `depth_support`)는 **적응하지 않았다** — C 앞 지주 그림자가 띠 앞부분을 가려 복귀 지지 실패가 늘 수 있다(미확인).

## 미완료(정직한 상태)

- **launch 배선 미구현:** `simulation.launch.py::_mount_trial`이 아직 `known_structure_mask in ('none','unknown')`만 받는다. 필요: `structure_aware` 값 허용(각도 계산은 unknown과 동일), 그 경우 `known_structure_policy: structure_aware_v1`을 lidar_safety 파라미터(약 1316행)와 controller_parameters(약 1390행) **두 곳 모두**에 전달. 기본 YAML 경로는 파라미터를 추가하지 않게 유지.
- **테스트 0건 작성·실행:** 필요 항목 — 설명된 구조 공백은 unknown 유지(NaN 불변), 옆이 끊긴 구조/구조 밖 결측은 v4 그대로 거절·1.2 m/s 규칙, 가까운/보이는 장애물 제동 유지, 회전 시 먼 그림자 상한·숨은 기억 상한, 기본 off 동일(FollowConfig·두 노드·`structure_excused` 없음), 정책 off에서 sectors 문자열 거절. 계산 비용(50 Hz 두 노드)도 측정 필요.
- **출처 계약 미작성:** `config/tests/lidar_mount_adapted_r2_sha256.json`(부모 `lidar_mount_trial_r1`, before=`runtime_before.zip`, v1 `prepare_contract.py` 복사) + `tests/test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS`에 추가(CRLF). 소스 최종 후에 생성.
- **프로필 미작성:** v2 `profiles/`에 공통 높이 75 mm·`chassis_visual_top_m: 0.065`·`known_structure_mask: structure_aware`로 JR(50; 앞 (+10,±62), 뒤 (−80,±52)), C 명목(30; (±80,±62) Ø7), C 전진 후보. 기구 판단 자료: C1 본체 55.6 mm 사각, 앞 외곽 x=+100 → 앞면 여유 x65 7.2 mm·x70 2.2 mm(케이블·마운트·조향·범퍼 미모델), C1 최소거리 50 mm로 x60 이상은 범퍼 앞 일부가 최소거리 안. 앞 지주 방위 x30 51.1°, x65 76.4°, x70 80.8°(여전히 ±100° 안 → 정책 필요). 앞 지주 후퇴(센서 뒤로) 선택지는 JR/D 계열로 수렴함을 설명할 것.
- **실행·평가 도구 미작성:** v1 `run_solo.sh/py`, `run_traffic.sh`, `evaluate.py`를 v2로 복사하고 SOURCES에 `known_structure.py`·변경 파일 추가, 평가에 safety status `partial_observation.known_structure`(상한 시간·기억 점수·미설명 구조) 집계 추가.
- 문서(README·현황·활동일지) 미갱신. v1 폴더는 손대지 않았다.

## 다음 단계(권장 순서)

1. launch 배선 → 단위 테스트 신규 파일(예: `tests/test_known_structure_policy.py`) → 기하 선별 스크립트(같은 모듈 함수로 센서 x 30–70의 그림자 방위·5 cm 거리·조향별 상한) → 계약·사슬 추가 → v1 `tests.sh` 목록+신규 테스트 실행(기존 미해결 2실패 `test_r0_safety_contract` 해시·`test_bypass_corridor_trial` 문자열은 수정하지 않음) → colcon 빌드.
2. 실행(동시 실행 금지, 각 회 `.exit` 영수증): C 명목 짧은 진단(25 s) → 출발하면 1랩 ×2, C 전진 최선안 ×2, 새 같은 빌드 JR(같은 정책) ×2, 성공 구성마다 전방/측방 교통 1장면(`run_traffic.sh <이름> close_cut_in <프로필>`, 기존 공식 각본). 실패 시도 보존, 원본 판정 유지.
3. 재현 형식(v1과 같음, WSL 저장소 루트): `bash artifacts/validation/2026-11-04/lidar_mount_adapted_v2/run_solo.sh <이름> <프로필.yaml> [최대초]`. Git Bash에서는 `wsl.exe -e bash -lc 'cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" && bash …'`.
