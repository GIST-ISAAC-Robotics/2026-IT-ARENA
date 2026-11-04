# 인계: 새 공식 월드 전방 교통

작성 2026-11-02 03:44 UTC. 5시간 사용량 경고(0.91, 7일 0.61)에 따른 Claude→GPT 인계다. 결과의 기준 보고서는 [README](README.md)이고 이 문서는 재개 정보만 담는다.

## 실행 상태

- 3 묶음 모두 **커밋되지 않았다.** HEAD는 `59c3c3b0`, 작업 트리에 변경 66항목이다(옛 방지턱 meshes 삭제 포함).
- 소유 PID·세션 **없음.** `gz sim`, `ros2 launch`, 검증기, 브리지, 신호 제어기, pytest 잔존 0을 03:43 UTC에 확인했다.
- Haiku 보조 실행은 모두 끝났다. 진행 중 작업 없음.
- 소스 계약 사슬: `official_forward_r1` 9/9 현재 바이트 일치. `official_facility_r1`과 `official_start_region_r1`의 `simulation.launch.py` 불일치는 후속 계약이 이어받은 정상 상태다.

## 인계한 변경의 범위

| 묶음 | 산출 | 상태 |
|---|---|---|
| 공식 시설 연결 | [official_facility_v1](../official_facility_v1/README.md) | GPT 주 에이전트 검토 수용. run1 35.606초는 수정 전 검출기 결과 |
| 선두 검출 영역 | [official_start_roi_v1](../official_start_roi_v1/README.md) | GPT 주 에이전트 검토 수용. 수평 선두 2칸은 물리 시야 미완료, 20°는 기구 후보 |
| 전방 교통 | 이 폴더 | **미완료. GPT 주 에이전트 검토 전** |

## 전방 장면 판정

| 장면 | 실행 | 옵션 | 판정 | 구체 사유 |
|---|---|---|---|---|
| A `moving_stop_restart` | a1 | OFF | 실패(run_valid true) | 재출발 순간 우회 진입, 7초 0.35 m/s, 출구 미도달, 방향 75°, final_gap 3.00 m |
| A | a2 | ON | **실패**(run_valid true) | `hold`·`final_gap`(1.73 m)·`curve_exit_reached`(0.60 m 전)·`inside_road` 실패. 접촉 0, 두 바퀴 잔디 3.19초 |
| B `cut_in` | b1 | OFF | 실행 무효 | 평가기의 정지 표본 요구 때문. 주행 게이트는 통과. 이후 평가기 수정 |
| B | b2 | ON | **통과** | 자차 약 1.2 m 뒤 합류. 근접 끼어들기 아님 |

`passed=false`인 실행을 통과로 다시 표시하지 않는다. a1·b1은 옵션 추가 전 소스의 기록이며 `source_snapshot.zip`에 보존했다.

## 실패의 원인과 수정 가설

**남은 결함(a2):** 앞차가 3초에 정지하고 자차는 약 5.5초에 BYPASS로 들어간다. 앞차는 8초에 재출발한다. 그런데 `BYPASS` 분기는 `entry_phase == 'centered'`가 되어야 벗어나므로 12.3초까지 0.35 m/s를 유지한다. 복귀 뒤에는 1.5 m/s까지 회복한다.

수정 가설(코드 미작성, 미검증):

1. `bypass_handoff.py` `_command` 끝의 BYPASS 경로에서 현재 전방 조각의 전진(또는 조각 소실과 G1 경로 복구)을 관측하면 우회 유지를 줄인다.
2. 단, a2의 앞차는 옆을 지나는 동안 자차 측면에 있었고 최소 간격 0.268 m였다. 옆면 관측이 없는 상태에서 복귀를 앞당기면 측방 위험이 있다. `normal` G1이 양수 명령과 경계를 낼 때만 허용하고 기존 `reacquire` 2스캔 확인을 유지해야 한다.
3. `rejoin` 단계 상한 0.35 m/s의 완화는 `bypass.config.max_speed_mps`를 쓰는 진입 검사와 분리해야 한다. 이 값은 진입 허용선이기도 하다.

**확인 안 된 것:**

- 전진 시작 조각의 진입 차단(`entry_motion_gate`)은 단위 시험만 통과했다. a2에서 조건이 나타나지 않았다. a1 재현 조건(앞차 근접 정지 후 재출발)이 필요하다.
- 기둥 앞 0명령은 옵션 ON에서도 남는다(A 0.28초, B 0.08초). 세계 고정 판정에 세 스캔이 필요해 첫 GUARD를 막지 못한다. 측방 창 진입 전 전방 반환을 추적하면 줄일 수 있다고 추정한다.
- 느린 측방 상대(약 0.25 m/s 이하)의 고정 오분류 위험은 미시험이다.
- MCU·모터축 단계별 대조를 하지 않았다. 보호 출력과 차체가 계획을 따른 것까지만 확인했다.

## 재개 명령

WSL 저장소 루트 기준이다. Git Bash에서는 `MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc 'bash "/mnt/c/.../스크립트" 인자'` 형식으로 호출한다.

```bash
A=artifacts/validation/2026-11-02/official_forward_v1
bash $A/build_test.sh                                                # 소스 수정 뒤 빌드·영향권 시험, build.exit 확인
bash $A/scene.sh moving_stop_restart a3 --handoff-observed-motion    # a3, 기존 이름 재사용 금지
bash $A/scene.sh cut_in b3 --handoff-observed-motion
python3 -I -B $A/analyze.py a3                                       # 1초 단위 계획/보호/차체, 게이트
python3 -B scripts/surface_contact_eval.py $A/a3 --output $A/a3_surface.json
```

`scene.sh`와 `analyze.py`는 WSL에서 실행한다. Windows Python에는 numpy가 없다. 소스를 고치면 `prepare_contract.py`의 기존 계약을 덮어쓰지 말고 새 후속 계약을 만들어 `tests/test_source_provenance_r0_plus_b3.py`의 `M3_SUCCESSORS`에 잇는다. 파일별 줄바꿈이 다르다(`simulation.launch.py`는 CRLF, 일부는 혼합). 바이트 단위로 고치고 Git diff로 전파일 변환이 없는지 확인한다.

## 남은 문서·회귀

- 문서: 이번 전방 결과는 [README](README.md), 현황, 계획, 활동 기록에 반영했다. a3 이후 결과로 교체할 항목은 `docs/PROJECT_CONTEXT.md`의 결과 표 첫 전방 행, 미해결 2번, 다음 작업 첫 항목이다.
- ADR: 관측 이동 옵션을 기본 활성화하기로 하면 ADR 또는 계획 판단 기록이 필요하다. 지금은 기본 OFF다.
- 회귀: 전방 묶음 수정 뒤 전체 회귀는 실행하지 않았다. 마지막 전체 결과는 시설 묶음의 7,334 통과·13 실패(변경 전과 같은 항목)다. 그 뒤 영향권만 돌렸고 `finish_checks.sh`는 264 통과다. 현재 코드 기준 전체 회귀 1회가 커밋 전에 필요하다.
- 범위 밖 미착수: `side_left`·E3 장면 이전, 교통 거리 시험의 새 월드 이전, 단독 기둥 구간 상한의 스캔 재현, `lidar`/`stereo` 데모 출발.
- 저장소 공개 전 확인: `.gitignore`에 세 폴더의 원시 기록 제외를 추가했다. `official_facility_v1/README.md`의 일부 링크는 실행 뒤 산출물을 가리키므로 커밋 범위에 넣어야 한다.
