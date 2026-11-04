# 임시 차량 JR60P20 채택과 배선 예약 외곽

**결론:** 2026-11-04 사용자 선택에 따라 최종 CAD 전의 신규 시뮬 개발(단독·교통)은 **JR60P20**을 기준으로 한다. 센서 x=+60 mm·광축 75 mm, 앞 지주 x=+20 mm, 뒤 지주 x=−80 mm, 후면 ID49 중심 140 mm다. 뒤 지주 바깥에는 넉넉한 배선 통로를 가정하고 그 가림을 Ø20 mm 원기둥 두 개로 예약했다. 이 작업은 **모델·설정·연결의 정적 검증까지**이며 새 주행 시험은 하지 않았다. 따라서 이 구성의 주행 성능은 측정되지 않았다. [결정 기록 ADR 0029](../../../../docs/decisions/0029-temporary-jr60p20-vehicle.md) · [상면도](jr60_adoption_top.png) · [기하 계산](geometry.py) → [geometry.json](geometry.json)

> **성능 근거 주의:** [JR 전진 후보](../lidar_jr_forward_v1/README.md)의 JR60P20 33.225 s 1회는 뒤 지주에 붙인 **Ø2 mm 합성 배선** 모델의 결과다. 배선 예약 외곽을 넣은 이 프리셋은 그와 다른 모델이며, 그 랩 기록을 이 프리셋의 성능으로 옮기지 않는다.

## 1. 채택한 좌표

[프리셋](../../../../src/arena_description/config/b0183_c1_jr60p20_sim.yaml)은 1랩 기록에 쓴 [JR60P20 시험 사본](../lidar_jr_forward_v1/profiles/jr60p20.yaml)과 비교해 `scan_plane_wires` 한 키만 다르다. 카메라·IMU·마스크 정책·차체 표시 높이 같은 나머지 키는 모두 같다. 단위는 mm이고 원점은 차체 중심, +x가 전방이다.

| 요소 | 위치 | 지름 | 비고 |
|---|---|---:|---|
| C1 회전축 | (+60, 0), 광축 75 | — | 본체 55.6 mm 정사각 가정 |
| 앞 지주 | (+20, ±62) | 7 | |
| 뒤 지주 | (−80, ±52) | 7 | |
| 배선 예약 외곽 | (−88.906, ±55.308) | 20 | 실제 U채널 CAD가 아닌 표시·광선 가림 예약 |
| 후면 ID49 | (−100.5, 0), 중심 z 140 | 50 판 | 아래끝 115 mm, `--pilot-marker-z 0.140` |

**배선 예약 외곽의 정의:** 각 뒤 지주 바깥에 U자 채널을 하나씩 둔다고 가정했다. 채널 외곽은 접선 방향 폭 16 mm, 반경 방향 깊이 12 mm, 벽 두께 2 mm이며 안쪽 공간은 약 12 × 10 mm다. 깊이 방향은 센서→뒤 지주 단위벡터 u=(−0.93743, ±0.34819)다. 사각형 중심은 지주 중심 + (3.5 + 6) mm·u에 두어 안쪽 면이 지주 외면에 접하게 했다. 사각형의 반대각선이 정확히 10 mm이므로 외접원 Ø20 mm가 체결부 여유를 포함한 16×12 외곽 전체를 덮는다. 중심은 µm 단위로 반올림했으며 오차는 0.513 µm다.

| 검사([geometry.json](geometry.json)) | 값 |
|---|---|
| 사각형 모서리–외곽 중심 최대 거리 | 10.000 mm ≤ 10 mm |
| 사각형 안쪽 면–지주 중심 | 3.500 mm(지주 외면 접함). 외곽 원은 지주보다 크며 지주 속에 숨지 않음 |
| 사각형 범위 | x −97.32 ~ −80.50, \|y\| 45.72 ~ 64.90 |
| Ø20 범위 | x −98.91 ~ −78.91, \|y\| 45.31 ~ 65.31. 200×150 외곽 안, 센서보다 뒤 |
| 마커와의 분리 | 판 아래끝 115 mm > 예약 표시 위끝 105 mm(원기둥 길이 60 mm) > 스캔면 75 mm. 평면에서도 판 \|y\| ≤ 25, 예약 \|y\| ≥ 45.3 |

## 2. 스캔면 가림

시뮬 마스크는 각 원기둥의 접선각에 양끝 0.5°를 더한다. 예약 외곽은 뒤 지주와 방위가 같다(159.624°). 반폭은 4.109°로, 뒤 지주의 1.843°를 그대로 덮는다(모두 마스크 여유 포함).

| 구성 | 정면 연속(물리/마스크) | 정후방 연속(물리/마스크) | 전 방향 가림 합(물리) |
|---|---:|---:|---:|
| **JR60P20 + Ø20 예약(채택)** | 240.2° / 239.2° | **33.5° / 32.5°** | 25.3° |
| JR60P20 + Ø2 배선(33.225 s 기록 모델) | 240.2° / 239.2° | 38.1° / 37.1° | 16.2° |
| JR50 + Ø2 배선(이전 기준) | 240.2° / 239.2° | 40.7° / 39.7° | 16.6° |

정면 연속각은 세 구성이 같다. 예약 외곽 때문에 정후방 연속 개방이 약 4.5° 줄고 그림자 합은 약 9° 늘지만, **정후방 중앙은 열려 있다**. 정후방 ±15°는 어떤 구조 각도에도 걸리지 않는다. 주행 손익은 측정하지 않았다.

## 3. 같은 기하가 표시·계획기·독립 보호층에 들어가는 경로

런타임 코드는 바꾸지 않았다. 기존 `simulation.launch.py`의 `_mount_trial`이 `scan_plane_wires` 각 항목을 표시 원기둥(`c1_mount_wire_N`, 길이 60 mm, 광축 높이 중심)으로 추가한다. 같은 반복문에서 그 원기둥의 구조 각도를 계산한다. 이렇게 만든 `lidar_blind_sectors_json` 하나를 지역 계획기와 독립 LiDAR 보호층에 함께 전달하고, `known_structure_mask: structure_aware`이면 두 노드 모두 `structure_aware_v1`을 쓴다. 가려진 광선은 NaN(unknown)으로 남는다. 거리나 자유 공간 광선을 만들지 않는다. 표시 형상만 추가하므로 질량·관성·충돌체는 바뀌지 않는다. 마커를 140 mm로 올리면 마커 각도는 추가되지 않아 구조 각도는 지주 4개와 예약 외곽 2개, 모두 6개다. 기본 90 mm 마커라면 정후방 마커 각도가 하나 더 생긴다.

## 4. 앞으로의 실행 인자

**단독 1랩**(Jazzy·작업공간을 source한 WSL): `python3 scripts/run_current_solo.py <새 결과 경로>`. [진입점](../../../../scripts/run_current_solo.py)은 이 프리셋과 `--pilot-marker-z 0.140`을 넣는다. 출력·프로필 경로를 빼면 33.225 s 기록의 등록 인자와 같다. `--print-args`는 실행하지 않고 조합만 출력한다. 이전 JR50 재현은 `--previous-jr50`을 쓴다.

**교통:** 교통 검사기에는 마커 높이 인자가 없었다. 단독 검사기와 같은 의미의 opt-in `--pilot-marker-z`를 [validate_g1_traffic.py](../../../../scripts/validate_g1_traffic.py)에 추가했다. 범위는 0.115~0.30 m이고 G1 계열 모드(g1/m3/g1g2)에서만 쓸 수 있다. 값은 `pilot_marker_z:=` launch 인자로 그대로 전달되며, `report.json`에는 센서 YAML 경로·해시와 마커 높이가 기록된다. 지정하지 않으면 이전과 같은 launch 인자를 쓴다. 11/4 JR 교통 장면과 같은 조합이면 다음과 같다.

```bash
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh /usr/bin/python3 -B scripts/validate_g1_traffic.py \
  --mode g1g2 --handoff-reserve paced --handoff-yield-progress --handoff-rear-sweep --official-signal-course \
  --config "$PWD/<장면 설정 json>" --case <장면> --seed 0 --record-follow-decisions --handoff-observed-motion \
  --side-departure-trial --current-lane-preference-trial \
  --local-sensor-config "$PWD/src/arena_description/config/b0183_c1_jr60p20_sim.yaml" --pilot-marker-z 0.140 \
  --output "$PWD/<새 결과 경로>"
```

`structure_aware` 프로필은 계획기와 보호층 모두에 `bounded_local_v4`를 요구한다. 위 g1g2 handoff 조합은 이 조건을 launch 인자로 넣는다. 일반 `--mode g1`을 쓸 때는 `--partial-observation-policy bounded_local_v4 --record-follow-decisions`를 함께 지정한다. 이 교통 명령은 **실행하지 않았다**.

## 5. 검증

모두 주행 없는 정적 검사다. Gazebo를 띄우지 않았고 교통을 실행하지 않았으며, 빌드도 하지 않았다. launch 파일은 `install/`에 심볼릭 링크로 설치되어 있고, 바꾼 파일은 YAML·검사기·진입점·시험뿐이다.

- [기하 계산](geometry.py) → [geometry.json](geometry.json)·[로그](geometry.log): 검사 8개가 모두 참이고 [exit 0](geometry.exit)이다. 검사 항목은 외곽 포함, 접함, 200×150 안, 센서 뒤, 정후방 개방, 지주 그림자 포함, 마커 높이/평면 분리다.
- [새 회귀](../../../../tests/test_jr60_adoption_profile.py) 7개가 대조하는 항목은 다음과 같다. 1랩 기록 프로필과의 키 차이(배선 한 키), 좌표와 외곽 포함·접함·외곽선 안, 실제 `_mount_trial`의 표시 원기둥 위치·반지름과 구조 각도의 일치, 질량·충돌체 불변, 같은 각도 문자열의 계획기/보호층 검증, 정후방 ±15° 미마스크와 예약 그림자 NaN, 마커 높이와 기본 90 mm일 때의 추가 각도, 진입점 인자와 33.225 s 등록 인자의 일치, 교통 검사기의 범위/모드 거절(exit 2)과 launch 사슬(g1_follow→local_pursuit→demo→simulation)의 `pilot_marker_z` 전달이다.
- [관련 회귀](tests.sh) 222개 중 **221 통과·1 실패**([로그](tests.log), [exit 1](tests.exit)). 실패 `test_r0_safety_contract.py::test_current_g1b_r0_contract_allows_only_the_registered_launch_change`는 변경 전 [기준 실행](baseline_before.log)(209 통과·1 실패)과 같은 해시 불일치이며, 이번 변경과 무관한 기존 미해결 항목이다. 출처 사슬 검사 `test_source_provenance_r0_plus_b3.py`는 통과했다.
- 교통 검사기 변경은 r5 뒤 후속 계약 [r6](../../../../config/tests/lidar_mount_traffic_marker_r6_sha256.json)에 기록했다. 변경 전 바이트는 [traffic_marker_before.zip](traffic_marker_before.zip)에 HEAD 바이트로 보존했다([등록 코드](register_contract.py)). 이전 계약과 등록 해시는 수정하지 않았다.

검사 명령: `MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc 'bash "<저장소>/artifacts/validation/2026-11-04/lidar_jr60_adoption_v1/tests.sh" tests'`. 기하는 WSL `python3 -I -B geometry.py`, 상면도는 `python3 -I -B draw_top.py`(Pillow)로 만들었다.

## 6. 한계와 이후 순서

- 예약 외곽은 보수적인 가림 예약이다. 실제 채널 형상·강도·체결·하네스 굵기를 실측하거나 CAD로 확인한 것은 아니다. 채널 외곽 \|y\| 64.9 mm는 뒤바퀴 안쪽면 61.5 mm보다 바깥이다. 스캔면 75 mm는 바퀴 위끝 65 mm보다 10 mm 높지만, 채널이 그 아래로 내려가면 바퀴와 간섭하는지 CAD에서 확인해야 한다. 표시 원기둥(45~105 mm)은 시뮬에서 뒤바퀴 표시와 일부 겹치지만, 질량·충돌이 없고 단일 스캔면만 보므로 광선 결과에는 영향이 없다.
- 새 주행 결과는 없다. 다음 성능 시험을 할 때는 위 인자로 새 경로에 기록하고, Ø2 mm 배선 기록과 구분해 비교한다.
- 하드웨어가 구조·안정성 이유로 이 배치를 거절하면, 대안은 C65 또는 앞 지주를 조금 더 앞으로 옮기는 안이다. 이 대안은 **나중에 시험할 후보**일 뿐이며 이번에 실행하거나 개발하지 않았다.
- 범용 `local_pursuit.launch.py` 기본(95 mm·후방 60°)과 과거 등록 실행은 각자의 설정을 유지한다.
