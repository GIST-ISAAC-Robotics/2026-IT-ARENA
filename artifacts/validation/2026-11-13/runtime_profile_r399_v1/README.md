# 최종 CAD 실물 운영 설정 전달 후보 r399

상태: **설정 생성기·검사 준비, 미실행**. Windows AST 확인만 수행했으며 단위검사·WSL·Jetson·ROS·장치는 실행하지 않았다. 공통 소스, r394 동결 판본, r398 정책과 기존 주행 기록은 수정하지 않았다. 산출물은 기하/관측 계약 overlay이며 전체 운영 launch 또는 실차 완료가 아니다.

동결 `source_manifest.json`은 관련27파일을 보존한다. manifest SHA256은 `4611e519114feb3375ddfb1fbcd9da51910d2bb0c38c0bb213bab1dab561ce90`, 생성기는 `bd19aac8812b2e4b21308de9f95b0d21c0b4b3174b1ebdacc00c1146a8d7907b`, 검사는 `0107cb3d281682303bc570bcb08f09c3cfe3d3e0172d49b16480c0215e6e81ea`다. 실행 전후 해당 manifest로 대조한다.

## 확인한 누락과 범위

| 경로 | 현재 관측 | 이번 준비 |
|---|---|---|
| `config/hardware/g1_input_shadow_v1.yaml` | `local_pursuit`만 설정한다. 축거 .135, preview IMU, 구조 mask 없음이다. gear/radius는 최종값이나 시각/실행은 preview 선택이다. `lidar_safety` 설정으로 읽히지 않는다. | 이 파일을 운영 설정으로 복사하지 않는다. 이름이 명시된 두 노드의 기하/추정시각 overlay를 새로 생성한다. |
| `vehicle_geometry.preview_parameters` | 최종 윤거·타이어폭·반경·축거·외곽·float32 송신 조향 상한을 변환한다. 기어비·lidar 위치/mask·운영 시각 정책은 의도적으로 다루지 않는다. | 이 함수를 그대로 사용하고 빠진 물리 입력/시각/gear를 각 YAML에서 명시 연결한다. |
| r347 `frontwire_6mm_nominal_mask.json` | marker Z=.14의 5구간이다. 최종 vehicle YAML Z=.09와 다른 선택이다. | 운영 선택은 최종 YAML의 4구간만 쓴다. .14 재현 5구간은 별도 `reproduction_only`에 보존한다. |
| preview C1/RGB 실행기 | domain 216 전용이며 topic도 `/hardware_preview/...`다. C1 실제 기본 frame은 `c1_unmounted_preview`, RGB는 `b0390_receive_time_unverified`다. | runtime topic은 명시 제안이며 생산자 연결은 남은 요구다. 원 frame을 `laser_frame`으로 자동 바꾸지 않는다. |
| r395 입력 계약 | `hardware_runtime_candidate_v1`/`host_estimated_v1`/`native_estimated_v1`/false verified를 구별한다. | planner와 독립 guard에 같은 명시 계약을 넣는다. 시간 교정/구동 허가는 생성하지 않는다. |
| r394/r397 모터 입력 | 별도 motor JSON을 명시해야 publisher가 생성된다. | 최종 28count·2.5×81/17·65mm에서 JSON을 만든다. encoder sign과 추정 나이/간격 한계는 명시 가정 파일에서 받는다. |

선택 기하는 wheelbase .128 m, track .1239 m, tire width .0261 m, wheel radius .0325 m, 계획 대칭 외곽 .207×.15 m다. `.207`은 실제 CAD bbox를 덮는 계획 외곽이며 검차 길이 확정이 아니다. 송신 중앙 등가 조향 상한은 기존 함수의 float32 내측 표현을 그대로 쓴다. vehicle의 시뮬 최고속도 5.555...m/s는 host CLI나 주행 정책으로 복사하지 않는다.

camera xyz는 `[.055,0,.198363]`, lidar xyz는 `[.0754,0,.1346]`로 입력 YAML에 남은 출처를 함께 보존한다. camera X/축·높이는 이전 대체 CAD의 추정이고 실제 B0390 설치 광학 중심이 아니다. 현재 local_pursuit RGB 처리에는 이 extrinsic을 받는 매개변수가 없으므로 camera pose를 ROS 입력에 적용했다고 표시하지 않는다. lidar XY는 두 노드의 `motion_lidar_*`와 planner의 `lidar_x_m`에 전달한다. raw ray 기준 yaw가 명시되면 mask 중심각만 같은 센서 좌표로 회전하며 반폭/거리대/여유는 그대로 둔다.

`assumptions.example.yaml`은 **명시 임시 가정**이다. encoder sign +1, lidar yaw 0, IMU roll/pitch 0은 교정 완료값이 아니다. 이 파일을 명시하지 않으면 생성하지 않는다. r394가 실제 발행하는 IMU frame `lsm6dsox_sensor`와 일치시킨다. native LiDAR frame은 현 consumer가 특정 문자열과 대조하는 ROS parameter를 제공하지 않으므로 원 frame과 이 한계를 registration에 남긴다. 이름만 맞춘다고 축이 맞아지는 것은 아니다.

## marker/SDF 정적 생성 경로

`simulation.launch.py`는 vehicle `identification_marker.xyz_m`를 읽은 뒤 비어 있지 않은 `pilot_marker_z`로 Z를 바꾼다. 이 **같은 최종 xyz**를 xacro의 `rear_marker_z`와 `cad_mount.mount` 양쪽에 전달한다. `model.sdf.xacro` 기본도 .090이며 chassis local visual 높이로 바꾸는 감산 뒤 실제 marker 중심은 선택한 ground Z다.

최종 YAML 기본 .09에서 50 mm 판의 높이는 .065..115 m여서 lidar .1346 m와 교차하지 않는다. .14 선택은 .115..165 m로 교차하여 다섯째 구간을 만든다. r377의 `launch_extra.json`/`launch_command.json`은 `pilot_marker_z:=0.140`을 명시했으므로 그 SDF와 5구간 실패를 .09 결과로 소급하지 않는다. r396/r398의 동결 r377 비교도 이번 산출물로 바꾸지 않는다.

현재 launch의 `pilot_marker_z` 명시값 검사는 .115 이상이다. **최종 YAML의 .09 기본값을 쓰려면 해당 override를 비워야 하며 `pilot_marker_z:=0.09`를 넣는 방식은 현재 경로에서 거절된다.** 이는 생성 경로 정적 확인이며 새로운 SDF/시뮬레이션을 생성하거나 실행한 결과는 아니다. r347 파일도 수정하지 않았다.

## 산출물과 검증 인계

`generate_profile.py`는 exact 가정 YAML과 최종 vehicle/sensor YAML을 읽고 기존 `preview_parameters`/`cad_mount.mount`를 재사용한다. 기존3개 mesh는 존재/해시 확인과 동일 CAD helper 호출에만 사용하며 재생성하지 않는다. 누락/legacy 치수·gear/count 불일치·다른 CAD provenance·marker 혼합을 거절한다. 출력은 새 폴더만 허용한다.

- `runtime_geometry.yaml`: `local_pursuit`, `lidar_safety` 각각의 명시 기하·운동·추정시각·거리 한정 구조 mask. planner의 윤거/타이어폭/반경까지 포함한다. guard는 현재 사용하는 몸체 외곽/축거와 운동 좌표를 받는다.
- `motor_feedback.json`: r394 `--motor-feedback-config` exact schema. 한계와 부호를 자동 확정하지 않는다.
- `selected_mask.json`: 최종 .09 선택의 4구간. 자기 반환 거리대 밖 유효 반환은 그대로이고 unknown을 자유로 채우지 않는다.
- `registration.json`: source 해시, 모든 소유 parameter, 원 sensor provenance, 5구간 재현 전용 자료, remap/host CLI 연결값, 미완료 항목.
- `output_manifest.json`: 생성된4파일의 해시. `check`는 이를 먼저 확인하고 외부 최종 profile/host JSON의 소유 필드가 누락·변경되지 않았는지 비교한다. 이 검사는 r398 알고리즘 정책 전체 또는 장치 상태 검증을 대신하지 않는다.

checkout root에서 아래 순서로 실행할 수 있도록 준비했다. 실행 담당자가 새 결과 폴더와 source 해시를 보존한다. 지금까지는 실행하지 않았다.

```bash
python3 -m pytest -q artifacts/validation/2026-11-13/runtime_profile_r399_v1/test_runtime_profile.py

python3 artifacts/validation/2026-11-13/runtime_profile_r399_v1/generate_profile.py generate \
  --vehicle src/arena_description/config/vehicle.final_cad_2026_11_13.yaml \
  --sensor src/arena_description/config/b0390_c1_cad_20261113_final_frontwire_6mm.yaml \
  --assumptions artifacts/validation/2026-11-13/runtime_profile_r399_v1/assumptions.example.yaml \
  --output artifacts/validation/2026-11-13/runtime_profile_r399_v1/generated_v1

python3 artifacts/validation/2026-11-13/runtime_profile_r399_v1/generate_profile.py check \
  --directory artifacts/validation/2026-11-13/runtime_profile_r399_v1/generated_v1
```

단위검사는 두 노드의 최종값, 4/5구간 분리·r347 재현, nonzero yaw의 중심각 변환, 먼 반환/unknown 보존, legacy geometry/gear/count/시각 혼합 거절, 원 source 불변, YAML 중복키/출력 덮기·변조 거절을 다룬다. r347 부동소수 계산값의 비교는 각 수치의 1e-12 범위 단위검사이며 mask 판정선/마진을 변경하지 않는다.

## r398 및 운영 실행자에게 필요한 연결

1. r398은 알고리즘/속도/출발·중지 정책을 별도로 제공한다. 최종 두 노드 profile에 이 overlay의 소유 필드를 명시 합성하고 `check --profile <최종 YAML> --motor-config <host JSON>`으로 대조한다. CLI의 뒤쪽 `-p` override도 같은 값이어야 하며 이 생성기가 임의 추가 CLI를 검사했다고 주장하지 않는다.
2. `/scan/native`를 registration의 runtime C1 topic으로 **두 노드 모두** remap한다. camera raw topic은 planner로, r394 IMU topic은 두 motion consumer로 연결한다. `/drive → 독립 guard → /drive/safe → r394`는 유지한다. native C1/RGB 생산자의 domain216-only preview 제약과 실제 topic/frame 전달은 별도 runtime 연결 작업이다.
3. r394 CLI에 생성된 motor JSON, 일치 IMU topic, 최종 송신 조향 상한을 넘긴다. `--max-speed-mps`는 r398의 명시 선택과 일치시킨다. `--enable-drive-wire`는 이 생성기로 추가하지 않으며 clock policy도 별도 명시 계약을 유지한다.
4. 원 source/time basis, 설치 전 encoder sign·IMU축·lidar yaw·camera 위치, effective rolling radius·CAD 조향과 실제 실장 차이는 미검증으로 남는다. 설정 파일 생성은 실물 동작·정지·전체 코스 승인 또는 완료가 아니다.
