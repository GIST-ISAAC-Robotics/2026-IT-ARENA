# r489 중단 인계

사용자 일시 중단 요청에 따라 stage만 보존했습니다. 공통 통합 및 WSL/Jetson/ROS 실행은 하지 않았습니다. 실행 중인 자기 소유 명령/PID는 없습니다. 비ROS 회귀도 아직 실행하지 않았으므로 검증 완료 후보가 아닙니다.

저장된 구현은 stage의 lidar_safety.py, partial_observation.py, known_structure.py 및 신규 independent_guard_route.py입니다. 원본 3개는 base에 있습니다. prepare_stage.py는 이 초기 stage 생성 스크립트이며 다시 실행하면 stage를 덮으므로 수정 후 무조건 재실행하지 않습니다. 파일 해시는 pause_receipt.json에 있습니다.

현재 stage: selected_r181+traffic_recovery 선택, 전방 반환 근거, 기본 정책과 공유하는 기존 소실 기억 처리, 선택 guard에서 가상 먼 구조 그림자 제한 제외가 구현됐습니다. 실제 숨은 점 기억은 유지합니다. 기본 정책 출력/기억 순서 동일성과 대표 위험 거절은 아직 검증하지 않았습니다.

중단 직전 root 검토 미반영 항목:
- lidar_safety의 새 min(swept_limit(current), swept_limit(previous))를 기존 현재 조향 swept_limit 한 번으로 되돌려야 합니다. 옛 조향 전체 제동 곡선 때문에 새 불필요 정지를 만들지 않기 위함입니다. 직전 조향은 기존 소실 기억 near_sweep에 남깁니다.
- front_evidence 입력은 on_scan→mask_scan의 CAD 거리 bands 처리 이후입니다. 자기 반환만/후방 반환만은 거절하고 같은 각도의 더 먼 외부 반환은 유지하는 반례가 필요합니다. angle sector 전체를 추가로 빼면 안 됩니다.
- scripts/runtime_installed_limits.py::derived_profile의 새 파생 YAML에서 planner selected_r181 및 양쪽 traffic_recovery가 명시된 경우 guard에도 선택값을 기록하는 최소 연결은 아직 구현하지 않았습니다. 기존 r401/r407 YAML을 덮지 않습니다.

refine_stage.py 생성/실행 호출은 dispatch 전에 중단되었습니다. 따라서 위 검토 수정 및 builder 연결은 적용되지 않았습니다.

r485 보존 입력은 ../solo_variation_recovery_r485_v1/failure_inputs/ 아래 있습니다. 최초 기록은 sim 50.245s, progress 41.88m, /drive .8→safe0, lateral_metric_outside_trial입니다. 정확한 guard timer/callback 순서·previous_safe_steering·내부 기억이 없으므로 full guard exact 재현으로 부를 수 없습니다. safety_clearance_m=0은 partial 거절 뒤 초기값이며 실제 충돌거리 0의 증거가 아닙니다.

다음 한 단계: 사용자 재개 후 root 검토사항(기존 current-steering 제동 복원과 최소 YAML 선택 연결)을 stage에만 반영하고, 자기 반환 경계 및 기본 정책 exact를 포함한 소수 비ROS 회귀를 실행합니다. 공통 통합은 root 검토 뒤 별도입니다.
