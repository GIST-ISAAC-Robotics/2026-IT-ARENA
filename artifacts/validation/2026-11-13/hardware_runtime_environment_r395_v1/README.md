# 실물 추정 시각의 선택형 운용 경로 r395

P0-2/P0-4의 연결 준비다. 기존 hardware_shadow_v1의 진단 출력 격리를 보존하고 별도 hardware_runtime_candidate_v1에서만 /drive → 독립 보호층 → /drive/safe를 연결한다. 실제 시각 교정 완료를 주장하지 않고 host_estimated_v1·native_estimated_v1·motion_scan_timing_verified=false를 강제한다. MCU ARM/출력 권한은 부여하지 않는다.

공통 코드는 아직 수정하지 않았다. stage의 __init__.py는 검사 시 공통 패키지와 결합하기 위한 overlay 전용이다. 새 선택형 경로는 기존 저장 시뮬레이션 결과를 실제 주행 성공으로 바꾸지 않는다. legacy shadow의 시각 미검증 거절도 그대로 둔다.

검증 준비: 선택 경계·기존 시각/보호층 검사·실제 ROS 노드 생성 및 종료를 수행한다. 새 실물/시뮬 주행은 아직 미실시. 실제 센서 입력·엔코더 출력과 운용 프로필 결합은 후속이다.

## 첫 검사와 기록 호환 수정

verify_v1은172통과·1실패다. 실패는 기존 공통 g1_g2_node에도 있던 verified_only 진단 필드 추가와 이전 shadow test의 정확 사전 비교 불일치다. 실제 새 경로 생성/종료·추정 시각 경계 검사는 통과했고 실행/동결197파일 불변을 확인했다. stage에서 legacy verified_only일 때 그 필드를 생략하고 명시 host_estimated_v1에서는 보존하도록 기존 status와 같은 기록 호환 규칙을 적용했다. 원 실패와 수정 전 소스를 보존한다.

verify_v2에서 영향받은10검사가 모두 통과했고197동결파일 전후불변을 확인했다. stage6개모듈과 검사파일을 공통에 이관했다. overlay 전용 __init__.py는 이관하지 않았다. 실제 ROS에서 /drive → /drive/safe의 노드생성/계약/종료까지이며 센서 양수 주행·실물 출력 검증은 후속이다.
