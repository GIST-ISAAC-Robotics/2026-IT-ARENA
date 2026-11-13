# r473 제한 독립 검토 — 후보 보류

사용자가 과도한 관측 관문을 크게 줄이도록 방향을 정정하여 **r473 세밀한 합성·양자 여유 후보의 채택을 보류**했다. 이 문서는 그 전에 수행한 Windows 읽기 전용 검토만 봉인한다. **공통 미통합·무실행**이며, 이 담당은 WSL·Jetson·ROS·새 시뮬이나 후보 시험을 실행하지 않았다. 주 담당이 source metadata 몇 줄을 stage에 추가했다고 전달했으나 그 후 재검토·추가 개선은 하지 않았다.

검토 범위는 `low_speed_observation.py`, `released_body_gap.py`, `current_path_recheck/_gaps.py`, `race_runtime_policy/current_region.py`, `known_structure.py`의 초안 차이와 직접 연결된 기존 source/profile 계약이다. 저장 입력 진단은 별도 [r472](../solo_stall_diagnostics_r472_v1/REVIEW.md)의 동결 r469 결과이며, r473 후보의 통과 근거가 아니다.

## 전달한 수정 의견

최초 초안의 `released_body_gap.check`는 기존 body proof가 `raw_scene_mismatch`로 거절돼도 새 `current_mask_shadow` 수용으로 덮을 수 있었다. 새 mixed 경로는 scene 자체 유효성과 원 +Inf를 확인했지만 raw scan과 scene의 frame·취득시각·원 angles·길이·반환거리 정합 대조는 빠져 있었다. 기존 `structure_region`의 source 대조와 `current_region`의 profile 검사를 거치지 않는 새 body 진입점이므로, 같은 현재 근거를 합성한다는 계약에서 대조를 유지할 것을 권고했다. 정상 r469 주행에서 입력 불일치가 발생했다는 보고가 아니며, 이 검토에서 반례 실행도 하지 않았다.

주 담당은 이 의견 뒤 source metadata 대조를 stage에 추가했다고 알렸다. 변경 후 정확한 검사 범위·해결 여부는 이 담당이 재확인하지 않았다. 후보 보류 뒤 이를 완성하거나 채택의 근거로 삼기 위한 추가 작업은 하지 않는다.

## 그 밖의 정적 관찰과 한계

초안은 기존 `current_lane_headroom_mps`를 입구 범위에 전달하며, 실제 속도를 clamp하는 변경은 보이지 않았다. 원 점유 padding·앞면 가림·측방 상태·운영 STOP을 직접 제거하는 수정도 검토한 다섯 파일에는 없었다. 기본 headroom 0 및 mixed 옵션 없는 경로는 유지하도록 작성됐지만, 실행 동등성은 검증하지 않았다.

작은 과제한 가능성도 있었다. 목표 .15 이하에서 기존 body proof가 이미 수용한 독립 +Inf 공백과 별도의 mask가 함께 있으면, 모든 nonfree query를 다시 mixed helper에 보내는 순서가 해당 공백에도 masked 성분을 요구할 수 있다. 이는 r469의 인접 혼합 성분과 다른 경우이며 정적 가능성으로만 남긴다. 추가 예외나 리팩터링으로 확대하지 않았다.

검토는 여기서 종료한다. 다음 방향은 주 담당의 지시에 따라 현재 센서 경로·실제 점유 baseline을 유지하면서 자기 가림·일부 무반환·과거 측방 이력만으로 추가 STOP하는 관문을 크게 줄이는 것이다. source metadata 계약·실제 충돌·경로 상실·운영 STOP·독립 guard는 유지한다. 이 방향의 설계·구현·실행은 본 r473 검토 범위에 포함하지 않는다.
