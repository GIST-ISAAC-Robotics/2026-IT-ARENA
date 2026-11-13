# r466 v2 — 중첩 검출의 실제 검은 코드 외곽 선택

상태: **stage 수정·Windows 32검사·ROS import2 및 제한 대표 통과**. WSL 합성 ROS만 사용했으며 Jetson/실물 장치는 사용하지 않았다. 원 `execution_v1/run1`, 원 `stage_source.zip`, 수정 전 stage(`stage_before/`)와 당시 문서를 보존했다.

## 최소 변경

runtime 변경은 `vehicle_marker_geometry.py` 한 모듈이다. 원 검출기나 신호·벽·free·주행 알고리즘을 바꾸지 않았다. 단일 검출은 기존 pose 경로를 유지하며, 같은 ID가 중복된 경우만 다음을 수행한다.

1. 평균 변 크기비≤1.25, 중심차≤큰 quad 변의0.10, IoU≥0.70, 대응 corner 차≤0.20변으로 같은 겹침 군인지 확인한다. 분리된 같은 ID나 decoded corner 방향이 다른 검출은 병합하지 않는다. 최대 면적 quad는 군의 기준일 뿐 자동 대표가 아니다.
2. 원 RGB를 후보 quad의 canonical 코드로 투영해 검은 border 연속성과 사전/ID의 내부 bit를 확인한다. 검은 border 비율·bit 일치율≥0.90, 흑백 대비≥20/255를 사용한다.
3. 원 영상의 네 변을 따라9쌍씩 안/밖 표본을 읽는다. 안쪽이 검고 바깥이 밝은 경계가3변 이상 있어야 한다. 후보 중 border 연속성→bit 일치→경계 대비 순으로 대표를 고른다. 면적 최댓값은 선택 점수가 아니다.
4. 선택된 **원 quad 좌표**를 기존 PnP에 넣는다. 새 corner나 물리 크기를 합성하지 않는다. PnP 재투영 검사는 유지하지만 이를 외곽 크기의 단독 근거로 사용하지 않는다.

이 수치는 겹친 영상 검출을 구별하는 국소 휴리스틱이다. 실제 차/마커 동일성이나 절대 거리 오차를 보장하지 않는다. 필수 대표 결과 뒤에 추가 CV 프로젝트로 확장하지 않는다.

## 원 실패와 반례 결과

`quad_comparison.json`과 저장 PNG는 **원 대표의 합성 RGB를 Windows에서 재구성한 것**이다. 실제 카메라 사진이나 실제 인쇄물 측정이 아니다.

| 후보 | 검은 border | 내부 bit | 외곽 경계 | 명목 camera depth | 선택 |
|---|---:|---:|---:|---:|---|
| 원79 px 검은 코드 | 100% | 100% | 4변 | 0.467351 m | 선택 |
| 원68.007 px 안쪽 검출 | 88.30% | 100% | 0변 | 0.542893 m | 제외 |
| 더 큰 흰 배경판89 px | 62.22% | 100% | 0변 | 0.414840 m | 제외 |
| 실제 바깥 검은 frame91 px | 72.92% | 100% | 4변 | 0.405722 m | 제외 |

원79/68의 중심차0, IoU0.7411은 같은 겹침 군으로 인정된다. 79 px 선택의 polygon과 거리는 동일 원 quad를 직접 넣은 PnP 결과와 exact다. 안쪽 검출을 고르면 이 합성 입력에서 camera depth가 약16.16% 커지고, 별도 frame을 고르면 약13.19% 작아진다. v2는 두 왜곡을 이번 입력에서 피했다.

각 후보의 PnP 재투영오차는 약10⁻⁹~10⁻⁶ px로 작았다. 특히 별도 검은 frame도 네 변 명암 대비255와 bit 일치100%였으므로, **큰 quad·경계 명암·재투영만으로는 코드 크기를 고를 수 없다**. border 안의 흰 틈을 포함한 연속성 표본이 이 반례를 구별했다.

`windows_duplicates_v3.log`/`windows_duplicates.xml`: **32 passed, exit0**. 원 좌표 순서 반전, 큰 흰 판/검은 frame, 분리 동일 ID, 큰 크기 차이, corner 순서 오류, 다른 code ID·빈 영상, 단일 검출 원결과와 기존 geometry/TTL/간이보정/CLI 검사를 포함한다. 기존197개 전부를 다시 수행한 결과라고 쓰지 않는다.

## 제한된 공간의 간이 보정

기존 `adjust_vehicle_marker_profile.py`는 정면 optical depth의 양수 표본1~2개를 받는다. 0.2 m와0.5 m의 한 표본을 받아 동일 초점거리 수식을 계산하는 계약을 검사했다. 약61 cm 상자 크기를 정답 거리나 새 camera K로 넣지 않았다. 카메라 높이의 마커를 정면에서 읽고 실제 광학 기준 거리를 입력하는 기존 절차를 그대로 사용할 수 있다. 실측 전 null 템플릿과 `approximate_v1`/정확도 미검증 표시는 유지한다.

## 실제 ROS 대표 결과

`execution_v2/source.zip`은 v1의232파일 snapshot에 geometry 한 파일만 교체했다. profile·ROS 검증 harness·입력·판정선은 동일하다. workspace만 `/home/jinhyeong/it-arena-r466-ros-b`, 결과 경로만 `execution_v2`로 분리한다. 원 v1 gate/실패를 지우지 않았다.

root의 WSL 단독권 전달 뒤 다음 고정 명령을 한 번 실행했고 exit0으로 종료했다. 아래는 실행 이력이며 추가 실행 지시가 아니다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/vehicle_marker_r466_v1/execution_v2/run_once.sh'
```

범위는 ROS import2·실제 VisionProcess→MarkerChannel→LidarSafety timer·부재/front/side/TTL·마지막0·정상 회수다. 별도 주행/전체 native replay/실물 비용·정확도 검증은 추가하지 않는다. 실제 합성 ROS에서 정면 cap이 적용되고 부재/측방/TTL 뒤 원출력이 유지됐다. 이 결과는 실차 주행이나 전체 plan 재생이 아니다.

`changes.json`은 현재 전체 이관 목록, `delta_from_v1.diff`는 이번 v2 수정만, `stage_source.zip`은 현재 이관 archive다. 최상위 기존 v1 archive는 그대로 보존한다.


| 실제 ROS 사례 | 출력 속도(m/s) | 판정 |
|---|---:|---|
| 부재 | .75 | 원출력 유지 |
| 정면 | .067812~.20 | vehicle_marker_braking |
| 측방 | .75 | 원출력 유지 |
| TTL 만료 | .75 | 보조 packet 소거·원출력 복귀 |

마지막0을 관측했다. RGB20/20 및 packet20, wheel/gyro 각517, 거절0이었다. 실제 ROS의 중복 metadata도 count2/support1, IoU.7411, 검은 border/bit100%, 지지 경계4변으로 남았다. [packet 발췌](ros_duplicate_evidence.json)는 원 `execution_v2/run1/marker_packets.jsonl`에서 추출했다.

같은 실행 묶음에서 exit0·3노드 destroy·Vision child forced=false/exit0·reported child 생존[]·232입력 불변을 확인했다. 후속 별도 `/proc` 감사는 **WSL 진입 E_UNEXPECTED exit1로 미수행**이다. [실패 영수증](../execution_v2/final_pid_audit_failure.json)을 보존하며 별도 전역 PID0 확인으로 바꾸지 않는다. 재시작/재호출 없이 WSL을 root에 반환했다.

프로젝트 `numeric native_v1`은 frozen source/명령 범위상 미선택이며 별도 backend 런타임 계측은 하지 않았다. OpenCV4.6 detector/PnP는 사용했다. 추가7×7/PnP/중복 표본 비용은 `stages_ms.markers` 밖이지만 전체 `duration_ms`에 포함된다. 처리20개·합계193.036918ms·최대26.989059ms는 이 합성 WSL 대표의 기록이며 Jetson/실제 주행 부하의 증명이 아니다.

소스 archive와 원 v1 네 근거, stage22파일의 SHA를 Windows에서 다시 대조했다([final_seal.json](final_seal.json)). 공통 미통합이며 후속은 root의 이관 검토다. 이관 후 실제 설치/간이 보정은 별도 작업으로 남긴다.
